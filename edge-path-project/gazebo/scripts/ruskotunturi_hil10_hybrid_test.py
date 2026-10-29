#!/usr/bin/env python3
"""HIL-9D regression with fresh-odometry gating and verified final stop.

Add-only: imports the existing Phase 6 route coordinator; writes its own logs.
Start a fresh Phase 6 Gazebo world and fresh R2100 processes for every run.
This process also sends B's raw LiDAR; do not run another LiDAR sender.
"""

import argparse
import math
from pathlib import Path
import time
import traceback

from gz.msgs10.odometry_pb2 import Odometry
import ruskotunturi_phase6_two_vehicle_test as phase6
from hil10_safety import HILLink, RunLog
from hil10_runtime import ManagedLidarSender, odometry_values, verify_stop, stop_repeatedly


class HILFault(RuntimeError):
    pass


class HIL10Hybrid(phase6.Phase6TwoVehicleCoordinator):
    def __init__(self, host, port, lidar_port, results):
        self.link = HILLink(host, port)
        self.log = RunLog(results, "hybrid")
        self.lidar = None
        self.phase6_summary = {}
        super().__init__()
        if not self.node.subscribe(Odometry, f"/model/{phase6.MODEL_B}/odometry", self.odom_callback):
            raise RuntimeError("B odometry subscription failed")
        self.lidar = ManagedLidarSender(host, lidar_port)

    def odom_callback(self, msg):
        self.link.odom.update(*odometry_values(msg))

    def record_stop(self):
        _brake, reason = self.link.decision()
        self.log.record(self.link, 0.0, 0.0, reason)

    def direct_stop(self):
        # Cleanup must not depend on the UDP connection that may have failed.
        phase6.Phase6TwoVehicleCoordinator.publish(self, self.pub_a, 0.0, 0.0)
        phase6.Phase6TwoVehicleCoordinator.publish(self, self.pub_b, 0.0, 0.0)

    def publish(self, publisher, speed, steering):
        if publisher is not self.pub_b:
            return super().publish(publisher, speed, steering)
        self.link.pump()
        applied, turn, reason = self.link.apply(speed, steering)
        super().publish(publisher, applied, turn)
        self.log.record(self.link, speed, applied, reason)
        if self.link.fault:
            self.direct_stop()
            raise HILFault(self.link.fault)
        if self.lidar is not None and self.lidar.error:
            self.direct_stop()
            raise HILFault("lidar_send_error: " + self.lidar.error)

    def reset_world(self):
        # Avoid a full Gazebo reset while sensor/transport callbacks are active.
        # Validate a newly started world instead; never teleport/reset a test.
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            self.direct_stop()
            self.link.pump()
            now = time.monotonic()
            with self.lock:
                a, b = dict(self.a), dict(self.b)
            ready = all(v["pose_received"] and v["lidar_received"]
                        and now - v["last_pose_time"] <= 0.5
                        and now - v["last_lidar_time"] <= 0.5 for v in (a, b))
            brake, _reason = self.link.decision()
            if ready and not brake:
                if (math.hypot(a["x"] - phase6.A_START[0], a["y"] - phase6.A_START[1]) > 1.5
                        or math.hypot(b["x"] - phase6.B_START[0], b["y"] - phase6.B_START[1]) > 1.5):
                    print("Start-position check failed. Restart the Gazebo world.")
                    return False
                print("Fresh world, B odometry and R2100 DRIVE state ready.")
                return True
            time.sleep(0.05)
        print("Startup failed: check fresh world, R2100 processes and LiDAR stream.")
        return False

    def print_result(self, duration):
        # Preserve Phase 6 acceptance conditions, but publish ONE final result
        # only after the additional HIL stop check. Do not write the SIL CSV.
        with self.lock:
            a, b = dict(self.a), dict(self.b)
            passed = (a["goal_reached"] and b["goal_reached"]
                      and self.coordinator_wait_events >= 1
                      and self.coordinator_release_events == 1
                      and not self.conflict_overlap_violation
                      and not self.terrain_violation and not self.lidar_violation
                      and a["minimum_clearance_seen"] >= phase6.HARD_BODY_CLEARANCE_M
                      and b["minimum_clearance_seen"] >= phase6.HARD_BODY_CLEARANCE_M
                      and self.end_reason == "both_goals_reached")
        finite = lambda value: value if math.isfinite(value) else None
        self.phase6_summary = dict(vehicle_a_goal=a["goal_reached"], vehicle_b_goal=b["goal_reached"],
            coordinator_wait_events=self.coordinator_wait_events,
            coordinator_release_events=self.coordinator_release_events,
            conflict_overlap=self.conflict_overlap_violation, terrain_violation=self.terrain_violation,
            lidar_violation=self.lidar_violation,
            vehicle_a_min_clearance_m=finite(a["minimum_clearance_seen"]),
            vehicle_b_min_clearance_m=finite(b["minimum_clearance_seen"]),
            phase6_duration_s=duration, phase6_pass=bool(passed))
        return passed

    def run(self):
        passed, stopped, reason = False, False, "not_started"
        print("HIL-10 HYBRID: HIL-9D regression + odometry timeout + fresh stop samples")
        try:
            base_pass = super().run()
            reason = self.end_reason
            if base_pass:
                stopped = verify_stop(self.link.odom, self.direct_stop, self.link.pump,
                                      on_sample=self.record_stop)
                self.link.decision()
                passed = stopped and self.link.fault is None
                reason = "hybrid_complete" if passed else (self.link.fault or "stop_not_verified")
        except HILFault as exc:
            reason = str(exc)
        except KeyboardInterrupt:
            reason = "interrupted"
        except Exception as exc:
            reason = "exception: " + str(exc)
            traceback.print_exc()
        finally:
            try:
                stop_repeatedly(self.direct_stop)
                self.record_stop()
            finally:
                try:
                    if self.lidar is not None:
                        self.lidar.close()
                finally:
                    self.link.close()
        sample = self.link.odom.snapshot()
        return self.log.finish(passed, reason, stop_verified=stopped,
            final_speed_mps=sample.speed if sample else None, fault=self.link.fault,
            stop_samples=self.link.odom.stop_samples, **self.phase6_summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--r2100-host", required=True)
    parser.add_argument("--udp-port", type=int, default=15000)
    parser.add_argument("--lidar-port", type=int, default=15010)
    parser.add_argument("--results-dir", type=Path, default=Path(__file__).resolve().parents[1] / "results" / "hil10")
    args = parser.parse_args()
    test = HIL10Hybrid(args.r2100_host, args.udp_port, args.lidar_port, args.results_dir)
    raise SystemExit(0 if test.run() else 1)


if __name__ == "__main__":
    main()
