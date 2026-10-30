#!/usr/bin/env python3
"""HIL-10 B route/AEB regression and controlled communication-loss tests.

Modes: route, aeb, fault-odometry, fault-brake, fault-lidar.
Runs its own B LiDAR sender. Never run a second sender or B controller.
Odometry-loss injection blocks the control input only; a separate observer
continues reading actual Gazebo odometry to verify the physical stop.
"""

import argparse
import math
from pathlib import Path
import threading
import time
import traceback

import ruskotunturi_hil_b_route_test as legacy
from ruskotunturi_hil9e_aeb_test import physical_clearance, REQUIRED_CLEARANCE_M
from hil10_safety import HILLink, OdometryMonitor, RunLog
from hil10_runtime import ManagedLidarSender, odometry_values, verify_stop, stop_repeatedly
from hil_startup_diagnostics import StartupReadiness

MODES = ("route", "aeb", "fault-odometry", "fault-brake", "fault-lidar")
DETECTION_LIMIT_S = 1.0
FAULT_STOP_LIMIT_S = 3.0
STOP_HOLD_S = 1.0


class HIL10B(legacy.RuskotunturiHILBRouteTest):
    def __init__(self, args):
        # Initialize all callback state before subscriptions are enabled.
        self.mode = args.mode
        self.fault_after = args.fault_after
        self.r2100_host = args.r2100_host
        self.link = HILLink(args.r2100_host, args.udp_port)
        self.observer = OdometryMonitor()
        self.log = RunLog(args.results_dir, self.mode)
        self.lock = threading.Lock()
        self.x = self.y = None
        self.yaw = 0.0
        self.last_pose_time = None
        self.drop_odometry = threading.Event()
        self.route = legacy.route_geometry(legacy.B_START, legacy.B_GOAL)
        self.node = legacy.Node()
        self.pub_b = self.node.advertise(legacy.CMD_B_TOPIC, legacy.Twist)
        if not self.node.subscribe(legacy.Pose_V, legacy.POSE_TOPIC, self.pose_callback):
            raise RuntimeError("B pose subscription failed")
        if not self.node.subscribe(legacy.Odometry, legacy.ODOM_B_TOPIC, self.odom_callback):
            raise RuntimeError("B odometry subscription failed")
        self.lidar = ManagedLidarSender(args.r2100_host, args.lidar_port)
        self.injected = None
        self.detected = None
        self.confirmed_stop = None
        self.measurements = {}
        self.stop_verified = False
        self.startup_failure_reason = "startup_not_ready"

    def record_stop(self):
        _brake, reason = self.link.decision()
        self.log.record(self.link, 0.0, 0.0, reason, self.observer,
                        fault_injected_s=self.injected)

    def odom_callback(self, msg):
        speed, stamp = odometry_values(msg)
        self.observer.update(speed, stamp)
        if not self.drop_odometry.is_set():
            self.link.odom.update(speed, stamp)

    def pose_fresh(self):
        with self.lock:
            return (self.last_pose_time is not None
                    and 0 <= time.monotonic() - self.last_pose_time <= 0.5
                    and all(math.isfinite(v) for v in (self.x, self.y, self.yaw)))

    def inject_fault(self):
        if self.mode == "fault-odometry":
            self.drop_odometry.set()
        elif self.mode == "fault-brake":
            self.link.drop_brakes = True
        elif self.mode == "fault-lidar":
            self.lidar.suspend()
        self.injected = time.monotonic()
        print(f"\n[FAULT] {self.mode} injected at {self.injected:.6f}", flush=True)

    def expected_fault(self):
        if self.mode == "fault-odometry":
            return (not self.link.odom.fresh()
                    and self.link.fault in {"odometry_timeout", "sensor_timeout"})
        if self.mode == "fault-brake":
            return not self.link.brake_fresh() and self.link.fault == "brake_timeout"
        return (self.link.brake_fresh() and self.link.brake_active
                and self.link.brake_reason == 3 and self.link.fault == "sensor_timeout")

    def startup_readiness(self):
        return StartupReadiness(
            r2100_host=self.r2100_host,
            pose_fresh=self.pose_fresh(),
            odometry_fresh=self.link.odom.fresh(),
            brake_reply_fresh=self.link.brake_fresh(),
            brake_active=self.link.brake_active,
            brake_reason=self.link.brake_reason,
            fault=self.link.fault,
            lidar_error=self.lidar.error,
        )

    def wait_ready(self):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            self.stop_b()
            self.link.pump()
            brake, _reason = self.link.decision()
            state = self.startup_readiness()

            if self.link.fault or self.lidar.error:
                self.startup_failure_reason = (
                    state.failure_reason() or "startup_not_ready"
                )
                print()
                print(state.format_report(), flush=True)
                return False

            if self.pose_fresh() and not brake:
                with self.lock:
                    distance = math.hypot(
                        self.x - legacy.B_START[0],
                        self.y - legacy.B_START[1],
                    )
                if distance > 1.5:
                    print("Start-position check failed. Restart the Gazebo world.")
                    self.startup_failure_reason = "startup_start_position_invalid"
                    print()
                    print(state.format_report(), flush=True)
                    return False
                return True

            time.sleep(0.05)

        state = self.startup_readiness()
        self.startup_failure_reason = (
            state.failure_reason() or "startup_not_ready"
        )
        print()
        print(state.format_report(), flush=True)
        return False

    def execute(self):
        if not self.wait_ready():
            return False, self.startup_failure_reason
        start = time.monotonic()
        moving_since = None
        last_print = 0.0
        aeb_stop_started = False
        while time.monotonic() - start < 90.0:
            now = time.monotonic()
            self.link.pump()
            if self.lidar.error:
                return False, "lidar_send_error"
            if not self.pose_fresh():
                return False, "pose_timeout"
            requested, steering, goal, route_s, lateral = self.route_command()
            applied, turn, reason = self.link.apply(requested, steering)
            observed = self.observer.snapshot()
            if not self.observer.fresh():
                return False, "verification_odometry_timeout"
            moving = applied > 0 and observed.speed >= 0.20
            if moving and moving_since is None:
                moving_since = now
            if self.mode.startswith("fault-") and self.injected is None:
                if moving and moving_since is not None and now - moving_since >= self.fault_after:
                    self.inject_fault()
            if self.mode == "route" and goal:
                stopped = verify_stop(self.observer, self.stop_b, self.link.pump,
                                      on_sample=self.record_stop)
                self.stop_verified = stopped
                self.link.decision()
                return (stopped and not self.link.fault,
                        "route_complete" if stopped and not self.link.fault else self.link.fault or "stop_not_verified")
            if goal:
                return False, "goal_reached_without_required_intervention"

            # A latched timeout is a FAIL in a normal run, an expected event
            # only after deliberate injection in the matching fault test.
            if self.link.fault and self.injected is None:
                return False, self.link.fault
            if self.mode == "aeb":
                valid_aeb = (moving_since is not None and self.link.brake_fresh()
                             and self.link.brake_active and self.link.brake_reason in (1, 2))
                if valid_aeb and not aeb_stop_started:
                    self.stop_b()
                    self.observer.begin_stop()
                    aeb_stop_started = True
                    self.measurements["aeb_reason"] = reason
                    self.measurements["trigger_route_s_m"] = route_s
                if aeb_stop_started and not valid_aeb:
                    return False, "aeb_state_lost"
                if aeb_stop_started and self.observer.stopped():
                    clearance = physical_clearance(route_s)
                    self.measurements.update(clearance_m=clearance, stop_route_s_m=route_s,
                                             required_clearance_m=REQUIRED_CLEARANCE_M)
                    self.stop_verified = True
                    return (clearance >= REQUIRED_CLEARANCE_M,
                            "aeb_stop_verified" if clearance >= REQUIRED_CLEARANCE_M else "clearance_violation")

            if self.injected is not None:
                elapsed = now - self.injected
                if self.expected_fault() and self.detected is None:
                    self.detected = now
                    self.stop_b()
                    self.observer.begin_stop()
                    print(f"[FAULT] detected: {reason}; delay={elapsed:.3f} s", flush=True)
                if self.detected is None and elapsed > DETECTION_LIMIT_S:
                    return False, "fault_not_detected_in_time"
                if self.detected is not None:
                    if self.detected - self.injected > DETECTION_LIMIT_S:
                        return False, "fault_detection_too_late"
                    if applied != 0.0 or turn != 0.0:
                        return False, "motion_command_after_fault"
                    if self.observer.stopped():
                        if self.confirmed_stop is None:
                            self.confirmed_stop = now
                        if self.confirmed_stop - self.injected > FAULT_STOP_LIMIT_S:
                            return False, "fault_stop_too_late"
                        if now - self.confirmed_stop >= STOP_HOLD_S:
                            self.stop_verified = True
                            return True, "fault_stop_and_hold_verified"
                    elif self.confirmed_stop is not None:
                        return False, "stop_not_held"
                    if self.confirmed_stop is None and elapsed > FAULT_STOP_LIMIT_S:
                        return False, "fault_stop_not_verified"

            self.publish_b(applied, turn)
            self.log.record(self.link, requested, applied, reason, self.observer,
                route_s_m=route_s, lateral_m=lateral, fault_injected_s=self.injected)
            if now - last_print >= 0.5:
                print(f"B s={route_s:.3f} m | measured={observed.speed:.3f} m/s | cmd={applied:.3f} | reason={reason}", flush=True)
                last_print = now
            time.sleep(0.05)
        return False, "mission_timeout"

    def run(self):
        print(f"HIL-10 B TEST: {self.mode}; integrated raw LiDAR sender; separate stop observer")
        passed, reason = False, "not_started"
        try:
            passed, reason = self.execute()
        except KeyboardInterrupt:
            reason = "interrupted"
        except Exception as exc:
            reason = "exception: " + str(exc)
            traceback.print_exc()
        finally:
            # Stopping must still run if writing the result trace fails.
            try:
                stop_repeatedly(self.stop_b)
                self.record_stop()
            finally:
                try:
                    self.lidar.close()
                finally:
                    self.link.close()
        sample = self.observer.snapshot()
        return self.log.finish(passed, reason, final_speed_mps=sample.speed if sample else None,
            stop_verified=self.stop_verified,
            observer_stop_samples=self.observer.stop_samples, fault=self.link.fault,
            detection_latency_s=self.detected - self.injected if self.detected is not None else None,
            stop_latency_s=self.confirmed_stop - self.injected if self.confirmed_stop is not None else None,
            detection_limit_s=DETECTION_LIMIT_S, fault_stop_limit_s=FAULT_STOP_LIMIT_S,
            stop_hold_s=STOP_HOLD_S, **self.measurements)


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, required=True)
    parser.add_argument("--r2100-host", required=True)
    parser.add_argument("--udp-port", type=int, default=15000)
    parser.add_argument("--lidar-port", type=int, default=15010)
    parser.add_argument("--fault-after", type=positive_seconds, default=2.0, help="seconds after observed motion before injecting the selected fault")
    parser.add_argument("--results-dir", type=Path, default=Path(__file__).resolve().parents[1] / "results" / "hil10")
    args = parser.parse_args()
    test = HIL10B(args)
    raise SystemExit(0 if test.run() else 1)


if __name__ == "__main__":
    main()
