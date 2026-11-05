#!/usr/bin/env python3
"""Experimental standalone Metsakone LiDAR-only SIL emergency-braking trial.

Launch the separate metsakone_aeb_world.sdf first. This trial is NOT the
validated common SIL/HIL scenario and does not connect to the physical R2100.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

from metsakone_aeb_policy import brake_decision, front_clearance, route_geometry


ROOT = Path(__file__).resolve().parents[3]
SCENARIO_PATH = (ROOT / "edge-path-project" / "gazebo"
                 / "scenarios" / "metsakone_emergency_braking.json")


def load_scenario():
    data = json.loads(SCENARIO_PATH.read_text())
    if data["schema_version"] != 1 or data["name"] != "metsakone_emergency_braking":
        raise ValueError("unexpected Metsakone scenario schema or name")
    return data


class MetsakoneAEB:
    def __init__(self, scenario, Node, Twist, Pose_V, Odometry, LaserScan, Contacts):
        self.config = scenario
        self.safety = scenario["safety"]
        self.acceptance = scenario["acceptance"]
        self.lock = threading.RLock()
        self.node = Node()
        self.Twist = Twist
        self.pose = None
        self.speed = None
        self.front = None
        self.pose_at = None
        self.odom_at = None
        self.lidar_at = None
        self.odom_seq = 0
        self.hit_obstacle = False
        self.command = self.node.advertise(scenario["topics"]["cmd_vel"], Twist)
        subs = (
            (Pose_V, scenario["topics"]["pose"], self.pose_callback),
            (Odometry, scenario["topics"]["odometry"], self.odom_callback),
            (LaserScan, scenario["topics"]["lidar"], self.lidar_callback),
            (Contacts, scenario["topics"]["front_contacts"], self.contact_callback),
        )
        for kind, topic, callback in subs:
            if not self.node.subscribe(kind, topic, callback):
                raise RuntimeError("Gazebo subscription failed: " + topic)

    def pose_callback(self, message):
        model = self.config["vehicle"]
        for pose in message.pose:
            if pose.name == model or pose.name.endswith("::" + model):
                with self.lock:
                    self.pose = (pose.position.x, pose.position.y)
                    self.pose_at = time.monotonic()
                return

    def odom_callback(self, message):
        linear = message.twist.linear
        speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        with self.lock:
            self.speed = speed
            self.odom_at = time.monotonic()
            self.odom_seq += 1

    def lidar_callback(self, message):
        clearance = front_clearance(
            message, self.safety["front_half_angle_deg"],
            self.safety["lidar_to_front_envelope_m"])
        with self.lock:
            self.front = clearance
            self.lidar_at = time.monotonic()

    def contact_callback(self, message):
        name = self.config["obstacle"]["name"]
        for contact in message.contact:
            if name in contact.collision1 or name in contact.collision2:
                with self.lock:
                    self.hit_obstacle = True

    def publish(self, speed):
        twist = self.Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # No skid-steering commands: articulated joint is independent.
        self.command.publish(twist)

    def stop(self):
        self.publish(0.0)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            data = dict(pose=self.pose, speed=self.speed, front=self.front,
                        odom_seq=self.odom_seq, hit=self.hit_obstacle)
            stamps = (self.pose_at, self.odom_at, self.lidar_at)
            data["fresh"] = (all(s is not None
                                 and 0 <= now - s <= self.safety["sensor_timeout_s"]
                                 for s in stamps)
                             and self.front is not None)
        return data

    def wait_ready(self):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            self.stop()
            state = self.snapshot()
            if state["fresh"]:
                start = self.config["start"]
                x, y = state["pose"]
                if math.hypot(x - start["x"], y - start["y"]) > 1.5:
                    raise RuntimeError(
                        "vehicle not at initial position; restart Gazebo world")
                return True
            time.sleep(0.05)
        return False

    def run(self):
        report = {"passed": False, "reason": "not_started",
                  "scenario": self.config["name"], "vehicle": self.config["vehicle"]}
        if not self.wait_ready():
            report["reason"] = "sensors_not_ready"
            self.stop()
            return report

        start_at = time.monotonic()
        brake_reason = None
        brake_at = None
        moved = False
        max_progress = 0.0
        stop_samples = 0
        last_stop_seq = None
        stopped_since = None
        required = self.acceptance["required_clearance_m"]
        last_status = 0.0
        try:
            while time.monotonic() - start_at < self.acceptance["trial_timeout_s"]:
                now = time.monotonic()
                state = self.snapshot()
                if not state["fresh"]:
                    report["reason"] = "sensor_timeout"
                    break
                if state["hit"]:
                    report["reason"] = "obstacle_contact"
                    break
                progress, lateral, estimated_gap = route_geometry(
                    self.config, *state["pose"])
                max_progress = max(max_progress, progress)
                if abs(lateral) > self.acceptance["maximum_lateral_error_m"]:
                    report["reason"] = "route_deviation"
                    break
                if state["speed"] >= 0.2 and progress >= 0.1:
                    moved = True
                if not brake_reason:
                    brake, reason, ttc = brake_decision(
                        state["speed"], state["front"], self.safety, required)
                    if brake:
                        brake_reason = reason
                        brake_at = now
                        print("AEB triggered: " + brake_reason
                              + " | estimated_gap=%.2f m" % estimated_gap,
                              flush=True)
                    else:
                        self.publish(self.safety["cruise_speed_mps"])
                if brake_reason:
                    self.stop()
                    if (state["odom_seq"] != last_stop_seq
                            and state["speed"] <= self.acceptance["stop_speed_mps"]):
                        stop_samples += 1
                        last_stop_seq = state["odom_seq"]
                    elif state["speed"] > self.acceptance["stop_speed_mps"]:
                        stop_samples = 0
                        stopped_since = None
                    if stop_samples >= self.acceptance["stop_confirm_samples"]:
                        if stopped_since is None:
                            stopped_since = now
                        if now - stopped_since >= self.acceptance["stop_hold_s"]:
                            report["passed"] = (
                                moved
                                and max_progress >= self.acceptance["minimum_travel_before_braking_m"]
                                and estimated_gap >= required
                                and state["front"] >= required
                                and not state["hit"])
                            report["reason"] = (
                                "aeb_stop_verified" if report["passed"]
                                else "clearance_or_motion_not_verified")
                            break
                if now - last_status >= 1.0:
                    print("speed=%.2f m/s | LiDAR clearance=%.2f m | "
                          "estimated body clearance=%.2f m | travel=%.2f m"
                          % (state["speed"], state["front"], estimated_gap, progress),
                          flush=True)
                    last_status = now
                time.sleep(0.05)
            else:
                report["reason"] = "trial_timeout"
        finally:
            for _ in range(5):
                self.stop()
                time.sleep(0.02)

        final = self.snapshot()
        progress, lateral, gap = route_geometry(self.config, *final["pose"])
        report.update(
            brake_reason=brake_reason, moved_before_braking=moved,
            maximum_route_progress_m=round(max_progress, 3),
            final_speed_mps=round(final["speed"], 4),
            final_lidar_clearance_m=(None if final["front"] is None
                                     else round(final["front"], 3)),
            estimated_ground_clearance_m=round(gap, 3),
            lateral_error_m=round(lateral, 3),
            obstacle_contact_observed=final["hit"],
            stop_samples=stop_samples,
            brake_elapsed_s=(None if brake_at is None
                             else round(time.monotonic() - brake_at, 3)),
            note="Estimated body clearance is an uncalibrated planar envelope, "
                 "not exact mesh separation; Gazebo dynamics still require validation.")
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true",
                        help="validate scenario and report inputs without Gazebo")
    parser.add_argument("--report", type=Path,
                        help="optional JSON report path for a new trial")
    args = parser.parse_args()
    scenario = load_scenario()
    if args.dry_run:
        world = ROOT / scenario["world"]
        model = ROOT / "edge-path-project/gazebo/models/metsakone/model.sdf"
        if not world.is_file() or not model.is_file():
            raise SystemExit("Metsakone world or model file missing")
        print("METSAKONE CONFIG: PASS (no Gazebo runtime test)")
        print("World: " + str(world))
        print("Model: " + str(model))
        return

    try:
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit("Gazebo Harmonic Python bindings required: " + str(exc))

    test = MetsakoneAEB(scenario, Node, Twist, Pose_V, Odometry, LaserScan, Contacts)
    try:
        report = test.run()
    except RuntimeError as exc:
        test.stop()
        report = {"passed": False, "reason": str(exc), "scenario": scenario["name"]}
    print("METSAKONE AEB SIL RESULT: " + ("PASS" if report["passed"] else "FAIL"))
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
