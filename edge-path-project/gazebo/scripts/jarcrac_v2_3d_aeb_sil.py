#!/usr/bin/env python3
"""Experimental JARCRAC v2 front-3D-LiDAR, flat-world AEB SIL trial.

Requires the JARCRAC v2 front-LiDAR model and jarcrac_v2_front_aeb_world.sdf.
Commands zero speed on AEB, timeout, interruption and exceptions. The model
uses velocity-controlled DiffDrive, NOT the real Mevea diesel/hydraulic brakes.
Do not run another world with /model/jarcrac topics at the same time.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_lidar_3d import summarize_forward
from jarcrac_v2_aeb_policy import (
    axial_geometry, angle_difference, braking_decision, yaw_from_quaternion,
)

ROOT = Path(__file__).resolve().parents[3]
GZ = ROOT / "edge-path-project" / "gazebo"
SCENARIO_FILE = GZ / "scenarios" / "jarcrac_v2_front_aeb.json"
CSV_FIELDS = (
    "wall_elapsed_s", "sim_time_s", "x_m", "y_m", "yaw_rad", "speed_mps",
    "front_obstacle_gap_m", "axial_body_gap_estimate_m", "progress_m",
    "forward_obstacle_points", "ground_points", "near_field_points",
    "command_mps", "phase", "obstacle_contact_observed",
)


def load_config():
    cfg = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != 1 or cfg.get("name") != "jarcrac_v2_front_3d_aeb":
        raise ValueError("wrong JARCRAC v2 scenario schema or name")
    if (cfg["safety"]["assumed_deceleration_mps2"] <= 0
            or cfg["acceptance"]["required_clearance_m"] <= 0):
        raise ValueError("invalid deceleration or clearance requirements")
    return cfg


def validate_files(cfg):
    """Validate that the 32-layer FRONT model is installed, not older model."""
    world_file = ROOT / cfg["world"]
    model_file = GZ / "models" / "jarcrac" / "model.sdf"
    for path in (world_file, model_file):
        if not path.is_file():
            raise FileNotFoundError(path)
    world = ET.parse(world_file).getroot().find("world")
    model = ET.parse(model_file).getroot().find("model")
    if world is None or world.get("name") != "jarcrac_v2_front_aeb":
        raise ValueError("wrong Gazebo AEB world")
    if model is None or model.get("name") != "jarcrac":
        raise ValueError("wrong Gazebo JARCRAC model")
    sensor = model.find("./link[@name='chassis']/sensor[@type='gpu_lidar']")
    plugin = model.find("./plugin[@name='gz::sim::systems::DiffDrive']")
    obs_sensor = world.find(
        "./model[@name='jarcrac_v2_aeb_obstacle']/link/sensor[@type='contact']")
    if sensor is None or plugin is None or obs_sensor is None:
        raise ValueError("missing LiDAR, DiffDrive, or obstacle contact sensor")
    if (sensor.findtext("topic") != cfg["topics"]["lidar"]
            or int(sensor.findtext("ray/scan/vertical/samples")) != 32
            or float(sensor.findtext("ray/scan/horizontal/min_angle")) > -1.5
            or float(sensor.findtext("ray/scan/horizontal/max_angle")) < 1.5
            or plugin.findtext("topic") != cfg["topics"]["cmd_vel"]
            or plugin.findtext("odom_topic") != cfg["topics"]["odometry"]
            or obs_sensor.findtext("topic") != cfg["topics"]["obstacle_contacts"]):
        raise ValueError("JARCRAC v2 model/scenario topics or scan layout mismatch")
    if len(plugin.findall("left_joint")) != 6 or len(plugin.findall("right_joint")) != 6:
        raise ValueError("expected normalized 12-wheel JARCRAC v2 drive")
    mount = [float(v) for v in sensor.findtext("pose").split()[:3]]
    chassis_z = float(model.find("./link[@name='chassis']/pose").text.split()[2])
    if (abs(mount[0] - 1.68) > 0.01 or abs(mount[1]) > 0.01
            or abs(mount[2] + chassis_z - cfg["sensor"]["height_above_flat_ground_m"]) > .02):
        raise ValueError("model is not configured with the expected front LiDAR mount")
    if world.find("./plugin[@name='gz::sim::systems::Contact']") is None:
        raise ValueError("world does not load Gazebo Contact system")
    obstacle = world.find("./model[@name='jarcrac_v2_aeb_obstacle']")
    if obstacle is None or obstacle.find("link/collision/geometry/box/size") is None:
        raise ValueError("AEB test obstacle missing")
    obst_xyz = [float(x) for x in obstacle.findtext("pose").split()[:3]]
    obs_half = [float(x)/2 for x in obstacle.findtext(
        "link/collision/geometry/box/size").split()]
    if (abs(obst_xyz[0] - cfg["obstacle"]["x"]) > 1e-5
            or abs(obst_xyz[1] - cfg["obstacle"]["y"]) > 1e-5
            or abs(obs_half[0] - cfg["obstacle"]["half_x"]) > 1e-5
            or abs(obs_half[1] - cfg["obstacle"]["half_y"]) > 1e-5):
        raise ValueError("AEB obstacle pose/dimensions do not match the scenario")
    return world_file, model_file


class JarcracV2AEB:
    def __init__(self, cfg, Node, Twist, Pose_V, Odometry, LaserScan, Contacts):
        self.cfg = cfg
        self.safety = cfg["safety"]
        self.acceptance = cfg["acceptance"]
        self.lock = threading.RLock()
        self.node = Node()
        self.twist_type = Twist
        self.pose = None
        self.speed = None
        self.scan_summary = None
        self.pose_at = self.odom_at = self.scan_at = None
        self.odom_seq = 0
        self.sim_time_s = None
        self.contact_messages = 0
        self.collision = False
        self.telemetry = []
        self.command_pub = self.node.advertise(cfg["topics"]["cmd_vel"], Twist)
        if self.command_pub is None:
            raise RuntimeError("could not advertise vehicle command topic")
        self.contact_subscription_established = False
        subs = [
            (Pose_V, cfg["topics"]["world_poses"], self.on_poses),
            (Odometry, cfg["topics"]["odometry"], self.on_odometry),
            (LaserScan, cfg["topics"]["lidar"], self.on_scan),
            (Contacts, cfg["topics"]["obstacle_contacts"], self.on_contacts),
        ]
        for msg, topic, callback in subs:
            if not self.node.subscribe(msg, topic, callback):
                raise RuntimeError("failed to subscribe to " + topic)
            if topic == cfg["topics"]["obstacle_contacts"]:
                self.contact_subscription_established = True

    def on_poses(self, message):
        for p in message.pose:
            if p.name == self.cfg["vehicle"] or p.name.endswith("::" + self.cfg["vehicle"]):
                yaw = yaw_from_quaternion(p.orientation)
                values = (p.position.x, p.position.y, yaw)
                if all(math.isfinite(v) for v in values):
                    with self.lock:
                        self.pose = values
                        self.pose_at = time.monotonic()
                return

    def on_odometry(self, message):
        v = message.twist.linear
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        if not math.isfinite(speed):
            return
        stamp = message.header.stamp
        sim_time = stamp.sec + stamp.nsec * 1e-9
        with self.lock:
            self.speed = speed
            self.odom_at = time.monotonic()
            self.odom_seq += 1
            self.sim_time_s = sim_time

    def on_scan(self, message):
        sensor = self.cfg["sensor"]
        summary = summarize_forward(
            message,
            sensor_height_m=sensor["height_above_flat_ground_m"],
            half_angle_deg=sensor["front_sector_half_angle_deg"],
            ground_z_m=sensor["ground_z_m"],
            min_obstacle_height_m=sensor["min_obstacle_height_m"],
            max_obstacle_height_m=sensor["max_obstacle_height_m"],
            body_self_radius_m=sensor["near_field_warning_radius_m"],
        )
        with self.lock:
            self.scan_summary = summary
            self.scan_at = time.monotonic()

    def on_contacts(self, message):
        with self.lock:
            self.contact_messages += 1
            if len(message.contact) > 0:
                self.collision = True

    def command(self, speed):
        msg = self.twist_type()
        msg.linear.x = float(speed)
        msg.angular.z = 0.0
        self.command_pub.publish(msg)

    def stop(self):
        self.command(0.0)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            s = self.scan_summary
            stamps = (self.pose_at, self.odom_at, self.scan_at)
            sensor_fresh = all(t is not None and 0 <= now-t <= self.safety["sensor_timeout_s"]
                               for t in stamps)
            valid_scan = (s is not None and s.valid
                          and s.obstacle_points >= self.cfg["sensor"]["minimum_obstacle_returns"]
                          and s.nearest_forward_obstacle_m is not None
                          and math.isfinite(s.nearest_forward_obstacle_m))
            return dict(
                fresh=sensor_fresh and valid_scan and self.pose is not None
                and self.speed is not None and math.isfinite(self.speed),
                pose=self.pose, speed=self.speed, summary=s, sim_time_s=self.sim_time_s,
                odom_seq=self.odom_seq, contact=self.collision,
                contact_messages=self.contact_messages,
            )

    def ready(self):
        start = time.monotonic()
        while time.monotonic()-start < self.acceptance["readiness_timeout_s"]:
            self.stop()
            snap = self.snapshot()
            if snap["fresh"]:
                x, y, yaw = snap["pose"]
                pose = self.cfg["start"]
                if (math.hypot(x-pose["x"], y-pose["y"]) > 0.7
                        or abs(angle_difference(yaw, pose["yaw"])) > 0.25):
                    raise RuntimeError("vehicle not at start; restart AEB world")
                if snap["speed"] > .05:
                    raise RuntimeError("vehicle already moving; restart AEB world")
                if (snap["summary"].near_points > 0
                        or snap["summary"].nearest_forward_obstacle_m < 3.0):
                    raise RuntimeError("unexpected near-field return at start; do not drive")
                return True
            time.sleep(.05)
        return False

    def record(self, elapsed, snap, progress, gap, phase, command):
        s = snap["summary"]
        x, y, yaw = snap["pose"]
        self.telemetry.append(dict(
            wall_elapsed_s=round(elapsed, 3),
            sim_time_s=round(snap["sim_time_s"], 4) if snap["sim_time_s"] is not None else None,
            x_m=round(x, 5), y_m=round(y, 5), yaw_rad=round(yaw, 5),
            speed_mps=round(snap["speed"], 5),
            front_obstacle_gap_m=round(s.nearest_forward_obstacle_m, 5),
            axial_body_gap_estimate_m=round(gap, 5), progress_m=round(progress, 5),
            forward_obstacle_points=s.obstacle_points, ground_points=s.ground_points,
            near_field_points=s.near_points, command_mps=command,
            phase=phase, obstacle_contact_observed=int(snap["contact"]),
        ))

    def run(self):
        report = dict(passed=False, reason="not_started",
                      scenario=self.cfg["name"], vehicle=self.cfg["vehicle"])
        brake_time = brake_sim = brake_progress = brake_reason = None
        reached_stop_sim = None
        hold_start_sim = None
        stop_count = 0
        last_stop_seq = None
        moved = False
        max_progress = 0.0
        min_obstacle_gap = math.inf
        min_body_gap = math.inf
        status_at = 0.0
        start_wall = None
        start_sim = None
        try:
            if not self.ready():
                report["reason"] = "sensors_not_ready_or_no_obstacle_returns"
                return report
            start_wall = time.monotonic()
            start_sim = self.snapshot()["sim_time_s"]
            while time.monotonic()-start_wall < self.acceptance["trial_timeout_s"]:
                now = time.monotonic()
                snap = self.snapshot()
                if snap["contact"]:
                    report["reason"] = "obstacle_contact"
                    break
                if not snap["fresh"]:
                    report["reason"] = "sensor_timeout_invalid_scan_or_obstacle_missing"
                    break
                progress, lateral, body_gap = axial_geometry(self.cfg, snap["pose"])
                front = snap["summary"].nearest_forward_obstacle_m
                min_obstacle_gap = min(min_obstacle_gap, front)
                min_body_gap = min(min_body_gap, body_gap)
                max_progress = max(max_progress, progress)
                if (abs(lateral) > self.acceptance["maximum_lateral_deviation_m"]
                        or abs(angle_difference(snap["pose"][2], self.cfg["start"]["yaw"]))
                        > self.acceptance["maximum_yaw_error_rad"]):
                    report["reason"] = "route_deviation"
                    break
                if body_gap <= 0:
                    report["reason"] = "estimated_footprint_overlap"
                    break
                if (snap["speed"] >= self.acceptance["minimum_speed_to_count_motion_mps"]
                        and progress >= 0.1):
                    moved = True
                cmd = self.safety["cruise_speed_mps"]
                phase = "CRUISE"
                if brake_time is None:
                    brake, reason, _ttc = braking_decision(
                        snap["speed"], front, self.safety,
                        self.acceptance["required_clearance_m"])
                    if brake:
                        brake_time, brake_sim, brake_progress, brake_reason = (
                            now, snap["sim_time_s"], progress, reason)
                        print("JARCRAC v2 3D AEB triggered:", reason,
                              "| speed=%.3f m/s | forward gap=%.3f m" %
                              (snap["speed"], front), flush=True)
                if brake_time is None:
                    self.command(cmd)
                else:
                    cmd = 0.0
                    self.stop()
                    phase = "BRAKE"
                    if snap["speed"] > self.acceptance["stop_speed_mps"]:
                        hold_start_sim = None
                        stop_count = 0
                    elif snap["odom_seq"] != last_stop_seq:
                        stop_count += 1
                        last_stop_seq = snap["odom_seq"]
                        if reached_stop_sim is None:
                            reached_stop_sim = snap["sim_time_s"]
                    if stop_count >= self.acceptance["stop_confirm_samples"]:
                        if hold_start_sim is None:
                            hold_start_sim = snap["sim_time_s"]
                        phase = "STOP_HOLD"
                        if (hold_start_sim is not None and snap["sim_time_s"] is not None
                                and snap["sim_time_s"] - hold_start_sim
                                >= self.acceptance["stop_hold_sim_s"]):
                            passed = (moved
                                and max_progress >= self.acceptance["minimum_travel_before_braking_m"]
                                and min_obstacle_gap >= self.acceptance["required_clearance_m"]
                                and min_body_gap >= self.acceptance["required_clearance_m"]
                                and not snap["contact"])
                            report["passed"] = passed
                            report["reason"] = (
                                "aeb_stop_and_estimated_clearance_verified" if passed
                                else "motion_or_clearance_not_verified")
                self.record(now-start_wall, snap, progress, body_gap, phase, cmd)
                if now-status_at >= 1.0:
                    print("speed=%.2f | 3D forward=%.2f m | estimated body gap=%.2f m | "
                          "progress=%.2f m | obstacles=%d | ground=%d | %s" %
                          (snap["speed"], front, body_gap, progress,
                           snap["summary"].obstacle_points,
                           snap["summary"].ground_points, phase), flush=True)
                    status_at = now
                if report["passed"] or report["reason"] != "not_started":
                    break
                time.sleep(.05)
            else:
                report["reason"] = "trial_timeout"
        finally:
            for _ in range(6):
                self.stop()
                time.sleep(.02)
            snap = self.snapshot()
            report.update(
                brake_reason=brake_reason,
                brake_start_progress_m=None if brake_progress is None else round(brake_progress, 4),
                maximum_route_progress_m=round(max_progress, 4),
                minimum_3d_forward_obstacle_gap_m=None if not math.isfinite(min_obstacle_gap)
                    else round(min_obstacle_gap, 4),
                minimum_estimated_body_gap_m=None if not math.isfinite(min_body_gap)
                    else round(min_body_gap, 4),
                obstacle_contact_observed=snap["contact"],
                contact_messages_received=snap["contact_messages"],
                contact_subscription_established=self.contact_subscription_established,
                contact_detection_positive_test_verified=False,
                moved_before_braking=moved,
                stop_confirmation_samples=stop_count,
                brake_to_first_low_speed_sim_s=(None if brake_sim is None
                    or reached_stop_sim is None else round(reached_stop_sim-brake_sim, 4)),
                note=("LiDAR and axial chassis gaps are flat-world estimates, not exact "
                      "mesh separation. Static obstacle contact sensor subscribed but not "
                      "positive-control tested. Velocity-controlled braking dynamics and "
                      "ground filtering are NOT calibrated for real forestry terrain."),
            )
            if snap["pose"] is not None:
                progress, lateral, gap = axial_geometry(self.cfg, snap["pose"])
                report.update(final_world_x_m=round(snap["pose"][0], 4),
                              final_world_y_m=round(snap["pose"][1], 4),
                              final_progress_m=round(progress, 4),
                              final_yaw_rad=round(snap["pose"][2], 5),
                              final_estimated_body_gap_m=round(gap, 4),
                              lateral_deviation_m=round(lateral, 4),
                              stopping_distance_m=None if brake_progress is None
                                  else round(max(0.0, progress-brake_progress), 4))
            if snap["speed"] is not None:
                report["final_speed_mps"] = round(snap["speed"], 5)
            if snap["summary"] is not None:
                report["final_forward_gap_m"] = snap["summary"].nearest_forward_obstacle_m
        return report


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--report", type=Path, help="JSON output file")
    parser.add_argument("--csv", type=Path, help="CSV telemetry output file")
    args = parser.parse_args()
    cfg = load_config()
    world, model = validate_files(cfg)
    if args.dry_run:
        print("JARCRAC V2 3D AEB CONFIG: PASS (not a runtime test)")
        print("World:", world)
        print("Model:", model)
        return
    try:
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit("Gazebo Harmonic Python bindings missing: " + str(exc))
    trial = JarcracV2AEB(cfg, Node, Twist, Pose_V, Odometry, LaserScan, Contacts)
    try:
        result = trial.run()
    except KeyboardInterrupt:
        trial.stop()
        result = {"passed": False, "reason": "interrupted", "scenario": cfg["name"]}
    except Exception as exc:
        trial.stop()
        result = {"passed": False, "reason": "exception", "detail": str(exc),
                  "scenario": cfg["name"]}
    print("JARCRAC V2 3D AEB SIL RESULT:", "PASS" if result["passed"] else "FAIL")
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
    if args.csv or args.report:
        csv_path = args.csv or args.report.with_suffix(".csv")
        write_csv(csv_path, trial.telemetry)
        print("CSV:", csv_path)
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
