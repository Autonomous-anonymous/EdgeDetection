#!/usr/bin/env python3
"""JARCRAC v2 contact sensor positive-control test (intentionally collides).

Run `prepare` with no simulator running, launch generated world separately,
then run `test` while that world is active.  No original repo files modified.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[3]
BASE_WORLD = REPO / "edge-path-project/gazebo/worlds/jarcrac_v2_front_aeb_world.sdf"
TEST_WORLD = Path.home() / "jarcrac_v2_contact_positive_world.sdf"
RESULT = REPO / "edge-path-project/gazebo/results/jarcrac/v2_contact_positive_trial.json"
CONTACT_TOPIC = "/world/jarcrac_v2_front_aeb/model/jarcrac_v2_aeb_obstacle/link/body/sensor/obstacle_contact/contact"
CMD_TOPIC = "/model/jarcrac/cmd_vel"
ODOM_TOPIC = "/model/jarcrac/odometry"
CHASSIS_COLLISION = "jarcrac::chassis::body_collision"
OBSTACLE_COLLISION = "jarcrac_v2_aeb_obstacle::body::collision"


def prepare():
    if not BASE_WORLD.is_file():
        raise SystemExit("Original world not found: " + str(BASE_WORLD))
    tree = ET.parse(BASE_WORLD, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root = tree.getroot()
    world = root.find("world")
    if world is None or world.get("name") != "jarcrac_v2_front_aeb":
        raise SystemExit("Unexpected base-world name")
    obstacle = world.find("./model[@name='jarcrac_v2_aeb_obstacle']")
    if obstacle is None:
        raise SystemExit("Original obstacle not found")
    sensor = obstacle.find("./link[@name='body']/sensor[@name='obstacle_contact']")
    if sensor is None or sensor.findtext("topic") != CONTACT_TOPIC:
        raise SystemExit("Original contact sensor/topic mismatch")
    if world.find("./plugin[@name='gz::sim::systems::Contact']") is None:
        raise SystemExit("Original Contact system missing")
    # Sensor / world name / model / topics unchanged; only obstacle pose changes.
    obstacle.find("pose").text = "3.0 0 0.75 0 0 0"
    tree.write(TEST_WORLD, encoding="utf-8", xml_declaration=True)
    test_world = ET.parse(TEST_WORLD).getroot()
    assert test_world.find("world/model[@name='jarcrac_v2_aeb_obstacle']/pose").text.startswith("3.0 ")
    print("Positive-contact world created:", TEST_WORLD)
    print("Same Gazebo world name and same contact topic as the validated AEB world.")
    print("Obstacle moved from x=8.0 m to x=3.0 m; intentional collision test only.")


def run_test(speed: float, timeout: float):
    try:
        from gz.transport13 import Node
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.twist_pb2 import Twist
    except ImportError as exc:
        raise SystemExit("Gazebo Harmonic Python bindings missing: " + str(exc))

    if not TEST_WORLD.is_file():
        raise SystemExit("First run: python3 ~/jarcrac_contact_positive.py prepare")
    node = Node()
    state_lock = threading.RLock()
    state = {
        "x": None,
        "speed": None,
        "max_measured_speed_mps": 0.0,
        "contact_messages": 0,
        "contact_events": 0,
        "matching_contact_events": 0,
        "collision_pairs": [],
        "matched_pair": None,
        "contact_x_m": None,
        "last_odom_at": None,
    }
    contact_seen = threading.Event()

    def on_odom(msg):
        v = msg.twist.linear
        speed_mps = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        if not math.isfinite(speed_mps):
            return
        with state_lock:
            state["x"] = msg.pose.position.x
            state["speed"] = speed_mps
            state["max_measured_speed_mps"] = max(state["max_measured_speed_mps"], speed_mps)
            state["last_odom_at"] = time.monotonic()

    def on_contact(msg):
        with state_lock:
            state["contact_messages"] += 1
            for contact in msg.contact:
                state["contact_events"] += 1
                one, two = contact.collision1.name, contact.collision2.name
                if len(state["collision_pairs"]) < 10:
                    state["collision_pairs"].append([one, two])
                matching = ((CHASSIS_COLLISION in one and OBSTACLE_COLLISION in two)
                            or (OBSTACLE_COLLISION in one and CHASSIS_COLLISION in two))
                if matching:
                    state["matching_contact_events"] += 1
                    if state["matched_pair"] is None:
                        state["matched_pair"] = [one, two]
                        state["contact_x_m"] = state["x"]
                    contact_seen.set()

    for message, topic, callback in ((Odometry, ODOM_TOPIC, on_odom),
                                      (Contacts, CONTACT_TOPIC, on_contact)):
        if not node.subscribe(message, topic, callback):
            raise SystemExit("Could not subscribe to " + topic)
    pub = node.advertise(CMD_TOPIC, Twist)
    if pub is None:
        raise SystemExit("Could not advertise " + CMD_TOPIC)

    def cmd(velocity):
        m = Twist()
        m.linear.x = velocity
        m.angular.z = 0.0
        pub.publish(m)

    print("Contact topic:", CONTACT_TOPIC, flush=True)
    print("Listening for actual JARCRAC chassis versus obstacle collisions.", flush=True)
    started = time.monotonic()
    reason = "timeout_without_confirmed_contact"
    start_x = None
    drove = False
    try:
        # Wait for odometry before sending any motion commands.
        while time.monotonic() - started < 12.0:
            cmd(0.0)
            with state_lock:
                x, v, stamp = state["x"], state["speed"], state["last_odom_at"]
            if x is not None and stamp is not None and time.monotonic() - stamp < 1.0:
                if not (-0.3 <= x <= 0.3) or v > 0.05:
                    reason = "incorrect_start_state_restart_world"
                    break
                start_x = x
                break
            time.sleep(0.05)
        if start_x is None and reason == "timeout_without_confirmed_contact":
            reason = "no_fresh_odometry"
        if start_x is not None:
            drive_start = time.monotonic()
            while time.monotonic() - drive_start < timeout:
                with state_lock:
                    x, v, stamp = state["x"], state["speed"], state["last_odom_at"]
                if contact_seen.is_set():
                    reason = "matching_chassis_obstacle_contact_observed"
                    break
                if stamp is None or time.monotonic() - stamp > 1.5:
                    reason = "stale_odometry"
                    break
                if x is not None and x - start_x > 2.0:
                    reason = "vehicle_passed_contact_limit_without_detection"
                    break
                if v is not None and v >= 0.05:
                    drove = True
                cmd(speed)
                time.sleep(0.05)
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        # Always command zero, including on timeout and interruption.
        for _ in range(25):
            cmd(0.0)
            time.sleep(0.04)
        with state_lock:
            snapshot = dict(state)
        stopped = (snapshot["speed"] is not None
                   and snapshot["speed"] <= 0.05
                   and snapshot["last_odom_at"] is not None
                   and time.monotonic() - snapshot["last_odom_at"] <= 1.5)
        passed = (reason == "matching_chassis_obstacle_contact_observed"
                  and drove and stopped and snapshot["matching_contact_events"] > 0)
        report = {
            "scenario": "jarcrac_v2_contact_sensor_positive_control",
            "passed": passed,
            "reason": reason,
            "intentional_contact": True,
            "aeb_controller_active": False,
            "same_contact_topic_as_original_aeb_world": True,
            "drive_speed_command_mps": speed,
            "movement_observed": drove,
            "observed_final_speed_mps": snapshot["speed"],
            "stop_verified": stopped,
            "contact_messages_received": snapshot["contact_messages"],
            "contact_events_received": snapshot["contact_events"],
            "matching_chassis_obstacle_contact_events": snapshot["matching_contact_events"],
            "matching_collision_pair": snapshot["matched_pair"],
            "first_contact_at_x_m": snapshot["contact_x_m"],
            "observed_collision_pairs_first_10": snapshot["collision_pairs"],
            "maximum_measured_speed_mps": snapshot["max_measured_speed_mps"],
            "elapsed_wall_s": round(time.monotonic() - started, 2),
            "note": "Positive contact-sensor plumbing test only. Deliberate Gazebo collision, not AEB acceptance or a safety demonstration.",
        }
        RESULT.parent.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print("JARCRAC CONTACT POSITIVE CONTROL:", "PASS" if passed else "FAIL", flush=True)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        print("Report:", RESULT, flush=True)
        if not passed:
            raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "test"))
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--timeout", type=float, default=40.0)
    args = parser.parse_args()
    if args.action == "prepare":
        prepare()
    else:
        if not 0.05 <= args.speed <= 0.25:
            parser.error("--speed must be between 0.05 and 0.25 m/s")
        if not 5 <= args.timeout <= 120:
            parser.error("--timeout must be between 5 and 120 s")
        run_test(args.speed, args.timeout)


if __name__ == "__main__":
    main()
