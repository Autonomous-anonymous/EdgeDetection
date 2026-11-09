#!/usr/bin/env python3
"""Controlled wheel-loader flat-world motion smoke test (Gazebo Harmonic SIL).

Requires the wheelloader_flat_test_world; starts its own controller bridge.
Publishes a 0.20 m/s straight-line command for 3 seconds, then repeatedly
publishes zero. Records odometry displacement and post-stop velocity. This is
not an AEB, dynamics calibration or safety-rated braking test.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from wheelloader_cmd_bridge import Bridge

ODOM_TOPIC = "/model/wheelloader/odometry"
CMD_TOPIC = "/model/wheelloader/cmd_vel"
PARAMS = Path(__file__).resolve().parents[1] / "models" / "wheelloader" / "wheelloader_params.json"
DEFAULT_REPORT = Path("/tmp/wheelloader_flat_smoke_recheck.json")


def run_smoke(report: Path) -> int:
    node = Node()
    lock = threading.Lock()
    latest = [None]
    messages = [0]

    def on_odom(msg: Odometry):
        try:
            point = (time.monotonic(), float(msg.pose.position.x),
                     float(msg.pose.position.y), float(msg.twist.linear.x))
            if all(math.isfinite(n) for n in point):
                with lock:
                    latest[0] = point
                    messages[0] += 1
        except (AttributeError, ValueError, TypeError):
            return

    if not node.subscribe(Odometry, ODOM_TOPIC, on_odom):
        raise SystemExit(f"Cannot subscribe to {ODOM_TOPIC}")
    publisher = node.advertise(CMD_TOPIC, Twist)

    def snapshot():
        with lock:
            return latest[0]

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        observed = snapshot()
        if observed is not None and time.monotonic() - observed[0] <= 0.5:
            break
        time.sleep(0.1)
    else:
        raise SystemExit("No fresh wheel-loader odometry; check Gazebo and the topic")

    beginning = snapshot()
    issue = None
    sent = 0
    def publish_speed(value: float):
        cmd = Twist()
        cmd.linear.x = value
        publisher.publish(cmd)

    print("Wheel-loader flat-world odometry smoke test: 0.20 m/s for 3 seconds")
    print("The wheel-loader controller bridge was started automatically.")
    try:
        for _ in range(30):
            current = snapshot()
            if current is None or time.monotonic() - current[0] > 0.5:
                issue = "odometry_stale_while_driving"
                break
            publish_speed(0.20)
            sent += 1
            time.sleep(0.10)
    except KeyboardInterrupt:
        issue = "interrupted"
    finally:
        for _ in range(15):
            publish_speed(0.0)
            time.sleep(0.10)

    final = snapshot()
    final_fresh = final is not None and time.monotonic() - final[0] < 0.5
    displacement = (math.hypot(final[1] - beginning[1],
                               final[2] - beginning[2])
                    if final is not None else None)
    speed = abs(final[3]) if final is not None else None
    passed = (issue is None and final_fresh and sent == 30
              and displacement is not None and 0.08 <= displacement <= 1.50
              and speed is not None and speed <= 0.08)
    result = {
        "test": "wheelloader_flat_world_motion_smoke",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "passed": passed,
        "status": "PASS" if passed else "INCOMPLETE_OR_FAIL",
        "issue": issue,
        "world": "wheelloader_flat_test_world",
        "commanded_speed_mps": 0.20,
        "command_duration_wall_s": 3.0,
        "commands_sent": sent,
        "odometry_messages": messages[0],
        "start_xy_m": [round(beginning[1], 5), round(beginning[2], 5)],
        "final_xy_m": ([round(final[1], 5), round(final[2], 5)]
                       if final is not None else None),
        "odometry_displacement_m": (round(displacement, 5)
                                    if displacement is not None else None),
        "final_forward_speed_mps": round(speed, 5) if speed is not None else None,
        "note": "Controlled flat-world SIL smoke test based on Gazebo odometry. "
                "Not independently measured ground-truth displacement, AEB "
                "acceptance, real braking calibration or terrain validation.",
    }
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved: {report}")
    return 0 if passed else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT,
                        help="result JSON path (defaults to /tmp, preserving committed evidence)")
    args = parser.parse_args(argv)

    bridge = Bridge(json.loads(PARAMS.read_text()), CMD_TOPIC, timeout_s=0.5)
    stop_event = threading.Event()
    bridge_errors = []

    def bridge_loop():
        try:
            bridge.run(stop_event)
        except Exception as exc:
            bridge_errors.append(exc)

    worker = threading.Thread(target=bridge_loop, name="wheel-loader-bridge", daemon=True)
    worker.start()
    try:
        result = run_smoke(args.report)
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        if worker.is_alive():
            raise RuntimeError("Wheel-loader bridge did not stop cleanly")
        bridge.close()
    if bridge_errors:
        raise RuntimeError(f"Wheel-loader bridge failed: {bridge_errors[0]}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
