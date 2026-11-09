#!/usr/bin/env python3
"""cmd_vel bridge for the articulated wheel loader.

Gazebo has no articulated-steering controller, so this node turns a standard
Twist on  /model/wheelloader/cmd_vel  into the wheel velocity commands and the
articulation-angle command that the model's joint controllers listen on.

    v = linear.x   speed of the REAR AXLE CENTRE (m/s)
    w = angular.z  yaw rate of the rear frame (rad/s, positive = left)

The geometry (pivot position, track, wheel radius, limits) comes from
wheelloader_params.json, which the generator writes.

Lift and bucket tilt need no bridge; publish an angle (rad) directly:
    gz topic -t /wheelloader/lift/cmd_pos        -m gz.msgs.Double -p 'data: 0.6'
    gz topic -t /wheelloader/bucket_tilt/cmd_pos -m gz.msgs.Double -p 'data: -0.8'

Run:
    python3 wheelloader_cmd_bridge.py --params ../models/wheelloader/wheelloader_params.json
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheelloader_kinematics as kin  # noqa: E402

WHEELS = ("wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr")


def build_commands(params: dict, v: float, omega: float) -> dict:
    """Map a (v, omega) command to {topic: value} for every controlled joint."""
    k = params["kinematics"]
    cmds = kin.twist_to_commands(v, omega, k, k["max_speed_mps"], k["articulation_limit_rad"])
    m = params["model"]
    out = {m["wheel_cmd_topics"][w]: cmds[w] for w in WHEELS}
    out[m["position_cmd_topics"]["articulation"]] = cmds["articulation"]
    return out


class Bridge:
    def __init__(self, params: dict, cmd_topic: str, timeout_s: float):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node

        self.params = params
        self.Double = Double
        self.timeout_s = timeout_s
        self.node = Node()
        topics = list(build_commands(params, 0.0, 0.0))
        self.pubs = {t: self.node.advertise(t, Double) for t in topics}
        if not self.node.subscribe(Twist, cmd_topic, self.on_cmd):
            raise SystemExit(f"could not subscribe to {cmd_topic}")
        self.last = time.monotonic()
        self.lock = threading.Lock()

    def send(self, v: float, omega: float):
        for topic, value in build_commands(self.params, v, omega).items():
            msg = self.Double()
            msg.data = float(value)
            self.pubs[topic].publish(msg)

    def on_cmd(self, msg):
        with self.lock:
            self.last = time.monotonic()
        self.send(msg.linear.x, msg.angular.z)

    def run(self):
        while True:  # watchdog: stop the wheels if commands cease
            time.sleep(0.1)
            with self.lock:
                stale = time.monotonic() - self.last > self.timeout_s
            if stale:
                self.send(0.0, 0.0)
                with self.lock:
                    self.last = time.monotonic()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, required=True)
    ap.add_argument("--cmd-topic", default="/model/wheelloader/cmd_vel")
    ap.add_argument("--timeout", type=float, default=0.5, help="seconds without a command before stopping")
    args = ap.parse_args(argv)
    Bridge(json.loads(args.params.read_text()), args.cmd_topic, args.timeout).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
