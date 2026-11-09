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
import fcntl
import os
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheelloader_kinematics as kin  # noqa: E402

WHEELS = ("wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr")
BRIDGE_LOCK = Path("/tmp") / f"camino_wheelloader_bridge_{os.getuid()}.lock"


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
        # Shared by the standalone bridge and the smoke-test's embedded bridge.
        # flock releases automatically if the process crashes.
        self._lockfile = BRIDGE_LOCK.open("a+")
        try:
            fcntl.flock(self._lockfile.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lockfile.close()
            self._lockfile = None
            raise SystemExit("Another wheel-loader bridge is already running; stop it before testing")

        try:
            from gz.msgs10.double_pb2 import Double
            from gz.msgs10.twist_pb2 import Twist
            from gz.transport13 import Node

            self.params = params
            self.Double = Double
            self.timeout_s = timeout_s
            self.last = time.monotonic()
            self.lock = threading.Lock()
            self.node = Node()
            topics = list(build_commands(params, 0.0, 0.0))
            self.pubs = {t: self.node.advertise(t, Double) for t in topics}
            if not self.node.subscribe(Twist, cmd_topic, self.on_cmd):
                raise SystemExit(f"could not subscribe to {cmd_topic}")
        except BaseException:
            self.close()
            raise

    def close(self):
        if self._lockfile is not None:
            fcntl.flock(self._lockfile.fileno(), fcntl.LOCK_UN)
            self._lockfile.close()
            self._lockfile = None

    def send(self, v: float, omega: float):
        for topic, value in build_commands(self.params, v, omega).items():
            msg = self.Double()
            msg.data = float(value)
            self.pubs[topic].publish(msg)

    def on_cmd(self, msg):
        with self.lock:
            self.last = time.monotonic()
        self.send(msg.linear.x, msg.angular.z)

    def run(self, stop_event: threading.Event | None = None):
        stop_event = stop_event or threading.Event()
        try:
            while not stop_event.wait(0.1):  # watchdog: stop if commands cease
                with self.lock:
                    stale = time.monotonic() - self.last > self.timeout_s
                if stale:
                    self.send(0.0, 0.0)
                    with self.lock:
                        self.last = time.monotonic()
        finally:
            self.send(0.0, 0.0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, required=True)
    ap.add_argument("--cmd-topic", default="/model/wheelloader/cmd_vel")
    ap.add_argument("--timeout", type=float, default=0.5, help="seconds without a command before stopping")
    args = ap.parse_args(argv)
    bridge = Bridge(json.loads(args.params.read_text()), args.cmd_topic, args.timeout)
    try:
        bridge.run()
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
