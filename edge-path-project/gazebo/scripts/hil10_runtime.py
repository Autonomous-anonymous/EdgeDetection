"""Gazebo adapters for the add-only HIL-10 tests."""

import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "hil"))
from gazebo_lidar_sender import GazeboLidarSender


def odometry_values(msg):
    vector = msg.twist.linear
    speed = math.hypot(vector.x, vector.y, vector.z)
    header = getattr(msg, "header", None)
    stamp = None
    if header is not None and header.HasField("stamp"):
        stamp = (header.stamp.sec, header.stamp.nsec)
    return speed, stamp


class ManagedLidarSender(GazeboLidarSender):
    """Own the raw B LiDAR stream; a fault test can suspend only this stream."""

    def __init__(self, host, port=15010):
        self.gate = threading.RLock()
        self.enabled = True
        self.error = None
        super().__init__(host, port, "/aeb360b/lidar")

    def lidar_callback(self, msg):
        with self.gate:
            if self.enabled:
                try:
                    super().lidar_callback(msg)
                except OSError as exc:
                    self.error = str(exc)
                    self.enabled = False

    def suspend(self):
        with self.gate:
            self.enabled = False

    def close(self):
        with self.gate:
            self.enabled = False
            self.sock.close()


def verify_stop(monitor, send_stop, pump, timeout=3.0, on_sample=None):
    """Require three fresh samples received AFTER the first zero command."""
    send_stop()
    monitor.begin_stop()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        send_stop()
        pump()
        if on_sample is not None:
            on_sample()
        if monitor.stopped():
            return True
        time.sleep(0.05)
    return False


def stop_repeatedly(send_stop):
    for _ in range(5):
        send_stop()
        time.sleep(0.02)
