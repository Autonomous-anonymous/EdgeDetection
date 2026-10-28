#!/usr/bin/env python3
"""Send raw Gazebo LiDAR scans to the R2100 over UDP.

HIL-7B1 only:
    Gazebo /aeb/lidar
        -> raw scan encoded as uint16 millimetres
        -> UDP
        -> R2100 perception process

This process does not control the vehicle and does not publish CAN signals.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node


LIDAR_TOPIC = "/aeb/lidar"

DEFAULT_PORT = 15010

MAGIC = b"HILR"

# magic, sequence, angle_min, angle_step, range_min, range_max, count
HEADER = struct.Struct("!4sIffffH")

INVALID_RANGE_MM = 0xFFFF

PRINT_INTERVAL_SECONDS = 1.0


class GazeboLidarSender:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port

        self.node = Node()

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.sock.connect((host, port))

        self.sequence = 0
        self.last_print_time = 0.0

        ok = self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback,
        )

        if not ok:
            raise RuntimeError(
                f"failed to subscribe to LiDAR topic: {LIDAR_TOPIC}"
            )

    @staticmethod
    def encode_range_mm(
        distance_m: float,
        range_min_m: float,
        range_max_m: float,
    ) -> int:
        if not math.isfinite(distance_m):
            return INVALID_RANGE_MM

        if distance_m < range_min_m:
            return INVALID_RANGE_MM

        if distance_m > range_max_m:
            return INVALID_RANGE_MM

        distance_mm = round(distance_m * 1000.0)

        if distance_mm < 0:
            return INVALID_RANGE_MM

        if distance_mm >= INVALID_RANGE_MM:
            return INVALID_RANGE_MM

        return distance_mm

    def lidar_callback(self, msg: LaserScan) -> None:
        count = len(msg.ranges)

        ranges_mm = [
            self.encode_range_mm(
                distance,
                msg.range_min,
                msg.range_max,
            )
            for distance in msg.ranges
        ]

        header = HEADER.pack(
            MAGIC,
            self.sequence,
            float(msg.angle_min),
            float(msg.angle_step),
            float(msg.range_min),
            float(msg.range_max),
            count,
        )

        payload = struct.pack(
            f"!{count}H",
            *ranges_mm,
        )

        packet = header + payload

        self.sock.send(packet)

        now = time.monotonic()

        if now - self.last_print_time >= PRINT_INTERVAL_SECONDS:
            print(
                f"TX raw lidar | "
                f"seq={self.sequence:6d} | "
                f"beams={count:3d} | "
                f"bytes={len(packet):4d} | "
                f"destination={self.host}:{self.port}"
            )
            self.last_print_time = now

        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

    def run(self) -> None:
        print()
        print("========================================")
        print(" GAZEBO RAW LIDAR UDP SENDER")
        print("========================================")
        print(f"Topic       : {LIDAR_TOPIC}")
        print(f"Destination : {self.host}:{self.port}")
        print("Encoding    : uint16 millimetres")
        print("Invalid     : 0xFFFF")
        print()
        print("Ctrl+C lopettaa.")
        print()

        try:
            while True:
                time.sleep(1.0)

        except KeyboardInterrupt:
            print("\nDone.")


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--r2100-host",
        required=True,
        help="R2100 IP address",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"UDP port (default: {DEFAULT_PORT})",
    )

    args = parser.parse_args()

    sender = GazeboLidarSender(
        host=args.r2100_host,
        port=args.port,
    )
    sender.run()


if __name__ == "__main__":
    main()
