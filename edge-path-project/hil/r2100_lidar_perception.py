#!/usr/bin/env python3
"""Receive raw Gazebo LiDAR scans and send front distance over CAN.

HIL-7B2A:
    Gazebo raw LiDAR
        -> UDP
        -> R2100 perception
        -> CAN 0x101 OBSTACLE_DISTANCE

This version does NOT perform the AEB decision.
"""

from __future__ import annotations

import argparse
import errno
import math
import socket
import struct
import time

from can_protocol import (
    CAN_ID_OBSTACLE_DISTANCE,
    encode_obstacle_distance,
)


DEFAULT_PORT = 15010
DEFAULT_CAN_INTERFACE = "can0"

MAGIC = b"HILR"

# magic, sequence, angle_min, angle_step, range_min, range_max, count
HEADER = struct.Struct("!4sIffffH")

INVALID_RANGE_MM = 0xFFFF

FRONT_HALF_ANGLE_DEG = 10.0
SENSOR_TIMEOUT_SECONDS = 0.50
PRINT_INTERVAL_SECONDS = 1.0

CAN_FRAME_FORMAT = "=IB3x8s"


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    if len(data) != 8:
        raise ValueError("CAN payload must contain exactly 8 bytes")

    return struct.pack(
        CAN_FRAME_FORMAT,
        can_id,
        len(data),
        data,
    )


class LidarPerceptionCAN:
    def __init__(
        self,
        port: int,
        can_interface: str,
    ) -> None:
        self.port = port
        self.can_interface = can_interface

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.25)

        self.can_sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )
        self.can_sock.bind((can_interface,))
        self.can_sock.setblocking(False)

        self.last_packet_time: float | None = None
        self.last_sequence: int | None = None
        self.last_front_distance_m: float | None = None

        self.rx_count = 0
        self.can_tx_drop_count = 0

        self.last_print_time = time.monotonic()
        self.stale_reported = False

    def send_distance_can(
        self,
        distance_m: float | None,
    ) -> bool:
        data = encode_obstacle_distance(distance_m)

        try:
            self.can_sock.send(
                pack_can_frame(
                    CAN_ID_OBSTACLE_DISTANCE,
                    data,
                )
            )
            return True

        except OSError as exc:
            if exc.errno in {
                errno.ENOBUFS,
                errno.EAGAIN,
                errno.EWOULDBLOCK,
            }:
                self.can_tx_drop_count += 1
                return False

            raise

    def process_packet(self, packet: bytes) -> None:
        if len(packet) < HEADER.size:
            print(f"DROP: packet too short ({len(packet)} bytes)")
            return

        (
            magic,
            sequence,
            angle_min,
            angle_step,
            range_min,
            range_max,
            count,
        ) = HEADER.unpack_from(packet)

        if magic != MAGIC:
            print(f"DROP: wrong magic {magic!r}")
            return

        expected_size = HEADER.size + count * 2

        if len(packet) != expected_size:
            print(
                f"DROP: wrong packet size "
                f"got={len(packet)} expected={expected_size}"
            )
            return

        if count == 0 or count > 4096:
            print(f"DROP: invalid beam count {count}")
            return

        ranges_mm = struct.unpack_from(
            f"!{count}H",
            packet,
            HEADER.size,
        )

        front_half_angle_rad = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        front_ranges_m: list[float] = []
        sector_beams = 0

        for index, distance_mm in enumerate(ranges_mm):
            angle = angle_min + index * angle_step

            if abs(angle) > front_half_angle_rad:
                continue

            sector_beams += 1

            if distance_mm == INVALID_RANGE_MM:
                continue

            distance_m = distance_mm / 1000.0

            if distance_m < range_min:
                continue

            if distance_m > range_max:
                continue

            front_ranges_m.append(distance_m)

        front_distance_m = (
            min(front_ranges_m)
            if front_ranges_m
            else None
        )

        # HIL-7B2A:
        # R2100 perception owns CAN ID 0x101.
        self.send_distance_can(front_distance_m)

        now = time.monotonic()

        self.last_packet_time = now
        self.last_sequence = sequence
        self.last_front_distance_m = front_distance_m

        self.rx_count += 1
        self.stale_reported = False

        elapsed = now - self.last_print_time

        if elapsed >= PRINT_INTERVAL_SECONDS:
            rate_hz = self.rx_count / elapsed

            if front_distance_m is None:
                distance_text = "none"
            else:
                distance_text = f"{front_distance_m:.3f} m"

            print(
                f"RX lidar | "
                f"seq={sequence:6d} | "
                f"beams={count:3d} | "
                f"sector={sector_beams:2d} | "
                f"front={distance_text:>9s} | "
                f"CAN=0x{CAN_ID_OBSTACLE_DISTANCE:03X} | "
                f"rate={rate_hz:4.1f} Hz | "
                f"can_tx_drops={self.can_tx_drop_count}"
            )

            self.rx_count = 0
            self.last_print_time = now

    def check_timeout(self) -> None:
        if self.last_packet_time is None:
            return

        age = time.monotonic() - self.last_packet_time

        if (
            age > SENSOR_TIMEOUT_SECONDS
            and not self.stale_reported
        ):
            print(
                f"STALE: no LiDAR packet for "
                f"{age:.3f} s"
            )

            # Do not transmit a replacement distance when LiDAR is stale.
            # The AEB controller will detect the missing 0x101 updates and
            # enter its sensor-timeout fail-safe state.
            self.stale_reported = True

    def run(self) -> None:
        print()
        print("========================================")
        print(" R2100 LIDAR PERCEPTION -> CAN")
        print("========================================")
        print(f"UDP port      : {self.port}")
        print(f"CAN interface : {self.can_interface}")
        print(
            f"Front sector  : "
            f"+/-{FRONT_HALF_ANGLE_DEG:.1f} deg"
        )
        print(
            f"CAN output    : "
            f"0x{CAN_ID_OBSTACLE_DISTANCE:03X}"
        )
        print()
        print("Ctrl+C lopettaa.")
        print()

        try:
            while True:
                try:
                    packet, _peer = self.sock.recvfrom(65535)
                    self.process_packet(packet)

                except socket.timeout:
                    self.check_timeout()

        except KeyboardInterrupt:
            print("\nDone.")

        finally:
            self.can_sock.close()
            self.sock.close()


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )

    parser.add_argument(
        "--can-interface",
        default=DEFAULT_CAN_INTERFACE,
    )

    args = parser.parse_args()

    perception = LidarPerceptionCAN(
        port=args.port,
        can_interface=args.can_interface,
    )

    perception.run()


if __name__ == "__main__":
    main()
