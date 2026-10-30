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

from signal_codec import encode_signal_value
from signal_mapping import (
    DEFAULT_CONFIG_PATH,
    SignalDefinition,
    load_signal_mapping,
)


DEFAULT_PORT = 15010

MAGIC = b"HILR"
CHUNK_MAGIC = b"HILC"

# magic, sequence, angle_min, angle_step, range_min, range_max,
# total_beams, start_index, chunk_beams
CHUNK_HEADER = struct.Struct("!4sIffffHHH")

# magic, sequence, angle_min, angle_step, range_min, range_max, count
HEADER = struct.Struct("!4sIffffH")

INVALID_RANGE_MM = 0xFFFF

FRONT_HALF_ANGLE_DEG = 10.0

# LiDAR is roof-centred. Convert sensor range to clearance from the
# physical vehicle body before transmitting CAN 0x101.
VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.60
SENSOR_TIMEOUT_SECONDS = 0.50
PRINT_INTERVAL_SECONDS = 1.0

CAN_FRAME_FORMAT = "=IB3x8s"



def body_radius_for_ray(angle_rad: float) -> float:
    cos_a = abs(math.cos(angle_rad))
    sin_a = abs(math.sin(angle_rad))

    candidates = []

    if cos_a > 1e-9:
        candidates.append(
            VEHICLE_HALF_LENGTH_M / cos_a
        )

    if sin_a > 1e-9:
        candidates.append(
            VEHICLE_HALF_WIDTH_M / sin_a
        )

    if not candidates:
        return 0.0

    return min(candidates)


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
        distance_signal: SignalDefinition,
        mapping_profile: str,
        mapping_bitrate: int,
    ) -> None:
        self.port = port
        self.can_interface = can_interface
        self.distance_signal = distance_signal
        self.distance_can_id = distance_signal.can_id
        self.mapping_profile = mapping_profile
        self.mapping_bitrate = mapping_bitrate

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

        # HIL-9: reassembly state for chunked LiDAR scans.
        self.chunk_sequence = None
        self.chunk_meta = None
        self.chunk_ranges = None
        self.chunk_received = None
        self.chunk_received_count = 0

    def send_distance_can(
        self,
        distance_m: float | None,
    ) -> bool:
        data = encode_signal_value(
            self.distance_signal,
            distance_m,
        )

        try:
            self.can_sock.send(
                pack_can_frame(
                    self.distance_can_id,
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

    def process_legacy_packet(self, packet: bytes) -> None:
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

            body_radius_m = body_radius_for_ray(angle)

            clearance_m = max(
                0.0,
                distance_m - body_radius_m,
            )

            front_ranges_m.append(clearance_m)

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
                f"CAN=0x{self.distance_can_id:03X} | "
                f"rate={rate_hz:4.1f} Hz | "
                f"can_tx_drops={self.can_tx_drop_count}"
            )

            self.rx_count = 0
            self.last_print_time = now

    def process_chunk_packet(self, packet: bytes) -> None:
        if len(packet) < CHUNK_HEADER.size:
            print(
                f"DROP: chunk packet too short "
                f"({len(packet)} bytes)"
            )
            return

        (
            magic,
            sequence,
            angle_min,
            angle_step,
            range_min,
            range_max,
            total_beams,
            start_index,
            chunk_beams,
        ) = CHUNK_HEADER.unpack_from(packet)

        if magic != CHUNK_MAGIC:
            print(f"DROP: wrong chunk magic {magic!r}")
            return

        if total_beams == 0 or total_beams > 4096:
            print(
                f"DROP: invalid total beam count "
                f"{total_beams}"
            )
            return

        if chunk_beams == 0:
            print("DROP: empty LiDAR chunk")
            return

        if start_index + chunk_beams > total_beams:
            print(
                f"DROP: invalid chunk range "
                f"start={start_index} "
                f"count={chunk_beams} "
                f"total={total_beams}"
            )
            return

        expected_size = (
            CHUNK_HEADER.size
            + chunk_beams * 2
        )

        if len(packet) != expected_size:
            print(
                f"DROP: wrong chunk size "
                f"got={len(packet)} "
                f"expected={expected_size}"
            )
            return

        chunk_ranges = struct.unpack_from(
            f"!{chunk_beams}H",
            packet,
            CHUNK_HEADER.size,
        )

        meta = (
            angle_min,
            angle_step,
            range_min,
            range_max,
            total_beams,
        )

        if (
            self.chunk_sequence != sequence
            or self.chunk_meta != meta
        ):
            self.chunk_sequence = sequence
            self.chunk_meta = meta
            self.chunk_ranges = [None] * total_beams
            self.chunk_received = bytearray(total_beams)
            self.chunk_received_count = 0

        assert self.chunk_ranges is not None
        assert self.chunk_received is not None

        for offset, distance_mm in enumerate(
            chunk_ranges
        ):
            index = start_index + offset

            if not self.chunk_received[index]:
                self.chunk_ranges[index] = distance_mm
                self.chunk_received[index] = 1
                self.chunk_received_count += 1

        if self.chunk_received_count != total_beams:
            return

        if any(
            value is None
            for value in self.chunk_ranges
        ):
            print(
                f"DROP: incomplete reassembly "
                f"seq={sequence}"
            )
            return

        ranges_mm = list(self.chunk_ranges)

        # Clear reassembly state before processing the
        # completed scan.
        self.chunk_sequence = None
        self.chunk_meta = None
        self.chunk_ranges = None
        self.chunk_received = None
        self.chunk_received_count = 0

        legacy_header = HEADER.pack(
            MAGIC,
            sequence,
            angle_min,
            angle_step,
            range_min,
            range_max,
            total_beams,
        )

        payload = struct.pack(
            f"!{total_beams}H",
            *ranges_mm,
        )

        self.process_legacy_packet(
            legacy_header + payload
        )

    def process_packet(self, packet: bytes) -> None:
        if len(packet) < 4:
            print(
                f"DROP: packet too short "
                f"({len(packet)} bytes)"
            )
            return

        magic = packet[:4]

        if magic == MAGIC:
            self.process_legacy_packet(packet)
            return

        if magic == CHUNK_MAGIC:
            self.process_chunk_packet(packet)
            return

        print(f"DROP: unknown packet magic {magic!r}")

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
        print(f"Mapping profile  : {self.mapping_profile}")
        print(f"UDP port         : {self.port}")
        print(f"CAN interface    : {self.can_interface}")
        print(f"Expected bitrate : {self.mapping_bitrate}")
        print(
            f"Front sector  : "
            f"+/-{FRONT_HALF_ANGLE_DEG:.1f} deg"
        )
        print(
            f"CAN output       : "
            f"0x{self.distance_can_id:03X}"
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
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="path to HIL signal mapping JSON",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )

    parser.add_argument(
        "--can-interface",
        default=None,
        help="override perception CAN interface from mapping",
    )

    args = parser.parse_args()

    mapping = load_signal_mapping(args.config)
    distance_signal = mapping.signal("obstacle_distance")

    can_interface = (
        args.can_interface
        if args.can_interface is not None
        else mapping.can.gateway_interface
    )

    perception = LidarPerceptionCAN(
        port=args.port,
        can_interface=can_interface,
        distance_signal=distance_signal,
        mapping_profile=mapping.profile,
        mapping_bitrate=mapping.can.bitrate,
    )

    perception.run()


if __name__ == "__main__":
    main()
