#!/usr/bin/env python3
"""R2100 network-to-CAN gateway for the first closed-loop HIL milestone.

Runs on the reComputer Industrial R2100.

Network side:
    Receives Gazebo sensor values over UDP from the simulation PC.
    Sends brake state back to the same UDP peer.

CAN side:
    TX on can0:
        0x100 vehicle speed
        0x101 front obstacle distance
    RX on can0:
        0x200 brake command from r2100_aeb_controller.py on can1

The physical bench connection remains:
    CAN0_H <-> CAN1_H
    CAN0_L <-> CAN1_L
"""

from __future__ import annotations

import argparse
import errno
import math
import select
import socket
import struct
import time

from can_protocol import (
    brake_reason_text,
    decode_brake_command,
    encode_obstacle_distance,
    encode_vehicle_speed,
)
from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_MASK = 0x1FFFFFFF

SENSOR_PACKET = struct.Struct("!4sIff")
BRAKE_PACKET = struct.Struct("!4sIBBBx")

SENSOR_MAGIC = b"HILS"
BRAKE_MAGIC = b"HILB"

STATUS_INTERVAL_SECONDS = 0.50


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    if len(data) != 8:
        raise ValueError("this HIL protocol uses exactly 8 data bytes")
    return struct.pack(CAN_FRAME_FORMAT, can_id, len(data), data)


def unpack_can_frame(frame: bytes) -> tuple[int, bytes]:
    can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
    return can_id & CAN_EFF_MASK, data[:dlc]


class R2100CANGateway:
    def __init__(
        self,
        can_interface: str,
        udp_port: int,
        speed_can_id: int,
        distance_can_id: int,
        brake_can_id: int,
        mapping_profile: str,
        mapping_bitrate: int,
        external_perception: bool = False,
    ) -> None:
        self.can_interface = can_interface
        self.udp_port = udp_port
        self.speed_can_id = speed_can_id
        self.distance_can_id = distance_can_id
        self.brake_can_id = brake_can_id
        self.mapping_profile = mapping_profile
        self.mapping_bitrate = mapping_bitrate
        self.external_perception = external_perception

        self.can_sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )
        self.can_sock.bind((self.can_interface,))
        self.can_sock.setblocking(False)

        self.udp_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.udp_sock.bind(("0.0.0.0", self.udp_port))
        self.udp_sock.setblocking(False)

        self.peer: tuple[str, int] | None = None
        self.last_sensor_seq = 0
        self.last_sensor_time: float | None = None

        self.last_speed_mps: float | None = None
        self.last_distance_m: float | None = None
        self.last_brake_active = True
        self.last_brake_reason = 3
        self.last_brake_counter = 0

        self.last_status_time = 0.0
        self.can_tx_drop_count = 0

    def send_can(self, can_id: int, data: bytes) -> bool:
        try:
            self.can_sock.send(
                pack_can_frame(
                    can_id,
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

    def handle_udp(self) -> None:
        packet, peer = self.udp_sock.recvfrom(1024)

        if len(packet) != SENSOR_PACKET.size:
            return

        magic, seq, speed_mps, distance_m = SENSOR_PACKET.unpack(packet)
        if magic != SENSOR_MAGIC:
            return

        if not math.isfinite(speed_mps) or speed_mps < 0.0:
            return

        if math.isnan(distance_m):
            distance: float | None = None
        elif math.isfinite(distance_m) and distance_m >= 0.0:
            distance = float(distance_m)
        else:
            return

        self.peer = peer
        self.last_sensor_seq = seq
        self.last_sensor_time = time.monotonic()
        self.last_speed_mps = float(speed_mps)
        self.last_distance_m = distance

        self.send_can(
            self.speed_can_id,
            encode_vehicle_speed(float(speed_mps)),
        )

        if not self.external_perception:
            self.send_can(
                self.distance_can_id,
                encode_obstacle_distance(distance),
            )

    def handle_can(self) -> None:
        frame = self.can_sock.recv(CAN_FRAME_SIZE)
        can_id, data = unpack_can_frame(frame)

        if can_id != self.brake_can_id or len(data) != 8:
            return

        active, reason, counter = decode_brake_command(data)

        self.last_brake_active = active
        self.last_brake_reason = reason
        self.last_brake_counter = counter

        if self.peer is None:
            return

        packet = BRAKE_PACKET.pack(
            BRAKE_MAGIC,
            self.last_sensor_seq,
            int(active),
            reason,
            counter,
        )
        self.udp_sock.sendto(packet, self.peer)

    def print_status(self, now: float) -> None:
        if now - self.last_status_time < STATUS_INTERVAL_SECONDS:
            return

        if self.last_sensor_time is None:
            age_text = "never"
        else:
            age_text = f"{now - self.last_sensor_time:.3f}s"

        speed_text = (
            "missing"
            if self.last_speed_mps is None
            else f"{self.last_speed_mps:.2f} m/s"
        )
        distance_text = (
            "no obstacle"
            if self.last_distance_m is None
            else f"{self.last_distance_m:.3f} m"
        )

        distance_label = (
            "bridge_distance"
            if self.external_perception
            else "distance"
        )

        print(
            f"UDP sensor age={age_text:>7s} | "
            f"speed={speed_text:>10s} | "
            f"{distance_label}={distance_text:>12s} | "
            f"brake={int(self.last_brake_active)} | "
            f"reason={brake_reason_text(self.last_brake_reason)} | "
            f"counter={self.last_brake_counter:3d} | "
            f"can_tx_drops={self.can_tx_drop_count}"
        )

        self.last_status_time = now

    def run(self) -> None:
        print()
        print("========================================")
        print(" R2100 HIL CAN GATEWAY")
        print("========================================")
        print(f"Mapping profile  : {self.mapping_profile}")
        print(f"CAN interface    : {self.can_interface}")
        print(f"Expected bitrate : {self.mapping_bitrate}")
        print(f"Speed CAN ID     : 0x{self.speed_can_id:03X}")
        print(f"Distance CAN ID  : 0x{self.distance_can_id:03X}")
        print(f"Brake CAN ID     : 0x{self.brake_can_id:03X}")
        print(f"UDP listen       : 0.0.0.0:{self.udp_port}")
        print(
            "Perception       : "
            + (
                f"external (0x{self.distance_can_id:03X} not transmitted)"
                if self.external_perception
                else (
                    "gateway distance -> "
                    f"CAN 0x{self.distance_can_id:03X}"
                )
            )
        )
        print()
        print("Waiting for Gazebo sensor packets. Ctrl+C stops.")
        print()

        try:
            while True:
                readable, _, _ = select.select(
                    [self.udp_sock, self.can_sock],
                    [],
                    [],
                    0.05,
                )

                for sock in readable:
                    if sock is self.udp_sock:
                        self.handle_udp()
                    elif sock is self.can_sock:
                        self.handle_can()

                self.print_status(time.monotonic())

        except KeyboardInterrupt:
            print()
            print("Gateway stopped by user.")

        finally:
            self.can_sock.close()
            self.udp_sock.close()
            print("Done.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="path to HIL signal mapping JSON",
    )
    parser.add_argument(
        "--can-interface",
        default=None,
        help="override gateway CAN interface from mapping",
    )
    parser.add_argument("--udp-port", type=int, default=15000)
    parser.add_argument(
        "--external-perception",
        action="store_true",
        help="do not transmit configured obstacle-distance CAN signal",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mapping = load_signal_mapping(args.config)

    speed_signal = mapping.signal("vehicle_speed")
    distance_signal = mapping.signal("obstacle_distance")
    brake_signal = mapping.signal("brake_command")

    can_interface = (
        args.can_interface
        if args.can_interface is not None
        else mapping.can.gateway_interface
    )

    gateway = R2100CANGateway(
        can_interface=can_interface,
        udp_port=args.udp_port,
        speed_can_id=speed_signal.can_id,
        distance_can_id=distance_signal.can_id,
        brake_can_id=brake_signal.can_id,
        mapping_profile=mapping.profile,
        mapping_bitrate=mapping.can.bitrate,
        external_perception=args.external_perception,
    )
    gateway.run()


if __name__ == "__main__":
    main()
