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
import math
import select
import socket
import struct
import time

from can_protocol import (
    CAN_ID_BRAKE_COMMAND,
    CAN_ID_OBSTACLE_DISTANCE,
    CAN_ID_VEHICLE_SPEED,
    brake_reason_text,
    decode_brake_command,
    encode_obstacle_distance,
    encode_vehicle_speed,
)

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
    def __init__(self, can_interface: str, udp_port: int) -> None:
        self.can_interface = can_interface
        self.udp_port = udp_port

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

        self.can_sock.send(
            pack_can_frame(
                CAN_ID_VEHICLE_SPEED,
                encode_vehicle_speed(float(speed_mps)),
            )
        )
        self.can_sock.send(
            pack_can_frame(
                CAN_ID_OBSTACLE_DISTANCE,
                encode_obstacle_distance(distance),
            )
        )

    def handle_can(self) -> None:
        frame = self.can_sock.recv(CAN_FRAME_SIZE)
        can_id, data = unpack_can_frame(frame)

        if can_id != CAN_ID_BRAKE_COMMAND or len(data) != 8:
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

        print(
            f"UDP sensor age={age_text:>7s} | "
            f"speed={speed_text:>10s} | "
            f"distance={distance_text:>12s} | "
            f"brake={int(self.last_brake_active)} | "
            f"reason={brake_reason_text(self.last_brake_reason)} | "
            f"counter={self.last_brake_counter:3d}"
        )

        self.last_status_time = now

    def run(self) -> None:
        print()
        print("========================================")
        print(" R2100 HIL CAN GATEWAY")
        print("========================================")
        print(f"CAN interface : {self.can_interface}")
        print(f"UDP listen    : 0.0.0.0:{self.udp_port}")
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
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--udp-port", type=int, default=15000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    gateway = R2100CANGateway(
        can_interface=args.can_interface,
        udp_port=args.udp_port,
    )
    gateway.run()


if __name__ == "__main__":
    main()
