#!/usr/bin/env python3
"""Minimal AEB controller for the reComputer Industrial R2100 HIL stage.

The controller is deliberately independent of Gazebo. It only knows CAN signals:

RX on can1:
    0x100 vehicle speed
    0x101 front obstacle distance

TX on can1:
    0x200 brake command

For the current bench setup, CAN1 is the controller side and CAN0 is the
simulation/bridge side. The two ports are physically connected H-H and L-L.

The AEB thresholds match the current Gazebo SIL AEB baseline.
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
    BRAKE_REASON_EMERGENCY_DISTANCE,
    BRAKE_REASON_NONE,
    BRAKE_REASON_SENSOR_TIMEOUT,
    BRAKE_REASON_TTC,
    brake_reason_text,
    decode_obstacle_distance,
    decode_vehicle_speed,
    encode_brake_command,
)
from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping

TTC_THRESHOLD_SECONDS = 1.7
MIN_SPEED_FOR_TTC_MPS = 0.10
EMERGENCY_DISTANCE_M = 0.80
SENSOR_TIMEOUT_SECONDS = 0.50
CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.25

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_MASK = 0x1FFFFFFF


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    if len(data) != 8:
        raise ValueError("this HIL protocol uses exactly 8 data bytes")
    return struct.pack(CAN_FRAME_FORMAT, can_id, len(data), data)


def unpack_can_frame(frame: bytes) -> tuple[int, bytes]:
    can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
    can_id &= CAN_EFF_MASK
    return can_id, data[:dlc]


class R2100AEBController:
    def __init__(
        self,
        interface: str,
        speed_can_id: int,
        distance_can_id: int,
        brake_can_id: int,
        mapping_profile: str,
        mapping_bitrate: int,
    ) -> None:
        self.interface = interface
        self.speed_can_id = speed_can_id
        self.distance_can_id = distance_can_id
        self.brake_can_id = brake_can_id
        self.mapping_profile = mapping_profile
        self.mapping_bitrate = mapping_bitrate

        self.sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )
        self.sock.bind((self.interface,))
        self.sock.setblocking(False)

        self.speed_mps: float | None = None
        self.front_distance_m: float | None = None

        self.last_speed_time: float | None = None
        self.last_distance_time: float | None = None

        self.aeb_latched = False
        self.latched_reason = BRAKE_REASON_NONE

        self.counter = 0
        self.can_tx_drop_count = 0
        self.last_print_time = 0.0

    def handle_frame(self, frame: bytes, now: float) -> None:
        can_id, data = unpack_can_frame(frame)

        if can_id == self.speed_can_id:
            if len(data) != 8:
                return
            self.speed_mps = decode_vehicle_speed(data)
            self.last_speed_time = now

        elif can_id == self.distance_can_id:
            if len(data) != 8:
                return
            self.front_distance_m = decode_obstacle_distance(data)
            self.last_distance_time = now

    def sensors_fresh(self, now: float) -> bool:
        if self.last_speed_time is None or self.last_distance_time is None:
            return False

        return (
            now - self.last_speed_time <= SENSOR_TIMEOUT_SECONDS
            and now - self.last_distance_time <= SENSOR_TIMEOUT_SECONDS
        )

    def evaluate(self, now: float) -> tuple[bool, int, float]:
        if not self.sensors_fresh(now):
            return True, BRAKE_REASON_SENSOR_TIMEOUT, math.inf

        assert self.speed_mps is not None

        ttc = math.inf
        if (
            self.front_distance_m is not None
            and self.speed_mps >= MIN_SPEED_FOR_TTC_MPS
        ):
            ttc = self.front_distance_m / self.speed_mps

        if not self.aeb_latched:
            reason = BRAKE_REASON_NONE

            if (
                self.front_distance_m is not None
                and self.front_distance_m <= EMERGENCY_DISTANCE_M
            ):
                reason = BRAKE_REASON_EMERGENCY_DISTANCE

            elif (
                self.front_distance_m is not None
                and self.speed_mps >= MIN_SPEED_FOR_TTC_MPS
                and ttc <= TTC_THRESHOLD_SECONDS
            ):
                reason = BRAKE_REASON_TTC

            if reason != BRAKE_REASON_NONE:
                self.aeb_latched = True
                self.latched_reason = reason

                print()
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(" AEB TRIGGERED")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"Reason   : {brake_reason_text(reason)}")
                print(
                    "Distance : "
                    + (
                        "no obstacle"
                        if self.front_distance_m is None
                        else f"{self.front_distance_m:.3f} m"
                    )
                )
                print(f"Speed    : {self.speed_mps:.2f} m/s")
                if math.isfinite(ttc):
                    print(f"TTC      : {ttc:.3f} s")
                print()

        if self.aeb_latched:
            return True, self.latched_reason, ttc

        return False, BRAKE_REASON_NONE, ttc

    def send_brake_command(self, active: bool, reason: int) -> bool:
        data = encode_brake_command(active, reason, self.counter)

        try:
            self.sock.send(
                pack_can_frame(
                    self.brake_can_id,
                    data,
                )
            )

        except OSError as exc:
            if exc.errno in {
                errno.ENOBUFS,
                errno.EAGAIN,
                errno.EWOULDBLOCK,
            }:
                self.can_tx_drop_count += 1
                return False

            raise

        self.counter = (self.counter + 1) & 0xFF
        return True

    def print_status(
        self,
        now: float,
        brake_active: bool,
        reason: int,
        ttc: float,
    ) -> None:
        if now - self.last_print_time < PRINT_INTERVAL_SECONDS:
            return

        speed_text = (
            "missing"
            if self.speed_mps is None
            else f"{self.speed_mps:.2f} m/s"
        )
        distance_text = (
            "no obstacle"
            if self.front_distance_m is None
            else f"{self.front_distance_m:.3f} m"
        )
        ttc_text = f"{ttc:.2f} s" if math.isfinite(ttc) else "inf"

        state = "BRAKE" if brake_active else "DRIVE"

        print(
            f"{state:5s} | "
            f"speed={speed_text:>10s} | "
            f"distance={distance_text:>12s} | "
            f"TTC={ttc_text:>7s} | "
            f"reason={brake_reason_text(reason)}"
        )

        self.last_print_time = now

    def run(self) -> None:
        print()
        print("========================================")
        print(" R2100 HIL AEB CONTROLLER")
        print("========================================")
        print(f"Mapping profile    : {self.mapping_profile}")
        print(f"CAN interface      : {self.interface}")
        print(f"Expected bitrate   : {self.mapping_bitrate}")
        print(f"Speed CAN ID       : 0x{self.speed_can_id:03X}")
        print(f"Distance CAN ID    : 0x{self.distance_can_id:03X}")
        print(f"Brake CAN ID       : 0x{self.brake_can_id:03X}")
        print(f"TTC threshold      : {TTC_THRESHOLD_SECONDS:.2f} s")
        print(f"Emergency distance : {EMERGENCY_DISTANCE_M:.2f} m")
        print(f"Sensor timeout     : {SENSOR_TIMEOUT_SECONDS:.2f} s")
        print()
        print("Waiting for CAN sensor frames. Ctrl+C stops.")
        print()

        next_control_time = time.monotonic()

        try:
            while True:
                now = time.monotonic()

                readable, _, _ = select.select([self.sock], [], [], 0.0)
                while readable:
                    frame = self.sock.recv(CAN_FRAME_SIZE)
                    self.handle_frame(frame, time.monotonic())
                    readable, _, _ = select.select([self.sock], [], [], 0.0)

                now = time.monotonic()

                if now >= next_control_time:
                    brake_active, reason, ttc = self.evaluate(now)
                    self.send_brake_command(brake_active, reason)
                    self.print_status(now, brake_active, reason, ttc)

                    next_control_time += CONTROL_PERIOD_SECONDS
                    if next_control_time < now:
                        next_control_time = now + CONTROL_PERIOD_SECONDS

                sleep_time = max(0.0, next_control_time - time.monotonic())
                time.sleep(min(sleep_time, 0.005))

        except KeyboardInterrupt:
            print()
            print("Controller stopped by user.")

        finally:
            print("Sending final fail-safe BRAKE commands...")
            for _ in range(5):
                self.send_brake_command(
                    True,
                    BRAKE_REASON_SENSOR_TIMEOUT,
                )
                time.sleep(0.02)

            print(
                f"CAN TX drops during run: {self.can_tx_drop_count}"
            )
            self.sock.close()
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
        help="override controller CAN interface from mapping",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mapping = load_signal_mapping(args.config)

    speed_signal = mapping.signal("vehicle_speed")
    distance_signal = mapping.signal("obstacle_distance")
    brake_signal = mapping.signal("brake_command")

    interface = (
        args.can_interface
        if args.can_interface is not None
        else mapping.can.controller_interface
    )

    controller = R2100AEBController(
        interface=interface,
        speed_can_id=speed_signal.can_id,
        distance_can_id=distance_signal.can_id,
        brake_can_id=brake_signal.can_id,
        mapping_profile=mapping.profile,
        mapping_bitrate=mapping.can.bitrate,
    )
    controller.run()


if __name__ == "__main__":
    main()
