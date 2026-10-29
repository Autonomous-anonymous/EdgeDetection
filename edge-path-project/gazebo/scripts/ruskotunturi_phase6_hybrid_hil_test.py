#!/usr/bin/env python3

import argparse
import math
import socket
import struct
import time

from gz.msgs10.odometry_pb2 import Odometry

from ruskotunturi_phase6_two_vehicle_test import (
    Phase6TwoVehicleCoordinator,
    MODEL_B,
)


ODOM_B_TOPIC = f"/model/{MODEL_B}/odometry"

DEFAULT_UDP_PORT = 15000

SENSOR_PACKET = struct.Struct("!4sIff")
BRAKE_PACKET = struct.Struct("!4sIBBBx")

SENSOR_MAGIC = b"HILS"
BRAKE_MAGIC = b"HILB"

BRAKE_TIMEOUT_S = 0.50
STOP_VERIFY_SPEED_MPS = 0.10
STOP_VERIFY_TIMEOUT_S = 2.0

BRAKE_REASON_TEXT = {
    0: "none",
    1: "emergency_distance",
    2: "ttc_threshold",
    3: "sensor_timeout",
}


class Phase6HybridHIL(Phase6TwoVehicleCoordinator):

    def __init__(
        self,
        r2100_host,
        udp_port,
    ):
        super().__init__()

        self.r2100_host = r2100_host
        self.udp_port = udp_port

        self.b_speed_mps = None
        self.last_b_odom_time = None

        self.brake_active = True
        self.brake_reason = 3
        self.brake_counter = 0
        self.last_brake_time = None

        self.sensor_sequence = 0

        self.aeb_events = 0
        self.previous_effective_brake = True

        if not self.node.subscribe(
            Odometry,
            ODOM_B_TOPIC,
            self.b_odom_callback,
        ):
            raise RuntimeError(
                f"Failed to subscribe to {ODOM_B_TOPIC}"
            )

        self.hil_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.hil_sock.connect(
            (
                self.r2100_host,
                self.udp_port,
            )
        )

        self.hil_sock.setblocking(False)

    def b_odom_callback(self, msg):
        self.b_speed_mps = abs(
            msg.twist.linear.x
        )
        self.last_b_odom_time = time.monotonic()

    def send_speed_to_r2100(self):
        if self.b_speed_mps is None:
            return

        packet = SENSOR_PACKET.pack(
            SENSOR_MAGIC,
            self.sensor_sequence,
            float(self.b_speed_mps),

            # Distance is owned by the separate
            # R2100 LiDAR perception process.
            math.nan,
        )

        self.hil_sock.send(packet)

        self.sensor_sequence = (
            self.sensor_sequence + 1
        ) & 0xFFFFFFFF

    def receive_brake_from_r2100(self):
        while True:
            try:
                packet = self.hil_sock.recv(1024)

            except BlockingIOError:
                return

            if len(packet) != BRAKE_PACKET.size:
                continue

            (
                magic,
                _sensor_sequence,
                active,
                reason,
                counter,
            ) = BRAKE_PACKET.unpack(packet)

            if magic != BRAKE_MAGIC:
                continue

            if active not in (0, 1):
                continue

            self.brake_active = bool(active)
            self.brake_reason = reason
            self.brake_counter = counter
            self.last_brake_time = time.monotonic()

    def brake_fresh(self):
        return (
            self.last_brake_time is not None
            and (
                time.monotonic()
                - self.last_brake_time
            )
            <= BRAKE_TIMEOUT_S
        )

    def publish(
        self,
        publisher,
        speed,
        steering,
    ):
        # Vehicle A is completely unchanged.
        if publisher is not self.pub_b:
            return super().publish(
                publisher,
                speed,
                steering,
            )

        # Vehicle B:
        # 1. Send actual B speed to R2100 -> CAN 0x100.
        # 2. Receive R2100 AEB result <- CAN 0x200.
        # 3. Apply brake to the Phase 6 route command.
        self.send_speed_to_r2100()
        self.receive_brake_from_r2100()

        brake_fresh = self.brake_fresh()

        effective_brake = (
            not brake_fresh
            or self.brake_active
        )

        requested_motion = speed > 0.0

        if requested_motion:
            if (
                effective_brake
                and not self.previous_effective_brake
            ):
                self.aeb_events += 1

                reason = (
                    "brake_timeout"
                    if not brake_fresh
                    else BRAKE_REASON_TEXT.get(
                        self.brake_reason,
                        f"unknown_{self.brake_reason}",
                    )
                )

                print()
                print(
                    "*** VEHICLE B R2100 AEB OVERRIDE ***"
                )
                print(
                    f"Reason                  : {reason}"
                )
                print()

            self.previous_effective_brake = (
                effective_brake
            )

        # During coordinator HOLD, speed is already zero.
        # During an R2100 brake, override both speed and steering.
        if effective_brake:
            applied_speed = 0.0
            applied_steering = 0.0

        else:
            applied_speed = speed
            applied_steering = steering

        return super().publish(
            publisher,
            applied_speed,
            applied_steering,
        )

    def verify_b_stopped(self):
        deadline = (
            time.monotonic()
            + STOP_VERIFY_TIMEOUT_S
        )

        while time.monotonic() < deadline:
            if (
                self.b_speed_mps is not None
                and self.b_speed_mps
                <= STOP_VERIFY_SPEED_MPS
            ):
                return True

            time.sleep(0.05)

        return False

    def run(self):
        print()
        print(
            "============================================================"
        )
        print(
            " RUSKOTUNTURI HIL-9D - FULL HYBRID TWO VEHICLE MISSION"
        )
        print(
            "============================================================"
        )
        print(
            "Vehicle A : SIL route + safety + coordinator priority"
        )
        print(
            "Vehicle B : SIL steering + R2100 longitudinal AEB"
        )
        print(
            "B LiDAR   : Gazebo -> chunked UDP -> R2100 -> CAN 0x101"
        )
        print(
            "B speed   : Gazebo odometry -> UDP -> gateway -> CAN 0x100"
        )
        print(
            "B brake   : R2100 AEB -> CAN 0x200 -> UDP -> WSL"
        )
        print()

        base_pass = super().run()

        stop_verified = self.verify_b_stopped()

        print()
        print(
            "============================================================"
        )
        print(
            " HIL-9D HYBRID VERIFICATION"
        )
        print(
            "============================================================"
        )

        print(
            f"Vehicle B final speed    : "
            + (
                "missing"
                if self.b_speed_mps is None
                else f"{self.b_speed_mps:.3f} m/s"
            )
        )

        print(
            f"Vehicle B stop verified  : "
            f"{'YES' if stop_verified else 'NO'}"
        )

        print(
            f"R2100 AEB overrides      : "
            f"{self.aeb_events}"
        )

        passed = (
            base_pass
            and stop_verified
        )

        print(
            "HIL-9D RESULT            : "
            + (
                "PASS"
                if passed
                else "FAIL"
            )
        )

        print(
            "============================================================"
        )

        self.hil_sock.close()

        return passed


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--r2100-host",
        required=True,
    )

    parser.add_argument(
        "--udp-port",
        type=int,
        default=DEFAULT_UDP_PORT,
    )

    args = parser.parse_args()

    controller = Phase6HybridHIL(
        r2100_host=args.r2100_host,
        udp_port=args.udp_port,
    )

    raise SystemExit(
        0 if controller.run() else 1
    )


if __name__ == "__main__":
    main()
