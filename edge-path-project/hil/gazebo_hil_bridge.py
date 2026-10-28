#!/usr/bin/env python3
"""Gazebo-to-R2100 bridge for the first closed-loop HIL AEB test.

Runs on the Gazebo / WSL PC.

Gazebo side:
    RX /aeb/lidar
    RX /model/aeb_vehicle/odometry
    TX /model/aeb_vehicle/cmd_vel

Network side:
    Sends speed + front obstacle distance to r2100_can_gateway.py over UDP.
    Receives the R2100 brake decision over UDP.

The R2100 gateway converts these network packets to/from the physical CAN loop.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

DEFAULT_LIDAR_TOPIC = "/aeb/lidar"
DEFAULT_ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
DEFAULT_CMD_VEL_TOPIC = "/model/aeb_vehicle/cmd_vel"

DEFAULT_TARGET_SPEED_MPS = 2.0
FRONT_HALF_ANGLE_DEG = 10.0

SENSOR_TIMEOUT_SECONDS = 0.50
BRAKE_TIMEOUT_SECONDS = 0.50
LOOP_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.25

SENSOR_PACKET = struct.Struct("!4sIff")
BRAKE_PACKET = struct.Struct("!4sIBBBx")

SENSOR_MAGIC = b"HILS"
BRAKE_MAGIC = b"HILB"

BRAKE_REASON_TEXT = {
    0: "none",
    1: "emergency_distance",
    2: "ttc_threshold",
    3: "sensor_timeout",
}


class GazeboHILBridge:
    def __init__(
        self,
        r2100_host: str,
        udp_port: int,
        lidar_topic: str,
        odometry_topic: str,
        cmd_vel_topic: str,
        target_speed_mps: float,
        goal_x_m: float | None = None,
        goal_tolerance_m: float = 0.40,
    ) -> None:
        self.r2100_host = r2100_host
        self.udp_port = udp_port
        self.lidar_topic = lidar_topic
        self.odometry_topic = odometry_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.target_speed_mps = target_speed_mps
        self.goal_x_m = goal_x_m
        self.goal_tolerance_m = goal_tolerance_m

        self.node = Node()
        self.lock = threading.Lock()

        self.speed_mps: float | None = None
        self.position_x_m: float | None = None
        self.front_distance_m: float | None = None
        self.last_lidar_time: float | None = None
        self.last_odometry_time: float | None = None

        self.brake_active = True
        self.brake_reason = 3
        self.brake_counter = 0
        self.last_brake_time: float | None = None

        self.sequence = 0
        self.last_print_time = 0.0

        self.cmd_publisher = self.node.advertise(
            self.cmd_vel_topic,
            Twist,
        )

        lidar_ok = self.node.subscribe(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
        )
        odometry_ok = self.node.subscribe(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
        )

        if not lidar_ok:
            raise RuntimeError(
                f"failed to subscribe to LiDAR topic: {self.lidar_topic}"
            )
        if not odometry_ok:
            raise RuntimeError(
                f"failed to subscribe to odometry topic: {self.odometry_topic}"
            )

        self.udp_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.udp_sock.connect((self.r2100_host, self.udp_port))
        self.udp_sock.setblocking(False)

    def lidar_callback(self, msg: LaserScan) -> None:
        front_half_angle_rad = math.radians(FRONT_HALF_ANGLE_DEG)
        valid_ranges: list[float] = []

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_step

            if abs(angle) > front_half_angle_rad:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min:
                continue
            if distance > msg.range_max:
                continue

            valid_ranges.append(distance)

        front_distance = min(valid_ranges) if valid_ranges else None
        now = time.monotonic()

        with self.lock:
            self.front_distance_m = front_distance
            self.last_lidar_time = now

    def odometry_callback(self, msg: Odometry) -> None:
        speed = abs(msg.twist.linear.x)
        position_x = msg.pose.position.x
        now = time.monotonic()

        with self.lock:
            self.speed_mps = speed
            self.position_x_m = position_x
            self.last_odometry_time = now

    def sensors_fresh(self, now: float) -> bool:
        with self.lock:
            last_lidar = self.last_lidar_time
            last_odometry = self.last_odometry_time

        if last_lidar is None or last_odometry is None:
            return False

        return (
            now - last_lidar <= SENSOR_TIMEOUT_SECONDS
            and now - last_odometry <= SENSOR_TIMEOUT_SECONDS
        )

    def send_sensor_packet(self) -> None:
        with self.lock:
            speed = self.speed_mps
            distance = self.front_distance_m

        if speed is None:
            return

        distance_wire = math.nan if distance is None else distance

        packet = SENSOR_PACKET.pack(
            SENSOR_MAGIC,
            self.sequence,
            float(speed),
            float(distance_wire),
        )

        self.udp_sock.send(packet)
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

    def receive_brake_packets(self) -> None:
        while True:
            try:
                packet = self.udp_sock.recv(1024)
            except BlockingIOError:
                return

            if len(packet) != BRAKE_PACKET.size:
                continue

            magic, _sensor_seq, active, reason, counter = BRAKE_PACKET.unpack(
                packet
            )
            if magic != BRAKE_MAGIC:
                continue
            if active not in (0, 1):
                continue

            self.brake_active = bool(active)
            self.brake_reason = reason
            self.brake_counter = counter
            self.last_brake_time = time.monotonic()

    def brake_fresh(self, now: float) -> bool:
        return (
            self.last_brake_time is not None
            and now - self.last_brake_time <= BRAKE_TIMEOUT_SECONDS
        )

    def publish_speed(self, speed_mps: float) -> None:
        command = Twist()
        command.linear.x = speed_mps
        command.angular.z = 0.0
        self.cmd_publisher.publish(command)

    def print_status(
        self,
        now: float,
        command_speed: float,
        effective_brake: bool,
        state_reason: str,
    ) -> None:
        if now - self.last_print_time < PRINT_INTERVAL_SECONDS:
            return

        with self.lock:
            speed = self.speed_mps
            distance = self.front_distance_m

        speed_text = "missing" if speed is None else f"{speed:.2f} m/s"
        distance_text = (
            "no obstacle"
            if distance is None
            else f"{distance:.3f} m"
        )

        print(
            f"{'BRAKE' if effective_brake else 'DRIVE':5s} | "
            f"speed={speed_text:>10s} | "
            f"distance={distance_text:>12s} | "
            f"cmd={command_speed:.2f} m/s | "
            f"reason={state_reason} | "
            f"counter={self.brake_counter:3d}"
        )

        self.last_print_time = now

    def run(self) -> None:
        print()
        print("========================================")
        print(" GAZEBO <-> R2100 HIL BRIDGE")
        print("========================================")
        print(f"R2100 gateway : {self.r2100_host}:{self.udp_port}")
        print(f"LiDAR topic   : {self.lidar_topic}")
        print(f"Odometry      : {self.odometry_topic}")
        print(f"Command       : {self.cmd_vel_topic}")
        print(f"Drive speed   : {self.target_speed_mps:.2f} m/s")
        if self.goal_x_m is None:
            print("Goal stop     : disabled")
        else:
            print(
                f"Goal stop     : x >= "
                f"{self.goal_x_m - self.goal_tolerance_m:.2f} m "
                f"(goal={self.goal_x_m:.2f}, tol={self.goal_tolerance_m:.2f})"
            )
        print()
        print("Fail-safe: stale sensors or stale R2100 brake response -> STOP")
        print("Ctrl+C stops.")
        print()

        try:
            while True:
                loop_start = time.monotonic()

                if self.sensors_fresh(loop_start):
                    try:
                        self.send_sensor_packet()
                    except OSError as exc:
                        print(f"UDP send failed: {exc}")

                try:
                    self.receive_brake_packets()
                except OSError as exc:
                    print(f"UDP receive failed: {exc}")

                now = time.monotonic()
                sensors_ok = self.sensors_fresh(now)
                brake_ok = self.brake_fresh(now)

                if not sensors_ok:
                    effective_brake = True
                    state_reason = "gazebo_sensor_timeout"
                elif not brake_ok:
                    effective_brake = True
                    state_reason = "r2100_response_timeout"
                else:
                    effective_brake = self.brake_active
                    state_reason = BRAKE_REASON_TEXT.get(
                        self.brake_reason,
                        f"unknown_{self.brake_reason}",
                    )

                with self.lock:
                    position_x = self.position_x_m

                goal_reached = (
                    self.goal_x_m is not None
                    and position_x is not None
                    and position_x
                    >= self.goal_x_m - self.goal_tolerance_m
                )

                if effective_brake:
                    command_speed = 0.0
                elif goal_reached:
                    command_speed = 0.0
                    state_reason = "goal_reached"
                else:
                    command_speed = self.target_speed_mps

                self.publish_speed(command_speed)

                self.print_status(
                    now,
                    command_speed,
                    effective_brake,
                    state_reason,
                )

                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, LOOP_PERIOD_SECONDS - elapsed))

        except KeyboardInterrupt:
            print()
            print("Bridge stopped by user.")

        finally:
            print("Sending final STOP commands...")
            for _ in range(5):
                self.publish_speed(0.0)
                time.sleep(0.02)

            self.udp_sock.close()
            print("Done.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--r2100-host",
        required=True,
        help="R2100 IP address or hostname reachable from the Gazebo PC",
    )
    parser.add_argument("--udp-port", type=int, default=15000)
    parser.add_argument(
        "--lidar-topic",
        default=DEFAULT_LIDAR_TOPIC,
    )
    parser.add_argument(
        "--odometry-topic",
        default=DEFAULT_ODOMETRY_TOPIC,
    )
    parser.add_argument(
        "--cmd-topic",
        default=DEFAULT_CMD_VEL_TOPIC,
    )
    parser.add_argument(
        "--target-speed",
        type=float,
        default=DEFAULT_TARGET_SPEED_MPS,
    )
    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        help="stop when odometry x reaches this goal (disabled by default)",
    )
    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.40,
        help="goal x tolerance in metres (default: 0.40)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = GazeboHILBridge(
        r2100_host=args.r2100_host,
        udp_port=args.udp_port,
        lidar_topic=args.lidar_topic,
        odometry_topic=args.odometry_topic,
        cmd_vel_topic=args.cmd_topic,
        target_speed_mps=args.target_speed,
        goal_x_m=args.goal_x,
        goal_tolerance_m=args.goal_tolerance,
    )
    bridge.run()


if __name__ == "__main__":
    main()
