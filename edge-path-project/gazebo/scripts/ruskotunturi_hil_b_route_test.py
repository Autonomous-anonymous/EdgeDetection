#!/usr/bin/env python3

import argparse
import math
import socket
import struct
import threading
import time

from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from ruskotunturi_phase6_two_vehicle_test import (
    WORLD_NAME,
    MODEL_B,
    CMD_B_TOPIC,
    B_START,
    B_GOAL,
    B_CRUISE_SPEED_MPS,
    FOLLOW_LOOKAHEAD_M,
    HEADING_GAIN,
    MAX_STEERING,
    GOAL_RADIUS_M,
    GOAL_LINE_MARGIN_M,
    GOAL_LATERAL_TOLERANCE_M,
    route_geometry,
    project_to_route,
    wrap_angle,
    distance_2d,
    quaternion_to_rpy,
)


POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
ODOM_B_TOPIC = f"/model/{MODEL_B}/odometry"

CONTROL_PERIOD_S = 0.05
PRINT_INTERVAL_S = 0.75
SENSOR_TIMEOUT_S = 2.0
BRAKE_TIMEOUT_S = 0.50
MISSION_TIMEOUT_S = 90.0

DEFAULT_UDP_PORT = 15000

SENSOR_PACKET = struct.Struct("!4sIff")
BRAKE_PACKET = struct.Struct("!4sIBBBx")

SENSOR_MAGIC = b"HILS"
BRAKE_MAGIC = b"HILB"

BRAKE_REASON = {
    0: "none",
    1: "emergency_distance",
    2: "ttc_threshold",
    3: "sensor_timeout",
}


class RuskotunturiHILBRouteTest:
    def __init__(self, r2100_host, udp_port):
        self.node = Node()
        self.lock = threading.Lock()

        # IMPORTANT:
        # Only this process publishes Vehicle B commands.
        # Vehicle A is never commanded here.
        self.pub_b = self.node.advertise(
            CMD_B_TOPIC,
            Twist,
        )

        if not self.node.subscribe(
            Pose_V,
            POSE_TOPIC,
            self.pose_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {POSE_TOPIC}"
            )

        if not self.node.subscribe(
            Odometry,
            ODOM_B_TOPIC,
            self.odom_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {ODOM_B_TOPIC}"
            )

        self.route = route_geometry(
            B_START,
            B_GOAL,
        )

        self.x = None
        self.y = None
        self.yaw = 0.0
        self.speed = None

        self.last_pose_time = None
        self.last_odom_time = None

        self.brake_active = True
        self.brake_reason = 3
        self.last_brake_time = None
        self.brake_counter = 0

        self.sequence = 0
        self.last_print_time = 0.0

        self.aeb_events = 0
        self.previous_effective_brake = True

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.sock.connect(
            (r2100_host, udp_port)
        )
        self.sock.setblocking(False)

    def pose_callback(self, msg):
        target_pose = None

        for pose in msg.pose:
            if (
                pose.name == MODEL_B
                or pose.name.endswith("::" + MODEL_B)
            ):
                target_pose = pose
                break

        if target_pose is None:
            return

        _roll, _pitch, yaw = quaternion_to_rpy(
            target_pose.orientation
        )

        now = time.monotonic()

        with self.lock:
            self.x = target_pose.position.x
            self.y = target_pose.position.y
            self.yaw = yaw
            self.last_pose_time = now

    def odom_callback(self, msg):
        now = time.monotonic()

        with self.lock:
            self.speed = abs(
                msg.twist.linear.x
            )
            self.last_odom_time = now

    def sensors_fresh(self, now):
        with self.lock:
            pose_time = self.last_pose_time
            odom_time = self.last_odom_time

        if pose_time is None or odom_time is None:
            return False

        return (
            now - pose_time <= SENSOR_TIMEOUT_S
            and now - odom_time <= SENSOR_TIMEOUT_S
        )

    def send_sensor_packet(self):
        with self.lock:
            speed = self.speed

        if speed is None:
            return

        # Distance is intentionally NaN.
        # R2100 gateway runs with --external-perception,
        # so CAN 0x101 is owned by r2100_lidar_perception.py.
        packet = SENSOR_PACKET.pack(
            SENSOR_MAGIC,
            self.sequence,
            float(speed),
            math.nan,
        )

        self.sock.send(packet)

        self.sequence = (
            self.sequence + 1
        ) & 0xFFFFFFFF

    def receive_brake_packets(self):
        while True:
            try:
                packet = self.sock.recv(1024)
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

    def brake_fresh(self, now):
        return (
            self.last_brake_time is not None
            and now - self.last_brake_time
            <= BRAKE_TIMEOUT_S
        )

    def route_command(self):
        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw

        if x is None or y is None:
            return 0.0, 0.0, False, 0.0, 0.0

        s, lateral = project_to_route(
            x,
            y,
            B_START,
            self.route,
        )

        goal_distance = distance_2d(
            x,
            y,
            B_GOAL[0],
            B_GOAL[1],
        )

        crossed_goal = (
            s
            >= (
                self.route["length"]
                - GOAL_LINE_MARGIN_M
            )
            and abs(lateral)
            <= GOAL_LATERAL_TOLERANCE_M
        )

        goal = (
            goal_distance <= GOAL_RADIUS_M
            or crossed_goal
        )

        if goal:
            return (
                0.0,
                0.0,
                True,
                s,
                lateral,
            )

        target_s = s + FOLLOW_LOOKAHEAD_M

        target_x = (
            B_START[0]
            + target_s * self.route["ux"]
        )

        target_y = (
            B_START[1]
            + target_s * self.route["uy"]
        )

        desired_heading = math.atan2(
            target_y - y,
            target_x - x,
        )

        heading_error = wrap_angle(
            desired_heading - yaw
        )

        steering = max(
            -MAX_STEERING,
            min(
                MAX_STEERING,
                HEADING_GAIN * heading_error,
            ),
        )

        return (
            B_CRUISE_SPEED_MPS,
            steering,
            False,
            s,
            lateral,
        )

    def publish_b(self, speed, steering):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steering

        self.pub_b.publish(msg)

    def stop_b(self):
        self.publish_b(
            0.0,
            0.0,
        )

    def run(self):
        print()
        print("============================================================")
        print(" RUSKOTUNTURI HIL-9C - VEHICLE B ROUTE TEST")
        print("============================================================")
        print(f"Vehicle B route : {B_START} -> {B_GOAL}")
        print(f"Cruise speed    : {B_CRUISE_SPEED_MPS:.2f} m/s")
        print("Steering        : SIL Phase 6 route controller")
        print("Longitudinal    : R2100 AEB over physical CAN")
        print("Vehicle A       : NOT commanded by this process")
        print()

        print("Waiting for Vehicle B pose / odometry...")

        ready_start = time.monotonic()

        while not self.sensors_fresh(
            time.monotonic()
        ):
            if (
                time.monotonic()
                - ready_start
                > 10.0
            ):
                print("RESULT: FAIL - B sensors not ready")
                self.stop_b()
                return False

            time.sleep(0.05)

        print("Vehicle B sensors ready.")
        print("Waiting for fresh R2100 brake state...")

        start = time.monotonic()

        try:
            while True:
                now = time.monotonic()

                if now - start > MISSION_TIMEOUT_S:
                    self.stop_b()
                    print()
                    print("RESULT: FAIL - mission timeout")
                    return False

                self.receive_brake_packets()
                self.send_sensor_packet()

                if not self.sensors_fresh(now):
                    self.stop_b()
                    print()
                    print("RESULT: FAIL - Gazebo sensor timeout")
                    return False

                (
                    requested_speed,
                    steering,
                    goal,
                    route_s,
                    lateral,
                ) = self.route_command()

                if goal:
                    self.stop_b()

                    with self.lock:
                        x = self.x
                        y = self.y
                        speed = self.speed

                    print()
                    print("*** VEHICLE B REACHED GOAL ***")
                    print(
                        f"B final = ({x:.2f}, {y:.2f})"
                    )
                    print(
                        f"Final speed = {speed:.3f} m/s"
                    )
                    print(
                        f"R2100 AEB events = {self.aeb_events}"
                    )
                    print()
                    print("RESULT: PASS")
                    return True

                brake_fresh = self.brake_fresh(now)

                effective_brake = (
                    not brake_fresh
                    or self.brake_active
                )

                if (
                    effective_brake
                    and not self.previous_effective_brake
                ):
                    self.aeb_events += 1

                self.previous_effective_brake = (
                    effective_brake
                )

                if effective_brake:
                    command_speed = 0.0
                    command_steering = 0.0
                else:
                    command_speed = requested_speed
                    command_steering = steering

                self.publish_b(
                    command_speed,
                    command_steering,
                )

                if (
                    now - self.last_print_time
                    >= PRINT_INTERVAL_S
                ):
                    with self.lock:
                        x = self.x
                        y = self.y
                        speed = self.speed

                    reason = (
                        "brake_timeout"
                        if not brake_fresh
                        else BRAKE_REASON.get(
                            self.brake_reason,
                            f"unknown_{self.brake_reason}",
                        )
                    )

                    state = (
                        "BRAKE"
                        if effective_brake
                        else "DRIVE"
                    )

                    print(
                        f"B x={x:6.2f} y={y:6.2f} | "
                        f"s={route_s:5.2f}/{self.route['length']:.2f} | "
                        f"d={lateral:+5.2f} | "
                        f"v={speed:4.2f} | "
                        f"cmd={command_speed:4.2f} | "
                        f"steer={command_steering:+5.3f} | "
                        f"{state} | "
                        f"reason={reason}"
                    )

                    self.last_print_time = now

                time.sleep(CONTROL_PERIOD_S)

        except KeyboardInterrupt:
            self.stop_b()
            print("\nStopped by user.")
            return False

        finally:
            self.stop_b()
            self.sock.close()


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

    test = RuskotunturiHILBRouteTest(
        r2100_host=args.r2100_host,
        udp_port=args.udp_port,
    )

    raise SystemExit(
        0 if test.run() else 1
    )


if __name__ == "__main__":
    main()
