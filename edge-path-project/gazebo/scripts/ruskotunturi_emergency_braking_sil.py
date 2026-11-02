#!/usr/bin/env python3

import json
import math
from pathlib import Path
import threading
import time

from gz.msgs10.laserscan_pb2 import LaserScan
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
    route_geometry,
    project_to_route,
    wrap_angle,
    quaternion_to_rpy,
)
from ruskotunturi_hil9e_aeb_test import physical_clearance


POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
ODOM_TOPIC = f"/model/{MODEL_B}/odometry"
LIDAR_TOPIC = "/aeb360b/lidar"

CONTROL_PERIOD_S = 0.05
SENSOR_TIMEOUT_S = 0.50
MISSION_TIMEOUT_S = 45.0

TTC_THRESHOLD_S = 1.7
MIN_SPEED_FOR_TTC_MPS = 0.10
EMERGENCY_DISTANCE_M = 0.80
FRONT_HALF_ANGLE_DEG = 10.0

VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.60

SCENARIO_PATH = (
    Path(__file__).resolve().parents[1]
    / "scenarios"
    / "emergency_braking.json"
)


def body_radius_for_ray(angle_rad):
    cos_a = abs(math.cos(angle_rad))
    sin_a = abs(math.sin(angle_rad))

    candidates = []

    if cos_a > 1e-9:
        candidates.append(VEHICLE_HALF_LENGTH_M / cos_a)

    if sin_a > 1e-9:
        candidates.append(VEHICLE_HALF_WIDTH_M / sin_a)

    return min(candidates) if candidates else 0.0


class EmergencyBrakingSIL:
    def __init__(self):
        scenario = json.loads(SCENARIO_PATH.read_text())
        acceptance = scenario["acceptance"]

        self.required_clearance_m = float(
            acceptance["required_clearance_m"]
        )
        self.stop_speed_mps = float(
            acceptance["stop_speed_mps"]
        )
        self.stop_confirm_samples = int(
            acceptance["stop_confirm_samples"]
        )

        self.node = Node()
        self.lock = threading.Lock()

        self.pub = self.node.advertise(
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
            ODOM_TOPIC,
            self.odom_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {ODOM_TOPIC}"
            )

        if not self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {LIDAR_TOPIC}"
            )

        self.route = route_geometry(
            B_START,
            B_GOAL,
        )

        self.x = None
        self.y = None
        self.yaw = 0.0
        self.speed = None
        self.front_clearance = None

        self.last_pose = None
        self.last_odom = None
        self.last_lidar = None

        self.aeb_latched = False
        self.aeb_reason = None
        self.trigger_route_s = None
        self.stop_samples = 0
        self.vehicle_moved = False

    def pose_callback(self, msg):
        target = None

        for pose in msg.pose:
            if (
                pose.name == MODEL_B
                or pose.name.endswith("::" + MODEL_B)
            ):
                target = pose
                break

        if target is None:
            return

        _roll, _pitch, yaw = quaternion_to_rpy(
            target.orientation
        )

        with self.lock:
            self.x = target.position.x
            self.y = target.position.y
            self.yaw = yaw
            self.last_pose = time.monotonic()

    def odom_callback(self, msg):
        speed = math.sqrt(
            msg.twist.linear.x ** 2
            + msg.twist.linear.y ** 2
            + msg.twist.linear.z ** 2
        )

        with self.lock:
            self.speed = speed
            self.last_odom = time.monotonic()

    def lidar_callback(self, msg):
        half_angle = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        clearances = []

        for index, distance in enumerate(msg.ranges):
            angle = (
                msg.angle_min
                + index * msg.angle_step
            )

            if abs(angle) > half_angle:
                continue

            if not math.isfinite(distance):
                continue

            if (
                distance < msg.range_min
                or distance > msg.range_max
            ):
                continue

            clearance = max(
                0.0,
                distance - body_radius_for_ray(angle),
            )

            clearances.append(clearance)

        with self.lock:
            self.front_clearance = (
                min(clearances)
                if clearances
                else None
            )
            self.last_lidar = time.monotonic()

    def sensors_fresh(self):
        now = time.monotonic()

        with self.lock:
            stamps = (
                self.last_pose,
                self.last_odom,
                self.last_lidar,
            )

        return all(
            stamp is not None
            and now - stamp <= SENSOR_TIMEOUT_S
            for stamp in stamps
        )

    def publish(self, speed, steering):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steering
        self.pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def route_command(self):
        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw

        if x is None or y is None:
            return 0.0, 0.0, 0.0

        route_s, _lateral = project_to_route(
            x,
            y,
            B_START,
            self.route,
        )

        target_s = route_s + FOLLOW_LOOKAHEAD_M

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

        error = wrap_angle(
            desired_heading - yaw
        )

        steering = max(
            -MAX_STEERING,
            min(
                MAX_STEERING,
                HEADING_GAIN * error,
            ),
        )

        return (
            B_CRUISE_SPEED_MPS,
            steering,
            route_s,
        )

    def evaluate_aeb(self, speed, clearance):
        if clearance is None:
            return False, None, math.inf

        ttc = math.inf

        if speed >= MIN_SPEED_FOR_TTC_MPS:
            ttc = clearance / speed

        if clearance <= EMERGENCY_DISTANCE_M:
            return True, "emergency_distance", ttc

        if (
            speed >= MIN_SPEED_FOR_TTC_MPS
            and ttc <= TTC_THRESHOLD_S
        ):
            return True, "ttc_threshold", ttc

        return False, None, ttc

    def wait_ready(self):
        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            self.stop()

            if self.sensors_fresh():
                return True

            time.sleep(0.05)

        return False

    def run(self):
        print("SIL emergency-braking scenario")
        print()
        print(f"Vehicle     : {MODEL_B}")
        print(f"LiDAR       : {LIDAR_TOPIC}")
        print(
            f"TTC trigger : {TTC_THRESHOLD_S:.2f} s"
        )
        print(
            f"Emergency   : {EMERGENCY_DISTANCE_M:.2f} m"
        )
        print(
            f"Required clearance: "
            f"{self.required_clearance_m:.2f} m"
        )
        print()

        if not self.wait_ready():
            print("STAGED SCENARIO RESULT: FAIL")
            print("End reason: sensors_not_ready")
            return False

        start = time.monotonic()
        last_print = 0.0

        try:
            while (
                time.monotonic() - start
                < MISSION_TIMEOUT_S
            ):
                if not self.sensors_fresh():
                    self.stop()
                    print(
                        "STAGED SCENARIO RESULT: FAIL"
                    )
                    print(
                        "End reason: sensor_timeout"
                    )
                    return False

                with self.lock:
                    speed = self.speed
                    clearance = self.front_clearance

                requested, steering, route_s = (
                    self.route_command()
                )

                if (
                    speed is not None
                    and speed >= 0.20
                ):
                    self.vehicle_moved = True

                brake, reason, ttc = self.evaluate_aeb(
                    speed or 0.0,
                    clearance,
                )

                if (
                    brake
                    and not self.aeb_latched
                ):
                    self.aeb_latched = True
                    self.aeb_reason = reason
                    self.trigger_route_s = route_s

                    print()
                    print("*** SIL AEB TRIGGERED ***")
                    print(f"Reason    : {reason}")
                    print(
                        "Clearance : "
                        f"{clearance:.3f} m"
                    )
                    print(
                        f"Speed     : {speed:.3f} m/s"
                    )
                    if math.isfinite(ttc):
                        print(f"TTC       : {ttc:.3f} s")
                    print()

                if self.aeb_latched:
                    self.stop()
                else:
                    self.publish(
                        requested,
                        steering,
                    )

                if self.aeb_latched:
                    if (
                        speed is not None
                        and speed
                        <= self.stop_speed_mps
                    ):
                        self.stop_samples += 1
                    else:
                        self.stop_samples = 0

                    if (
                        self.stop_samples
                        >= self.stop_confirm_samples
                    ):
                        clearance_gt = physical_clearance(
                            route_s
                        )

                        passed = (
                            self.vehicle_moved
                            and self.aeb_reason
                            in {
                                "emergency_distance",
                                "ttc_threshold",
                            }
                            and clearance_gt
                            >= self.required_clearance_m
                        )

                        print()
                        print(
                            "STAGED SCENARIO RESULT: "
                            + ("PASS" if passed else "FAIL")
                        )
                        print(
                            "End reason: "
                            + (
                                "aeb_stop_verified"
                                if passed
                                else "clearance_violation"
                            )
                        )
                        print(
                            f"AEB reason: "
                            f"{self.aeb_reason}"
                        )
                        print(
                            "Ground-truth clearance: "
                            f"{clearance_gt:.3f} m"
                        )

                        return passed

                now = time.monotonic()

                if now - last_print >= 0.5:
                    clearance_text = (
                        "none"
                        if clearance is None
                        else f"{clearance:.3f}"
                    )

                    print(
                        f"B s={route_s:.3f} m"
                        f" | speed={(speed or 0.0):.3f}"
                        f" | lidar_clearance={clearance_text}"
                        f" | brake={self.aeb_latched}"
                    )

                    last_print = now

                time.sleep(CONTROL_PERIOD_S)

            self.stop()
            print("STAGED SCENARIO RESULT: FAIL")
            print("End reason: mission_timeout")
            return False

        except KeyboardInterrupt:
            print()
            print("Interrupted.")
            return False

        finally:
            for _ in range(5):
                self.stop()
                time.sleep(0.02)


def main():
    test = EmergencyBrakingSIL()
    raise SystemExit(
        0 if test.run() else 1
    )


if __name__ == "__main__":
    main()
