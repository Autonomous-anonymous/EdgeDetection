#!/usr/bin/env python3

import math
import threading
import time

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


# ============================================================
# TOPICS
# ============================================================

A_ODOM_TOPIC = "/model/aeb_vehicle/odometry"
A_LIDAR_TOPIC = "/aeb/lidar"
A_CMD_TOPIC = "/model/aeb_vehicle/cmd_vel"

B_ODOM_TOPIC = "/model/aeb_vehicle_b/odometry"

# IMPORTANT:
# There is intentionally NO Vehicle B command topic here.
# Vehicle B is controlled exclusively by the R2100 HIL bridge.


# ============================================================
# WORLD / COORDINATION
# ============================================================

A_WORLD_OFFSET_Y = 2.0
B_WORLD_OFFSET_Y = -2.0

A_RELEASE_WHEN_B_X_M = 8.0

A_TARGET_SPEED_MPS = 1.8

GOAL_X_M = 28.5
GOAL_X_TOLERANCE_M = 0.40
GOAL_STOP_X_M = GOAL_X_M - GOAL_X_TOLERANCE_M
STOPPED_SPEED_MPS = 0.10

MIN_ALLOWED_CENTER_DISTANCE_M = 3.0


# ============================================================
# A SIL AEB SAFETY
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7
MIN_CLOSING_SPEED_MPS = 0.10
EMERGENCY_DISTANCE_M = 0.80
FRONT_HALF_ANGLE_DEG = 8.0
CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# TIMING
# ============================================================

CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.50
SENSOR_TIMEOUT_SECONDS = 0.75

HIL_MOTION_WAIT_SECONDS = 60.0
MISSION_TIMEOUT_SECONDS = 70.0


class HybridHILMissionTest:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()

        self.a_x = None
        self.a_y = None
        self.a_speed = 0.0

        self.b_x = None
        self.b_y = None
        self.b_speed = 0.0

        self.last_a_odom_time = None
        self.last_b_odom_time = None

        self.a_front_distance = None
        self.a_closing_speed = None
        self.a_previous_distance = None
        self.a_previous_lidar_time = None
        self.last_a_lidar_time = None

        self.a_aeb_active = False
        self.a_aeb_interrupts = 0

        self.a_released = False
        self.a_wait_events = 0

        self.a_goal = False
        self.b_goal = False

        self.minimum_center_distance = None
        self.estimated_collision = False

        # Only Vehicle A is controlled by this process.
        self.publisher_a = self.node.advertise(
            A_CMD_TOPIC,
            Twist,
        )

        if not self.node.subscribe(
            Odometry,
            A_ODOM_TOPIC,
            self.odom_a_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {A_ODOM_TOPIC}"
            )

        if not self.node.subscribe(
            Odometry,
            B_ODOM_TOPIC,
            self.odom_b_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {B_ODOM_TOPIC}"
            )

        if not self.node.subscribe(
            LaserScan,
            A_LIDAR_TOPIC,
            self.lidar_a_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {A_LIDAR_TOPIC}"
            )

    def odom_a_callback(self, msg):
        now = time.monotonic()

        with self.lock:
            self.a_x = msg.pose.position.x
            self.a_y = (
                msg.pose.position.y
                + A_WORLD_OFFSET_Y
            )
            self.a_speed = abs(msg.twist.linear.x)
            self.last_a_odom_time = now

    def odom_b_callback(self, msg):
        now = time.monotonic()

        with self.lock:
            self.b_x = msg.pose.position.x
            self.b_y = (
                msg.pose.position.y
                + B_WORLD_OFFSET_Y
            )
            self.b_speed = abs(msg.twist.linear.x)
            self.last_b_odom_time = now

    def lidar_a_callback(self, msg):
        half_angle = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        valid = []

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

            valid.append(distance)

        distance = min(valid) if valid else None

        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec * 1e-9
        )

        wall_time = time.monotonic()

        with self.lock:
            if distance is not None:
                if (
                    self.a_previous_distance is not None
                    and self.a_previous_lidar_time is not None
                ):
                    dt = (
                        sim_time
                        - self.a_previous_lidar_time
                    )

                    if 0.01 < dt < 1.0:
                        raw_closing = (
                            self.a_previous_distance
                            - distance
                        ) / dt

                        raw_closing = max(
                            -20.0,
                            min(20.0, raw_closing),
                        )

                        if self.a_closing_speed is None:
                            filtered = raw_closing
                        else:
                            alpha = (
                                CLOSING_SPEED_FILTER_ALPHA
                            )
                            filtered = (
                                alpha * raw_closing
                                + (1.0 - alpha)
                                * self.a_closing_speed
                            )

                        self.a_closing_speed = filtered

                self.a_previous_distance = distance
                self.a_previous_lidar_time = sim_time

            self.a_front_distance = distance
            self.last_a_lidar_time = wall_time

    def publish_a_speed(self, speed):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.publisher_a.publish(msg)

    def stop_a(self):
        for _ in range(5):
            self.publish_a_speed(0.0)
            time.sleep(0.02)

    def sensors_fresh(self, now):
        with self.lock:
            times = (
                self.last_a_odom_time,
                self.last_b_odom_time,
                self.last_a_lidar_time,
            )

        if any(value is None for value in times):
            return False

        return all(
            now - value <= SENSOR_TIMEOUT_SECONDS
            for value in times
        )

    def update_a_aeb(self):
        with self.lock:
            distance = self.a_front_distance
            closing = self.a_closing_speed
            currently_active = self.a_aeb_active

        if (
            distance is not None
            and closing is not None
            and closing > MIN_CLOSING_SPEED_MPS
        ):
            ttc = distance / closing
        else:
            ttc = math.inf

        trigger = (
            (
                distance is not None
                and distance <= EMERGENCY_DISTANCE_M
            )
            or (
                math.isfinite(ttc)
                and ttc <= TTC_THRESHOLD_SECONDS
            )
        )

        if trigger and not currently_active:
            with self.lock:
                self.a_aeb_active = True
                self.a_aeb_interrupts += 1

            print()
            print("*** VEHICLE A SIL AEB TRIGGERED ***")
            print(
                "Distance:",
                "none"
                if distance is None
                else f"{distance:.3f} m",
            )
            print(
                "TTC:",
                "inf"
                if not math.isfinite(ttc)
                else f"{ttc:.3f} s",
            )
            print()

        if currently_active:
            safe_to_release = (
                distance is None
                or (
                    distance > 3.0
                    and (
                        closing is None
                        or closing
                        <= MIN_CLOSING_SPEED_MPS
                        or ttc > 2.5
                    )
                )
            )

            if safe_to_release:
                with self.lock:
                    self.a_aeb_active = False

        with self.lock:
            return self.a_aeb_active

    def run(self):
        print()
        print("============================================================")
        print(" TWO-VEHICLE HYBRID HIL MISSION TEST")
        print("============================================================")
        print("Vehicle A : SIL coordination + SIL safety")
        print("Vehicle B : R2100 HIL perception + AEB + drive")
        print()
        print(
            f"A waits until B x >= "
            f"{A_RELEASE_WHEN_B_X_M:.1f} m."
        )
        print(
            f"Both goals: x >= "
            f"{GOAL_STOP_X_M:.1f} m and speed <= "
            f"{STOPPED_SPEED_MPS:.1f} m/s."
        )
        print()
        print("This process NEVER publishes Vehicle B cmd_vel.")
        print()

        wait_start = time.monotonic()
        mission_start = None
        last_print = 0.0

        try:
            while True:
                now = time.monotonic()

                if not self.sensors_fresh(now):
                    self.publish_a_speed(0.0)

                    if (
                        mission_start is None
                        and now - wait_start
                        > HIL_MOTION_WAIT_SECONDS
                    ):
                        print()
                        print("RESULT: FAIL")
                        print(
                            "Reason: required sensor data "
                            "not available"
                        )
                        return False

                    time.sleep(CONTROL_PERIOD_SECONDS)
                    continue

                with self.lock:
                    a_x = self.a_x
                    a_y = self.a_y
                    a_speed = self.a_speed

                    b_x = self.b_x
                    b_y = self.b_y
                    b_speed = self.b_speed

                # Wait until externally controlled B has
                # actually started moving.
                if mission_start is None:
                    self.publish_a_speed(0.0)

                    if (
                        b_x is not None
                        and b_x > 0.25
                        and b_speed > 0.10
                    ):
                        mission_start = now

                        print()
                        print(
                            "*** VEHICLE B HIL MOTION "
                            "DETECTED ***"
                        )
                        print(
                            f"B x={b_x:.3f} m, "
                            f"speed={b_speed:.2f} m/s"
                        )
                        print(
                            "Starting full hybrid mission."
                        )
                        print()

                    else:
                        if (
                            now - wait_start
                            > HIL_MOTION_WAIT_SECONDS
                        ):
                            print()
                            print("RESULT: FAIL")
                            print(
                                "Reason: Vehicle B HIL "
                                "motion not detected"
                            )
                            return False

                        time.sleep(
                            CONTROL_PERIOD_SECONDS
                        )
                        continue

                if (
                    now - mission_start
                    > MISSION_TIMEOUT_SECONDS
                ):
                    print()
                    print("RESULT: FAIL")
                    print(
                        "Reason: hybrid mission timeout"
                    )
                    return False

                # --------------------------------------------
                # Vehicle separation
                # --------------------------------------------

                center_distance = math.hypot(
                    a_x - b_x,
                    a_y - b_y,
                )

                if (
                    self.minimum_center_distance is None
                    or center_distance
                    < self.minimum_center_distance
                ):
                    self.minimum_center_distance = (
                        center_distance
                    )

                if (
                    center_distance
                    < MIN_ALLOWED_CENTER_DISTANCE_M
                ):
                    self.estimated_collision = True

                    print()
                    print(
                        "*** UNSAFE VEHICLE PROXIMITY ***"
                    )
                    print(
                        f"Center distance = "
                        f"{center_distance:.3f} m"
                    )
                    print("RESULT: FAIL")
                    return False

                # --------------------------------------------
                # Release A
                # --------------------------------------------

                if (
                    not self.a_released
                    and b_x
                    >= A_RELEASE_WHEN_B_X_M
                ):
                    self.a_released = True

                    print()
                    print(
                        "*** HYBRID COORDINATION: "
                        "VEHICLE A RELEASED ***"
                    )
                    print(
                        f"Vehicle B x = {b_x:.3f} m"
                    )
                    print()

                # --------------------------------------------
                # Vehicle B goal observation
                # --------------------------------------------

                if (
                    not self.b_goal
                    and b_x >= GOAL_STOP_X_M
                    and b_speed <= STOPPED_SPEED_MPS
                ):
                    self.b_goal = True

                    print()
                    print(
                        "*** VEHICLE B HIL "
                        "REACHED LOADING ZONE ***"
                    )
                    print(
                        f"B x={b_x:.3f} m, "
                        f"speed={b_speed:.3f} m/s"
                    )
                    print()

                # --------------------------------------------
                # Vehicle A control
                # --------------------------------------------

                if (
                    a_x >= GOAL_STOP_X_M
                ):
                    self.publish_a_speed(0.0)
                    state_a = "GOAL"

                    if (
                        not self.a_goal
                        and a_speed
                        <= STOPPED_SPEED_MPS
                    ):
                        self.a_goal = True

                        print()
                        print(
                            "*** VEHICLE A REACHED "
                            "LOADING ZONE ***"
                        )
                        print(
                            f"A x={a_x:.3f} m, "
                            f"speed={a_speed:.3f} m/s"
                        )
                        print()

                elif not self.a_released:
                    self.publish_a_speed(0.0)
                    state_a = "WAIT"

                    if self.a_wait_events == 0:
                        self.a_wait_events = 1

                elif self.update_a_aeb():
                    self.publish_a_speed(0.0)
                    state_a = "AEB"

                else:
                    self.publish_a_speed(
                        A_TARGET_SPEED_MPS
                    )
                    state_a = "DRIVE"

                # --------------------------------------------
                # Status
                # --------------------------------------------

                if (
                    now - last_print
                    >= PRINT_INTERVAL_SECONDS
                ):
                    state_b = (
                        "GOAL"
                        if self.b_goal
                        else "HIL"
                    )

                    print(
                        f"A: x={a_x:5.1f} "
                        f"{state_a:5s} | "
                        f"B: x={b_x:5.1f} "
                        f"{state_b:4s} "
                        f"v={b_speed:.2f} | "
                        f"gap={center_distance:.2f} m"
                    )

                    last_print = now

                # --------------------------------------------
                # Mission complete
                # --------------------------------------------

                if self.a_goal and self.b_goal:
                    break

                time.sleep(CONTROL_PERIOD_SECONDS)

            self.stop_a()

            duration = (
                time.monotonic()
                - mission_start
            )

            passed = (
                self.a_goal
                and self.b_goal
                and self.a_released
                and self.a_wait_events >= 1
                and not self.estimated_collision
            )

            print()
            print("============================================================")
            print(" HYBRID HIL MISSION RESULT")
            print("============================================================")
            print(
                f"Vehicle A goal           : "
                f"{'YES' if self.a_goal else 'NO'}"
            )
            print(
                f"Vehicle B goal           : "
                f"{'YES' if self.b_goal else 'NO'}"
            )
            print(
                f"Vehicle A released       : "
                f"{'YES' if self.a_released else 'NO'}"
            )
            print(
                f"Coordination wait events : "
                f"{self.a_wait_events}"
            )
            print(
                f"Vehicle A AEB interrupts : "
                f"{self.a_aeb_interrupts}"
            )
            print(
                f"Minimum center distance  : "
                f"{self.minimum_center_distance:.3f} m"
            )
            print(
                f"Estimated collision      : "
                f"{'YES' if self.estimated_collision else 'NO'}"
            )
            print(
                f"Duration                 : "
                f"{duration:.2f} s"
            )
            print(
                f"RESULT: "
                f"{'PASS' if passed else 'FAIL'}"
            )
            print("============================================================")

            return passed

        except KeyboardInterrupt:
            print()
            print("Mission stopped by user.")
            return False

        finally:
            self.stop_a()


def main():
    test = HybridHILMissionTest()
    success = test.run()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
