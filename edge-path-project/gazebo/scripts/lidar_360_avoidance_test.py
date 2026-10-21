#!/usr/bin/env python3

import csv
import math
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


WORLD_NAME = "lidar_360_avoidance_world"

LIDAR_TOPIC = "/aeb360/lidar"
ODOMETRY_TOPIC = "/model/aeb_vehicle_360/odometry"
CMD_TOPIC = "/model/aeb_vehicle_360/cmd_vel"


CRUISE_SPEED_MPS = 2.0
AVOIDANCE_SPEED_MPS = 1.4
RETURN_SPEED_MPS = 1.8

GOAL_X_M = 62.0

AVOIDANCE_OFFSET_M = 4.1
LANE_TOLERANCE_M = 0.30

LOOKAHEAD_X_M = 5.0
HEADING_GAIN = 1.7
MAX_STEERING_COMMAND = 0.38


FRONT_DETECTION_DISTANCE_M = 10.0

LANE_CHANGE_MIN_CLEARANCE_M = 6.0
RETURN_MIN_CLEARANCE_M = 4.0

MIN_PASS_DISTANCE_M = 13.0


TTC_THRESHOLD_SECONDS = 1.7
MIN_CLOSING_SPEED_MPS = 0.10
EMERGENCY_FRONT_DISTANCE_M = 0.80
CLOSING_SPEED_FILTER_ALPHA = 0.35

VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.60

MIN_BODY_CLEARANCE_M = 1.0
PLANNING_BODY_CLEARANCE_M = 1.25


CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.50
TEST_TIMEOUT_SECONDS = 70.0
RESET_TIMEOUT_SECONDS = 10.0
SENSOR_TIMEOUT_SECONDS = 0.75

MAX_ALLOWED_ABS_Y_M = 6.0
MAX_ALLOWED_X_M = 72.0

EXPECTED_DIRECTIONS = [
    "RIGHT",
    "LEFT",
]


SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "lidar_360_avoidance_results.csv"
)


SECTOR_NAMES = [
    "FRONT",
    "FRONT_LEFT",
    "LEFT",
    "REAR_LEFT",
    "REAR",
    "REAR_RIGHT",
    "RIGHT",
    "FRONT_RIGHT",
]


def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(
            maximum,
            value
        )
    )


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def normalize_degrees(angle):
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (
            q.w * q.z
            + q.x * q.y
        ),
        1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )
    )


def sector_name(angle_deg):
    angle_deg = normalize_degrees(
        angle_deg
    )

    if -15.0 <= angle_deg <= 15.0:
        return "FRONT"

    if 15.0 < angle_deg <= 60.0:
        return "FRONT_LEFT"

    if 60.0 < angle_deg <= 120.0:
        return "LEFT"

    if 120.0 < angle_deg <= 165.0:
        return "REAR_LEFT"

    if (
        angle_deg > 165.0
        or angle_deg < -165.0
    ):
        return "REAR"

    if -165.0 <= angle_deg < -120.0:
        return "REAR_RIGHT"

    if -120.0 <= angle_deg < -60.0:
        return "RIGHT"

    return "FRONT_RIGHT"


def body_radius_for_ray(angle_rad):
    cos_a = abs(
        math.cos(angle_rad)
    )

    sin_a = abs(
        math.sin(angle_rad)
    )

    candidates = []

    if cos_a > 1e-9:
        candidates.append(
            VEHICLE_HALF_LENGTH_M
            / cos_a
        )

    if sin_a > 1e-9:
        candidates.append(
            VEHICLE_HALF_WIDTH_M
            / sin_a
        )

    if not candidates:
        return 0.0

    return min(
        candidates
    )


class Lidar360AvoidanceTest:

    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()

        self.publisher = self.node.advertise(
            CMD_TOPIC,
            Twist
        )

        if not self.node.subscribe(
            Odometry,
            ODOMETRY_TOPIC,
            self.odometry_callback
        ):
            raise RuntimeError(
                "Failed to subscribe to odometry"
            )

        if not self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback
        ):
            raise RuntimeError(
                "Failed to subscribe to 360 LiDAR"
            )

        self.reset_internal_state()


    def reset_internal_state(self):
        with self.lock:
            self.x = None
            self.y = None
            self.yaw = 0.0
            self.speed = 0.0

            self.sectors = {
                name: 30.0
                for name in SECTOR_NAMES
            }

            self.sector_body_clearance = {
                name: math.inf
                for name in SECTOR_NAMES
            }

            self.front_distance = None

            self.minimum_body_clearance = (
                math.inf
            )

            self.previous_front_distance = None
            self.previous_lidar_time = None
            self.closing_speed = None

            self.odom_received = False
            self.lidar_received = False

            self.last_odom_wall_time = 0.0
            self.last_lidar_wall_time = 0.0

            self.state = "CRUISE"

            self.avoidance_target_y = 0.0
            self.avoidance_direction = None
            self.avoidance_start_x = None

            self.avoidance_directions = []
            self.completed_avoidances = 0

            self.waiting_for_return_clearance = (
                False
            )

            self.no_path_detected = False

            self.aeb_active = False
            self.aeb_interrupts = 0

            self.goal_reached = False
            self.safety_violation = False


    def odometry_callback(self, msg):
        now = time.monotonic()

        with self.lock:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y

            self.yaw = quaternion_to_yaw(
                msg.pose.orientation
            )

            self.speed = math.hypot(
                msg.twist.linear.x,
                msg.twist.linear.y
            )

            self.odom_received = True
            self.last_odom_wall_time = now


    def lidar_callback(self, msg):
        sector_ranges = {
            name: []
            for name in SECTOR_NAMES
        }

        sector_clearances = {
            name: []
            for name in SECTOR_NAMES
        }

        minimum_body_clearance = math.inf

        for index, distance in enumerate(
            msg.ranges
        ):
            if not math.isfinite(
                distance
            ):
                continue

            if (
                distance < msg.range_min
                or distance > msg.range_max
            ):
                continue

            angle_rad = (
                msg.angle_min
                + index
                * msg.angle_step
            )

            angle_deg = math.degrees(
                angle_rad
            )

            name = sector_name(
                angle_deg
            )

            sector_ranges[
                name
            ].append(
                distance
            )

            body_clearance = max(
                0.0,
                distance
                - body_radius_for_ray(
                    angle_rad
                )
            )

            sector_clearances[
                name
            ].append(
                body_clearance
            )

            minimum_body_clearance = min(
                minimum_body_clearance,
                body_clearance
            )

        sectors = {}
        body_clearance = {}

        for name in SECTOR_NAMES:
            sectors[name] = (
                min(
                    sector_ranges[name]
                )
                if sector_ranges[name]
                else msg.range_max
            )

            body_clearance[name] = (
                min(
                    sector_clearances[name]
                )
                if sector_clearances[name]
                else math.inf
            )

        front = sectors[
            "FRONT"
        ]

        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec
            * 1e-9
        )

        now = time.monotonic()

        with self.lock:
            if (
                self.previous_front_distance
                is not None
                and self.previous_lidar_time
                is not None
            ):
                dt = (
                    sim_time
                    - self.previous_lidar_time
                )

                if 0.01 < dt < 1.0:
                    raw_closing = (
                        self.previous_front_distance
                        - front
                    ) / dt

                    raw_closing = clamp(
                        raw_closing,
                        -20.0,
                        20.0
                    )

                    if self.closing_speed is None:
                        self.closing_speed = (
                            raw_closing
                        )
                    else:
                        alpha = (
                            CLOSING_SPEED_FILTER_ALPHA
                        )

                        self.closing_speed = (
                            alpha
                            * raw_closing
                            + (
                                1.0
                                - alpha
                            )
                            * self.closing_speed
                        )

            self.previous_front_distance = (
                front
            )

            self.previous_lidar_time = (
                sim_time
            )

            self.sectors = sectors

            self.sector_body_clearance = (
                body_clearance
            )

            self.front_distance = front

            self.minimum_body_clearance = min(
                self.minimum_body_clearance,
                minimum_body_clearance
            )

            self.lidar_received = True
            self.last_lidar_wall_time = now


    def publish_command(
        self,
        speed,
        steering
    ):
        msg = Twist()

        msg.linear.x = speed
        msg.angular.z = steering

        self.publisher.publish(
            msg
        )


    def stop_vehicle(self):
        self.publish_command(
            0.0,
            0.0
        )


    def stop_repeatedly(self):
        for _ in range(10):
            self.stop_vehicle()
            time.sleep(
                0.02
            )


    def call_world_control(
        self,
        request
    ):
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD_NAME}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "8000",
            "--req",
            request,
        ]

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=10.0
        )

        output = (
            result.stdout
            + result.stderr
        )

        return (
            result.returncode == 0
            and "data: true" in output
        )


    def reset_world(self):
        print(
            "Resetting 360-degree LiDAR world..."
        )

        self.stop_repeatedly()
        self.reset_internal_state()

        if not self.call_world_control(
            "reset: {all: true}"
        ):
            print(
                "ERROR: world reset failed"
            )
            return False

        if not self.call_world_control(
            "pause: false"
        ):
            print(
                "ERROR: failed to unpause world"
            )
            return False

        start = time.monotonic()

        while True:
            if (
                time.monotonic()
                - start
                > RESET_TIMEOUT_SECONDS
            ):
                print(
                    "ERROR: reset timeout"
                )
                return False

            with self.lock:
                ready = (
                    self.odom_received
                    and self.lidar_received
                )

                x = self.x
                y = self.y
                speed = self.speed

            if (
                ready
                and x is not None
                and y is not None
                and abs(x) < 0.75
                and abs(y) < 0.75
                and speed < 0.10
            ):
                print(
                    f"Reset OK | "
                    f"x={x:.2f} | "
                    f"y={y:.2f}"
                )
                return True

            time.sleep(
                0.05
            )


    def left_path_score(
        self,
        sectors
    ):
        return min(
            sectors["FRONT_LEFT"],
            sectors["LEFT"],
            sectors["REAR_LEFT"],
        )


    def right_path_score(
        self,
        sectors
    ):
        return min(
            sectors["FRONT_RIGHT"],
            sectors["RIGHT"],
            sectors["REAR_RIGHT"],
        )


    def choose_avoidance_side(
        self,
        sectors
    ):
        left_score = (
            self.left_path_score(
                sectors
            )
        )

        right_score = (
            self.right_path_score(
                sectors
            )
        )

        left_open = (
            left_score
            >= LANE_CHANGE_MIN_CLEARANCE_M
        )

        right_open = (
            right_score
            >= LANE_CHANGE_MIN_CLEARANCE_M
        )

        print()
        print(
            "*** 360-DEGREE AVOIDANCE DECISION ***"
        )

        print(
            f"FRONT       : "
            f"{sectors['FRONT']:.2f} m"
        )

        print(
            f"FRONT_LEFT  : "
            f"{sectors['FRONT_LEFT']:.2f} m"
        )

        print(
            f"LEFT        : "
            f"{sectors['LEFT']:.2f} m"
        )

        print(
            f"REAR_LEFT   : "
            f"{sectors['REAR_LEFT']:.2f} m"
        )

        print(
            f"REAR        : "
            f"{sectors['REAR']:.2f} m"
        )

        print(
            f"REAR_RIGHT  : "
            f"{sectors['REAR_RIGHT']:.2f} m"
        )

        print(
            f"RIGHT       : "
            f"{sectors['RIGHT']:.2f} m"
        )

        print(
            f"FRONT_RIGHT : "
            f"{sectors['FRONT_RIGHT']:.2f} m"
        )

        print(
            f"LEFT path score : "
            f"{left_score:.2f} m"
        )

        print(
            f"RIGHT path score: "
            f"{right_score:.2f} m"
        )

        if (
            not left_open
            and not right_open
        ):
            print(
                "Decision: NO SAFE SIDE"
            )
            print()
            return None

        if (
            left_open
            and not right_open
        ):
            direction = "LEFT"

        elif (
            right_open
            and not left_open
        ):
            direction = "RIGHT"

        elif left_score >= right_score:
            direction = "LEFT"

        else:
            direction = "RIGHT"

        print(
            f"Decision: {direction}"
        )

        print()

        return direction


    def return_path_clear(
        self,
        sectors
    ):
        if self.avoidance_direction == "LEFT":
            score = min(
                sectors["FRONT_RIGHT"],
                sectors["RIGHT"],
                sectors["REAR_RIGHT"],
            )

        else:
            score = min(
                sectors["FRONT_LEFT"],
                sectors["LEFT"],
                sectors["REAR_LEFT"],
            )

        return (
            score
            >= RETURN_MIN_CLEARANCE_M
        )


    def update_state_machine(self):
        with self.lock:
            x = self.x
            y = self.y

            sectors = dict(
                self.sectors
            )

            state = self.state

        if state == "CRUISE":
            if (
                sectors["FRONT"]
                <= FRONT_DETECTION_DISTANCE_M
            ):
                direction = (
                    self.choose_avoidance_side(
                        sectors
                    )
                )

                if direction is None:
                    self.no_path_detected = True
                    self.state = "NO_PATH"

                    print(
                        "*** LOCAL PLANNER STOP ***"
                    )

                    print(
                        "No safe lane-change corridor exists."
                    )

                    print()
                    return

                self.avoidance_direction = (
                    direction
                )

                self.avoidance_directions.append(
                    direction
                )

                self.avoidance_start_x = x

                if direction == "LEFT":
                    self.avoidance_target_y = (
                        AVOIDANCE_OFFSET_M
                    )
                else:
                    self.avoidance_target_y = (
                        -AVOIDANCE_OFFSET_M
                    )

                self.state = "MOVE_OUT"

        elif state == "MOVE_OUT":
            if (
                abs(
                    y
                    - self.avoidance_target_y
                )
                <= LANE_TOLERANCE_M
            ):
                self.state = "PASS_OBSTACLE"

                print(
                    f"*** AVOIDANCE LANE REACHED | "
                    f"{self.avoidance_direction} ***"
                )

        elif state == "PASS_OBSTACLE":
            travelled = (
                x
                - self.avoidance_start_x
            )

            if (
                travelled
                >= MIN_PASS_DISTANCE_M
            ):
                if self.return_path_clear(
                    sectors
                ):
                    self.waiting_for_return_clearance = (
                        False
                    )

                    self.state = "RETURN"

                    print(
                        "*** 360 CHECK: RETURN CORRIDOR CLEAR ***"
                    )

                else:
                    if not self.waiting_for_return_clearance:
                        print(
                            "*** 360 CHECK: WAITING BEFORE RETURN ***"
                        )

                        print(
                            "Object detected beside / behind "
                            "the return corridor."
                        )

                        self.waiting_for_return_clearance = (
                            True
                        )

        elif state == "RETURN":
            if (
                abs(y)
                <= LANE_TOLERANCE_M
            ):
                self.completed_avoidances += 1

                print(
                    "*** ORIGINAL LANE REACHED | "
                    f"completed={self.completed_avoidances} ***"
                )

                self.state = "CRUISE"

                self.avoidance_direction = None
                self.avoidance_target_y = 0.0
                self.avoidance_start_x = None


    def steering_to_y(
        self,
        target_y
    ):
        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw

        target_x = (
            x
            + LOOKAHEAD_X_M
        )

        desired_heading = math.atan2(
            target_y - y,
            target_x - x
        )

        heading_error = wrap_angle(
            desired_heading
            - yaw
        )

        return clamp(
            HEADING_GAIN
            * heading_error,
            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND
        )


    def update_aeb(self):
        with self.lock:
            distance = self.front_distance
            closing_speed = (
                self.closing_speed
            )

            front_body_clearance = (
                self.sector_body_clearance[
                    "FRONT"
                ]
            )

        if (
            closing_speed is not None
            and closing_speed
            > MIN_CLOSING_SPEED_MPS
        ):
            ttc = (
                distance
                / closing_speed
            )
        else:
            ttc = math.inf

        trigger = (
            distance
            <= EMERGENCY_FRONT_DISTANCE_M
        )

        if (
            front_body_clearance
            <= MIN_BODY_CLEARANCE_M
        ):
            trigger = True

        if (
            math.isfinite(ttc)
            and ttc
            <= TTC_THRESHOLD_SECONDS
        ):
            trigger = True

        if (
            trigger
            and not self.aeb_active
        ):
            self.aeb_active = True
            self.aeb_interrupts += 1

            print()
            print(
                "*** AEB SAFETY OVERRIDE ***"
            )

            print(
                f"Front range: "
                f"{distance:.2f} m"
            )

            print(
                f"Front body clearance: "
                f"{front_body_clearance:.2f} m"
            )

            if math.isfinite(
                ttc
            ):
                print(
                    f"TTC: {ttc:.2f} s"
                )

            print()

        return self.aeb_active


    def minimum_360_body_clearance(self):
        with self.lock:
            clearance = dict(
                self.sector_body_clearance
            )

        finite_values = [
            value
            for value in clearance.values()
            if math.isfinite(value)
        ]

        if not finite_values:
            return math.inf

        return min(
            finite_values
        )


    def intended_lateral_direction(
        self,
        state,
        steering
    ):
        # During MOVE_OUT and PASS_OBSTACLE the intended corridor is
        # the selected avoidance side. A small opposite steering
        # correction is only path stabilisation and must not be
        # interpreted as a lane change toward the obstacle.
        if state in (
            "MOVE_OUT",
            "PASS_OBSTACLE",
        ):
            return self.avoidance_direction

        # During RETURN the intended lateral movement is back toward
        # the original lane, i.e. opposite to the avoidance side.
        if state == "RETURN":
            if self.avoidance_direction == "LEFT":
                return "RIGHT"

            if self.avoidance_direction == "RIGHT":
                return "LEFT"

        # In ordinary cruise use the actual steering sign.
        if steering > 0.05:
            return "LEFT"

        if steering < -0.05:
            return "RIGHT"

        return None


    def steering_side_safe(
        self,
        state,
        steering
    ):
        with self.lock:
            clearance = dict(
                self.sector_body_clearance
            )

        direction = (
            self.intended_lateral_direction(
                state,
                steering
            )
        )

        if direction == "LEFT":
            minimum = min(
                clearance["FRONT_LEFT"],
                clearance["LEFT"],
                clearance["REAR_LEFT"],
            )

        elif direction == "RIGHT":
            minimum = min(
                clearance["FRONT_RIGHT"],
                clearance["RIGHT"],
                clearance["REAR_RIGHT"],
            )

        else:
            return True

        return (
            minimum
            >= PLANNING_BODY_CLEARANCE_M
        )


    def save_result(
        self,
        result,
        end_reason,
        duration
    ):
        RESULTS_DIR.mkdir(
            parents=True,
            exist_ok=True
        )

        file_exists = (
            RESULTS_FILE.exists()
        )

        fields = [
            "timestamp",
            "avoidance_directions",
            "completed_avoidances",
            "goal_reached",
            "no_path_detected",
            "aeb_interrupts",
            "minimum_body_clearance_m",
            "required_body_clearance_m",
            "planning_body_clearance_m",
            "safety_margin_ok",
            "end_reason",
            "duration_s",
            "result",
        ]

        with RESULTS_FILE.open(
            "a",
            newline="",
            encoding="utf-8"
        ) as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=fields
            )

            if not file_exists:
                writer.writeheader()

            writer.writerow(
                {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),

                    "avoidance_directions":
                        ",".join(
                            self.avoidance_directions
                        ),

                    "completed_avoidances":
                        self.completed_avoidances,

                    "goal_reached":
                        self.goal_reached,

                    "no_path_detected":
                        self.no_path_detected,

                    "aeb_interrupts":
                        self.aeb_interrupts,

                    "minimum_body_clearance_m":
                        (
                            ""
                            if not math.isfinite(
                                self.minimum_body_clearance
                            )
                            else
                            f"{self.minimum_body_clearance:.3f}"
                        ),

                    "required_body_clearance_m":
                        f"{MIN_BODY_CLEARANCE_M:.3f}",

                    "planning_body_clearance_m":
                        f"{PLANNING_BODY_CLEARANCE_M:.3f}",

                    "safety_margin_ok":
                        (
                            math.isfinite(
                                self.minimum_body_clearance
                            )
                            and self.minimum_body_clearance
                            >= MIN_BODY_CLEARANCE_M
                        ),

                    "end_reason":
                        end_reason,

                    "duration_s":
                        f"{duration:.3f}",

                    "result":
                        result,
                }
            )


    def run(self):
        print()
        print(
            "============================================================"
        )

        print(
            " 360-DEGREE LIDAR OBSTACLE AVOIDANCE TEST"
        )

        print(
            "============================================================"
        )

        print(
            "Expected first avoidance : RIGHT"
        )

        print(
            "Expected second avoidance: LEFT"
        )

        print(
            "Rear-side sectors participate in "
            "lane-change decisions."
        )

        print()

        if not self.reset_world():
            print(
                "RESULT: FAIL"
            )
            return False

        start_time = (
            time.monotonic()
        )

        last_print = 0.0
        end_reason = None

        while True:
            now = time.monotonic()

            if (
                now
                - start_time
                > TEST_TIMEOUT_SECONDS
            ):
                end_reason = "timeout"
                break

            with self.lock:
                x = self.x
                y = self.y
                state = self.state

                sectors = dict(
                    self.sectors
                )

                sensors_fresh = (
                    self.odom_received
                    and self.lidar_received
                    and (
                        now
                        - self.last_odom_wall_time
                        <= SENSOR_TIMEOUT_SECONDS
                    )
                    and (
                        now
                        - self.last_lidar_wall_time
                        <= SENSOR_TIMEOUT_SECONDS
                    )
                )

            if not sensors_fresh:
                self.stop_vehicle()

                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )
                continue

            if (
                abs(y)
                > MAX_ALLOWED_ABS_Y_M
                or x
                > MAX_ALLOWED_X_M
            ):
                end_reason = (
                    "route_deviation"
                )
                break

            current_360_clearance = (
                self.minimum_360_body_clearance()
            )

            if (
                current_360_clearance
                < MIN_BODY_CLEARANCE_M
            ):
                self.safety_violation = True

                self.stop_vehicle()

                end_reason = (
                    "360_global_clearance_stop"
                )

                print()
                print(
                    "*** 360 GLOBAL SAFETY STOP ***"
                )

                print(
                    f"Minimum body clearance: "
                    f"{current_360_clearance:.3f} m"
                )

                print(
                    f"Required body clearance: "
                    f"{MIN_BODY_CLEARANCE_M:.3f} m"
                )

                print()
                break

            if (
                x >= GOAL_X_M
                and abs(y) <= 0.75
                and self.completed_avoidances >= 2
                and self.state == "CRUISE"
            ):
                self.goal_reached = True
                end_reason = "goal_reached"
                break

            self.update_state_machine()

            with self.lock:
                state = self.state

            if self.update_aeb():
                self.stop_vehicle()
                drive_state = "AEB"

            elif state == "NO_PATH":
                self.stop_vehicle()
                drive_state = "NO_PATH"

            else:
                if state == "CRUISE":
                    target_y = 0.0
                    command_speed = (
                        CRUISE_SPEED_MPS
                    )

                elif state == "MOVE_OUT":
                    target_y = (
                        self.avoidance_target_y
                    )
                    command_speed = (
                        AVOIDANCE_SPEED_MPS
                    )

                elif state == "PASS_OBSTACLE":
                    target_y = (
                        self.avoidance_target_y
                    )
                    command_speed = (
                        AVOIDANCE_SPEED_MPS
                    )

                elif state == "RETURN":
                    target_y = 0.0
                    command_speed = (
                        RETURN_SPEED_MPS
                    )

                else:
                    target_y = 0.0
                    command_speed = 0.0

                steering = (
                    self.steering_to_y(
                        target_y
                    )
                )

                if not self.steering_side_safe(
                    state,
                    steering
                ):
                    self.safety_violation = True

                    self.stop_vehicle()

                    end_reason = (
                        "360_planning_clearance_stop"
                    )

                    print()
                    print(
                        "*** 360 PLANNING SAFETY STOP ***"
                    )

                    print(
                        "The intended lateral corridor "
                        "does not have the planning reserve."
                    )

                    print(
                        f"Planning reserve required: "
                        f"{PLANNING_BODY_CLEARANCE_M:.2f} m"
                    )

                    print()
                    break

                self.publish_command(
                    command_speed,
                    steering
                )

                drive_state = state

            if (
                now
                - last_print
                >= PRINT_INTERVAL_SECONDS
            ):
                print(
                    f"x={x:5.1f} "
                    f"y={y:5.2f} | "
                    f"{drive_state:13s} | "
                    f"F={sectors['FRONT']:5.1f} "
                    f"FL={sectors['FRONT_LEFT']:5.1f} "
                    f"L={sectors['LEFT']:5.1f} "
                    f"RL={sectors['REAR_LEFT']:5.1f} "
                    f"R={sectors['REAR']:5.1f} "
                    f"RR={sectors['REAR_RIGHT']:5.1f} "
                    f"Rt={sectors['RIGHT']:5.1f} "
                    f"FR={sectors['FRONT_RIGHT']:5.1f}"
                )

                last_print = now

            time.sleep(
                CONTROL_PERIOD_SECONDS
            )

        self.stop_repeatedly()

        duration = (
            time.monotonic()
            - start_time
        )

        directions_correct = (
            self.avoidance_directions
            == EXPECTED_DIRECTIONS
        )

        safety_margin_ok = (
            math.isfinite(
                self.minimum_body_clearance
            )
            and self.minimum_body_clearance
            >= MIN_BODY_CLEARANCE_M
        )

        passed = (
            self.goal_reached
            and self.completed_avoidances == 2
            and directions_correct
            and self.aeb_interrupts == 0
            and not self.no_path_detected
            and not self.safety_violation
            and safety_margin_ok
            and end_reason == "goal_reached"
        )

        result = (
            "PASS"
            if passed
            else "FAIL"
        )

        print()
        print(
            "============================================================"
        )

        print(
            " 360-DEGREE LIDAR RESULT"
        )

        print(
            "============================================================"
        )

        print(
            f"Avoidance directions     : "
            f"{self.avoidance_directions}"
        )

        print(
            f"Expected directions      : "
            f"{EXPECTED_DIRECTIONS}"
        )

        print(
            f"Completed avoidances     : "
            f"{self.completed_avoidances}"
        )

        print(
            f"Goal reached             : "
            f"{'YES' if self.goal_reached else 'NO'}"
        )

        print(
            f"No-path detected          : "
            f"{'YES' if self.no_path_detected else 'NO'}"
        )

        print(
            f"AEB interrupts            : "
            f"{self.aeb_interrupts}"
        )

        print(
            f"360 side safety violation : "
            f"{'YES' if self.safety_violation else 'NO'}"
        )

        if math.isfinite(
            self.minimum_body_clearance
        ):
            print(
                f"Minimum body clearance   : "
                f"{self.minimum_body_clearance:.3f} m"
            )

        print(
            f"Required body clearance  : "
            f"{MIN_BODY_CLEARANCE_M:.3f} m"
        )

        print(
            f"Planning reserve         : "
            f"{PLANNING_BODY_CLEARANCE_M:.3f} m"
        )

        print(
            f"Safety margin OK         : "
            f"{'YES' if safety_margin_ok else 'NO'}"
        )

        print(
            f"End reason                : "
            f"{end_reason}"
        )

        print(
            f"Duration                  : "
            f"{duration:.2f} s"
        )

        print()

        print(
            f"RESULT: {result}"
        )

        print(
            "============================================================"
        )

        self.save_result(
            result,
            end_reason,
            duration
        )

        print(
            f"CSV: {RESULTS_FILE}"
        )

        return passed


def main():
    test = Lidar360AvoidanceTest()

    success = test.run()

    raise SystemExit(
        0 if success else 1
    )


if __name__ == "__main__":
    main()
