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


# ============================================================
# CONFIGURATION
# ============================================================

WORLD_NAME = "two_vehicle_world"

A_ODOM_TOPIC = "/model/aeb_vehicle/odometry"
A_CMD_TOPIC = "/model/aeb_vehicle/cmd_vel"
A_LIDAR_TOPIC = "/aeb/lidar"

B_ODOM_TOPIC = "/model/aeb_vehicle_b/odometry"
B_CMD_TOPIC = "/model/aeb_vehicle_b/cmd_vel"
B_LIDAR_TOPIC = "/aeb_b/lidar"


# ============================================================
# WORLD COORDINATE OFFSETS
# ============================================================
#
# Ackermann odometry is local to each vehicle.
#
# World start positions:
#
# A = (0, +2)
# B = (0, -2)
# ============================================================

A_WORLD_OFFSET_X = 0.0
A_WORLD_OFFSET_Y = 2.0

B_WORLD_OFFSET_X = 0.0
B_WORLD_OFFSET_Y = -2.0


# ============================================================
# MERGE GEOMETRY
# ============================================================

A_START_LANE_Y_M = 2.0
B_START_LANE_Y_M = -2.0

MERGED_LANE_Y_M = 0.0

# Both vehicles begin moving toward the common lane
# after this longitudinal position.
MERGE_START_X_M = 18.0

# Vehicle A must wait here until B has cleared the merge.
A_HOLD_X_M = 15.5

# B is considered safely through the merge only when
# it is this far forward and close to the center lane.
B_CLEAR_X_M = 25.0
B_CLEAR_MAX_ABS_Y_M = 0.75


# ============================================================
# LOADING-ZONE GOALS
# ============================================================
#
# The green loading zone covers approximately:
#
# x = 27 ... 33
# y = -3 ... +3
#
# Both vehicles finish on the common center lane,
# but at different longitudinal positions.
# ============================================================

A_GOAL_X_M = 27.2
A_GOAL_Y_M = 0.0

B_GOAL_X_M = 32.3
B_GOAL_Y_M = 0.0

GOAL_Y_TOLERANCE_M = 0.75


# ============================================================
# SPEEDS
# ============================================================

A_CRUISE_SPEED_MPS = 1.6
B_CRUISE_SPEED_MPS = 1.6

# Slow approach to final parked positions.
A_FINAL_SPEED_MPS = 0.60
B_FINAL_SPEED_MPS = 0.70

A_FINAL_SLOWDOWN_X_M = 24.5
B_FINAL_SLOWDOWN_X_M = 30.0


# ============================================================
# PATH FOLLOWING
# ============================================================
#
# Instead of chasing fixed waypoints, the controller always
# aims at a point LOOKAHEAD_X metres ahead on the desired lane.
#
# This avoids the "circle around the waypoint" failure.
# ============================================================

LOOKAHEAD_X_M = 4.0

HEADING_GAIN = 1.8

MAX_STEERING_COMMAND = 0.40


# ============================================================
# CONTROL
# ============================================================

CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.50

TEST_TIMEOUT_SECONDS = 60.0

SENSOR_TIMEOUT_SECONDS = 0.75
RESET_TIMEOUT_SECONDS = 10.0


# ============================================================
# AEB SAFETY OVERRIDE
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 10.0

EMERGENCY_DISTANCE_M = 0.80

CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# VEHICLE-TO-VEHICLE SAFETY
# ============================================================

# Cars are roughly 2.6 m long.
# 3.0 m center distance is treated as unsafe.
MIN_ALLOWED_CENTER_DISTANCE_M = 3.0


# ============================================================
# ROUTE LIMIT
# ============================================================

# Protection against uncontrolled runaway.
MAX_ALLOWED_X_M = 36.0
MAX_ALLOWED_ABS_Y_M = 6.0


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "two_vehicle_merge_results.csv"
)


# ============================================================
# HELPERS
# ============================================================

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

        angle -= (
            2.0
            * math.pi
        )


    while angle < -math.pi:

        angle += (
            2.0
            * math.pi
        )


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


def distance_2d(
    x1,
    y1,
    x2,
    y2
):

    return math.hypot(
        x2 - x1,
        y2 - y1
    )


# ============================================================
# TEST CLASS
# ============================================================

class TwoVehicleMergeTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()


        self.vehicles = {

            "A": {
                "x": None,
                "y": None,

                "yaw": 0.0,
                "speed": 0.0,

                "front_distance": None,
                "closing_speed": None,

                "previous_distance": None,
                "previous_lidar_time": None,

                "odom_received": False,
                "lidar_received": False,

                "last_odom_wall_time": 0.0,
                "last_lidar_wall_time": 0.0,

                "goal_reached": False,

                "aeb_active": False,
                "aeb_interrupts": 0,
            },

            "B": {
                "x": None,
                "y": None,

                "yaw": 0.0,
                "speed": 0.0,

                "front_distance": None,
                "closing_speed": None,

                "previous_distance": None,
                "previous_lidar_time": None,

                "odom_received": False,
                "lidar_received": False,

                "last_odom_wall_time": 0.0,
                "last_lidar_wall_time": 0.0,

                "goal_reached": False,

                "aeb_active": False,
                "aeb_interrupts": 0,
            }
        }


        self.publisher_a = self.node.advertise(
            A_CMD_TOPIC,
            Twist
        )

        self.publisher_b = self.node.advertise(
            B_CMD_TOPIC,
            Twist
        )


        if not self.node.subscribe(
            Odometry,
            A_ODOM_TOPIC,
            self.odom_a_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to vehicle A odometry"
            )


        if not self.node.subscribe(
            Odometry,
            B_ODOM_TOPIC,
            self.odom_b_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to vehicle B odometry"
            )


        if not self.node.subscribe(
            LaserScan,
            A_LIDAR_TOPIC,
            self.lidar_a_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to vehicle A LiDAR"
            )


        if not self.node.subscribe(
            LaserScan,
            B_LIDAR_TOPIC,
            self.lidar_b_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to vehicle B LiDAR"
            )


        self.minimum_center_distance_m = None

        self.estimated_collision = False

        self.coordination_wait_events = 0

        self.a_waiting = False

        self.a_released = False

        self.b_clear_announced = False


    # ========================================================
    # ODOMETRY
    # ========================================================

    def update_odometry(
        self,
        vehicle_name,
        msg
    ):

        local_x = (
            msg.pose.position.x
        )

        local_y = (
            msg.pose.position.y
        )


        if vehicle_name == "A":

            world_x = (
                local_x
                + A_WORLD_OFFSET_X
            )

            world_y = (
                local_y
                + A_WORLD_OFFSET_Y
            )

        else:

            world_x = (
                local_x
                + B_WORLD_OFFSET_X
            )

            world_y = (
                local_y
                + B_WORLD_OFFSET_Y
            )


        yaw = quaternion_to_yaw(
            msg.pose.orientation
        )


        speed = math.hypot(
            msg.twist.linear.x,
            msg.twist.linear.y
        )


        now = time.monotonic()


        with self.lock:

            vehicle = self.vehicles[
                vehicle_name
            ]

            vehicle["x"] = world_x
            vehicle["y"] = world_y

            vehicle["yaw"] = yaw

            vehicle["speed"] = speed

            vehicle["odom_received"] = True

            vehicle[
                "last_odom_wall_time"
            ] = now


    def odom_a_callback(
        self,
        msg
    ):

        self.update_odometry(
            "A",
            msg
        )


    def odom_b_callback(
        self,
        msg
    ):

        self.update_odometry(
            "B",
            msg
        )


    # ========================================================
    # LIDAR
    # ========================================================

    def update_lidar(
        self,
        vehicle_name,
        msg
    ):

        half_angle = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        valid_ranges = []


        for index, distance in enumerate(
            msg.ranges
        ):

            angle = (
                msg.angle_min
                + index
                * msg.angle_step
            )


            if abs(angle) > half_angle:

                continue


            if not math.isfinite(
                distance
            ):

                continue


            if distance < msg.range_min:

                continue


            if distance > msg.range_max:

                continue


            valid_ranges.append(
                distance
            )


        if valid_ranges:

            distance = min(
                valid_ranges
            )

        else:

            distance = None


        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec
            * 1e-9
        )


        wall_time = time.monotonic()


        with self.lock:

            vehicle = self.vehicles[
                vehicle_name
            ]


            if distance is not None:

                previous_distance = (
                    vehicle[
                        "previous_distance"
                    ]
                )

                previous_time = (
                    vehicle[
                        "previous_lidar_time"
                    ]
                )


                if (
                    previous_distance is not None
                    and previous_time is not None
                ):

                    dt = (
                        sim_time
                        - previous_time
                    )


                    if (
                        dt > 0.01
                        and dt < 1.0
                    ):

                        raw_closing_speed = (
                            previous_distance
                            - distance
                        ) / dt


                        raw_closing_speed = clamp(
                            raw_closing_speed,
                            -20.0,
                            20.0
                        )


                        previous_filtered = (
                            vehicle[
                                "closing_speed"
                            ]
                        )


                        if previous_filtered is None:

                            filtered = (
                                raw_closing_speed
                            )

                        else:

                            alpha = (
                                CLOSING_SPEED_FILTER_ALPHA
                            )


                            filtered = (
                                alpha
                                * raw_closing_speed
                                + (
                                    1.0
                                    - alpha
                                )
                                * previous_filtered
                            )


                        vehicle[
                            "closing_speed"
                        ] = filtered


                vehicle[
                    "previous_distance"
                ] = distance

                vehicle[
                    "previous_lidar_time"
                ] = sim_time


            vehicle[
                "front_distance"
            ] = distance

            vehicle[
                "lidar_received"
            ] = True

            vehicle[
                "last_lidar_wall_time"
            ] = wall_time


    def lidar_a_callback(
        self,
        msg
    ):

        self.update_lidar(
            "A",
            msg
        )


    def lidar_b_callback(
        self,
        msg
    ):

        self.update_lidar(
            "B",
            msg
        )


    # ========================================================
    # COMMANDS
    # ========================================================

    def publish_command(
        self,
        vehicle_name,
        speed,
        steering
    ):

        msg = Twist()

        msg.linear.x = speed
        msg.angular.z = steering


        if vehicle_name == "A":

            self.publisher_a.publish(
                msg
            )

        else:

            self.publisher_b.publish(
                msg
            )


    def stop_vehicle(
        self,
        vehicle_name
    ):

        self.publish_command(
            vehicle_name,
            0.0,
            0.0
        )


    def stop_both(self):

        for _ in range(10):

            self.stop_vehicle(
                "A"
            )

            self.stop_vehicle(
                "B"
            )

            time.sleep(
                0.02
            )


    # ========================================================
    # WORLD CONTROL
    # ========================================================

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
            "5000",

            "--req",
            request
        ]


        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=8.0
        )


        output = (
            result.stdout
            + result.stderr
        )


        if result.returncode != 0:

            print(
                "World control failed:"
            )

            print(
                output
            )

            return False


        if "data: true" not in output:

            print(
                "Unexpected world-control response:"
            )

            print(
                output
            )

            return False


        return True


    def reset_internal_state(self):

        with self.lock:

            for vehicle in (
                self.vehicles.values()
            ):

                vehicle["x"] = None
                vehicle["y"] = None

                vehicle["yaw"] = 0.0
                vehicle["speed"] = 0.0

                vehicle[
                    "front_distance"
                ] = None

                vehicle[
                    "closing_speed"
                ] = None

                vehicle[
                    "previous_distance"
                ] = None

                vehicle[
                    "previous_lidar_time"
                ] = None

                vehicle[
                    "odom_received"
                ] = False

                vehicle[
                    "lidar_received"
                ] = False

                vehicle[
                    "last_odom_wall_time"
                ] = 0.0

                vehicle[
                    "last_lidar_wall_time"
                ] = 0.0

                vehicle[
                    "goal_reached"
                ] = False

                vehicle[
                    "aeb_active"
                ] = False

                vehicle[
                    "aeb_interrupts"
                ] = 0


        self.minimum_center_distance_m = None

        self.estimated_collision = False

        self.coordination_wait_events = 0

        self.a_waiting = False

        self.a_released = False

        self.b_clear_announced = False


    def reset_world(self):

        print(
            "Resetting two-vehicle world..."
        )


        self.stop_both()

        self.reset_internal_state()


        if not self.call_world_control(
            "reset: {all: true}"
        ):

            return False


        if not self.call_world_control(
            "pause: false"
        ):

            return False


        start_time = time.monotonic()


        while True:

            if (
                time.monotonic()
                - start_time
                > RESET_TIMEOUT_SECONDS
            ):

                print(
                    "ERROR: reset timeout"
                )

                return False


            with self.lock:

                ready = all(
                    vehicle[
                        "odom_received"
                    ]
                    and vehicle[
                        "lidar_received"
                    ]

                    for vehicle
                    in self.vehicles.values()
                )


                a_x = (
                    self.vehicles[
                        "A"
                    ]["x"]
                )

                a_y = (
                    self.vehicles[
                        "A"
                    ]["y"]
                )

                b_x = (
                    self.vehicles[
                        "B"
                    ]["x"]
                )

                b_y = (
                    self.vehicles[
                        "B"
                    ]["y"]
                )


            positions_ok = (
                a_x is not None
                and a_y is not None

                and b_x is not None
                and b_y is not None

                and abs(a_x) < 0.75
                and abs(a_y - 2.0) < 0.75

                and abs(b_x) < 0.75
                and abs(b_y + 2.0) < 0.75
            )


            if ready and positions_ok:

                initial_distance = (
                    distance_2d(
                        a_x,
                        a_y,
                        b_x,
                        b_y
                    )
                )


                print(
                    f"Reset OK | "
                    f"A=({a_x:.2f}, {a_y:.2f}) | "
                    f"B=({b_x:.2f}, {b_y:.2f}) | "
                    f"gap={initial_distance:.2f} m"
                )


                time.sleep(
                    0.50
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # SENSOR FRESHNESS
    # ========================================================

    def sensors_fresh(
        self,
        now,
        a,
        b
    ):

        return (
            a["odom_received"]
            and a["lidar_received"]

            and b["odom_received"]
            and b["lidar_received"]

            and (
                now
                - a["last_odom_wall_time"]
                <= SENSOR_TIMEOUT_SECONDS
            )

            and (
                now
                - a["last_lidar_wall_time"]
                <= SENSOR_TIMEOUT_SECONDS
            )

            and (
                now
                - b["last_odom_wall_time"]
                <= SENSOR_TIMEOUT_SECONDS
            )

            and (
                now
                - b["last_lidar_wall_time"]
                <= SENSOR_TIMEOUT_SECONDS
            )
        )


    # ========================================================
    # AEB
    # ========================================================

    def update_aeb(
        self,
        vehicle_name
    ):

        with self.lock:

            vehicle = self.vehicles[
                vehicle_name
            ]

            distance = vehicle[
                "front_distance"
            ]

            closing_speed = vehicle[
                "closing_speed"
            ]

            currently_active = vehicle[
                "aeb_active"
            ]


        if (
            distance is not None
            and closing_speed is not None
            and closing_speed
            > MIN_CLOSING_SPEED_MPS
        ):

            ttc = (
                distance
                / closing_speed
            )

        else:

            ttc = math.inf


        trigger = False


        if (
            distance is not None
            and distance
            <= EMERGENCY_DISTANCE_M
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
            and not currently_active
        ):

            with self.lock:

                self.vehicles[
                    vehicle_name
                ]["aeb_active"] = True

                self.vehicles[
                    vehicle_name
                ]["aeb_interrupts"] += 1


            print()
            print(
                f"*** SAFETY OVERRIDE: "
                f"Vehicle {vehicle_name} AEB ***"
            )


            if distance is not None:

                print(
                    f"Distance: "
                    f"{distance:.2f} m"
                )


            if math.isfinite(ttc):

                print(
                    f"TTC: "
                    f"{ttc:.2f} s"
                )


            print()


        if currently_active:

            safe_to_release = (
                distance is None

                or (
                    distance > 3.0

                    and (
                        closing_speed is None

                        or closing_speed
                        <= MIN_CLOSING_SPEED_MPS

                        or ttc > 2.5
                    )
                )
            )


            if safe_to_release:

                with self.lock:

                    self.vehicles[
                        vehicle_name
                    ]["aeb_active"] = False


        with self.lock:

            return self.vehicles[
                vehicle_name
            ]["aeb_active"]


    # ========================================================
    # PATH FOLLOWING
    # ========================================================

    def lane_target_y(
        self,
        vehicle_name,
        x
    ):

        if x < MERGE_START_X_M:

            if vehicle_name == "A":

                return (
                    A_START_LANE_Y_M
                )

            return (
                B_START_LANE_Y_M
            )


        return (
            MERGED_LANE_Y_M
        )


    def steering_command(
        self,
        vehicle_name,
        x,
        y,
        yaw
    ):

        target_y = self.lane_target_y(
            vehicle_name,
            x
        )


        # Virtual point always stays in front of the car.
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


        steering = clamp(
            HEADING_GAIN
            * heading_error,

            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND
        )


        return (
            steering,
            target_y
        )


    # ========================================================
    # GOAL CHECK
    # ========================================================

    def check_goal(
        self,
        vehicle_name,
        x,
        y
    ):

        if vehicle_name == "A":

            goal_x = A_GOAL_X_M
            goal_y = A_GOAL_Y_M

        else:

            goal_x = B_GOAL_X_M
            goal_y = B_GOAL_Y_M


        return (
            x >= goal_x

            and abs(
                y - goal_y
            )
            <= GOAL_Y_TOLERANCE_M
        )


    # ========================================================
    # VEHICLE CONTROL
    # ========================================================

    def drive_vehicle(
        self,
        vehicle_name,
        vehicle
    ):

        x = vehicle["x"]
        y = vehicle["y"]
        yaw = vehicle["yaw"]


        if vehicle_name == "A":

            if x >= A_FINAL_SLOWDOWN_X_M:

                speed_command = (
                    A_FINAL_SPEED_MPS
                )

            else:

                speed_command = (
                    A_CRUISE_SPEED_MPS
                )


        else:

            if x >= B_FINAL_SLOWDOWN_X_M:

                speed_command = (
                    B_FINAL_SPEED_MPS
                )

            else:

                speed_command = (
                    B_CRUISE_SPEED_MPS
                )


        steering, target_y = (
            self.steering_command(
                vehicle_name,
                x,
                y,
                yaw
            )
        )


        if self.update_aeb(
            vehicle_name
        ):

            self.stop_vehicle(
                vehicle_name
            )

            return (
                "AEB",
                target_y
            )


        self.publish_command(
            vehicle_name,
            speed_command,
            steering
        )


        if x < MERGE_START_X_M:

            return (
                "APPROACH",
                target_y
            )


        if (
            abs(
                y - MERGED_LANE_Y_M
            )
            > 0.60
        ):

            return (
                "MERGING",
                target_y
            )


        return (
            "MERGED",
            target_y
        )


    # ========================================================
    # CSV
    # ========================================================

    def save_result(
        self,
        result,
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
            "vehicle_a_goal",
            "vehicle_b_goal",
            "coordination_wait_events",
            "vehicle_a_aeb_interrupts",
            "vehicle_b_aeb_interrupts",
            "minimum_center_distance_m",
            "estimated_collision",
            "duration_s",
            "result"
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


            with self.lock:

                a_goal = (
                    self.vehicles[
                        "A"
                    ]["goal_reached"]
                )

                b_goal = (
                    self.vehicles[
                        "B"
                    ]["goal_reached"]
                )

                a_aeb = (
                    self.vehicles[
                        "A"
                    ]["aeb_interrupts"]
                )

                b_aeb = (
                    self.vehicles[
                        "B"
                    ]["aeb_interrupts"]
                )


            writer.writerow(
                {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),

                    "vehicle_a_goal":
                        a_goal,

                    "vehicle_b_goal":
                        b_goal,

                    "coordination_wait_events":
                        self.coordination_wait_events,

                    "vehicle_a_aeb_interrupts":
                        a_aeb,

                    "vehicle_b_aeb_interrupts":
                        b_aeb,

                    "minimum_center_distance_m":
                        (
                            ""
                            if self.minimum_center_distance_m
                            is None
                            else
                            f"{self.minimum_center_distance_m:.3f}"
                        ),

                    "estimated_collision":
                        self.estimated_collision,

                    "duration_s":
                        f"{duration:.3f}",

                    "result":
                        result
                }
            )


    # ========================================================
    # RUN
    # ========================================================

    def run(self):

        print()
        print(
            "============================================================"
        )
        print(
            " GAZEBO TWO-VEHICLE MERGE TEST"
        )
        print(
            "============================================================"
        )

        print(
            "Vehicle B has merge priority."
        )

        print(
            f"Vehicle A holds at x >= "
            f"{A_HOLD_X_M:.1f} m."
        )

        print(
            f"Merge begins at x = "
            f"{MERGE_START_X_M:.1f} m."
        )

        print(
            f"B clears merge at x >= "
            f"{B_CLEAR_X_M:.1f} m "
            f"and |y| <= "
            f"{B_CLEAR_MAX_ABS_Y_M:.2f} m."
        )

        print(
            f"A final slot = "
            f"({A_GOAL_X_M:.1f}, "
            f"{A_GOAL_Y_M:.1f})"
        )

        print(
            f"B final slot = "
            f"({B_GOAL_X_M:.1f}, "
            f"{B_GOAL_Y_M:.1f})"
        )

        print(
            "LiDAR relative-TTC AEB remains active."
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

        last_print_time = 0.0

        end_reason = None


        try:

            while True:

                now = time.monotonic()


                if (
                    now - start_time
                    > TEST_TIMEOUT_SECONDS
                ):

                    end_reason = (
                        "timeout"
                    )

                    break


                with self.lock:

                    a = dict(
                        self.vehicles["A"]
                    )

                    b = dict(
                        self.vehicles["B"]
                    )


                # ====================================================
                # SENSOR CHECK
                # ====================================================

                if not self.sensors_fresh(
                    now,
                    a,
                    b
                ):

                    self.stop_both()

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # ====================================================
                # ROUTE-DEVIATION PROTECTION
                # ====================================================

                route_deviation = (
                    a["x"] > MAX_ALLOWED_X_M

                    or b["x"] > MAX_ALLOWED_X_M

                    or abs(a["y"])
                    > MAX_ALLOWED_ABS_Y_M

                    or abs(b["y"])
                    > MAX_ALLOWED_ABS_Y_M
                )


                if route_deviation:

                    print()
                    print(
                        "*** ROUTE DEVIATION ***"
                    )

                    print(
                        f"A = "
                        f"({a['x']:.2f}, "
                        f"{a['y']:.2f})"
                    )

                    print(
                        f"B = "
                        f"({b['x']:.2f}, "
                        f"{b['y']:.2f})"
                    )

                    print()


                    end_reason = (
                        "route_deviation"
                    )

                    break


                # ====================================================
                # VEHICLE-TO-VEHICLE DISTANCE
                # ====================================================

                center_distance = (
                    distance_2d(
                        a["x"],
                        a["y"],
                        b["x"],
                        b["y"]
                    )
                )


                if (
                    self.minimum_center_distance_m
                    is None

                    or center_distance
                    < self.minimum_center_distance_m
                ):

                    self.minimum_center_distance_m = (
                        center_distance
                    )


                if (
                    center_distance
                    < MIN_ALLOWED_CENTER_DISTANCE_M
                ):

                    self.estimated_collision = (
                        True
                    )


                    print()
                    print(
                        "*** UNSAFE VEHICLE PROXIMITY ***"
                    )

                    print(
                        f"A = "
                        f"({a['x']:.2f}, "
                        f"{a['y']:.2f})"
                    )

                    print(
                        f"B = "
                        f"({b['x']:.2f}, "
                        f"{b['y']:.2f})"
                    )

                    print(
                        f"Center distance = "
                        f"{center_distance:.3f} m"
                    )

                    print()


                    end_reason = (
                        "unsafe_proximity"
                    )

                    break


                # ====================================================
                # B CLEAR CONDITION
                # ====================================================

                b_merge_clear = (
                    b["x"]
                    >= B_CLEAR_X_M

                    and abs(
                        b["y"]
                    )
                    <= B_CLEAR_MAX_ABS_Y_M
                )


                if (
                    b_merge_clear
                    and not self.b_clear_announced
                ):

                    self.b_clear_announced = (
                        True
                    )


                    print()
                    print(
                        "*** VEHICLE B CLEAR OF MERGE ***"
                    )

                    print(
                        f"B position: "
                        f"({b['x']:.2f}, "
                        f"{b['y']:.2f})"
                    )

                    print()


                # ====================================================
                # GOAL DETECTION
                # ====================================================

                if (
                    not a["goal_reached"]

                    and self.check_goal(
                        "A",
                        a["x"],
                        a["y"]
                    )
                ):

                    with self.lock:

                        self.vehicles[
                            "A"
                        ]["goal_reached"] = True


                    print()
                    print(
                        "*** VEHICLE A REACHED "
                        "LOADING-ZONE SLOT ***"
                    )

                    print(
                        f"A position: "
                        f"({a['x']:.2f}, "
                        f"{a['y']:.2f})"
                    )

                    print()


                if (
                    not b["goal_reached"]

                    and self.check_goal(
                        "B",
                        b["x"],
                        b["y"]
                    )
                ):

                    with self.lock:

                        self.vehicles[
                            "B"
                        ]["goal_reached"] = True


                    print()
                    print(
                        "*** VEHICLE B REACHED "
                        "LOADING-ZONE SLOT ***"
                    )

                    print(
                        f"B position: "
                        f"({b['x']:.2f}, "
                        f"{b['y']:.2f})"
                    )

                    print()


                with self.lock:

                    a_goal = (
                        self.vehicles[
                            "A"
                        ]["goal_reached"]
                    )

                    b_goal = (
                        self.vehicles[
                            "B"
                        ]["goal_reached"]
                    )


                if (
                    a_goal
                    and b_goal
                ):

                    end_reason = (
                        "both_reached_loading_zone"
                    )

                    break


                # ====================================================
                # VEHICLE B
                # ====================================================

                if b_goal:

                    self.stop_vehicle(
                        "B"
                    )

                    state_b = "GOAL"


                else:

                    state_b, _ = (
                        self.drive_vehicle(
                            "B",
                            b
                        )
                    )


                # ====================================================
                # VEHICLE A MERGE COORDINATION
                # ====================================================

                if (
                    not self.a_released
                    and b_merge_clear
                ):

                    self.a_released = (
                        True
                    )


                    if self.a_waiting:

                        print()
                        print(
                            "*** COORDINATION: "
                            "Vehicle B cleared merge - "
                            "Vehicle A released ***"
                        )

                        print()


                    self.a_waiting = (
                        False
                    )


                if a_goal:

                    self.stop_vehicle(
                        "A"
                    )

                    state_a = "GOAL"


                elif (
                    not self.a_released

                    and a["x"]
                    >= A_HOLD_X_M
                ):

                    self.stop_vehicle(
                        "A"
                    )


                    if not self.a_waiting:

                        self.a_waiting = True

                        self.coordination_wait_events += 1


                        print()
                        print(
                            "*** COORDINATION: "
                            "Vehicle A waiting before merge ***"
                        )

                        print(
                            f"A position: "
                            f"({a['x']:.2f}, "
                            f"{a['y']:.2f})"
                        )

                        print()


                    state_a = (
                        "COORD WAIT"
                    )


                else:

                    state_a, _ = (
                        self.drive_vehicle(
                            "A",
                            a
                        )
                    )


                # ====================================================
                # STATUS
                # ====================================================

                if (
                    now - last_print_time
                    >= PRINT_INTERVAL_SECONDS
                ):

                    print(
                        f"A: "
                        f"x={a['x']:5.1f} "
                        f"y={a['y']:5.1f} "
                        f"{state_a:10s} | "
                        f"B: "
                        f"x={b['x']:5.1f} "
                        f"y={b['y']:5.1f} "
                        f"{state_b:10s} | "
                        f"gap={center_distance:5.2f} m"
                    )


                    last_print_time = (
                        now
                    )


                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )


        except KeyboardInterrupt:

            end_reason = (
                "user_interrupt"
            )


        self.stop_both()


        duration = (
            time.monotonic()
            - start_time
        )


        with self.lock:

            a_goal = (
                self.vehicles[
                    "A"
                ]["goal_reached"]
            )

            b_goal = (
                self.vehicles[
                    "B"
                ]["goal_reached"]
            )

            a_aeb = (
                self.vehicles[
                    "A"
                ]["aeb_interrupts"]
            )

            b_aeb = (
                self.vehicles[
                    "B"
                ]["aeb_interrupts"]
            )

            a_x = (
                self.vehicles[
                    "A"
                ]["x"]
            )

            a_y = (
                self.vehicles[
                    "A"
                ]["y"]
            )

            b_x = (
                self.vehicles[
                    "B"
                ]["x"]
            )

            b_y = (
                self.vehicles[
                    "B"
                ]["y"]
            )


        passed = (
            a_goal
            and b_goal

            and self.coordination_wait_events
            >= 1

            and not self.estimated_collision

            and end_reason
            == "both_reached_loading_zone"
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
            " TWO-VEHICLE MERGE RESULT"
        )
        print(
            "============================================================"
        )

        print(
            f"Vehicle A goal            : "
            f"{'YES' if a_goal else 'NO'}"
        )

        print(
            f"Vehicle B goal            : "
            f"{'YES' if b_goal else 'NO'}"
        )


        if (
            a_x is not None
            and a_y is not None
        ):

            print(
                f"Vehicle A final position  : "
                f"({a_x:.2f}, {a_y:.2f})"
            )


        if (
            b_x is not None
            and b_y is not None
        ):

            print(
                f"Vehicle B final position  : "
                f"({b_x:.2f}, {b_y:.2f})"
            )


        print(
            f"Coordination wait events  : "
            f"{self.coordination_wait_events}"
        )

        print(
            f"Vehicle A AEB interrupts  : "
            f"{a_aeb}"
        )

        print(
            f"Vehicle B AEB interrupts  : "
            f"{b_aeb}"
        )


        if (
            self.minimum_center_distance_m
            is not None
        ):

            print(
                f"Minimum center distance   : "
                f"{self.minimum_center_distance_m:.3f} m"
            )


        print(
            f"Estimated collision       : "
            f"{'YES' if self.estimated_collision else 'NO'}"
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


        if passed:

            if (
                a_aeb == 0
                and b_aeb == 0
            ):

                print(
                    "Merge coordination prevented "
                    "the need for emergency braking."
                )

            else:

                print(
                    "Both vehicles completed the merge, "
                    "but AEB intervention occurred."
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
            duration
        )


        print(
            f"CSV: {RESULTS_FILE}"
        )


        return passed


# ============================================================
# MAIN
# ============================================================

def main():

    test = TwoVehicleMergeTest()

    success = test.run()


    if success:

        raise SystemExit(0)


    raise SystemExit(1)


if __name__ == "__main__":

    main()
