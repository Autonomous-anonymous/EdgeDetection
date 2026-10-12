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
# Both Ackermann odometry frames start at local (0, 0).
#
# World starts:
#
# A = (0, +2)
# B = (0, -2)
# ============================================================

A_WORLD_OFFSET_X = 0.0
A_WORLD_OFFSET_Y = 2.0

B_WORLD_OFFSET_X = 0.0
B_WORLD_OFFSET_Y = -2.0


# ============================================================
# COORDINATION
# ============================================================

# B gets priority and starts first.
#
# A remains stopped until B has this much longitudinal lead.
A_RELEASE_WHEN_B_X_M = 8.0

A_TARGET_SPEED_MPS = 1.8
B_TARGET_SPEED_MPS = 1.8


# ============================================================
# GOALS
# ============================================================
#
# Loading zone is centered around x = 30, y = 0.
#
# Both y = +/-2 lanes are inside the 6 x 6 m loading zone.
# Vehicles stop before the far edge.
# ============================================================

A_GOAL_X_M = 28.5
B_GOAL_X_M = 28.5

A_GOAL_Y_M = 2.0
B_GOAL_Y_M = -2.0

GOAL_X_TOLERANCE_M = 0.40
GOAL_Y_TOLERANCE_M = 0.75


# ============================================================
# CONTROL
# ============================================================

CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.50

TEST_TIMEOUT_SECONDS = 45.0

SENSOR_TIMEOUT_SECONDS = 0.75
RESET_TIMEOUT_SECONDS = 10.0


# ============================================================
# AEB SAFETY OVERRIDE
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 8.0

EMERGENCY_DISTANCE_M = 0.80

CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# VEHICLE SAFETY
# ============================================================
#
# Vehicles are about 2.6 m long.
#
# Use a substantially larger center-distance warning than
# before. Normal lateral separation is 4 m.
# ============================================================

MIN_ALLOWED_CENTER_DISTANCE_M = 3.0


# ============================================================
# RESULTS
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "two_vehicle_coordination_results.csv"
)


# ============================================================
# HELPERS
# ============================================================

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

class TwoVehicleCoordinationTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()


        self.vehicles = {

            "A": {
                "x": None,
                "y": None,

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
                "Failed to subscribe to A odometry"
            )


        if not self.node.subscribe(
            Odometry,
            B_ODOM_TOPIC,
            self.odom_b_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to B odometry"
            )


        if not self.node.subscribe(
            LaserScan,
            A_LIDAR_TOPIC,
            self.lidar_a_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to A LiDAR"
            )


        if not self.node.subscribe(
            LaserScan,
            B_LIDAR_TOPIC,
            self.lidar_b_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to B LiDAR"
            )


        self.minimum_center_distance_m = None

        self.estimated_collision = False

        self.coordination_wait_events = 0

        self.a_waiting = False

        self.a_released = False


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


                        raw_closing_speed = max(
                            -20.0,
                            min(
                                20.0,
                                raw_closing_speed
                            )
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

    def publish_speed(
        self,
        vehicle_name,
        speed
    ):

        msg = Twist()

        msg.linear.x = speed

        # Straight lane following.
        msg.angular.z = 0.0


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

        self.publish_speed(
            vehicle_name,
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

            for vehicle in self.vehicles.values():

                vehicle["x"] = None
                vehicle["y"] = None

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


                a_x = self.vehicles[
                    "A"
                ]["x"]

                a_y = self.vehicles[
                    "A"
                ]["y"]

                b_x = self.vehicles[
                    "B"
                ]["x"]

                b_y = self.vehicles[
                    "B"
                ]["y"]


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
                    f"Distance: {distance:.2f} m"
                )

            if math.isfinite(ttc):

                print(
                    f"TTC: {ttc:.2f} s"
                )

            print()


        # Safety release with hysteresis.
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
    # GOAL CHECK
    # ========================================================

    def goal_reached(
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


        # X is monotonic in this straight-line test.
        #
        # Therefore "x >= goal_x - tolerance" prevents
        # overshooting from turning into endless circles.
        return (
            x
            >= goal_x
            - GOAL_X_TOLERANCE_M

            and abs(
                y - goal_y
            )
            <= GOAL_Y_TOLERANCE_M
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
            " GAZEBO TWO-VEHICLE COORDINATION TEST"
        )
        print(
            "============================================================"
        )

        print(
            "Coordination strategy:"
        )

        print(
            "  Vehicle B starts first."
        )

        print(
            f"  Vehicle A waits until B reaches "
            f"x >= {A_RELEASE_WHEN_B_X_M:.1f} m."
        )

        print(
            "  Both vehicles then continue on separate "
            "parallel lanes."
        )

        print(
            "  Both destinations are inside the same "
            "loading zone."
        )

        print(
            "  LiDAR TTC AEB remains the safety override."
        )

        print()


        if not self.reset_world():

            print(
                "RESULT: FAIL"
            )

            return False


        start_time = time.monotonic()

        last_print_time = 0.0

        end_reason = None


        try:

            while True:

                now = time.monotonic()


                if (
                    now - start_time
                    > TEST_TIMEOUT_SECONDS
                ):

                    end_reason = "timeout"

                    break


                with self.lock:

                    a = dict(
                        self.vehicles["A"]
                    )

                    b = dict(
                        self.vehicles["B"]
                    )


                # ====================================================
                # SENSOR FRESHNESS
                # ====================================================

                sensors_fresh = (

                    a["odom_received"]
                    and a["lidar_received"]

                    and b["odom_received"]
                    and b["lidar_received"]

                    and (
                        now
                        - a[
                            "last_odom_wall_time"
                        ]
                        <= SENSOR_TIMEOUT_SECONDS
                    )

                    and (
                        now
                        - a[
                            "last_lidar_wall_time"
                        ]
                        <= SENSOR_TIMEOUT_SECONDS
                    )

                    and (
                        now
                        - b[
                            "last_odom_wall_time"
                        ]
                        <= SENSOR_TIMEOUT_SECONDS
                    )

                    and (
                        now
                        - b[
                            "last_lidar_wall_time"
                        ]
                        <= SENSOR_TIMEOUT_SECONDS
                    )
                )


                if not sensors_fresh:

                    self.stop_both()

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # ====================================================
                # VEHICLE DISTANCE
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

                    self.estimated_collision = True

                    print()
                    print(
                        "*** UNSAFE VEHICLE PROXIMITY ***"
                    )

                    print(
                        f"A = "
                        f"({a['x']:.2f}, {a['y']:.2f})"
                    )

                    print(
                        f"B = "
                        f"({b['x']:.2f}, {b['y']:.2f})"
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
                # GOALS
                # ====================================================

                if (
                    not a["goal_reached"]
                    and self.goal_reached(
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
                        "LOADING ZONE ***"
                    )

                    print(
                        f"A position: "
                        f"({a['x']:.2f}, {a['y']:.2f})"
                    )

                    print()


                if (
                    not b["goal_reached"]
                    and self.goal_reached(
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
                        "LOADING ZONE ***"
                    )

                    print(
                        f"B position: "
                        f"({b['x']:.2f}, {b['y']:.2f})"
                    )

                    print()


                with self.lock:

                    a_goal = self.vehicles[
                        "A"
                    ]["goal_reached"]

                    b_goal = self.vehicles[
                        "B"
                    ]["goal_reached"]


                if (
                    a_goal
                    and b_goal
                ):

                    end_reason = (
                        "both_reached_loading_zone"
                    )

                    break


                # ====================================================
                # VEHICLE B CONTROL
                # ====================================================

                if b_goal:

                    self.stop_vehicle(
                        "B"
                    )

                    state_b = "GOAL"

                else:

                    if self.update_aeb(
                        "B"
                    ):

                        self.stop_vehicle(
                            "B"
                        )

                        state_b = "AEB"

                    else:

                        self.publish_speed(
                            "B",
                            B_TARGET_SPEED_MPS
                        )

                        state_b = "DRIVE"


                # ====================================================
                # VEHICLE A COORDINATION
                # ====================================================

                if (
                    not self.a_released
                    and b["x"]
                    >= A_RELEASE_WHEN_B_X_M
                ):

                    self.a_released = True

                    self.a_waiting = False


                    print()
                    print(
                        "*** COORDINATION: "
                        "Vehicle B has safe lead - "
                        "Vehicle A released ***"
                    )

                    print(
                        f"B lead position: "
                        f"x={b['x']:.2f} m"
                    )

                    print()


                if a_goal:

                    self.stop_vehicle(
                        "A"
                    )

                    state_a = "GOAL"


                elif not self.a_released:

                    self.stop_vehicle(
                        "A"
                    )


                    if not self.a_waiting:

                        self.a_waiting = True

                        self.coordination_wait_events += 1


                        print()
                        print(
                            "*** COORDINATION: "
                            "Vehicle A waiting at start ***"
                        )

                        print()


                    state_a = "COORD WAIT"


                else:

                    if self.update_aeb(
                        "A"
                    ):

                        self.stop_vehicle(
                            "A"
                        )

                        state_a = "AEB"

                    else:

                        self.publish_speed(
                            "A",
                            A_TARGET_SPEED_MPS
                        )

                        state_a = "DRIVE"


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


                    last_print_time = now


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

            a_goal = self.vehicles[
                "A"
            ]["goal_reached"]

            b_goal = self.vehicles[
                "B"
            ]["goal_reached"]

            a_aeb = self.vehicles[
                "A"
            ]["aeb_interrupts"]

            b_aeb = self.vehicles[
                "B"
            ]["aeb_interrupts"]

            a_x = self.vehicles[
                "A"
            ]["x"]

            a_y = self.vehicles[
                "A"
            ]["y"]

            b_x = self.vehicles[
                "B"
            ]["x"]

            b_y = self.vehicles[
                "B"
            ]["y"]


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
            " TWO-VEHICLE COORDINATION RESULT"
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


        if (
            passed
            and a_aeb == 0
            and b_aeb == 0
        ):

            print(
                "Coordination prevented the need "
                "for emergency braking."
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

                a_goal = self.vehicles[
                    "A"
                ]["goal_reached"]

                b_goal = self.vehicles[
                    "B"
                ]["goal_reached"]

                a_aeb = self.vehicles[
                    "A"
                ]["aeb_interrupts"]

                b_aeb = self.vehicles[
                    "B"
                ]["aeb_interrupts"]


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


# ============================================================
# MAIN
# ============================================================

def main():

    test = TwoVehicleCoordinationTest()

    success = test.run()

    if success:

        raise SystemExit(0)

    raise SystemExit(1)


if __name__ == "__main__":

    main()
