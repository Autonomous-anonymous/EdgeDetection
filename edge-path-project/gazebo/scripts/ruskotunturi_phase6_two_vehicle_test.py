#!/usr/bin/env python3

import csv
import math
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


# ============================================================
# WORLD / VEHICLES / TOPICS
# ============================================================

WORLD_NAME = (
    "ruskotunturi_phase6_two_vehicle_world"
)

POSE_TOPIC = (
    "/world/"
    + WORLD_NAME
    + "/pose/info"
)

MODEL_A = "aeb_vehicle_360"
MODEL_B = "aeb_vehicle_360_b"

CMD_A_TOPIC = (
    "/model/"
    + MODEL_A
    + "/cmd_vel"
)

CMD_B_TOPIC = (
    "/model/"
    + MODEL_B
    + "/cmd_vel"
)

LIDAR_A_TOPIC = "/aeb360/lidar"
LIDAR_B_TOPIC = "/aeb360b/lidar"


# ============================================================
# ROUTES
# ============================================================

A_START = (
    8.0,
    34.0
)

A_GOAL = (
    -8.0,
    24.0
)

B_START = (
    -8.0,
    36.0
)

B_GOAL = (
    8.0,
    24.0
)

CONFLICT_CENTER = (
    0.7272727273,
    29.4545454545
)

CONFLICT_RADIUS_M = 3.0

# Vehicle A receives coordinator priority.
PRIORITY_VEHICLE = "A"

# Vehicle B stops this far before the centre of the conflict zone.
B_STOP_LINE_DISTANCE_M = 4.5

# A must move this far beyond the route intersection before B
# receives permission to continue.
A_CLEAR_MARGIN_M = 3.0


# ============================================================
# MOTION / CONTROL
# ============================================================

A_CRUISE_SPEED_MPS = 0.90
B_CRUISE_SPEED_MPS = 0.85

B_APPROACH_SPEED_MPS = 0.40

FOLLOW_LOOKAHEAD_M = 3.0

HEADING_GAIN = 1.60
MAX_STEERING = 0.40

GOAL_RADIUS_M = 1.8
GOAL_LINE_MARGIN_M = 0.40
GOAL_LATERAL_TOLERANCE_M = 0.90

CONTROL_PERIOD_S = 0.05
PRINT_INTERVAL_S = 0.75
MISSION_TIMEOUT_S = 120.0

RESET_SETTLE_S = 1.0
SENSOR_TIMEOUT_S = 2.0


# ============================================================
# SAFETY
# ============================================================

VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.60

HARD_BODY_CLEARANCE_M = 1.00

MAX_ABS_ROLL_DEG = 25.0
MAX_ABS_PITCH_DEG = 30.0


# ============================================================
# RESULTS
# ============================================================

RESULT_FILE = (
    Path(__file__).resolve().parent.parent
    / "results"
    / "ruskotunturi_phase6_two_vehicle_results.csv"
)


# ============================================================
# HELPERS
# ============================================================

def clamp(
    value,
    minimum,
    maximum
):

    return max(
        minimum,
        min(
            maximum,
            value
        )
    )


def wrap_angle(
    angle
):

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


def quaternion_to_rpy(
    q
):

    x = q.x
    y = q.y
    z = q.z
    w = q.w


    sinr_cosp = (
        2.0
        * (
            w * x
            + y * z
        )
    )

    cosr_cosp = (
        1.0
        - 2.0
        * (
            x * x
            + y * y
        )
    )

    roll = math.atan2(
        sinr_cosp,
        cosr_cosp
    )


    sinp = (
        2.0
        * (
            w * y
            - z * x
        )
    )

    if abs(
        sinp
    ) >= 1.0:

        pitch = math.copysign(
            math.pi / 2.0,
            sinp
        )

    else:

        pitch = math.asin(
            sinp
        )


    siny_cosp = (
        2.0
        * (
            w * z
            + x * y
        )
    )

    cosy_cosp = (
        1.0
        - 2.0
        * (
            y * y
            + z * z
        )
    )

    yaw = math.atan2(
        siny_cosp,
        cosy_cosp
    )


    return (
        roll,
        pitch,
        yaw
    )


def body_radius_for_ray(
    angle_rad
):

    cos_a = abs(
        math.cos(
            angle_rad
        )
    )

    sin_a = abs(
        math.sin(
            angle_rad
        )
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


def route_geometry(
    start,
    goal
):

    dx = (
        goal[0]
        - start[0]
    )

    dy = (
        goal[1]
        - start[1]
    )

    length = math.hypot(
        dx,
        dy
    )

    ux = dx / length
    uy = dy / length

    nx = -uy
    ny = ux

    heading = math.atan2(
        dy,
        dx
    )


    return {
        "dx": dx,
        "dy": dy,
        "length": length,
        "ux": ux,
        "uy": uy,
        "nx": nx,
        "ny": ny,
        "heading": heading,
    }


def project_to_route(
    x,
    y,
    start,
    geometry
):

    rx = (
        x
        - start[0]
    )

    ry = (
        y
        - start[1]
    )

    s = (
        rx
        * geometry[
            "ux"
        ]
        + ry
        * geometry[
            "uy"
        ]
    )

    d = (
        rx
        * geometry[
            "nx"
        ]
        + ry
        * geometry[
            "ny"
        ]
    )


    return (
        s,
        d
    )


def conflict_s(
    start,
    geometry
):

    s, _ = project_to_route(
        CONFLICT_CENTER[0],
        CONFLICT_CENTER[1],
        start,
        geometry
    )

    return s


# ============================================================
# CONTROLLER
# ============================================================

class Phase6TwoVehicleCoordinator:

    def __init__(
        self
    ):

        self.node = Node()
        self.lock = threading.Lock()

        self.pub_a = (
            self.node.advertise(
                CMD_A_TOPIC,
                Twist
            )
        )

        self.pub_b = (
            self.node.advertise(
                CMD_B_TOPIC,
                Twist
            )
        )


        if not self.node.subscribe(
            Pose_V,
            POSE_TOPIC,
            self.pose_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to "
                + POSE_TOPIC
            )


        if not self.node.subscribe(
            LaserScan,
            LIDAR_A_TOPIC,
            self.lidar_a_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to "
                + LIDAR_A_TOPIC
            )


        if not self.node.subscribe(
            LaserScan,
            LIDAR_B_TOPIC,
            self.lidar_b_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to "
                + LIDAR_B_TOPIC
            )


        self.route_a = (
            route_geometry(
                A_START,
                A_GOAL
            )
        )

        self.route_b = (
            route_geometry(
                B_START,
                B_GOAL
            )
        )

        self.conflict_s_a = (
            conflict_s(
                A_START,
                self.route_a
            )
        )

        self.conflict_s_b = (
            conflict_s(
                B_START,
                self.route_b
            )
        )

        self.reset_state()


    # ========================================================
    # STATE
    # ========================================================

    def blank_vehicle_state(
        self
    ):

        return {
            "x": None,
            "y": None,
            "z": None,
            "yaw": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,

            "pose_received": False,
            "lidar_received": False,

            "last_pose_time": 0.0,
            "last_lidar_time": 0.0,

            "min_clearance": math.inf,
            "minimum_clearance_seen": math.inf,

            "max_abs_roll_deg": 0.0,
            "max_abs_pitch_deg": 0.0,

            "goal_reached": False,
            "safety_stop": False,
        }


    def reset_state(
        self
    ):

        with self.lock:

            self.a = (
                self.blank_vehicle_state()
            )

            self.b = (
                self.blank_vehicle_state()
            )

            self.b_waiting = False
            self.b_released = False

            self.coordinator_wait_events = 0
            self.coordinator_release_events = 0

            self.conflict_overlap_violation = False
            self.terrain_violation = False
            self.lidar_violation = False

            self.end_reason = "running"


    # ========================================================
    # CALLBACKS
    # ========================================================

    def pose_matches(
        self,
        pose_name,
        model_name
    ):

        return (
            pose_name == model_name
            or pose_name.endswith(
                "::"
                + model_name
            )
        )


    def update_pose_state(
        self,
        state,
        pose,
        now
    ):

        roll, pitch, yaw = (
            quaternion_to_rpy(
                pose.orientation
            )
        )

        state[
            "x"
        ] = pose.position.x

        state[
            "y"
        ] = pose.position.y

        state[
            "z"
        ] = pose.position.z

        state[
            "yaw"
        ] = yaw

        state[
            "roll_deg"
        ] = math.degrees(
            roll
        )

        state[
            "pitch_deg"
        ] = math.degrees(
            pitch
        )

        state[
            "max_abs_roll_deg"
        ] = max(
            state[
                "max_abs_roll_deg"
            ],
            abs(
                state[
                    "roll_deg"
                ]
            )
        )

        state[
            "max_abs_pitch_deg"
        ] = max(
            state[
                "max_abs_pitch_deg"
            ],
            abs(
                state[
                    "pitch_deg"
                ]
            )
        )

        state[
            "pose_received"
        ] = True

        state[
            "last_pose_time"
        ] = now


    def pose_callback(
        self,
        msg
    ):

        pose_a = None
        pose_b = None


        for pose in msg.pose:

            if self.pose_matches(
                pose.name,
                MODEL_A
            ):

                pose_a = pose


            if self.pose_matches(
                pose.name,
                MODEL_B
            ):

                pose_b = pose


        now = time.monotonic()


        with self.lock:

            if pose_a is not None:

                self.update_pose_state(
                    self.a,
                    pose_a,
                    now
                )


            if pose_b is not None:

                self.update_pose_state(
                    self.b,
                    pose_b,
                    now
                )


    def process_lidar(
        self,
        msg
    ):

        minimum_clearance = math.inf


        for index, distance in enumerate(
            msg.ranges
        ):

            if not math.isfinite(
                distance
            ):

                continue


            if (
                distance
                < msg.range_min
                or distance
                > msg.range_max
            ):

                continue


            angle = (
                msg.angle_min
                + index
                * msg.angle_step
            )

            clearance = max(
                0.0,
                distance
                - body_radius_for_ray(
                    angle
                )
            )

            minimum_clearance = min(
                minimum_clearance,
                clearance
            )


        return minimum_clearance


    def lidar_a_callback(
        self,
        msg
    ):

        clearance = (
            self.process_lidar(
                msg
            )
        )

        now = time.monotonic()


        with self.lock:

            self.a[
                "min_clearance"
            ] = clearance

            self.a[
                "minimum_clearance_seen"
            ] = min(
                self.a[
                    "minimum_clearance_seen"
                ],
                clearance
            )

            self.a[
                "lidar_received"
            ] = True

            self.a[
                "last_lidar_time"
            ] = now


    def lidar_b_callback(
        self,
        msg
    ):

        clearance = (
            self.process_lidar(
                msg
            )
        )

        now = time.monotonic()


        with self.lock:

            self.b[
                "min_clearance"
            ] = clearance

            self.b[
                "minimum_clearance_seen"
            ] = min(
                self.b[
                    "minimum_clearance_seen"
                ],
                clearance
            )

            self.b[
                "lidar_received"
            ] = True

            self.b[
                "last_lidar_time"
            ] = now


    # ========================================================
    # COMMANDS
    # ========================================================

    def publish(
        self,
        publisher,
        speed,
        steering
    ):

        msg = Twist()

        msg.linear.x = speed
        msg.angular.z = steering

        publisher.publish(
            msg
        )


    def stop_both(
        self
    ):

        self.publish(
            self.pub_a,
            0.0,
            0.0
        )

        self.publish(
            self.pub_b,
            0.0,
            0.0
        )


    def stop_both_repeatedly(
        self
    ):

        for _ in range(
            10
        ):

            self.stop_both()

            time.sleep(
                0.02
            )


    # ========================================================
    # WORLD RESET
    # ========================================================

    def world_control(
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


        try:

            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=10.0
            )

        except subprocess.TimeoutExpired:

            return False


        output = (
            result.stdout
            + result.stderr
        )


        return (
            result.returncode == 0
            and "data: true"
            in output
        )


    def reset_world(
        self
    ):

        print(
            "Resetting Phase 6 two-vehicle world..."
        )

        self.stop_both_repeatedly()
        self.reset_state()


        if not self.world_control(
            "reset: {all: true}"
        ):

            print(
                "ERROR: world reset failed"
            )

            return False


        if not self.world_control(
            "pause: false"
        ):

            print(
                "ERROR: failed to unpause world"
            )

            return False


        time.sleep(
            RESET_SETTLE_S
        )


        start = time.monotonic()


        while (
            time.monotonic()
            - start
            < 8.0
        ):

            with self.lock:

                ready = (
                    self.a[
                        "pose_received"
                    ]
                    and self.a[
                        "lidar_received"
                    ]
                    and self.b[
                        "pose_received"
                    ]
                    and self.b[
                        "lidar_received"
                    ]
                )


            if ready:

                print(
                    "Reset OK | "
                    f"A=({self.a['x']:.2f}, "
                    f"{self.a['y']:.2f}) | "
                    f"B=({self.b['x']:.2f}, "
                    f"{self.b['y']:.2f})"
                )

                return True


            time.sleep(
                0.05
            )


        print(
            "ERROR: both vehicles / LiDARs "
            "did not become ready after reset."
        )

        return False


    # ========================================================
    # ROUTE CONTROL
    # ========================================================

    def route_status(
        self,
        state,
        start,
        goal,
        geometry
    ):

        s, lateral = (
            project_to_route(
                state[
                    "x"
                ],
                state[
                    "y"
                ],
                start,
                geometry
            )
        )

        goal_distance = (
            distance_2d(
                state[
                    "x"
                ],
                state[
                    "y"
                ],
                goal[
                    0
                ],
                goal[
                    1
                ]
            )
        )

        crossed_goal = (
            s
            >= (
                geometry[
                    "length"
                ]
                - GOAL_LINE_MARGIN_M
            )
            and abs(
                lateral
            )
            <= GOAL_LATERAL_TOLERANCE_M
        )


        return (
            s,
            lateral,
            goal_distance,
            crossed_goal
        )


    def route_command(
        self,
        state,
        start,
        goal,
        geometry,
        desired_speed
    ):

        s, lateral, goal_distance, crossed_goal = (
            self.route_status(
                state,
                start,
                goal,
                geometry
            )
        )


        if (
            goal_distance
            <= GOAL_RADIUS_M
            or crossed_goal
        ):

            return (
                0.0,
                0.0,
                True,
                s,
                lateral,
                goal_distance
            )


        target_s = (
            s
            + FOLLOW_LOOKAHEAD_M
        )

        target_x = (
            start[
                0
            ]
            + target_s
            * geometry[
                "ux"
            ]
        )

        target_y = (
            start[
                1
            ]
            + target_s
            * geometry[
                "uy"
            ]
        )

        desired_heading = math.atan2(
            target_y
            - state[
                "y"
            ],
            target_x
            - state[
                "x"
            ]
        )

        heading_error = wrap_angle(
            desired_heading
            - state[
                "yaw"
            ]
        )

        steering = clamp(
            HEADING_GAIN
            * heading_error,
            -MAX_STEERING,
            MAX_STEERING
        )


        return (
            desired_speed,
            steering,
            False,
            s,
            lateral,
            goal_distance
        )


    # ========================================================
    # MAIN TEST
    # ========================================================

    def run(
        self
    ):

        print()
        print(
            "============================================================"
        )

        print(
            " RUSKOTUNTURI PHASE 6 - TWO VEHICLE COORDINATION"
        )

        print(
            "============================================================"
        )

        print(
            f"Priority vehicle        : "
            f"{PRIORITY_VEHICLE}"
        )

        print(
            f"Vehicle A route         : "
            f"{A_START} -> {A_GOAL}"
        )

        print(
            f"Vehicle B route         : "
            f"{B_START} -> {B_GOAL}"
        )

        print(
            f"Conflict centre         : "
            f"({CONFLICT_CENTER[0]:.2f}, "
            f"{CONFLICT_CENTER[1]:.2f})"
        )

        print(
            f"Conflict radius         : "
            f"{CONFLICT_RADIUS_M:.2f} m"
        )

        print(
            f"B stop-line distance    : "
            f"{B_STOP_LINE_DISTANCE_M:.2f} m"
        )

        print(
            f"A clear margin          : "
            f"{A_CLEAR_MARGIN_M:.2f} m"
        )

        print(
            f"Vehicle A speed         : "
            f"{A_CRUISE_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Vehicle B speed         : "
            f"{B_CRUISE_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Hard body clearance     : "
            f"{HARD_BODY_CLEARANCE_M:.2f} m"
        )

        print(
            "Local safety            : "
            "independent 360 LiDAR on both vehicles"
        )

        print()


        if not self.reset_world():

            self.end_reason = (
                "reset_failed"
            )

            self.print_result(
                0.0
            )

            return False


        start_time = (
            time.monotonic()
        )

        last_print = 0.0


        while True:

            now = (
                time.monotonic()
            )

            elapsed = (
                now
                - start_time
            )


            if elapsed > MISSION_TIMEOUT_S:

                with self.lock:

                    self.end_reason = (
                        "timeout"
                    )

                break


            with self.lock:

                a = dict(
                    self.a
                )

                b = dict(
                    self.b
                )

                b_released = (
                    self.b_released
                )

                b_waiting = (
                    self.b_waiting
                )


            # --------------------------------------------
            # Sensor freshness
            # --------------------------------------------

            sensor_timeout = (
                now
                - a[
                    "last_pose_time"
                ]
                > SENSOR_TIMEOUT_S
                or now
                - a[
                    "last_lidar_time"
                ]
                > SENSOR_TIMEOUT_S
                or now
                - b[
                    "last_pose_time"
                ]
                > SENSOR_TIMEOUT_S
                or now
                - b[
                    "last_lidar_time"
                ]
                > SENSOR_TIMEOUT_S
            )


            if sensor_timeout:

                with self.lock:

                    self.end_reason = (
                        "sensor_timeout"
                    )

                break


            # --------------------------------------------
            # Terrain safety
            # --------------------------------------------

            terrain_bad = (
                abs(
                    a[
                        "roll_deg"
                    ]
                )
                > MAX_ABS_ROLL_DEG
                or abs(
                    a[
                        "pitch_deg"
                    ]
                )
                > MAX_ABS_PITCH_DEG
                or abs(
                    b[
                        "roll_deg"
                    ]
                )
                > MAX_ABS_ROLL_DEG
                or abs(
                    b[
                        "pitch_deg"
                    ]
                )
                > MAX_ABS_PITCH_DEG
            )


            if terrain_bad:

                with self.lock:

                    self.terrain_violation = True
                    self.end_reason = (
                        "terrain_attitude_stop"
                    )

                break


            # --------------------------------------------
            # LiDAR hard safety
            # --------------------------------------------

            if (
                a[
                    "min_clearance"
                ]
                < HARD_BODY_CLEARANCE_M
            ):

                with self.lock:

                    self.a[
                        "safety_stop"
                    ] = True

                    self.lidar_violation = True

                    self.end_reason = (
                        "vehicle_a_lidar_stop"
                    )

                print()
                print(
                    "*** VEHICLE A LOCAL LIDAR SAFETY STOP ***"
                )

                print(
                    f"Body clearance          : "
                    f"{a['min_clearance']:.3f} m"
                )

                break


            if (
                b[
                    "min_clearance"
                ]
                < HARD_BODY_CLEARANCE_M
            ):

                with self.lock:

                    self.b[
                        "safety_stop"
                    ] = True

                    self.lidar_violation = True

                    self.end_reason = (
                        "vehicle_b_lidar_stop"
                    )

                print()
                print(
                    "*** VEHICLE B LOCAL LIDAR SAFETY STOP ***"
                )

                print(
                    f"Body clearance          : "
                    f"{b['min_clearance']:.3f} m"
                )

                break


            # --------------------------------------------
            # Route progress
            # --------------------------------------------

            (
                a_s,
                a_d,
                a_goal_distance,
                a_crossed_goal,
            ) = self.route_status(
                a,
                A_START,
                A_GOAL,
                self.route_a
            )

            (
                b_s,
                b_d,
                b_goal_distance,
                b_crossed_goal,
            ) = self.route_status(
                b,
                B_START,
                B_GOAL,
                self.route_b
            )


            # --------------------------------------------
            # Conflict occupancy monitor
            # --------------------------------------------

            a_conflict_distance = (
                distance_2d(
                    a[
                        "x"
                    ],
                    a[
                        "y"
                    ],
                    CONFLICT_CENTER[
                        0
                    ],
                    CONFLICT_CENTER[
                        1
                    ]
                )
            )

            b_conflict_distance = (
                distance_2d(
                    b[
                        "x"
                    ],
                    b[
                        "y"
                    ],
                    CONFLICT_CENTER[
                        0
                    ],
                    CONFLICT_CENTER[
                        1
                    ]
                )
            )

            a_in_conflict = (
                a_conflict_distance
                <= CONFLICT_RADIUS_M
            )

            b_in_conflict = (
                b_conflict_distance
                <= CONFLICT_RADIUS_M
            )


            if (
                a_in_conflict
                and b_in_conflict
            ):

                with self.lock:

                    self.conflict_overlap_violation = True

                    self.end_reason = (
                        "conflict_overlap"
                    )

                print()
                print(
                    "*** COORDINATOR CONFLICT VIOLATION ***"
                )

                print(
                    "Both vehicles entered "
                    "the conflict zone simultaneously."
                )

                break


            # --------------------------------------------
            # A has priority. Release B only when A has
            # moved beyond the intersection by a margin.
            # --------------------------------------------

            a_cleared = (
                a_s
                >= (
                    self.conflict_s_a
                    + A_CLEAR_MARGIN_M
                )
                or a[
                    "goal_reached"
                ]
            )


            if (
                a_cleared
                and not b_released
            ):

                with self.lock:

                    self.b_released = True

                    self.coordinator_release_events += 1

                    self.b_waiting = False

                b_released = True
                b_waiting = False

                print()
                print(
                    "*** COORDINATOR: VEHICLE B RELEASED ***"
                )

                print(
                    f"Vehicle A route progress: "
                    f"{a_s:.2f} m"
                )

                print(
                    f"A clear threshold       : "
                    f"{self.conflict_s_a + A_CLEAR_MARGIN_M:.2f} m"
                )

                print()


            # --------------------------------------------
            # Vehicle A command
            # --------------------------------------------

            (
                a_speed,
                a_steering,
                a_goal,
                a_s,
                a_d,
                a_goal_distance,
            ) = self.route_command(
                a,
                A_START,
                A_GOAL,
                self.route_a,
                A_CRUISE_SPEED_MPS
            )


            if a_goal:

                a_speed = 0.0
                a_steering = 0.0

                with self.lock:

                    self.a[
                        "goal_reached"
                    ] = True


            # --------------------------------------------
            # Vehicle B coordinator command
            # --------------------------------------------

            b_remaining_to_conflict = (
                self.conflict_s_b
                - b_s
            )


            if not b_released:

                if (
                    b_remaining_to_conflict
                    <= B_STOP_LINE_DISTANCE_M
                ):

                    b_speed = 0.0
                    b_steering = 0.0

                    if not b_waiting:

                        with self.lock:

                            self.b_waiting = True

                            self.coordinator_wait_events += 1

                        b_waiting = True

                        print()
                        print(
                            "*** COORDINATOR: VEHICLE B HOLD ***"
                        )

                        print(
                            f"Distance to conflict    : "
                            f"{b_remaining_to_conflict:.2f} m"
                        )

                        print(
                            "Reason                  : "
                            "Vehicle A has priority"
                        )

                        print()

                else:

                    if (
                        b_remaining_to_conflict
                        <= (
                            B_STOP_LINE_DISTANCE_M
                            + 2.0
                        )
                    ):

                        desired_b_speed = (
                            B_APPROACH_SPEED_MPS
                        )

                    else:

                        desired_b_speed = (
                            B_CRUISE_SPEED_MPS
                        )


                    (
                        b_speed,
                        b_steering,
                        b_goal,
                        b_s,
                        b_d,
                        b_goal_distance,
                    ) = self.route_command(
                        b,
                        B_START,
                        B_GOAL,
                        self.route_b,
                        desired_b_speed
                    )

            else:

                (
                    b_speed,
                    b_steering,
                    b_goal,
                    b_s,
                    b_d,
                    b_goal_distance,
                ) = self.route_command(
                    b,
                    B_START,
                    B_GOAL,
                    self.route_b,
                    B_CRUISE_SPEED_MPS
                )


                if b_goal:

                    b_speed = 0.0
                    b_steering = 0.0

                    with self.lock:

                        self.b[
                            "goal_reached"
                        ] = True


            # Vehicle B cannot complete before release in this test,
            # but keep the state correct if geometry is changed later.
            if (
                not b_released
                and (
                    b_goal_distance
                    <= GOAL_RADIUS_M
                    or b_crossed_goal
                )
            ):

                with self.lock:

                    self.b[
                        "goal_reached"
                    ] = True

                b_speed = 0.0
                b_steering = 0.0


            self.publish(
                self.pub_a,
                a_speed,
                a_steering
            )

            self.publish(
                self.pub_b,
                b_speed,
                b_steering
            )


            with self.lock:

                a_goal_reached = (
                    self.a[
                        "goal_reached"
                    ]
                )

                b_goal_reached = (
                    self.b[
                        "goal_reached"
                    ]
                )


            if (
                a_goal_reached
                and b_goal_reached
            ):

                with self.lock:

                    self.end_reason = (
                        "both_goals_reached"
                    )

                print()
                print(
                    "*** BOTH VEHICLES REACHED THEIR GOALS ***"
                )

                break


            if (
                now
                - last_print
                >= PRINT_INTERVAL_S
            ):

                coordinator_state = (
                    "B_RELEASED"
                    if b_released
                    else (
                        "B_HOLD"
                        if b_waiting
                        else "A_PRIORITY"
                    )
                )

                print(
                    f"A x={a['x']:6.2f} "
                    f"y={a['y']:6.2f} "
                    f"s={a_s:5.2f}/{self.route_a['length']:.2f} "
                    f"clr={a['min_clearance']:4.1f} | "
                    f"B x={b['x']:6.2f} "
                    f"y={b['y']:6.2f} "
                    f"s={b_s:5.2f}/{self.route_b['length']:.2f} "
                    f"clr={b['min_clearance']:4.1f} | "
                    f"coord={coordinator_state}"
                )

                last_print = now


            time.sleep(
                CONTROL_PERIOD_S
            )


        self.stop_both_repeatedly()

        duration = (
            time.monotonic()
            - start_time
        )

        return self.print_result(
            duration
        )


    # ========================================================
    # RESULT
    # ========================================================

    def write_result_csv(
        self,
        passed,
        duration
    ):

        RESULT_FILE.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        exists = (
            RESULT_FILE.exists()
        )


        row = {
            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "result":
                "PASS"
                if passed
                else "FAIL",

            "duration_s":
                f"{duration:.3f}",

            "vehicle_a_goal":
                self.a[
                    "goal_reached"
                ],

            "vehicle_b_goal":
                self.b[
                    "goal_reached"
                ],

            "coordinator_wait_events":
                self.coordinator_wait_events,

            "coordinator_release_events":
                self.coordinator_release_events,

            "conflict_overlap_violation":
                self.conflict_overlap_violation,

            "vehicle_a_min_clearance_m":
                f"{self.a['minimum_clearance_seen']:.3f}",

            "vehicle_b_min_clearance_m":
                f"{self.b['minimum_clearance_seen']:.3f}",

            "vehicle_a_max_roll_deg":
                f"{self.a['max_abs_roll_deg']:.3f}",

            "vehicle_a_max_pitch_deg":
                f"{self.a['max_abs_pitch_deg']:.3f}",

            "vehicle_b_max_roll_deg":
                f"{self.b['max_abs_roll_deg']:.3f}",

            "vehicle_b_max_pitch_deg":
                f"{self.b['max_abs_pitch_deg']:.3f}",

            "terrain_violation":
                self.terrain_violation,

            "lidar_violation":
                self.lidar_violation,

            "end_reason":
                self.end_reason,
        }


        with RESULT_FILE.open(
            "a",
            newline="",
            encoding="utf-8"
        ) as handle:

            writer = csv.DictWriter(
                handle,
                fieldnames=list(
                    row.keys()
                )
            )


            if not exists:

                writer.writeheader()


            writer.writerow(
                row
            )


    def print_result(
        self,
        duration
    ):

        with self.lock:

            a_goal = (
                self.a[
                    "goal_reached"
                ]
            )

            b_goal = (
                self.b[
                    "goal_reached"
                ]
            )

            a_min = (
                self.a[
                    "minimum_clearance_seen"
                ]
            )

            b_min = (
                self.b[
                    "minimum_clearance_seen"
                ]
            )


        safety_margin_ok = (
            a_min
            >= HARD_BODY_CLEARANCE_M
            and b_min
            >= HARD_BODY_CLEARANCE_M
        )


        passed = (
            a_goal
            and b_goal
            and self.coordinator_wait_events
            >= 1
            and self.coordinator_release_events
            == 1
            and not self.conflict_overlap_violation
            and not self.terrain_violation
            and not self.lidar_violation
            and safety_margin_ok
            and self.end_reason
            == "both_goals_reached"
        )


        self.write_result_csv(
            passed,
            duration
        )


        print()
        print(
            "============================================================"
        )

        print(
            " RUSKOTUNTURI PHASE 6 TWO VEHICLE RESULT"
        )

        print(
            "============================================================"
        )

        print(
            f"Vehicle A goal reached   : "
            f"{'YES' if a_goal else 'NO'}"
        )

        print(
            f"Vehicle B goal reached   : "
            f"{'YES' if b_goal else 'NO'}"
        )

        print(
            f"Coordinator wait events  : "
            f"{self.coordinator_wait_events}"
        )

        print(
            f"Coordinator releases     : "
            f"{self.coordinator_release_events}"
        )

        print(
            f"Conflict overlap         : "
            f"{'YES' if self.conflict_overlap_violation else 'NO'}"
        )

        print(
            f"Vehicle A min clearance  : "
            f"{a_min:.3f} m"
        )

        print(
            f"Vehicle B min clearance  : "
            f"{b_min:.3f} m"
        )

        print(
            f"Required clearance       : "
            f"{HARD_BODY_CLEARANCE_M:.3f} m"
        )

        print(
            f"Safety margin OK         : "
            f"{'YES' if safety_margin_ok else 'NO'}"
        )

        print(
            f"Vehicle A max roll       : "
            f"{self.a['max_abs_roll_deg']:.3f} deg"
        )

        print(
            f"Vehicle A max pitch      : "
            f"{self.a['max_abs_pitch_deg']:.3f} deg"
        )

        print(
            f"Vehicle B max roll       : "
            f"{self.b['max_abs_roll_deg']:.3f} deg"
        )

        print(
            f"Vehicle B max pitch      : "
            f"{self.b['max_abs_pitch_deg']:.3f} deg"
        )

        print(
            f"Terrain safety violation : "
            f"{'YES' if self.terrain_violation else 'NO'}"
        )

        print(
            f"LiDAR safety violation   : "
            f"{'YES' if self.lidar_violation else 'NO'}"
        )

        print(
            f"End reason               : "
            f"{self.end_reason}"
        )

        print(
            f"Duration                 : "
            f"{duration:.2f} s"
        )

        print()

        print(
            "RESULT: "
            + (
                "PASS"
                if passed
                else "FAIL"
            )
        )

        print(
            "============================================================"
        )


        return passed


# ============================================================
# MAIN
# ============================================================

def main():

    controller = (
        Phase6TwoVehicleCoordinator()
    )

    passed = (
        controller.run()
    )

    raise SystemExit(
        0
        if passed
        else 1
    )


if __name__ == "__main__":

    main()
