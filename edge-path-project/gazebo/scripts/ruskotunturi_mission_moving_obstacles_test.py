#!/usr/bin/env python3

import csv
import math
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


# ============================================================
# WORLD / MODEL / TOPICS
# ============================================================

WORLD_NAME = "ruskotunturi_mission_moving_world"
MODEL_NAME = "aeb_vehicle_360"

POSE_TOPIC = (
    "/world/"
    + WORLD_NAME
    + "/pose/info"
)

CMD_TOPIC = (
    "/model/"
    + MODEL_NAME
    + "/cmd_vel"
)

LIDAR_TOPIC = "/aeb360/lidar"
CAMERA_TOPIC = "/aeb360/camera/image"

MOVING_CROSSING_MODEL = "moving_crossing_obstacle"
MOVING_SLOW_MODEL = "moving_slow_obstacle"

SET_POSE_VECTOR_SERVICE = (
    "/world/"
    + WORLD_NAME
    + "/set_pose_vector"
)


# ============================================================
# MISSION ROUTE
# ============================================================

WAYPOINTS = [
    (20.000, 38.000),
    (16.000, 36.000),
    (12.000, 36.000),
    (8.000, 34.000),
    (4.000, 32.000),
    (0.000, 32.000),
    (-4.000, 28.000),
    (-8.000, 24.000),
    (-12.000, 20.000),
    (-16.000, 16.000),
    (-18.000, 12.000),
    (-18.000, 8.000),
    (-18.000, 4.000),
    (-18.000, 0.000),
    (-20.000, -4.000),
    (-22.000, -8.000),
    (-26.000, -12.000),
    (-28.000, -16.000),
    (-32.000, -20.000),
    (-36.000, -24.000),
    (-38.000, -28.000),
    (-40.000, -32.000),
    (-40.000, -36.000)
]

START_TERRAIN_Z_M = -10.516900
GOAL_TERRAIN_Z_M = 4.305869

PLANNED_ELEVATION_GAIN_M = (
    GOAL_TERRAIN_Z_M
    - START_TERRAIN_Z_M
)


# ============================================================
# MISSION SETTINGS
# ============================================================

WAYPOINT_REACHED_M = 2.0
GOAL_REACHED_M = 2.5

# Follow the route as a continuous polyline instead of steering
# directly at one waypoint at a time. This prevents the controller
# from turning back toward a waypoint that was passed during a detour.
FOLLOW_ROUTE_LOOKAHEAD_M = 3.0
RETURN_ROUTE_LOOKAHEAD_M = 3.5
RETURN_HEADING_TOLERANCE_DEG = 35.0

CRUISE_SPEED_MPS = 1.10
MODERATE_TURN_SPEED_MPS = 0.75
SHARP_TURN_SPEED_MPS = 0.50
TERRAIN_REDUCED_SPEED_MPS = 0.70

HEADING_GAIN = 1.55
MAX_STEERING_COMMAND = 0.40

MODERATE_TURN_DEG = 22.0
SHARP_TURN_DEG = 45.0

CONTROL_PERIOD_S = 0.05
PRINT_INTERVAL_S = 0.75

MISSION_TIMEOUT_S = 300.0
RESET_TIMEOUT_S = 12.0
SETTLE_TIME_S = 1.0

POSE_TIMEOUT_S = 1.0
LIDAR_TIMEOUT_S = 1.0
CAMERA_TIMEOUT_S = 1.5


# ============================================================
# TERRAIN SAFETY
# ============================================================

MAX_ABS_ROLL_DEG = 25.0
MAX_ABS_PITCH_DEG = 30.0

TERRAIN_SLOW_ROLL_DEG = 12.0
TERRAIN_SLOW_PITCH_DEG = 15.0


# ============================================================
# 360-DEGREE LIDAR / LOCAL AVOIDANCE
# ============================================================

OBSTACLE_DETECTION_RANGE_M = 6.8

# Baseline mission produced natural front-LiDAR returns down to about
# 7.18 m on this terrain. Therefore an 8 m trigger caused the hill
# itself to be classified as an obstacle. Require a clearly closer
# return and several consecutive scans before starting a detour.
OBSTACLE_CONFIRM_SCANS = 4

VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.60

MIN_BODY_CLEARANCE_M = 1.00

# 1.45 m is the preferred planning reserve, not a hard stop.
# A corridor may still be used if the predicted body clearance is
# at least 1.10 m. In that reduced-reserve case the manoeuvre is
# executed at a lower speed. The real-time 1.00 m hard clearance
# remains unchanged throughout the mission.
PLANNING_BODY_CLEARANCE_M = 1.45
MIN_PREDICTED_CORRIDOR_CLEARANCE_M = 1.10
RETURN_CORRIDOR_CLEARANCE_M = 1.10

# The local planner chooses the lateral offset dynamically.
# Smaller offsets help when the outer side corridor is constrained;
# larger offsets provide more separation from a centerline obstacle.
AVOIDANCE_OFFSET_CANDIDATES_M = (
    2.80,
    3.00,
    3.20,
    3.40,
)

AVOIDANCE_OFFSET_TOLERANCE_M = 0.35

# Predict the full swept side corridor from the current route center
# toward the requested lateral offset. A side is accepted only if the
# LiDAR point cloud leaves enough body clearance along this corridor.
CORRIDOR_LOOKAHEAD_M = 9.0
CORRIDOR_TRANSITION_FORWARD_M = 4.5
CORRIDOR_MIN_FORWARD_M = 0.8

# Planner horizon must exceed the detection trigger distance so that
# the triggering obstacle is included in swept-corridor evaluation.
if CORRIDOR_LOOKAHEAD_M <= OBSTACLE_DETECTION_RANGE_M:
    raise RuntimeError(
        "CORRIDOR_LOOKAHEAD_M must be greater than "
        "OBSTACLE_DETECTION_RANGE_M"
    )

AVOIDANCE_OUT_SPEED_MPS = 0.35
AVOIDANCE_PASS_SPEED_MPS = 0.70
AVOIDANCE_RETURN_SPEED_MPS = 0.60
REDUCED_RESERVE_SPEED_MPS = 0.45

AVOIDANCE_LOOKAHEAD_M = 1.6
AVOIDANCE_RETURN_LOOKAHEAD_M = 2.5

AVOIDANCE_HEADING_GAIN = 1.65
AVOIDANCE_MAX_STEERING = 0.40

# AVOID_OUT must create lateral separation before reaching the
# obstacle. Use a stronger but slower initial lane-change command.
AVOIDANCE_OUT_HEADING_GAIN = 2.00
AVOIDANCE_OUT_MAX_STEERING = 0.45
AVOIDANCE_URGENT_FRONT_RANGE_M = 4.5
AVOIDANCE_URGENT_OUT_SPEED_MPS = 0.25
AVOIDANCE_MIN_URGENT_LATERAL_M = 2.8

# Detection occurs roughly 6-7 m before the obstacle surface.
# Stay on the offset line until the obstacle is clearly behind.
PASS_FORWARD_FROM_TRIGGER_M = 9.0

FRONT_CLEAR_TO_RETURN_M = 4.0
RETURN_ROUTE_TOLERANCE_M = 0.55

SIDE_SCORE_TIE_M = 0.50
TIE_BREAK_DIRECTION = "RIGHT"

# If both predicted corridors are usable and their scores are close,
# use the lateral position of the primary front obstacle as an
# additional decision input. The vehicle then prefers the side away
# from the obstacle instead of using a blind RIGHT tie-break.
OBSTACLE_SIDE_BIAS_MIN_M = 0.15
OBSTACLE_SIDE_SCORE_MARGIN_M = 0.35
PRIMARY_OBSTACLE_HALF_WINDOW_M = 0.80
PRIMARY_OBSTACLE_MAX_LATERAL_M = 2.00

EMERGENCY_FRONT_RANGE_M = 0.70
AVOIDANCE_COOLDOWN_S = 1.0


# ============================================================
# MOVING-OBSTACLE TEST HARNESS / DYNAMIC RESPONSE
# ============================================================

EXPECTED_STATIC_AVOIDANCES = 0
EXPECTED_MOVING_WAIT_EVENTS = 1
EXPECTED_MOVING_OVERTAKES = 1
EXPECTED_TOTAL_AVOIDANCES = (
    EXPECTED_STATIC_AVOIDANCES
    + EXPECTED_MOVING_OVERTAKES
)

# Generic dynamic response:
# after a confirmed close obstacle is detected, stop briefly.
# If the route clears, continue without a detour. If it remains
# blocked, use the normal 360-degree local planner.
CROSSING_WAIT_TIMEOUT_S = 15.0
SLOW_WAIT_TIMEOUT_S = 4.0

# WAIT_CLEAR does not rely on one absolute range threshold because
# Ruskotunturi terrain itself can create front returns around 7 m.
# Instead, remember the closest range while waiting and require the
# front distance to open up substantially from that minimum.
DYNAMIC_CLEAR_MIN_RANGE_M = 5.8
DYNAMIC_CLEAR_DELTA_M = 1.5
DYNAMIC_CLEAR_CONFIRM_SCANS = 5

MOVING_CONTEXT_RADIUS_M = 9.0

# Scripted-pose update rate.
SCRIPTED_POSE_UPDATE_PERIOD_S = 0.20
SCRIPTED_POSE_SERVICE_TIMEOUT_MS = 2000

# Crossing obstacle test.
# It starts west of the route and moves continuously east across the
# route at y=10. The Z values follow the Ruskotunturi collision mesh.
CROSSING_INTERSECTION_X = -18.0
CROSSING_INTERSECTION_Y = 10.0
CROSSING_ACTIVATION_DISTANCE_M = 8.0
CROSSING_SPEED_MPS = 0.55

CROSSING_PATH = (
    (-23.0, 10.0, -7.527213),
    (-21.0, 10.0, -7.572128),
    (-19.0, 10.0, -7.703717),
    (-18.0, 10.0, -7.761079),
    (-17.0, 10.0, -7.794066),
    (-15.0, 10.0, -7.881089),
    (-13.0, 10.0, -7.817413),
)

# Slow same-direction obstacle test.
# It follows the lower mission route continuously once activated.
SLOW_ACTIVATION_WAYPOINT_INDEX = 1
SLOW_SPEED_MPS = 0.12
SLOW_MAX_MOVE_DURATION_S = 60.0

SLOW_PATH = (
    (6.0, 33.0, -9.487282),
    (4.0, 32.0, -9.443136),
    (0.0, 32.0, -9.502729),
    (-4.0, 28.0, -9.446475),
    (-8.0, 24.0, -9.238916),
    (-12.0, 20.0, -9.153407),
)


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "ruskotunturi_mission_moving_obstacles_results.csv"
)


# ============================================================
# SECTORS
# ============================================================

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


def normalize_degrees(
    angle_deg
):

    while angle_deg > 180.0:

        angle_deg -= 360.0


    while angle_deg < -180.0:

        angle_deg += 360.0


    return angle_deg


def quaternion_to_roll_pitch_yaw(
    q
):

    sinr_cosp = (
        2.0
        * (
            q.w * q.x
            + q.y * q.z
        )
    )

    cosr_cosp = (
        1.0
        - 2.0
        * (
            q.x * q.x
            + q.y * q.y
        )
    )

    roll = math.atan2(
        sinr_cosp,
        cosr_cosp
    )


    sinp = (
        2.0
        * (
            q.w * q.y
            - q.z * q.x
        )
    )

    sinp = max(
        -1.0,
        min(
            1.0,
            sinp
        )
    )

    pitch = math.asin(
        sinp
    )


    siny_cosp = (
        2.0
        * (
            q.w * q.z
            + q.x * q.y
        )
    )

    cosy_cosp = (
        1.0
        - 2.0
        * (
            q.y * q.y
            + q.z * q.z
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


def pose_name_matches(
    pose_name
):

    if pose_name == MODEL_NAME:

        return True


    if pose_name.endswith(
        "::" + MODEL_NAME
    ):

        return True


    return False


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


def sector_name(
    angle_deg
):

    angle_deg = normalize_degrees(
        angle_deg
    )


    if (
        -15.0
        <= angle_deg
        <= 15.0
    ):

        return "FRONT"


    if (
        15.0
        < angle_deg
        <= 60.0
    ):

        return "FRONT_LEFT"


    if (
        60.0
        < angle_deg
        <= 120.0
    ):

        return "LEFT"


    if (
        120.0
        < angle_deg
        <= 165.0
    ):

        return "REAR_LEFT"


    if (
        angle_deg
        > 165.0
        or angle_deg
        < -165.0
    ):

        return "REAR"


    if (
        -165.0
        <= angle_deg
        < -120.0
    ):

        return "REAR_RIGHT"


    if (
        -120.0
        <= angle_deg
        < -60.0
    ):

        return "RIGHT"


    return "FRONT_RIGHT"


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


# ============================================================
# CONTROLLER
# ============================================================

class RuskotunturiMovingObstacleMission:

    def __init__(
        self
    ):

        self.node = Node()
        self.lock = threading.Lock()


        self.publisher = self.node.advertise(
            CMD_TOPIC,
            Twist
        )

        if not self.node.subscribe(
            Pose_V,
            POSE_TOPIC,
            self.pose_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to world pose: "
                + POSE_TOPIC
            )


        if not self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to 360 LiDAR: "
                + LIDAR_TOPIC
            )


        if not self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.camera_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to camera: "
                + CAMERA_TOPIC
            )


        self.reset_internal_state()


    # ========================================================
    # STATE
    # ========================================================

    def reset_internal_state(
        self
    ):

        with self.lock:

            self.x = None
            self.y = None
            self.z = None

            self.roll_deg = 0.0
            self.pitch_deg = 0.0
            self.yaw_rad = 0.0

            self.pose_received = False
            self.lidar_received = False
            self.camera_received = False

            self.last_pose_time = 0.0
            self.last_lidar_time = 0.0
            self.last_camera_time = 0.0

            self.camera_frames = 0

            self.sectors = {
                name: 30.0
                for name in SECTOR_NAMES
            }

            self.sector_clearance = {
                name: math.inf
                for name in SECTOR_NAMES
            }

            self.current_min_body_clearance = math.inf
            self.minimum_body_clearance = math.inf

            # Latest valid 360-degree LiDAR points in the vehicle
            # coordinate frame: x forward, y left.
            self.lidar_points_xy = []

            self.minimum_z_m = None
            self.maximum_z_m = None

            self.max_abs_roll_deg = 0.0
            self.max_abs_pitch_deg = 0.0

            self.target_waypoint_index = 1
            self.waypoint_progress = 1

            self.mode = "FOLLOW"

            self.avoid_origin_x = None
            self.avoid_origin_y = None
            self.avoid_heading_rad = None

            self.avoid_side = None
            self.avoid_side_sign = 0.0

            self.avoid_target_offset_m = (
                max(
                    AVOIDANCE_OFFSET_CANDIDATES_M
                )
            )

            self.avoidance_directions = []
            self.completed_avoidances = 0

            self.reduced_reserve_active = False
            self.reduced_reserve_events = 0

            self.cooldown_until = 0.0

            self.obstacle_confirm_count = 0

            # Generic wait-before-avoid state.
            self.wait_started_time = None
            self.wait_clear_count = 0
            self.wait_context = "unknown"
            self.wait_min_front_range = math.inf

            # Moving obstacle poses from the world Pose_V topic.
            self.crossing_x = None
            self.crossing_y = None
            self.slow_x = None
            self.slow_y = None

            # Environment event orchestration.
            self.crossing_event_started = False
            self.crossing_event_finished = False
            self.crossing_event_start_time = None

            self.slow_event_started = False
            self.slow_event_finished = False
            self.slow_event_start_time = None
            self.slow_event_stop_elapsed = None

            self.last_scripted_pose_update = 0.0
            self.scripted_pose_service_failures = 0
            self.last_scripted_pose_error = ""

            # Evaluation counters. These do not drive ego planning.
            self.moving_wait_events_completed = 0
            self.moving_overtakes_completed = 0
            self.active_avoidance_context = "static"

            self.goal_reached = False

            self.terrain_safety_violation = False
            self.lidar_safety_violation = False
            self.no_path_detected = False
            self.emergency_stop = False


    # ========================================================
    # CALLBACKS
    # ========================================================

    def pose_callback(
        self,
        msg
    ):

        selected = None
        crossing_pose = None
        slow_pose = None


        for pose in msg.pose:

            if pose_name_matches(
                pose.name
            ):

                selected = pose


            if (
                pose.name
                == MOVING_CROSSING_MODEL
                or pose.name.endswith(
                    "::" + MOVING_CROSSING_MODEL
                )
            ):

                crossing_pose = pose


            if (
                pose.name
                == MOVING_SLOW_MODEL
                or pose.name.endswith(
                    "::" + MOVING_SLOW_MODEL
                )
            ):

                slow_pose = pose


        if selected is None:

            return


        roll, pitch, yaw = (
            quaternion_to_roll_pitch_yaw(
                selected.orientation
            )
        )


        now = time.monotonic()


        with self.lock:

            self.x = selected.position.x
            self.y = selected.position.y
            self.z = selected.position.z

            self.roll_deg = math.degrees(
                roll
            )

            self.pitch_deg = math.degrees(
                pitch
            )

            self.yaw_rad = yaw

            self.max_abs_roll_deg = max(
                self.max_abs_roll_deg,
                abs(
                    self.roll_deg
                )
            )

            self.max_abs_pitch_deg = max(
                self.max_abs_pitch_deg,
                abs(
                    self.pitch_deg
                )
            )


            if self.minimum_z_m is None:

                self.minimum_z_m = self.z
                self.maximum_z_m = self.z

            else:

                self.minimum_z_m = min(
                    self.minimum_z_m,
                    self.z
                )

                self.maximum_z_m = max(
                    self.maximum_z_m,
                    self.z
                )


            if crossing_pose is not None:

                self.crossing_x = (
                    crossing_pose.position.x
                )

                self.crossing_y = (
                    crossing_pose.position.y
                )


            if slow_pose is not None:

                self.slow_x = (
                    slow_pose.position.x
                )

                self.slow_y = (
                    slow_pose.position.y
                )


            self.pose_received = True
            self.last_pose_time = now


    def lidar_callback(
        self,
        msg
    ):

        sector_ranges = {
            name: []
            for name in SECTOR_NAMES
        }

        sector_clearances = {
            name: []
            for name in SECTOR_NAMES
        }

        minimum_clearance = math.inf

        lidar_points_xy = []


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


            angle_rad = (
                msg.angle_min
                + index
                * msg.angle_step
            )

            angle_deg = math.degrees(
                angle_rad
            )


            point_x = (
                distance
                * math.cos(
                    angle_rad
                )
            )

            point_y = (
                distance
                * math.sin(
                    angle_rad
                )
            )

            lidar_points_xy.append(
                (
                    point_x,
                    point_y
                )
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


            minimum_clearance = min(
                minimum_clearance,
                body_clearance
            )


        sectors = {}
        clearances = {}


        for name in SECTOR_NAMES:

            sectors[
                name
            ] = (
                min(
                    sector_ranges[
                        name
                    ]
                )
                if sector_ranges[
                    name
                ]
                else msg.range_max
            )


            clearances[
                name
            ] = (
                min(
                    sector_clearances[
                        name
                    ]
                )
                if sector_clearances[
                    name
                ]
                else math.inf
            )


        now = time.monotonic()


        with self.lock:

            self.sectors = sectors
            self.sector_clearance = clearances

            self.current_min_body_clearance = (
                minimum_clearance
            )

            self.lidar_points_xy = (
                lidar_points_xy
            )

            self.minimum_body_clearance = min(
                self.minimum_body_clearance,
                minimum_clearance
            )

            self.lidar_received = True
            self.last_lidar_time = now


    def camera_callback(
        self,
        msg
    ):

        now = time.monotonic()


        with self.lock:

            self.camera_frames += 1
            self.camera_received = True
            self.last_camera_time = now


    # ========================================================
    # VEHICLE COMMANDS
    # ========================================================

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


    def stop_vehicle(
        self
    ):

        self.publish_command(
            0.0,
            0.0
        )


    def stop_repeatedly(
        self
    ):

        for _ in range(
            10
        ):

            self.stop_vehicle()

            time.sleep(
                0.02
            )


    # ========================================================
    # SCRIPTED MOVING OBSTACLE CONTROL
    # ========================================================

    def path_pose_at_distance(
        self,
        path_points,
        distance_m
    ):

        if len(
            path_points
        ) < 2:

            x, y, z = (
                path_points[
                    0
                ]
            )

            return (
                x,
                y,
                z,
                0.0,
                True
            )


        remaining = max(
            0.0,
            distance_m
        )


        for index in range(
            len(
                path_points
            )
            - 1
        ):

            x0, y0, z0 = (
                path_points[
                    index
                ]
            )

            x1, y1, z1 = (
                path_points[
                    index + 1
                ]
            )


            segment_length = math.hypot(
                x1 - x0,
                y1 - y0
            )


            if segment_length <= 1e-9:

                continue


            yaw = math.atan2(
                y1 - y0,
                x1 - x0
            )


            if remaining <= segment_length:

                ratio = (
                    remaining
                    / segment_length
                )


                return (
                    x0
                    + ratio
                    * (
                        x1
                        - x0
                    ),

                    y0
                    + ratio
                    * (
                        y1
                        - y0
                    ),

                    z0
                    + ratio
                    * (
                        z1
                        - z0
                    ),

                    yaw,
                    False
                )


            remaining -= (
                segment_length
            )


        x0, y0, z0 = (
            path_points[
                -2
            ]
        )

        x1, y1, z1 = (
            path_points[
                -1
            ]
        )


        yaw = math.atan2(
            y1 - y0,
            x1 - x0
        )


        return (
            x1,
            y1,
            z1,
            yaw,
            True
        )


    def append_pose_request(
        self,
        request,
        name,
        x,
        y,
        z,
        yaw
    ):

        pose = (
            request.pose.add()
        )

        pose.name = name

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        half_yaw = (
            0.5
            * yaw
        )

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(
            half_yaw
        )

        pose.orientation.w = math.cos(
            half_yaw
        )


    def set_scripted_poses(
        self,
        poses
    ):

        request = Pose_V()


        for (
            name,
            x,
            y,
            z,
            yaw
        ) in poses:

            self.append_pose_request(
                request,
                name,
                x,
                y,
                z,
                yaw
            )


        try:

            result, response = (
                self.node.request(
                    SET_POSE_VECTOR_SERVICE,
                    request,
                    Pose_V,
                    Boolean,
                    SCRIPTED_POSE_SERVICE_TIMEOUT_MS
                )
            )

        except Exception as exc:

            with self.lock:

                self.last_scripted_pose_error = (
                    str(
                        exc
                    )
                )

            return False


        return (
            bool(
                result
            )
            and bool(
                response.data
            )
        )


    def stop_moving_obstacles(
        self
    ):

        # No velocity command is required. Scripted actors only move
        # when update_moving_obstacles() advances their pose.
        return


    def moving_context_near_ego(
        self
    ):

        with self.lock:

            x = self.x
            y = self.y

            crossing_x = self.crossing_x
            crossing_y = self.crossing_y

            slow_x = self.slow_x
            slow_y = self.slow_y

            crossing_active = (
                self.crossing_event_started
            )

            slow_active = (
                self.slow_event_started
                and not self.slow_event_finished
            )


        candidates = []


        if (
            crossing_active
            and crossing_x is not None
            and crossing_y is not None
        ):

            candidates.append(
                (
                    distance_2d(
                        x,
                        y,
                        crossing_x,
                        crossing_y
                    ),
                    "crossing"
                )
            )


        if (
            slow_active
            and slow_x is not None
            and slow_y is not None
        ):

            candidates.append(
                (
                    distance_2d(
                        x,
                        y,
                        slow_x,
                        slow_y
                    ),
                    "slow"
                )
            )


        if not candidates:

            return "static"


        distance, context = min(
            candidates,
            key=lambda item: item[0]
        )


        if distance <= MOVING_CONTEXT_RADIUS_M:

            return context


        return "static"


    def update_moving_obstacles(
        self,
        now
    ):

        with self.lock:

            x = self.x
            y = self.y

            waypoint_index = (
                self.target_waypoint_index
            )

            crossing_started = (
                self.crossing_event_started
            )

            crossing_finished = (
                self.crossing_event_finished
            )

            crossing_start_time = (
                self.crossing_event_start_time
            )

            slow_started = (
                self.slow_event_started
            )

            slow_finished = (
                self.slow_event_finished
            )

            slow_start_time = (
                self.slow_event_start_time
            )

            slow_stop_elapsed = (
                self.slow_event_stop_elapsed
            )

            moving_overtakes = (
                self.moving_overtakes_completed
            )

            last_update = (
                self.last_scripted_pose_update
            )


        # ---------------- Crossing event activation ----------------

        if (
            not crossing_started
            and distance_2d(
                x,
                y,
                CROSSING_INTERSECTION_X,
                CROSSING_INTERSECTION_Y
            )
            <= CROSSING_ACTIVATION_DISTANCE_M
        ):

            with self.lock:

                self.crossing_event_started = True
                self.crossing_event_start_time = now

            crossing_started = True
            crossing_start_time = now

            print()
            print(
                "*** TEST HARNESS: SCRIPTED CROSSING STARTED ***"
            )

            print(
                "Actor trajectory is scripted; "
                "ego response still uses LiDAR only."
            )

            print()


        # ---------------- Slow event activation ----------------

        if (
            not slow_started
            and waypoint_index
            >= SLOW_ACTIVATION_WAYPOINT_INDEX
        ):

            with self.lock:

                self.slow_event_started = True
                self.slow_event_start_time = now

            slow_started = True
            slow_start_time = now

            print()
            print(
                "*** TEST HARNESS: SCRIPTED SLOW MOVER STARTED ***"
            )

            print(
                "Actor trajectory is scripted; "
                "ego response still uses LiDAR only."
            )

            print()


        # Only make service requests at the scripted pose update rate.
        if (
            now
            - last_update
            < SCRIPTED_POSE_UPDATE_PERIOD_S
        ):

            return


        # ---------------- Crossing pose ----------------

        if crossing_started:

            crossing_elapsed = max(
                0.0,
                now
                - crossing_start_time
            )

            crossing_distance = (
                CROSSING_SPEED_MPS
                * crossing_elapsed
            )

        else:

            crossing_distance = 0.0


        (
            crossing_x,
            crossing_y,
            crossing_z,
            crossing_yaw,
            crossing_path_finished
        ) = self.path_pose_at_distance(
            CROSSING_PATH,
            crossing_distance
        )


        # ---------------- Slow pose ----------------

        if slow_started:

            if slow_stop_elapsed is not None:

                slow_elapsed = (
                    slow_stop_elapsed
                )

            else:

                slow_elapsed = min(
                    max(
                        0.0,
                        now
                        - slow_start_time
                    ),
                    SLOW_MAX_MOVE_DURATION_S
                )

        else:

            slow_elapsed = 0.0


        slow_distance = (
            SLOW_SPEED_MPS
            * slow_elapsed
        )


        (
            slow_x,
            slow_y,
            slow_z,
            slow_yaw,
            slow_path_finished
        ) = self.path_pose_at_distance(
            SLOW_PATH,
            slow_distance
        )


        # Freeze the slow mover after the expected overtake or after
        # the configured maximum scripted duration.
        if (
            slow_started
            and not slow_finished
            and (
                moving_overtakes
                >= EXPECTED_MOVING_OVERTAKES
                or slow_elapsed
                >= SLOW_MAX_MOVE_DURATION_S
                or slow_path_finished
            )
        ):

            with self.lock:

                self.slow_event_finished = True
                self.slow_event_stop_elapsed = (
                    slow_elapsed
                )

            slow_finished = True

            print(
                "*** TEST HARNESS: SLOW MOVER TRAJECTORY FINISHED ***"
            )


        poses = [
            (
                MOVING_CROSSING_MODEL,
                crossing_x,
                crossing_y,
                crossing_z,
                crossing_yaw
            ),

            (
                MOVING_SLOW_MODEL,
                slow_x,
                slow_y,
                slow_z,
                slow_yaw
            ),
        ]


        if not self.set_scripted_poses(
            poses
        ):

            with self.lock:

                self.scripted_pose_service_failures += 1

                failures = (
                    self.scripted_pose_service_failures
                )


            if (
                failures
                <= 3
                or failures
                % 10
                == 0
            ):

                with self.lock:

                    last_error = (
                        self.last_scripted_pose_error
                    )


                if last_error:

                    print(
                        f"WARNING: scripted pose service failed "
                        f"({failures}): {last_error}"
                    )

                else:

                    print(
                        f"WARNING: scripted pose service failed "
                        f"({failures})"
                    )


                if failures == 3:

                    print(
                        "Check that the world contains the "
                        "UserCommands system and that this service "
                        "exists:"
                    )

                    print(
                        f"  {SET_POSE_VECTOR_SERVICE}"
                    )


        with self.lock:

            self.last_scripted_pose_update = now


            # Use the commanded pose immediately for scenario context;
            # Pose_V callbacks will independently confirm world state.
            self.crossing_x = crossing_x
            self.crossing_y = crossing_y

            self.slow_x = slow_x
            self.slow_y = slow_y


            if (
                crossing_started
                and crossing_path_finished
                and not self.crossing_event_finished
            ):

                self.crossing_event_finished = True

                print(
                    "*** TEST HARNESS: CROSSING TRAJECTORY FINISHED ***"
                )


    # ========================================================
    # DYNAMIC OBSTACLE RESPONSE
    # ========================================================

    def start_obstacle_wait(
        self,
        now
    ):

        context = (
            self.moving_context_near_ego()
        )


        with self.lock:

            self.mode = "WAIT_CLEAR"
            self.wait_started_time = now
            self.wait_clear_count = 0
            self.wait_context = context

            self.wait_min_front_range = (
                self.sectors[
                    "FRONT"
                ]
            )

            self.obstacle_confirm_count = 0


        print()
        print(
            "*** CONFIRMED OBSTACLE - WAITING FOR CLEARANCE ***"
        )

        print(
            f"Evaluation context       : "
            f"{context}"
        )

        if context == "crossing":

            wait_timeout_s = (
                CROSSING_WAIT_TIMEOUT_S
            )

        else:

            wait_timeout_s = (
                SLOW_WAIT_TIMEOUT_S
            )


        print(
            f"Wait timeout             : "
            f"{wait_timeout_s:.1f} s"
        )

        print()


    def dynamic_wait_control(
        self,
        now
    ):

        with self.lock:

            front_range = self.sectors[
                "FRONT"
            ]

            wait_started_time = (
                self.wait_started_time
            )

            wait_clear_count = (
                self.wait_clear_count
            )

            context = (
                self.wait_context
            )

            wait_min_front_range = (
                self.wait_min_front_range
            )


        wait_min_front_range = min(
            wait_min_front_range,
            front_range
        )


        clear_threshold = max(
            DYNAMIC_CLEAR_MIN_RANGE_M,
            wait_min_front_range
            + DYNAMIC_CLEAR_DELTA_M
        )


        if (
            front_range
            >= clear_threshold
        ):

            wait_clear_count += 1

        else:

            wait_clear_count = 0


        with self.lock:

            self.wait_clear_count = (
                wait_clear_count
            )

            self.wait_min_front_range = (
                wait_min_front_range
            )


        if (
            wait_clear_count
            >= DYNAMIC_CLEAR_CONFIRM_SCANS
        ):

            print()
            print(
                "*** LIDAR FRONT CORRIDOR OPENED ***"
            )

            print(
                f"Closest wait range       : "
                f"{wait_min_front_range:.2f} m"
            )

            print(
                f"Current front range      : "
                f"{front_range:.2f} m"
            )

            print(
                f"Clear threshold          : "
                f"{clear_threshold:.2f} m"
            )


            with self.lock:

                self.mode = "FOLLOW"
                self.cooldown_until = (
                    now
                    + AVOIDANCE_COOLDOWN_S
                )

                if context == "crossing":

                    self.moving_wait_events_completed += 1

                completed_waits = (
                    self.moving_wait_events_completed
                )


            print()
            print(
                "*** ROUTE CLEARED - CONTINUING WITHOUT DETOUR ***"
            )

            print(
                f"Context                  : "
                f"{context}"
            )

            print(
                f"Moving wait events       : "
                f"{completed_waits}/"
                f"{EXPECTED_MOVING_WAIT_EVENTS}"
            )

            print()


            return (
                "cleared",
                0.0,
                0.0
            )


        if context == "crossing":

            wait_timeout_s = (
                CROSSING_WAIT_TIMEOUT_S
            )

        else:

            wait_timeout_s = (
                SLOW_WAIT_TIMEOUT_S
            )


        if (
            wait_started_time is not None
            and (
                now
                - wait_started_time
            )
            >= wait_timeout_s
        ):

            print()
            print(
                "*** OBSTACLE PERSISTENT - LOCAL DETOUR REQUIRED ***"
            )

            print(
                f"Context                  : "
                f"{context}"
            )


            avoidance_choice = (
                self.choose_avoidance_side()
            )


            if avoidance_choice is None:

                return (
                    "no_path",
                    0.0,
                    0.0
                )


            (
                direction,
                target_offset_m
            ) = avoidance_choice


            self.start_avoidance(
                direction,
                target_offset_m,
                context=context
            )


            speed, steering = (
                self.avoidance_control()
            )


            return (
                "avoid",
                speed,
                steering
            )


        if (
            int(
                (
                    now
                    - wait_started_time
                )
                * 2.0
            )
            % 4
            == 0
        ):

            print(
                f"WAIT_CLEAR | "
                f"context={context} | "
                f"front={front_range:.2f} m | "
                f"min={wait_min_front_range:.2f} m | "
                f"clear>={clear_threshold:.2f} m | "
                f"confirm={wait_clear_count}/"
                f"{DYNAMIC_CLEAR_CONFIRM_SCANS}"
            )


        return (
            "waiting",
            0.0,
            0.0
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
            "Resetting Phase 3 moving-obstacle mission world..."
        )


        self.stop_repeatedly()
        self.stop_moving_obstacles()
        self.reset_internal_state()


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


        start = time.monotonic()


        while True:

            if (
                time.monotonic()
                - start
                > RESET_TIMEOUT_S
            ):

                print(
                    "ERROR: reset timeout"
                )

                return False


            with self.lock:

                ready = (
                    self.pose_received
                    and self.lidar_received
                    and self.camera_received
                    and self.x is not None
                )


            if ready:

                time.sleep(
                    SETTLE_TIME_S
                )


                with self.lock:

                    self.minimum_z_m = self.z
                    self.maximum_z_m = self.z

                    self.max_abs_roll_deg = abs(
                        self.roll_deg
                    )

                    self.max_abs_pitch_deg = abs(
                        self.pitch_deg
                    )

                    self.minimum_body_clearance = (
                        self.current_min_body_clearance
                    )


                print(
                    f"Reset OK | "
                    f"x={self.x:.2f}, "
                    f"y={self.y:.2f}, "
                    f"z={self.z:.2f}"
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # WAYPOINT ROUTE / POLYLINE FOLLOWING
    # ========================================================

    def closest_future_route_projection(
        self,
        x=None,
        y=None
    ):

        if (
            x is None
            or y is None
        ):

            with self.lock:

                x = self.x
                y = self.y

                current_target = (
                    self.target_waypoint_index
                )

        else:

            with self.lock:

                current_target = (
                    self.target_waypoint_index
                )


        # Never search segments behind the current mission progress.
        # This makes route progress monotonic and prevents the vehicle
        # from trying to return to an already passed waypoint.
        first_segment = max(
            0,
            min(
                len(
                    WAYPOINTS
                )
                - 2,
                current_target
                - 1
            )
        )


        best = None


        for segment_index in range(
            first_segment,
            len(
                WAYPOINTS
            )
            - 1
        ):

            x0, y0 = (
                WAYPOINTS[
                    segment_index
                ]
            )

            x1, y1 = (
                WAYPOINTS[
                    segment_index
                    + 1
                ]
            )


            dx = (
                x1
                - x0
            )

            dy = (
                y1
                - y0
            )


            length_sq = (
                dx
                * dx
                + dy
                * dy
            )


            if length_sq <= 1e-9:

                continue


            t = clamp(
                (
                    (
                        x
                        - x0
                    )
                    * dx
                    + (
                        y
                        - y0
                    )
                    * dy
                )
                / length_sq,
                0.0,
                1.0
            )


            projection_x = (
                x0
                + t
                * dx
            )

            projection_y = (
                y0
                + t
                * dy
            )


            distance = distance_2d(
                x,
                y,
                projection_x,
                projection_y
            )


            if (
                best is None
                or distance
                < best[
                    "distance"
                ]
            ):

                best = {
                    "segment_index":
                        segment_index,

                    "t":
                        t,

                    "x":
                        projection_x,

                    "y":
                        projection_y,

                    "distance":
                        distance,

                    "heading":
                        math.atan2(
                            dy,
                            dx
                        ),
                }


        return best


    def route_lookahead_target(
        self,
        lookahead_m
    ):

        projection = (
            self.closest_future_route_projection()
        )


        if projection is None:

            with self.lock:

                index = (
                    self.target_waypoint_index
                )


            target_x, target_y = (
                WAYPOINTS[
                    index
                ]
            )


            return (
                target_x,
                target_y,
                math.inf,
                max(
                    0,
                    index - 1
                ),
                self.route_heading_for_current_target()
            )


        segment_index = (
            projection[
                "segment_index"
            ]
        )

        current_x = (
            projection[
                "x"
            ]
        )

        current_y = (
            projection[
                "y"
            ]
        )

        remaining = max(
            0.0,
            lookahead_m
        )

        target_heading = (
            projection[
                "heading"
            ]
        )


        while (
            segment_index
            < len(
                WAYPOINTS
            )
            - 1
        ):

            end_x, end_y = (
                WAYPOINTS[
                    segment_index
                    + 1
                ]
            )


            distance_to_end = (
                distance_2d(
                    current_x,
                    current_y,
                    end_x,
                    end_y
                )
            )


            if (
                remaining
                <= distance_to_end
                and distance_to_end
                > 1e-9
            ):

                ratio = (
                    remaining
                    / distance_to_end
                )


                target_x = (
                    current_x
                    + ratio
                    * (
                        end_x
                        - current_x
                    )
                )

                target_y = (
                    current_y
                    + ratio
                    * (
                        end_y
                        - current_y
                    )
                )


                target_heading = math.atan2(
                    end_y
                    - current_y,
                    end_x
                    - current_x
                )


                return (
                    target_x,
                    target_y,
                    projection[
                        "distance"
                    ],
                    segment_index,
                    target_heading
                )


            remaining -= (
                distance_to_end
            )

            current_x = end_x
            current_y = end_y

            segment_index += 1


            if (
                segment_index
                >= len(
                    WAYPOINTS
                )
                - 1
            ):

                break


        target_x, target_y = (
            WAYPOINTS[
                -1
            ]
        )


        if len(
            WAYPOINTS
        ) >= 2:

            previous_x, previous_y = (
                WAYPOINTS[
                    -2
                ]
            )

            target_heading = math.atan2(
                target_y
                - previous_y,
                target_x
                - previous_x
            )


        return (
            target_x,
            target_y,
            projection[
                "distance"
            ],
            len(
                WAYPOINTS
            )
            - 2,
            target_heading
        )


    def synchronize_route_progress(
        self,
        announce=False
    ):

        with self.lock:

            x = self.x
            y = self.y

            old_target = (
                self.target_waypoint_index
            )


        projection = (
            self.closest_future_route_projection(
                x,
                y
            )
        )


        if projection is None:

            return


        # If the closest point lies on segment i -> i+1, waypoint i
        # is already behind or alongside us and i+1 is the next
        # meaningful route target.
        new_target = max(
            old_target,
            projection[
                "segment_index"
            ]
            + 1
        )


        # If the end of that segment is already very close, advance
        # one more waypoint instead of commanding a turn back to it.
        while (
            new_target
            < len(
                WAYPOINTS
            )
            - 1
        ):

            target_x, target_y = (
                WAYPOINTS[
                    new_target
                ]
            )


            if (
                distance_2d(
                    x,
                    y,
                    target_x,
                    target_y
                )
                > WAYPOINT_REACHED_M
            ):

                break


            new_target += 1


        new_target = min(
            new_target,
            len(
                WAYPOINTS
            )
            - 1
        )


        with self.lock:

            self.target_waypoint_index = (
                new_target
            )

            # target index N means waypoint indices 0..N-1 are
            # already behind the vehicle.
            self.waypoint_progress = max(
                self.waypoint_progress,
                new_target
            )


        if (
            announce
            and new_target
            != old_target
        ):

            print(
                f"Route resync -> target waypoint "
                f"{new_target + 1}/"
                f"{len(WAYPOINTS)}"
            )


    def advance_waypoint_if_reached(
        self
    ):

        self.synchronize_route_progress(
            announce=False
        )


        with self.lock:

            x = self.x
            y = self.y

            index = (
                self.target_waypoint_index
            )


        goal_x, goal_y = (
            WAYPOINTS[
                -1
            ]
        )


        goal_distance = (
            distance_2d(
                x,
                y,
                goal_x,
                goal_y
            )
        )


        if (
            index
            == len(
                WAYPOINTS
            )
            - 1
            and goal_distance
            <= GOAL_REACHED_M
        ):

            with self.lock:

                self.waypoint_progress = len(
                    WAYPOINTS
                )

                self.goal_reached = True


            print()
            print(
                "*** MISSION GOAL AREA REACHED ***"
            )


    def resync_waypoint_after_avoidance(
        self
    ):

        self.synchronize_route_progress(
            announce=True
        )


    def route_heading_for_current_target(
        self
    ):

        with self.lock:

            index = (
                self.target_waypoint_index
            )


        previous_index = max(
            0,
            index - 1
        )


        x0, y0 = (
            WAYPOINTS[
                previous_index
            ]
        )

        x1, y1 = (
            WAYPOINTS[
                index
            ]
        )


        return math.atan2(
            y1 - y0,
            x1 - x0
        )


    def follow_route_control(
        self
    ):

        with self.lock:

            x = self.x
            y = self.y
            yaw = self.yaw_rad

            roll_deg = self.roll_deg
            pitch_deg = self.pitch_deg


        (
            target_x,
            target_y,
            route_distance,
            _segment_index,
            _route_heading
        ) = self.route_lookahead_target(
            FOLLOW_ROUTE_LOOKAHEAD_M
        )


        desired_heading = math.atan2(
            target_y
            - y,
            target_x
            - x
        )


        heading_error = wrap_angle(
            desired_heading
            - yaw
        )


        heading_error_deg = abs(
            math.degrees(
                heading_error
            )
        )


        steering = clamp(
            HEADING_GAIN
            * heading_error,
            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND
        )


        if (
            heading_error_deg
            >= SHARP_TURN_DEG
        ):

            speed = (
                SHARP_TURN_SPEED_MPS
            )


        elif (
            heading_error_deg
            >= MODERATE_TURN_DEG
        ):

            speed = (
                MODERATE_TURN_SPEED_MPS
            )


        else:

            speed = (
                CRUISE_SPEED_MPS
            )


        if (
            abs(
                roll_deg
            )
            >= TERRAIN_SLOW_ROLL_DEG
            or abs(
                pitch_deg
            )
            >= TERRAIN_SLOW_PITCH_DEG
        ):

            speed = min(
                speed,
                TERRAIN_REDUCED_SPEED_MPS
            )


        return (
            speed,
            steering,
            heading_error_deg
        )


    # ========================================================
    # LOCAL AVOIDANCE FRAME
    # ========================================================

    def avoidance_local_coordinates(
        self
    ):

        with self.lock:

            x = self.x
            y = self.y

            origin_x = self.avoid_origin_x
            origin_y = self.avoid_origin_y
            heading = self.avoid_heading_rad


        dx = (
            x
            - origin_x
        )

        dy = (
            y
            - origin_y
        )


        cos_h = math.cos(
            heading
        )

        sin_h = math.sin(
            heading
        )


        forward = (
            dx
            * cos_h
            + dy
            * sin_h
        )

        lateral = (
            -dx
            * sin_h
            + dy
            * cos_h
        )


        return (
            forward,
            lateral
        )


    def avoidance_local_to_world(
        self,
        forward,
        lateral
    ):

        with self.lock:

            origin_x = self.avoid_origin_x
            origin_y = self.avoid_origin_y
            heading = self.avoid_heading_rad


        cos_h = math.cos(
            heading
        )

        sin_h = math.sin(
            heading
        )


        x = (
            origin_x
            + forward
            * cos_h
            - lateral
            * sin_h
        )

        y = (
            origin_y
            + forward
            * sin_h
            + lateral
            * cos_h
        )


        return (
            x,
            y
        )


    def steering_to_world_target(
        self,
        target_x,
        target_y,
        heading_gain=None,
        max_steering=None
    ):

        with self.lock:

            x = self.x
            y = self.y
            yaw = self.yaw_rad


        if heading_gain is None:

            heading_gain = (
                AVOIDANCE_HEADING_GAIN
            )


        if max_steering is None:

            max_steering = (
                AVOIDANCE_MAX_STEERING
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
            heading_gain
            * heading_error,
            -max_steering,
            max_steering
        )


        return steering


    # ========================================================
    # 360 SWEPT-CORRIDOR ASSESSMENT
    # ========================================================

    def swept_corridor_clearance(
        self,
        direction,
        offset_m
    ):

        # LiDAR points are measured in the VEHICLE frame, but the
        # predicted avoidance corridor is defined relative to the
        # current mission-route segment. Transform every point:
        #
        # vehicle frame -> world delta -> route frame
        #
        # This is important in waypoint turns where vehicle yaw and
        # route heading are not exactly equal.

        route_heading = (
            self.route_heading_for_current_target()
        )


        with self.lock:

            points = list(
                self.lidar_points_xy
            )

            vehicle_yaw = self.yaw_rad


        if direction == "LEFT":

            side_sign = 1.0

        else:

            side_sign = -1.0


        cos_yaw = math.cos(
            vehicle_yaw
        )

        sin_yaw = math.sin(
            vehicle_yaw
        )

        cos_route = math.cos(
            route_heading
        )

        sin_route = math.sin(
            route_heading
        )


        minimum_clearance = math.inf
        limiting_point = None


        for vehicle_x, vehicle_y in points:

            # Vehicle-local LiDAR point -> world-oriented delta.
            world_dx = (
                vehicle_x
                * cos_yaw
                - vehicle_y
                * sin_yaw
            )

            world_dy = (
                vehicle_x
                * sin_yaw
                + vehicle_y
                * cos_yaw
            )


            # World-oriented delta -> current route frame.
            route_forward = (
                world_dx
                * cos_route
                + world_dy
                * sin_route
            )

            route_lateral = (
                -world_dx
                * sin_route
                + world_dy
                * cos_route
            )


            if (
                route_forward
                < CORRIDOR_MIN_FORWARD_M
                or route_forward
                > CORRIDOR_LOOKAHEAD_M
            ):

                continue


            transition_ratio = clamp(
                route_forward
                / CORRIDOR_TRANSITION_FORWARD_M,
                0.0,
                1.0
            )


            corridor_center_lateral = (
                side_sign
                * offset_m
                * transition_ratio
            )


            clearance = (
                abs(
                    route_lateral
                    - corridor_center_lateral
                )
                - VEHICLE_HALF_WIDTH_M
            )


            if clearance < minimum_clearance:

                minimum_clearance = clearance

                limiting_point = (
                    route_forward,
                    route_lateral,
                    corridor_center_lateral
                )


        return (
            minimum_clearance,
            limiting_point
        )


    def best_offset_for_side(
        self,
        direction,
        sector_score
    ):

        evaluations = []


        for offset_m in (
            AVOIDANCE_OFFSET_CANDIDATES_M
        ):

            corridor_score, limiting_point = (
                self.swept_corridor_clearance(
                    direction,
                    offset_m
                )
            )


            total_score = min(
                sector_score,
                corridor_score
            )


            evaluations.append(
                (
                    total_score,
                    corridor_score,
                    offset_m,
                    limiting_point
                )
            )


        # Prefer the candidate with the highest predicted minimum
        # clearance. If scores are equal, prefer the smaller offset
        # to reduce unnecessary route deviation.
        evaluations.sort(
            key=lambda item: (
                item[0],
                -item[2]
            ),
            reverse=True
        )


        return evaluations[
            0
        ], evaluations


    def primary_front_obstacle_lateral(
        self
    ):

        # Estimate the lateral position of the closest object that
        # caused the front obstacle trigger.
        #
        # LiDAR points:
        # vehicle frame -> world-oriented delta -> route frame.

        route_heading = (
            self.route_heading_for_current_target()
        )


        with self.lock:

            points = list(
                self.lidar_points_xy
            )

            vehicle_yaw = self.yaw_rad


        cos_yaw = math.cos(
            vehicle_yaw
        )

        sin_yaw = math.sin(
            vehicle_yaw
        )

        cos_route = math.cos(
            route_heading
        )

        sin_route = math.sin(
            route_heading
        )


        candidates = []


        for vehicle_x, vehicle_y in points:

            world_dx = (
                vehicle_x
                * cos_yaw
                - vehicle_y
                * sin_yaw
            )

            world_dy = (
                vehicle_x
                * sin_yaw
                + vehicle_y
                * cos_yaw
            )


            route_forward = (
                world_dx
                * cos_route
                + world_dy
                * sin_route
            )

            route_lateral = (
                -world_dx
                * sin_route
                + world_dy
                * cos_route
            )


            if (
                route_forward
                < 0.5
                or route_forward
                > (
                    OBSTACLE_DETECTION_RANGE_M
                    + 1.0
                )
            ):

                continue


            if (
                abs(
                    route_lateral
                )
                > PRIMARY_OBSTACLE_MAX_LATERAL_M
            ):

                continue


            candidates.append(
                (
                    route_forward,
                    route_lateral
                )
            )


        if not candidates:

            return None


        closest_forward = min(
            item[0]
            for item in candidates
        )


        cluster_lateral = [
            lateral
            for forward, lateral
            in candidates
            if forward
            <= (
                closest_forward
                + PRIMARY_OBSTACLE_HALF_WINDOW_M
            )
        ]


        if not cluster_lateral:

            return None


        return (
            sum(
                cluster_lateral
            )
            / len(
                cluster_lateral
            )
        )


    # ========================================================
    # 360 SIDE ASSESSMENT
    # ========================================================

    def choose_avoidance_side(
        self
    ):

        with self.lock:

            clearance = dict(
                self.sector_clearance
            )

            sectors = dict(
                self.sectors
            )


        left_sector_score = min(
            clearance[
                "FRONT_LEFT"
            ],
            clearance[
                "LEFT"
            ],
        )

        right_sector_score = min(
            clearance[
                "FRONT_RIGHT"
            ],
            clearance[
                "RIGHT"
            ],
        )


        (
            left_best,
            left_evaluations
        ) = self.best_offset_for_side(
            "LEFT",
            left_sector_score
        )

        (
            right_best,
            right_evaluations
        ) = self.best_offset_for_side(
            "RIGHT",
            right_sector_score
        )


        (
            left_score,
            left_corridor,
            left_offset,
            left_point
        ) = left_best


        (
            right_score,
            right_corridor,
            right_offset,
            right_point
        ) = right_best


        left_preferred = (
            left_score
            >= PLANNING_BODY_CLEARANCE_M
        )

        right_preferred = (
            right_score
            >= PLANNING_BODY_CLEARANCE_M
        )


        left_usable = (
            left_score
            >= MIN_PREDICTED_CORRIDOR_CLEARANCE_M
        )

        right_usable = (
            right_score
            >= MIN_PREDICTED_CORRIDOR_CLEARANCE_M
        )


        primary_obstacle_lateral = (
            self.primary_front_obstacle_lateral()
        )


        print()
        print(
            "*** STATIC OBSTACLE DETECTED ***"
        )

        print(
            f"Front range             : "
            f"{sectors['FRONT']:.2f} m"
        )

        print(
            f"LEFT sector clearance   : "
            f"{left_sector_score:.2f} m"
        )

        print(
            f"LEFT best corridor      : "
            f"{left_corridor:.2f} m "
            f"@ offset {left_offset:.2f} m"
        )

        print(
            "LEFT offset candidates  : "
            + ", ".join(
                f"{item[2]:.1f}m->{item[0]:.2f}m"
                for item in left_evaluations
            )
        )

        print(
            f"RIGHT sector clearance  : "
            f"{right_sector_score:.2f} m"
        )

        print(
            f"RIGHT best corridor     : "
            f"{right_corridor:.2f} m "
            f"@ offset {right_offset:.2f} m"
        )

        print(
            "RIGHT offset candidates : "
            + ", ".join(
                f"{item[2]:.1f}m->{item[0]:.2f}m"
                for item in right_evaluations
            )
        )


        if left_point is not None:

            print(
                f"LEFT limiting point     : "
                f"route s={left_point[0]:.2f}, "
                f"d={left_point[1]:+.2f}"
            )


        if right_point is not None:

            print(
                f"RIGHT limiting point    : "
                f"route s={right_point[0]:.2f}, "
                f"d={right_point[1]:+.2f}"
            )


        print(
            f"Preferred reserve       : "
            f"{PLANNING_BODY_CLEARANCE_M:.2f} m"
        )

        print(
            f"Minimum predicted safe  : "
            f"{MIN_PREDICTED_CORRIDOR_CLEARANCE_M:.2f} m"
        )


        if primary_obstacle_lateral is None:

            print(
                "Primary obstacle lateral: UNKNOWN"
            )

        else:

            print(
                f"Primary obstacle lateral: "
                f"{primary_obstacle_lateral:+.2f} m "
                f"(+LEFT / -RIGHT)"
            )


        direction = None
        chosen_score = None
        chosen_offset = None


        if (
            not left_usable
            and not right_usable
        ):

            print(
                "Decision                : NO SAFE SIDE"
            )

            print()

            return None


        if (
            left_usable
            and not right_usable
        ):

            direction = "LEFT"
            chosen_score = left_score
            chosen_offset = left_offset


        elif (
            right_usable
            and not left_usable
        ):

            direction = "RIGHT"
            chosen_score = right_score
            chosen_offset = right_offset


        else:

            # Both sides are usable.
            #
            # If the corridor scores are reasonably close, prefer the
            # side AWAY from the primary obstacle. This avoids cases
            # where a tiny numerical score advantage sends the vehicle
            # around the tighter side of an off-centre obstacle.

            side_aware_direction = None


            if (
                primary_obstacle_lateral
                is not None
            ):

                if (
                    primary_obstacle_lateral
                    <= -OBSTACLE_SIDE_BIAS_MIN_M
                ):

                    # Obstacle is right of route center -> go LEFT.
                    side_aware_direction = "LEFT"


                elif (
                    primary_obstacle_lateral
                    >= OBSTACLE_SIDE_BIAS_MIN_M
                ):

                    # Obstacle is left of route center -> go RIGHT.
                    side_aware_direction = "RIGHT"


            score_difference = abs(
                left_score
                - right_score
            )


            if (
                side_aware_direction
                is not None
                and score_difference
                <= OBSTACLE_SIDE_SCORE_MARGIN_M
            ):

                direction = (
                    side_aware_direction
                )

                chosen_score = (
                    left_score
                    if direction == "LEFT"
                    else right_score
                )

                chosen_offset = (
                    left_offset
                    if direction == "LEFT"
                    else right_offset
                )


            elif (
                left_preferred
                and not right_preferred
            ):

                direction = "LEFT"
                chosen_score = left_score
                chosen_offset = left_offset


            elif (
                right_preferred
                and not left_preferred
            ):

                direction = "RIGHT"
                chosen_score = right_score
                chosen_offset = right_offset


            elif (
                abs(
                    left_score
                    - right_score
                )
                <= SIDE_SCORE_TIE_M
            ):

                direction = (
                    TIE_BREAK_DIRECTION
                )

                chosen_score = (
                    left_score
                    if direction == "LEFT"
                    else right_score
                )

                chosen_offset = (
                    left_offset
                    if direction == "LEFT"
                    else right_offset
                )


            elif left_score > right_score:

                direction = "LEFT"
                chosen_score = left_score
                chosen_offset = left_offset


            else:

                direction = "RIGHT"
                chosen_score = right_score
                chosen_offset = right_offset


        if (
            direction is None
            or chosen_score is None
            or chosen_offset is None
        ):

            raise RuntimeError(
                "Avoidance planner failed to produce "
                "a complete direction/score/offset decision"
            )


        reduced_reserve = (
            chosen_score
            < PLANNING_BODY_CLEARANCE_M
        )


        with self.lock:

            self.reduced_reserve_active = (
                reduced_reserve
            )

            self.avoid_target_offset_m = (
                chosen_offset
            )

            if reduced_reserve:

                self.reduced_reserve_events += 1


        print(
            f"Decision                : "
            f"{direction}"
        )

        print(
            f"Chosen predicted clear. : "
            f"{chosen_score:.2f} m"
        )

        print(
            f"Chosen lateral offset   : "
            f"{chosen_offset:.2f} m"
        )

        print(
            f"Reserve mode            : "
            f"{'REDUCED / SLOW' if reduced_reserve else 'PREFERRED'}"
        )

        print()


        return (
            direction,
            chosen_offset
        )


    def return_corridor_clear(
        self
    ):

        with self.lock:

            clearance = dict(
                self.sector_clearance
            )

            side = self.avoid_side


        if side == "LEFT":

            score = min(
                clearance[
                    "FRONT_RIGHT"
                ],
                clearance[
                    "RIGHT"
                ],
                clearance[
                    "REAR_RIGHT"
                ],
            )


        else:

            score = min(
                clearance[
                    "FRONT_LEFT"
                ],
                clearance[
                    "LEFT"
                ],
                clearance[
                    "REAR_LEFT"
                ],
            )


        return (
            score
            >= RETURN_CORRIDOR_CLEARANCE_M
        )


    # ========================================================
    # AVOIDANCE STATE MACHINE
    # ========================================================

    def start_avoidance(
        self,
        direction,
        target_offset_m,
        context="static"
    ):

        # IMPORTANT:
        # Do not call route_heading_for_current_target() while
        # holding self.lock. That function also acquires self.lock.
        # With threading.Lock this would deadlock the controller and
        # Gazebo would keep executing the last steering command.

        avoid_heading_rad = (
            self.route_heading_for_current_target()
        )


        with self.lock:

            self.avoid_origin_x = self.x
            self.avoid_origin_y = self.y

            self.avoid_heading_rad = (
                avoid_heading_rad
            )

            self.avoid_side = direction

            self.avoid_side_sign = (
                1.0
                if direction == "LEFT"
                else -1.0
            )

            self.avoid_target_offset_m = (
                target_offset_m
            )

            self.mode = "AVOID_OUT"

            self.active_avoidance_context = (
                context
            )

            self.obstacle_confirm_count = 0

            self.avoidance_directions.append(
                direction
            )

            origin_x = self.avoid_origin_x
            origin_y = self.avoid_origin_y


        print(
            f"*** LOCAL DETOUR STARTED: "
            f"{direction} ***"
        )

        print(
            f"Detour origin           : "
            f"x={origin_x:.2f}, "
            f"y={origin_y:.2f}"
        )

        print(
            f"Detour route heading    : "
            f"{math.degrees(avoid_heading_rad):.2f} deg"
        )

        print(
            f"Detour target offset    : "
            f"{target_offset_m:.2f} m"
        )

        print(
            f"Detour context          : "
            f"{context}"
        )


    def avoidance_control(
        self
    ):

        with self.lock:

            mode = self.mode
            side_sign = self.avoid_side_sign

            front_range = self.sectors[
                "FRONT"
            ]

            reduced_reserve = (
                self.reduced_reserve_active
            )

            selected_offset_m = (
                self.avoid_target_offset_m
            )


        forward, lateral = (
            self.avoidance_local_coordinates()
        )


        target_lateral = (
            side_sign
            * selected_offset_m
        )


        if mode == "AVOID_OUT":

            if (
                abs(
                    lateral
                    - target_lateral
                )
                <= AVOIDANCE_OFFSET_TOLERANCE_M
            ):

                with self.lock:

                    self.mode = "AVOID_PASS"


                mode = "AVOID_PASS"

                print(
                    "*** OFFSET CORRIDOR REACHED ***"
                )


            speed = (
                AVOIDANCE_OUT_SPEED_MPS
            )


            # If the obstacle is getting close but the vehicle has
            # not yet created enough lateral separation, slow down
            # further instead of continuing forward at the nominal
            # lane-change speed.
            if (
                front_range
                <= AVOIDANCE_URGENT_FRONT_RANGE_M
                and abs(
                    lateral
                )
                < AVOIDANCE_MIN_URGENT_LATERAL_M
            ):

                speed = min(
                    speed,
                    AVOIDANCE_URGENT_OUT_SPEED_MPS
                )


            if reduced_reserve:

                speed = min(
                    speed,
                    REDUCED_RESERVE_SPEED_MPS
                )


            target_forward = (
                forward
                + AVOIDANCE_LOOKAHEAD_M
            )

            target_d = (
                target_lateral
            )


            target_x, target_y = (
                self.avoidance_local_to_world(
                    target_forward,
                    target_d
                )
            )


            steering = (
                self.steering_to_world_target(
                    target_x,
                    target_y,
                    heading_gain=(
                        AVOIDANCE_OUT_HEADING_GAIN
                    ),
                    max_steering=(
                        AVOIDANCE_OUT_MAX_STEERING
                    )
                )
            )


            return (
                speed,
                steering
            )


        elif mode == "AVOID_PASS":

            if (
                forward
                >= PASS_FORWARD_FROM_TRIGGER_M
                and front_range
                >= FRONT_CLEAR_TO_RETURN_M
                and self.return_corridor_clear()
            ):

                with self.lock:

                    self.mode = "AVOID_RETURN"


                mode = "AVOID_RETURN"

                print(
                    "*** OBSTACLE PASSED - RETURNING TO MISSION ROUTE ***"
                )


            if mode == "AVOID_PASS":

                speed = (
                    AVOIDANCE_PASS_SPEED_MPS
                )


                if reduced_reserve:

                    speed = min(
                        speed,
                        REDUCED_RESERVE_SPEED_MPS
                    )


                target_forward = (
                    forward
                    + AVOIDANCE_LOOKAHEAD_M
                )

                target_d = (
                    target_lateral
                )


                target_x, target_y = (
                    self.avoidance_local_to_world(
                        target_forward,
                        target_d
                    )
                )


                steering = (
                    self.steering_to_world_target(
                        target_x,
                        target_y
                    )
                )


                return (
                    speed,
                    steering
                )


        if mode == "AVOID_RETURN":

            (
                target_x,
                target_y,
                route_distance,
                route_segment,
                route_heading
            ) = self.route_lookahead_target(
                RETURN_ROUTE_LOOKAHEAD_M
            )


            with self.lock:

                yaw = self.yaw_rad


            route_heading_error_deg = abs(
                math.degrees(
                    wrap_angle(
                        route_heading
                        - yaw
                    )
                )
            )


            steering = (
                self.steering_to_world_target(
                    target_x,
                    target_y
                )
            )


            if (
                route_distance
                <= RETURN_ROUTE_TOLERANCE_M
                and route_heading_error_deg
                <= RETURN_HEADING_TOLERANCE_DEG
            ):

                with self.lock:

                    self.completed_avoidances += 1
                    self.mode = "FOLLOW"

                    self.reduced_reserve_active = False

                    if (
                        self.active_avoidance_context
                        == "slow"
                    ):

                        self.moving_overtakes_completed += 1

                    moving_overtakes = (
                        self.moving_overtakes_completed
                    )

                    self.active_avoidance_context = "static"

                    completed = (
                        self.completed_avoidances
                    )

                    self.cooldown_until = (
                        time.monotonic()
                        + AVOIDANCE_COOLDOWN_S
                    )


                self.resync_waypoint_after_avoidance()


                print(
                    f"*** MISSION ROUTE REJOINED | "
                    f"avoidances={completed}/"
                    f"{EXPECTED_TOTAL_AVOIDANCES} | "
                    f"moving overtakes={moving_overtakes}/"
                    f"{EXPECTED_MOVING_OVERTAKES} | "
                    f"route distance={route_distance:.2f} m | "
                    f"heading error={route_heading_error_deg:.1f} deg ***"
                )


                return (
                    0.0,
                    0.0
                )


            return_speed = (
                AVOIDANCE_RETURN_SPEED_MPS
            )


            if reduced_reserve:

                return_speed = min(
                    return_speed,
                    REDUCED_RESERVE_SPEED_MPS
                )


            return (
                return_speed,
                steering
            )


        return (
            0.0,
            0.0
        )


    def limiting_clearance_sector(
        self
    ):

        with self.lock:

            clearances = dict(
                self.sector_clearance
            )


        finite = [
            (
                name,
                value
            )
            for name, value
            in clearances.items()
            if math.isfinite(
                value
            )
        ]


        if not finite:

            return (
                "UNKNOWN",
                math.inf
            )


        return min(
            finite,
            key=lambda item: item[1]
        )


    # ========================================================
    # SAFETY
    # ========================================================

    def check_safety(
        self
    ):

        with self.lock:

            roll_deg = self.roll_deg
            pitch_deg = self.pitch_deg

            front_range = self.sectors[
                "FRONT"
            ]

            min_clearance = (
                self.current_min_body_clearance
            )


        if (
            abs(
                roll_deg
            )
            > MAX_ABS_ROLL_DEG
            or abs(
                pitch_deg
            )
            > MAX_ABS_PITCH_DEG
        ):

            self.terrain_safety_violation = True

            print()
            print(
                "*** TERRAIN SAFETY STOP ***"
            )

            print(
                f"roll={roll_deg:.2f} deg | "
                f"pitch={pitch_deg:.2f} deg"
            )

            print()

            return (
                False,
                "terrain_attitude_stop"
            )


        if (
            front_range
            <= EMERGENCY_FRONT_RANGE_M
        ):

            self.emergency_stop = True

            print()
            print(
                "*** EMERGENCY FRONT STOP ***"
            )

            print(
                f"Front range: "
                f"{front_range:.3f} m"
            )

            print()

            return (
                False,
                "emergency_front_stop"
            )


        if (
            math.isfinite(
                min_clearance
            )
            and min_clearance
            < MIN_BODY_CLEARANCE_M
        ):

            self.lidar_safety_violation = True

            limiting_sector, sector_clearance = (
                self.limiting_clearance_sector()
            )


            with self.lock:

                mode = self.mode
                x = self.x
                y = self.y

                avoid_side = self.avoid_side


            print()
            print(
                "*** 360 BODY-CLEARANCE STOP ***"
            )

            print(
                f"Body clearance          : "
                f"{min_clearance:.3f} m"
            )

            print(
                f"Required                : "
                f"{MIN_BODY_CLEARANCE_M:.3f} m"
            )

            print(
                f"Limiting sector         : "
                f"{limiting_sector} "
                f"({sector_clearance:.3f} m)"
            )

            print(
                f"Controller mode         : "
                f"{mode}"
            )

            print(
                f"Avoidance side          : "
                f"{avoid_side}"
            )

            print(
                f"World position          : "
                f"x={x:.2f}, y={y:.2f}"
            )


            if mode != "FOLLOW":

                try:

                    local_forward, local_lateral = (
                        self.avoidance_local_coordinates()
                    )

                    print(
                        f"Detour local position   : "
                        f"s={local_forward:.2f} m, "
                        f"d={local_lateral:+.2f} m"
                    )

                except Exception:

                    pass


            print()

            return (
                False,
                "360_body_clearance_stop"
            )


        return (
            True,
            None
        )


    # ========================================================
    # CSV
    # ========================================================

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


        z_variation = (
            0.0
            if (
                self.minimum_z_m is None
                or self.maximum_z_m is None
            )
            else (
                self.maximum_z_m
                - self.minimum_z_m
            )
        )


        fields = [
            "timestamp",
            "waypoint_progress",
            "waypoints_total",
            "goal_reached",
            "expected_static_avoidances",
            "expected_total_avoidances",
            "completed_avoidances",
            "moving_wait_events_completed",
            "moving_overtakes_completed",
            "crossing_event_started",
            "slow_event_started",
            "avoidance_directions",
            "camera_frames",
            "max_abs_roll_deg",
            "max_abs_pitch_deg",
            "world_z_variation_m",
            "minimum_body_clearance_m",
            "required_body_clearance_m",
            "terrain_safety_violation",
            "lidar_safety_violation",
            "no_path_detected",
            "emergency_stop",
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

                    "waypoint_progress":
                        self.waypoint_progress,

                    "waypoints_total":
                        len(
                            WAYPOINTS
                        ),

                    "goal_reached":
                        self.goal_reached,

                    "expected_static_avoidances":
                        EXPECTED_STATIC_AVOIDANCES,

                    "expected_total_avoidances":
                        EXPECTED_TOTAL_AVOIDANCES,

                    "completed_avoidances":
                        self.completed_avoidances,

                    "moving_wait_events_completed":
                        self.moving_wait_events_completed,

                    "moving_overtakes_completed":
                        self.moving_overtakes_completed,

                    "crossing_event_started":
                        self.crossing_event_started,

                    "slow_event_started":
                        self.slow_event_started,

                    "avoidance_directions":
                        ",".join(
                            self.avoidance_directions
                        ),

                    "camera_frames":
                        self.camera_frames,

                    "max_abs_roll_deg":
                        f"{self.max_abs_roll_deg:.3f}",

                    "max_abs_pitch_deg":
                        f"{self.max_abs_pitch_deg:.3f}",

                    "world_z_variation_m":
                        f"{z_variation:.3f}",

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

                    "terrain_safety_violation":
                        self.terrain_safety_violation,

                    "lidar_safety_violation":
                        self.lidar_safety_violation,

                    "no_path_detected":
                        self.no_path_detected,

                    "emergency_stop":
                        self.emergency_stop,

                    "end_reason":
                        end_reason,

                    "duration_s":
                        f"{duration:.3f}",

                    "result":
                        result,
                }
            )


    # ========================================================
    # RUN
    # ========================================================

    def run(
        self
    ):

        print()
        print(
            "============================================================"
        )

        print(
            " RUSKOTUNTURI AUTONOMOUS MISSION - MOVING OBSTACLES"
        )

        print(
            "============================================================"
        )

        print(
            "Controller version      : Phase 3 v7 (fixed corridor horizon)"
        )

        print(
            f"Waypoints               : "
            f"{len(WAYPOINTS)}"
        )

        print(
            f"Expected moving detours : "
            f"{EXPECTED_TOTAL_AVOIDANCES}"
        )

        print(
            f"Expected moving waits   : "
            f"{EXPECTED_MOVING_WAIT_EVENTS}"
        )

        print(
            f"Expected moving overtake: "
            f"{EXPECTED_MOVING_OVERTAKES}"
        )

        print(
            f"Planned elevation gain : "
            f"{PLANNED_ELEVATION_GAIN_M:.2f} m"
        )

        print(
            f"Preferred clearance    : "
            f"{PLANNING_BODY_CLEARANCE_M:.2f} m"
        )

        print(
            f"Minimum predicted      : "
            f"{MIN_PREDICTED_CORRIDOR_CLEARANCE_M:.2f} m"
        )

        print(
            f"Hard body clearance    : "
            f"{MIN_BODY_CLEARANCE_M:.2f} m"
        )

        print(
            f"Detection range        : "
            f"{OBSTACLE_DETECTION_RANGE_M:.2f} m"
        )

        print(
            f"Obstacle confirmation  : "
            f"{OBSTACLE_CONFIRM_SCANS} consecutive scans"
        )

        print(
            "Obstacle side estimate : route-frame front cluster"
        )

        print(
            "Avoidance offsets      : "
            + ", ".join(
                f"{value:.2f} m"
                for value in AVOIDANCE_OFFSET_CANDIDATES_M
            )
        )

        print(
            f"AVOID_OUT speed        : "
            f"{AVOIDANCE_OUT_SPEED_MPS:.2f} m/s"
        )

        print(
            f"AVOID_OUT lookahead    : "
            f"{AVOIDANCE_LOOKAHEAD_M:.2f} m"
        )

        print(
            f"Urgent lateral speed   : "
            f"{AVOIDANCE_URGENT_OUT_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Corridor lookahead     : "
            f"{CORRIDOR_LOOKAHEAD_M:.2f} m"
        )

        print(
            f"Route follow lookahead : "
            f"{FOLLOW_ROUTE_LOOKAHEAD_M:.2f} m"
        )

        print(
            f"Route return lookahead : "
            f"{RETURN_ROUTE_LOOKAHEAD_M:.2f} m"
        )

        print(
            "Route progress         : monotonic polyline projection"
        )

        print(
            "Side selection         : confirmed close obstacle + 360 route corridor"
        )

        print(
            "Camera                  : ACTIVE, not used for control"
        )

        print(
            "Moving actors           : scripted UserCommands poses"
        )

        print(
            f"Pose service            : "
            f"{SET_POSE_VECTOR_SERVICE}"
        )

        print(
            "Static test obstacles   : NONE (validated in Phase 2)"
        )

        print(
            f"Crossing wait timeout   : "
            f"{CROSSING_WAIT_TIMEOUT_S:.1f} s"
        )

        print(
            f"Slow-object wait timeout: "
            f"{SLOW_WAIT_TIMEOUT_S:.1f} s"
        )

        print(
            f"Wait clear delta        : "
            f"+{DYNAMIC_CLEAR_DELTA_M:.1f} m from closest range"
        )

        print(
            f"Scripted pose update    : "
            f"{1.0 / SCRIPTED_POSE_UPDATE_PERIOD_S:.1f} Hz"
        )

        print(
            f"Obstacle detect range   : "
            f"{OBSTACLE_DETECTION_RANGE_M:.2f} m"
        )

        print(
            f"Corridor lookahead      : "
            f"{CORRIDOR_LOOKAHEAD_M:.2f} m"
        )

        print(
            f"Crossing speed          : "
            f"{CROSSING_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Slow mover speed        : "
            f"{SLOW_SPEED_MPS:.2f} m/s"
        )

        print(
            "Ego moving-object input : 360 LiDAR only"
        )

        print()


        if not self.reset_world():

            print(
                "RESULT: FAIL"
            )

            return False


        start_time = time.monotonic()
        last_print = 0.0

        end_reason = None


        while True:

            now = time.monotonic()


            if (
                now
                - start_time
                > MISSION_TIMEOUT_S
            ):

                end_reason = "timeout"

                break


            with self.lock:

                pose_fresh = (
                    self.pose_received
                    and (
                        now
                        - self.last_pose_time
                        <= POSE_TIMEOUT_S
                    )
                )

                lidar_fresh = (
                    self.lidar_received
                    and (
                        now
                        - self.last_lidar_time
                        <= LIDAR_TIMEOUT_S
                    )
                )

                camera_fresh = (
                    self.camera_received
                    and (
                        now
                        - self.last_camera_time
                        <= CAMERA_TIMEOUT_S
                    )
                )

                mode = self.mode

                x = self.x
                y = self.y
                z = self.z

                roll_deg = self.roll_deg
                pitch_deg = self.pitch_deg

                front_range = self.sectors[
                    "FRONT"
                ]

                target_index = (
                    self.target_waypoint_index
                )

                cooldown_until = (
                    self.cooldown_until
                )


            if not (
                pose_fresh
                and lidar_fresh
                and camera_fresh
            ):

                self.stop_vehicle()
                end_reason = "sensor_timeout"

                break


            self.update_moving_obstacles(
                now
            )


            safe, safety_reason = (
                self.check_safety()
            )


            if not safe:

                self.stop_vehicle()
                end_reason = safety_reason

                break


            if mode == "FOLLOW":

                self.advance_waypoint_if_reached()


                with self.lock:

                    if self.goal_reached:

                        end_reason = "goal_reached"
                        break

                    front_range = self.sectors[
                        "FRONT"
                    ]


                if (
                    now
                    >= cooldown_until
                    and front_range
                    <= OBSTACLE_DETECTION_RANGE_M
                ):

                    with self.lock:

                        self.obstacle_confirm_count += 1

                        confirm_count = (
                            self.obstacle_confirm_count
                        )


                else:

                    with self.lock:

                        self.obstacle_confirm_count = 0

                        confirm_count = 0


                if (
                    confirm_count
                    >= OBSTACLE_CONFIRM_SCANS
                ):

                    self.start_obstacle_wait(
                        now
                    )

                    speed = 0.0
                    steering = 0.0


                else:

                    speed, steering, _ = (
                        self.follow_route_control()
                    )


            elif mode == "WAIT_CLEAR":

                (
                    wait_status,
                    speed,
                    steering
                ) = self.dynamic_wait_control(
                    now
                )


                if wait_status == "no_path":

                    self.no_path_detected = True
                    self.stop_vehicle()

                    end_reason = "no_safe_side"

                    break


            else:

                speed, steering = (
                    self.avoidance_control()
                )


            self.publish_command(
                speed,
                steering
            )


            if (
                now
                - last_print
                >= PRINT_INTERVAL_S
            ):

                with self.lock:

                    mode = self.mode

                    target_index = (
                        self.target_waypoint_index
                    )

                    completed = (
                        self.completed_avoidances
                    )

                    min_clearance = (
                        self.current_min_body_clearance
                    )


                if target_index < len(
                    WAYPOINTS
                ):

                    target_x, target_y = (
                        WAYPOINTS[
                            target_index
                        ]
                    )

                    target_distance = distance_2d(
                        x,
                        y,
                        target_x,
                        target_y
                    )

                else:

                    target_distance = 0.0


                print(
                    f"WP {target_index + 1:02d}/"
                    f"{len(WAYPOINTS):02d} | "
                    f"{mode:12s} | "
                    f"x={x:6.1f} y={y:6.1f} "
                    f"z={z:6.1f} | "
                    f"wp={target_distance:4.1f} m | "
                    f"F={front_range:4.1f} m | "
                    f"clr={min_clearance:4.1f} m | "
                    f"avoid={completed}/"
                    f"{EXPECTED_TOTAL_AVOIDANCES} | "
                    f"roll={roll_deg:5.1f} "
                    f"pitch={pitch_deg:5.1f}"
                )


                last_print = now


            time.sleep(
                CONTROL_PERIOD_S
            )


        self.stop_repeatedly()
        self.stop_moving_obstacles()


        duration = (
            time.monotonic()
            - start_time
        )


        z_variation = (
            0.0
            if (
                self.minimum_z_m is None
                or self.maximum_z_m is None
            )
            else (
                self.maximum_z_m
                - self.minimum_z_m
            )
        )


        camera_active = (
            self.camera_frames
            > 0
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
            and self.completed_avoidances
            == EXPECTED_TOTAL_AVOIDANCES
            and self.moving_wait_events_completed
            >= EXPECTED_MOVING_WAIT_EVENTS
            and self.moving_overtakes_completed
            >= EXPECTED_MOVING_OVERTAKES
            and camera_active
            and safety_margin_ok
            and not self.terrain_safety_violation
            and not self.lidar_safety_violation
            and not self.no_path_detected
            and not self.emergency_stop
            and end_reason
            == "goal_reached"
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
            " RUSKOTUNTURI MOVING-OBSTACLE MISSION RESULT"
        )

        print(
            "============================================================"
        )

        print(
            f"Route waypoint progress  : "
            f"{self.waypoint_progress}/"
            f"{len(WAYPOINTS)}"
        )

        print(
            f"Goal reached             : "
            f"{'YES' if self.goal_reached else 'NO'}"
        )

        print(
            f"Expected moving detours  : "
            f"{EXPECTED_TOTAL_AVOIDANCES}"
        )

        print(
            f"Completed detours        : "
            f"{self.completed_avoidances}"
        )

        print(
            f"Moving wait events       : "
            f"{self.moving_wait_events_completed}/"
            f"{EXPECTED_MOVING_WAIT_EVENTS}"
        )

        print(
            f"Moving overtakes         : "
            f"{self.moving_overtakes_completed}/"
            f"{EXPECTED_MOVING_OVERTAKES}"
        )

        print(
            f"Avoidance directions     : "
            f"{self.avoidance_directions}"
        )

        print(
            f"Reduced-reserve events   : "
            f"{self.reduced_reserve_events}"
        )

        print(
            f"Maximum absolute roll    : "
            f"{self.max_abs_roll_deg:.3f} deg"
        )

        print(
            f"Roll limit               : "
            f"{MAX_ABS_ROLL_DEG:.3f} deg"
        )

        print(
            f"Maximum absolute pitch   : "
            f"{self.max_abs_pitch_deg:.3f} deg"
        )

        print(
            f"Pitch limit              : "
            f"{MAX_ABS_PITCH_DEG:.3f} deg"
        )

        print(
            f"World Z variation        : "
            f"{z_variation:.3f} m"
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
            f"Safety margin OK         : "
            f"{'YES' if safety_margin_ok else 'NO'}"
        )

        print(
            f"Camera frames received   : "
            f"{self.camera_frames}"
        )

        print(
            f"Camera active             : "
            f"{'YES' if camera_active else 'NO'}"
        )

        print(
            "Camera used for control  : NO"
        )

        print(
            f"No-path detected          : "
            f"{'YES' if self.no_path_detected else 'NO'}"
        )

        print(
            f"Terrain safety violation : "
            f"{'YES' if self.terrain_safety_violation else 'NO'}"
        )

        print(
            f"360 safety violation     : "
            f"{'YES' if self.lidar_safety_violation else 'NO'}"
        )

        print(
            f"Emergency stop           : "
            f"{'YES' if self.emergency_stop else 'NO'}"
        )

        print(
            f"End reason               : "
            f"{end_reason}"
        )

        print(
            f"Duration                 : "
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

    test = (
        RuskotunturiMovingObstacleMission()
    )

    success = test.run()

    raise SystemExit(
        0 if success else 1
    )


if __name__ == "__main__":

    main()
