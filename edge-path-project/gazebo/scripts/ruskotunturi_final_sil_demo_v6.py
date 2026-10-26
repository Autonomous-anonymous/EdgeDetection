#!/usr/bin/env python3

import csv
import sys
import json
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.stringmsg_pb2 import StringMsg
from final_sil_v6_checks import point_clearance, scan_valid, scene_clearance, scene_clearance_detail, margins_ok, starting_scene_issue, loading_approach_clearance
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
# WORLD / MODELS / TOPICS
# ============================================================

WORLD_NAME = "ruskotunturi_final_sil_demo_v6_world"

POSE_TOPIC = (
    "/world/"
    + WORLD_NAME
    + "/pose/info"
)

SET_POSE_VECTOR_SERVICE = (
    "/world/"
    + WORLD_NAME
    + "/set_pose_vector"
)

MODEL_A = "aeb_vehicle_360"
MODEL_B = "aeb_vehicle_360_b"

MOVING_OBSTACLE_MODEL = "final_moving_obstacle"

CMD_A_TOPIC = (
    "/model/"
    + MODEL_A
    + "/requested_cmd_vel_v6"
)

CMD_B_TOPIC = (
    "/model/"
    + MODEL_B
    + "/requested_cmd_vel_v6"
)

LIDAR_A_TOPIC = "/aeb360/lidar"
LIDAR_B_TOPIC = "/aeb360b/lidar"
CAMERA_A_TOPIC = "/aeb360/camera/image"


# ============================================================
# LARGE-AREA ROUTES
# ============================================================

A_ROUTE = (
    # Private A branch
    (20.0, 40.0),
    (16.0, 38.0),
    (12.0, 36.0),
    (4.0, 32.0),      # static obstacle lies between 12,36 and 4,32
    (0.0, 30.0),
    (-4.0, 28.0),     # MERGE

    # Exact shared route after merge
    (-8.0, 24.0),
    (-12.0, 20.0),
    (-16.0, 16.0),
    (-18.0, 12.0),
    (-18.0, 8.0),
    (-18.0, 4.0),
    (-18.0, 0.0),
    (-20.0, -4.0),
    (-22.0, -8.0),
    (-26.0, -12.0),
    (-28.0, -16.0),
    (-32.0, -20.0),
    (-36.0, -24.0),
    (-38.0, -28.0),
    (-40.0, -32.0),
    (-40.0, -36.0),   # COMMON SUMMIT LOADING ZONE
    (-48.0, -36.0),   # Clear B final approach, including goal tolerance
)

B_ROUTE = (
    # Private B branch
    (20.0, 24.0),
    (16.0, 24.0),
    (12.0, 24.0),
    (4.0, 24.0),      # moving obstacle crosses at x=8
    (0.0, 26.0),
    (-4.0, 28.0),     # MERGE

    # Exact same shared route as Vehicle A
    (-8.0, 24.0),
    (-12.0, 20.0),
    (-16.0, 16.0),
    (-18.0, 12.0),
    (-18.0, 8.0),
    (-18.0, 4.0),
    (-18.0, 0.0),
    (-20.0, -4.0),
    (-22.0, -8.0),
    (-26.0, -12.0),
    (-28.0, -16.0),
    (-32.0, -20.0),
    (-36.0, -24.0),
    (-38.0, -28.0),
    (-40.0, -32.0),
    (-40.0, -36.0),   # SAME COMMON SUMMIT LOADING ZONE
)

A_STATIC_ACTIVE_INDICES = (1, 2, 3)

A_MERGE_INDEX = 5
A_SHARED_INDEX = 6
A_LOADING_INDEX = 21
A_EXIT_INDEX = 22

B_MOVING_ENCOUNTER_INDEX = 3
B_PREMERGE_INDEX = 4
B_MERGE_INDEX = 5
B_SHARED_INDEX = 6
B_LOADING_INDEX = 21

MERGE_POINT = (-4.0, 28.0)
LOADING_ZONE = (-40.0, -36.0)

LOADING_ZONE_RADIUS_M = 2.2
A_LOADING_DWELL_S = 2.0
LOADING_APPROACH_GATE_DISTANCE_M = 12.0
LOADING_APPROACH_RESERVED_MARGIN_M = 1.25

MERGE_RELEASE_HEADWAY_M = 7.0
MERGE_CAUTION_DISTANCE_M = 14.0
MERGE_SLOW_DISTANCE_M = 10.0
MERGE_STOP_DISTANCE_M = 7.0

B_MERGE_CAUTION_SPEED_MPS = 0.55
B_MERGE_SLOW_SPEED_MPS = 0.28
B_SHARED_SPEED_MPS = 0.72

SHARED_HEADWAY_SLOW_M = 8.0
SHARED_HEADWAY_STOP_M = 5.0


# ============================================================
# MOTION
# ============================================================

A_CRUISE_SPEED_MPS = 0.90
B_CRUISE_SPEED_MPS = 0.85
B_APPROACH_SPEED_MPS = 0.45

WAYPOINT_REACHED_M = 1.55
GOAL_REACHED_M = 1.80

HEADING_GAIN = 1.65
MAX_STEERING = 0.42

CONTROL_PERIOD_S = 0.05
PRINT_INTERVAL_S = 0.80
MISSION_TIMEOUT_S = 280.0

RESET_SETTLE_S = 1.0
SENSOR_TIMEOUT_S = 2.0


# ============================================================
# HARD SAFETY
# ============================================================

VEHICLE_HALF_LENGTH_M = 1.30
VEHICLE_HALF_WIDTH_M = 0.81

HARD_BODY_CLEARANCE_M = 1.00

MAX_ABS_ROLL_DEG = 25.0
MAX_ABS_PITCH_DEG = 30.0

TERRAIN_SLOW_ROLL_DEG = 12.0
TERRAIN_SLOW_PITCH_DEG = 15.0
TERRAIN_REDUCED_SPEED_MPS = 0.60


# ============================================================
# A: STATIC OBSTACLE + CAMERA/LIDAR FUSION
# ============================================================

STATIC_DETECT_FRONT_M = 10.5
STATIC_LIDAR_FAILSAFE_M = 3.0

STATIC_APPROACH_SLOW_FRONT_M = 11.0
STATIC_APPROACH_SLOW_SPEED_MPS = 0.35

STATIC_LINE_START = (20.0, 40.0)
STATIC_LINE_END = (0.0, 30.0)

STATIC_OFFSET_M = 4.60
STATIC_OUT_SPEED_MPS = 0.35
STATIC_PASS_SPEED_MPS = 0.65
STATIC_RETURN_SPEED_MPS = 0.58

STATIC_LINE_LOOKAHEAD_M = 2.0
STATIC_OUT_TOLERANCE_M = 0.45

STATIC_PASS_MARGIN_M = 3.5
STATIC_RETURN_COMPLETE_MARGIN_M = 6.0
STATIC_RETURN_LATERAL_TOLERANCE_M = 0.55

STATIC_MAX_STEERING = 0.72

CAMERA_ROI_X_MIN_FRAC = 0.15
CAMERA_ROI_X_MAX_FRAC = 0.85
CAMERA_ROI_Y_MIN_FRAC = 0.15
CAMERA_ROI_Y_MAX_FRAC = 0.85

CAMERA_SAMPLE_STRIDE_PX = 3

CAMERA_ORANGE_R_MIN = 70
CAMERA_ORANGE_RG_DELTA_MIN = 35
CAMERA_ORANGE_GB_DELTA_MIN = 8

CAMERA_ORANGE_MIN_SAMPLES = 12
CAMERA_CONFIRM_FRAMES = 3
CAMERA_CONFIRM_MAX_AGE_S = 0.65


# ============================================================
# B: MOVING OBSTACLE
# ============================================================

# Perpendicular crossing of B's route near (-32,-20).
# Coordinates include obstacle-centre height, not terrain surface.
MOVING_PATH = (
    (8.0, 20.0, -8.490700),
    (8.0, 22.0, -9.137000),
    (8.0, 24.0, -9.202800),
    (8.0, 26.0, -9.195000),
    (8.0, 28.0, -9.321000),
)

MOVING_SPEED_MPS = 0.45
MOVING_CENTER_DWELL_S = 3.0

MOVING_POSE_UPDATE_PERIOD_S = 0.40
MOVING_POSE_TIMEOUT_MS = 150

B_DYNAMIC_STOP_FRONT_M = 5.5
B_DYNAMIC_RESUME_FRONT_M = 7.0
B_DYNAMIC_RESUME_CONFIRM_CYCLES = 5


# ============================================================
# RESULTS
# ============================================================

RESULT_FILE = (
    Path(__file__).resolve().parent.parent
    / "results"
    / "ruskotunturi_final_sil_demo_v6_4_results.csv"
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


# ============================================================
# CONTROLLER
# ============================================================

class FinalSilDemoLargeArea:

    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()
        self.reset_state()
        self.sim_time = None
        self.clock_wall = 0.0
        self.harness = None
        self.watchdog = None
        self.harness_wall = 0.0
        self.observed = {}
        self.ground_min = math.inf
        self.ground_min_pair = "unavailable"
        self.invalid_scan_seen = False
        self.encounter_stop_verified = False
        self.encounter_resume_verified = False
        self.trace_handle = None
        self.trace_writer = None
        self.trace_path = ""
        self.pub_a = self.node.advertise(CMD_A_TOPIC, Twist)
        self.pub_b = self.node.advertise(CMD_B_TOPIC, Twist)
        self.emergency_pubs = [self.node.advertise(f"/model/{model}/cmd_vel", Twist) for model in (MODEL_A, MODEL_B)]
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self.pose_callback):
            raise RuntimeError('Failed to subscribe to pose topic')
        if not self.node.subscribe(LaserScan, LIDAR_A_TOPIC, self.lidar_a_callback):
            raise RuntimeError('Failed to subscribe to A LiDAR')
        if not self.node.subscribe(LaserScan, LIDAR_B_TOPIC, self.lidar_b_callback):
            raise RuntimeError('Failed to subscribe to B LiDAR')
        if not self.node.subscribe(Image, CAMERA_A_TOPIC, self.camera_callback):
            raise RuntimeError('Failed to subscribe to A camera')
        self.node.subscribe(Clock, "/world/" + WORLD_NAME + "/clock", self.clock_callback)
        self.node.subscribe(StringMsg, "/final_sil_v6/harness", self.harness_callback)


    # ========================================================
    # STATE
    # ========================================================

    def blank_vehicle(
        self,
        route
    ):

        return {
            "route": route,
            "target_index": 1,

            "x": None,
            "y": None,
            "z": None,
            "yaw": 0.0,

            "roll_deg": 0.0,
            "pitch_deg": 0.0,

            "max_abs_roll_deg": 0.0,
            "max_abs_pitch_deg": 0.0,

            "pose_received": False,
            "lidar_received": False,

            "last_pose_time": 0.0,
            "last_lidar_time": 0.0,

            "front_range": math.inf,
            "left_range": math.inf,
            "right_range": math.inf,
            "rear_range": math.inf,

            "min_clearance": math.inf,
            "minimum_clearance_seen": math.inf,

            "goal_reached": False,
        }


    def reset_state(
        self
    ):

        with self.lock:

            self.a = (
                self.blank_vehicle(
                    A_ROUTE
                )
            )

            self.b = (
                self.blank_vehicle(
                    B_ROUTE
                )
            )

            # A static avoidance.
            self.a_mode = "FOLLOW"
            self.a_detour_side = None
            self.a_detour_phase = "NONE"

            self.static_line_ux = 0.0
            self.static_line_uy = 0.0
            self.static_line_nx = 0.0
            self.static_line_ny = 0.0

            self.static_desired_offset_m = 0.0
            self.static_obstacle_s_m = None

            self.static_avoidance_started = False
            self.static_avoidance_completed = False

            # A camera fusion.
            self.camera_frames = 0
            self.camera_target_frames = 0
            self.camera_target_streak = 0
            self.camera_target_confirmed = False
            self.camera_last_target_time = 0.0
            self.camera_max_orange_samples = 0

            self.camera_used_for_control = False
            self.camera_fallback_used = False

            # Shared loading zone / coordinator.
            self.a_loading_arrived = False
            self.a_loading_arrival_time = None
            self.a_loading_departed = False

            self.b_waiting = False
            self.b_released = False

            # Dynamic merge coordination. There is no dedicated
            # coordinator waiting area.
            self.coordinator_profile_events = 0
            self.coordinator_stop_events = 0
            self.coordinator_release_events = 0

            self.coordinator_profile_active = False
            self.coordinator_stop_active = False

            # B moving obstacle.
            self.moving_started = False
            self.moving_completed = False
            self.moving_start_time = None
            self.last_moving_update = 0.0
            self.moving_pose_failures = 0

            self.b_dynamic_hold = False
            self.b_dynamic_hold_events = 0
            self.b_dynamic_resume_events = 0
            self.b_dynamic_clear_count = 0

            # Safety / result.
            self.loading_overlap_violation = False
            self.terrain_violation = False
            self.lidar_violation = False

            self.loading_gate_hold = False
            self.loading_gate_hold_events = 0
            self.loading_gate_clearance = None
            self.end_reason = "running"


    # ========================================================
    # SENSOR CALLBACKS
    # ========================================================

    def pose_matches(
        self,
        name,
        model
    ):

        return (
            name == model
            or name.endswith(
                "::"
                + model
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


    def pose_callback(self, msg):
        now = time.monotonic()
        pose_a = None
        pose_b = None
        for pose in msg.pose:
            if self.pose_matches(pose.name, MODEL_A):
                pose_a = pose
            elif self.pose_matches(pose.name, MODEL_B):
                pose_b = pose
        with self.lock:
            for p in msg.pose:
                self.observed[p.name] = (p.position.x, p.position.y, quaternion_to_rpy(p.orientation)[2], quaternion_to_rpy(p.orientation)[0], quaternion_to_rpy(p.orientation)[1])
            detail = scene_clearance_detail(self.observed)
            if detail is not None and detail[0] < self.ground_min:
                self.ground_min, self.ground_min_pair = detail
            if pose_a is not None:
                self.update_pose_state(self.a, pose_a, now)
            if pose_b is not None:
                self.update_pose_state(self.b, pose_b, now)


    def lidar_summary(self, msg):
        front = math.inf
        left = math.inf
        right = math.inf
        rear = math.inf
        minimum_clearance = math.inf
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_step
            angle_deg = math.degrees(wrap_angle(angle))
            minimum_clearance = min(minimum_clearance, point_clearance(distance, angle))
            if abs(angle_deg) <= 20.0:
                front = min(front, distance)
            elif 20.0 < angle_deg <= 100.0:
                left = min(left, distance)
            elif -100.0 <= angle_deg < -20.0:
                right = min(right, distance)
            elif abs(angle_deg) >= 135.0:
                rear = min(rear, distance)
        return (front, left, right, rear, minimum_clearance)


    def update_lidar(self, state, msg):
        front, left, right, rear, minimum_clearance = self.lidar_summary(msg)
        state['scan_valid'] = scan_valid(msg)
        self.invalid_scan_seen = self.invalid_scan_seen or not state['scan_valid']
        state['front_range'] = front
        state['left_range'] = left
        state['right_range'] = right
        state['rear_range'] = rear
        state['min_clearance'] = minimum_clearance
        state['minimum_clearance_seen'] = min(state['minimum_clearance_seen'], minimum_clearance)
        state['lidar_received'] = True
        state['last_lidar_time'] = time.monotonic()


    def lidar_a_callback(
        self,
        msg
    ):

        with self.lock:

            self.update_lidar(
                self.a,
                msg
            )


    def lidar_b_callback(
        self,
        msg
    ):

        with self.lock:

            self.update_lidar(
                self.b,
                msg
            )


    def camera_callback(self, msg):
        now = self.sim_time if self.sim_time is not None else 0.0
        width = int(msg.width)
        height = int(msg.height)
        row_step = int(msg.step)
        data = msg.data
        orange_samples = 0
        if width > 0 and height > 0 and (row_step >= width * 3) and (len(data) >= row_step * height):
            x_min = int(width * CAMERA_ROI_X_MIN_FRAC)
            x_max = int(width * CAMERA_ROI_X_MAX_FRAC)
            y_min = int(height * CAMERA_ROI_Y_MIN_FRAC)
            y_max = int(height * CAMERA_ROI_Y_MAX_FRAC)
            for y in range(y_min, y_max, CAMERA_SAMPLE_STRIDE_PX):
                row = y * row_step
                for x in range(x_min, x_max, CAMERA_SAMPLE_STRIDE_PX):
                    offset = row + x * 3
                    r = data[offset]
                    g = data[offset + 1]
                    b = data[offset + 2]
                    if r >= CAMERA_ORANGE_R_MIN and int(r) - int(g) >= CAMERA_ORANGE_RG_DELTA_MIN and (int(g) - int(b) >= CAMERA_ORANGE_GB_DELTA_MIN):
                        orange_samples += 1
        visible = orange_samples >= CAMERA_ORANGE_MIN_SAMPLES
        with self.lock:
            self.camera_frames += 1
            self.camera_max_orange_samples = max(self.camera_max_orange_samples, orange_samples)
            if visible:
                self.camera_target_frames += 1
                self.camera_target_streak += 1
                self.camera_last_target_time = now
            else:
                self.camera_target_streak = 0
            self.camera_target_confirmed = self.camera_target_streak >= CAMERA_CONFIRM_FRAMES


    # ========================================================
    # BASIC COMMANDS / RESET
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


    def stop_repeatedly(
        self
    ):

        for _ in range(
            10
        ):

            self.stop_both()

            time.sleep(
                0.02
            )




    def prepare_fresh_world(self):
        print('Checking fresh world (no world reset will be requested)...')
        print('Start Gazebo with -r, or press Play. Restart the world for each trial.')
        self.stop_repeatedly()
        self.reset_state()
        start=time.monotonic()
        last_issue='waiting for fresh sensors and an advancing simulation clock'
        stable_since=None
        stable_pose=None
        while time.monotonic()-start<15.0:
            now=time.monotonic()
            with self.lock:
                ready=(self.sim_time is not None and now-self.clock_wall<.5
                    and self.a.get('scan_valid',False) and self.b.get('scan_valid',False)
                    and self.a['pose_received'] and self.b['pose_received']
                    and now-self.a['last_pose_time']<.5 and now-self.b['last_pose_time']<.5
                    and now-self.a['last_lidar_time']<.5 and now-self.b['last_lidar_time']<.5
                    and self.camera_frames>0 and scene_clearance(self.observed) is not None)
                if ready:
                    issue=starting_scene_issue(self.observed)
                    if issue:
                        print('START REFUSED: '+issue,flush=True)
                        self.end_reason='initial_scene_mismatch'
                        return False
                    positions=(self.a['x'],self.a['y'],self.b['x'],self.b['y'])
                    if stable_pose is None or math.dist(positions,stable_pose)>.08:
                        stable_pose=positions;stable_since=now
                    if now-stable_since>=.5:
                        self.invalid_scan_seen=False
                        self.ground_min,self.ground_min_pair=scene_clearance_detail(self.observed)
                        for vehicle in (self.a,self.b):
                            vehicle['minimum_clearance_seen']=vehicle['min_clearance']
                            vehicle['max_abs_roll_deg']=abs(vehicle['roll_deg'])
                            vehicle['max_abs_pitch_deg']=abs(vehicle['pitch_deg'])
                        print(f"Fresh start OK | A=({self.a['x']:.1f},{self.a['y']:.1f}) | B=({self.b['x']:.1f},{self.b['y']:.1f})",flush=True)
                        return True
                    last_issue='waiting for starting vehicles to settle'
                else:
                    stable_pose=None;stable_since=None
            time.sleep(.05)
        self.end_reason='fresh_world_not_ready'
        print('START REFUSED: '+last_issue,flush=True)
        return False



    # ========================================================
    # ROUTE FOLLOWING
    # ========================================================

    def steer_to(
        self,
        state,
        target,
        max_steering=MAX_STEERING
    ):

        desired_heading = math.atan2(
            target[
                1
            ]
            - state[
                "y"
            ],
            target[
                0
            ]
            - state[
                "x"
            ]
        )

        error = wrap_angle(
            desired_heading
            - state[
                "yaw"
            ]
        )

        return clamp(
            HEADING_GAIN
            * error,
            -max_steering,
            max_steering
        )


    def waypoint_control(
        self,
        state,
        speed
    ):

        route = state[
            "route"
        ]

        index = state[
            "target_index"
        ]

        target = route[
            index
        ]

        distance = distance_2d(
            state[
                "x"
            ],
            state[
                "y"
            ],
            target[
                0
            ],
            target[
                1
            ]
        )

        steering = self.steer_to(
            state,
            target
        )

        return (
            speed,
            steering,
            distance
        )


    def advance_waypoint(
        self,
        state,
        threshold=WAYPOINT_REACHED_M
    ):

        index = state[
            "target_index"
        ]

        target = state[
            "route"
        ][
            index
        ]

        distance = distance_2d(
            state[
                "x"
            ],
            state[
                "y"
            ],
            target[
                0
            ],
            target[
                1
            ]
        )

        if distance > threshold:

            return False

        if (
            index
            < len(
                state[
                    "route"
                ]
            )
            - 1
        ):

            state[
                "target_index"
            ] += 1

            return True

        state[
            "goal_reached"
        ] = True

        return True


    def terrain_limited_speed(
        self,
        state,
        desired_speed
    ):

        if (
            abs(
                state[
                    "roll_deg"
                ]
            )
            >= TERRAIN_SLOW_ROLL_DEG
            or abs(
                state[
                    "pitch_deg"
                ]
            )
            >= TERRAIN_SLOW_PITCH_DEG
        ):

            return min(
                desired_speed,
                TERRAIN_REDUCED_SPEED_MPS
            )

        return desired_speed


    # ========================================================
    # A STATIC OBSTACLE - CONTINUOUS OFFSET-LINE AVOIDANCE
    # ========================================================

    def camera_recent(
        self,
        now
    ):

        return (
            self.camera_target_confirmed
            and (
                now
                - self.camera_last_target_time
            )
            <= CAMERA_CONFIRM_MAX_AGE_S
        )


    def static_line_geometry(
        self
    ):

        dx = (
            STATIC_LINE_END[0]
            - STATIC_LINE_START[0]
        )

        dy = (
            STATIC_LINE_END[1]
            - STATIC_LINE_START[1]
        )

        length = math.hypot(
            dx,
            dy
        )

        ux = dx / length
        uy = dy / length

        nx = -uy
        ny = ux

        return (
            ux,
            uy,
            nx,
            ny
        )


    def static_line_project(
        self,
        x,
        y
    ):

        rx = (
            x
            - STATIC_LINE_START[0]
        )

        ry = (
            y
            - STATIC_LINE_START[1]
        )

        s = (
            rx * self.static_line_ux
            + ry * self.static_line_uy
        )

        d = (
            rx * self.static_line_nx
            + ry * self.static_line_ny
        )

        return (
            s,
            d
        )


    def static_line_target(
        self,
        current_s,
        desired_offset
    ):

        target_s = (
            current_s
            + STATIC_LINE_LOOKAHEAD_M
        )

        return (
            STATIC_LINE_START[0]
            + target_s * self.static_line_ux
            + desired_offset * self.static_line_nx,

            STATIC_LINE_START[1]
            + target_s * self.static_line_uy
            + desired_offset * self.static_line_ny,
        )


    def start_a_detour(
        self,
        now
    ):

        a = self.a

        (
            self.static_line_ux,
            self.static_line_uy,
            self.static_line_nx,
            self.static_line_ny,
        ) = self.static_line_geometry()


        if (
            a[
                "right_range"
            ]
            >= a[
                "left_range"
            ]
        ):

            self.a_detour_side = "RIGHT"
            offset_sign = -1.0

        else:

            self.a_detour_side = "LEFT"
            offset_sign = 1.0


        self.static_desired_offset_m = (
            offset_sign
            * STATIC_OFFSET_M
        )


        estimated_obstacle_x = (
            a[
                "x"
            ]
            + a[
                "front_range"
            ]
            * math.cos(
                a[
                    "yaw"
                ]
            )
        )

        estimated_obstacle_y = (
            a[
                "y"
            ]
            + a[
                "front_range"
            ]
            * math.sin(
                a[
                    "yaw"
                ]
            )
        )

        obstacle_s, _ = (
            self.static_line_project(
                estimated_obstacle_x,
                estimated_obstacle_y
            )
        )

        current_s, _ = (
            self.static_line_project(
                a[
                    "x"
                ],
                a[
                    "y"
                ]
            )
        )

        self.static_obstacle_s_m = max(
            obstacle_s,
            current_s + 2.5
        )

        self.a_mode = "STATIC_DETOUR"
        self.a_detour_phase = "OUT"
        self.static_avoidance_started = True

        camera_ok = (
            self.camera_recent(
                now
            )
        )

        if camera_ok:

            self.camera_used_for_control = True

        else:

            self.camera_fallback_used = True


        print()
        print(
            "*** A: STATIC OBSTACLE OFFSET-LINE AVOIDANCE STARTED ***"
        )

        print(
            f"Front range             : "
            f"{a['front_range']:.2f} m"
        )

        print(
            f"Camera confirmed        : "
            f"{'YES' if camera_ok else 'NO - LIDAR FAILSAFE'}"
        )

        print(
            f"Avoidance side          : "
            f"{self.a_detour_side}"
        )

        print(
            f"Target lateral offset   : "
            f"{abs(self.static_desired_offset_m):.2f} m"
        )

        print(
            "Strategy                : "
            "continuous OUT -> PASS -> RETURN"
        )

        print()


    def static_detour_control(self):
        a = self.a
        current_s, current_d = self.static_line_project(a['x'], a['y'])
        if self.a_detour_phase == 'OUT':
            desired_offset = self.static_desired_offset_m
            speed = STATIC_OUT_SPEED_MPS
            if abs(current_d - self.static_desired_offset_m) <= STATIC_OUT_TOLERANCE_M:
                self.a_detour_phase = 'PASS'
                print()
                print('*** A: SAFE LATERAL OFFSET ESTABLISHED ***')
                print(f'Current lateral offset  : {current_d:.2f} m')
                print(f"Current body clearance  : {a['min_clearance']:.2f} m")
                print()
        elif self.a_detour_phase == 'PASS':
            desired_offset = self.static_desired_offset_m
            speed = STATIC_PASS_SPEED_MPS
            if current_s >= self.static_obstacle_s_m + STATIC_PASS_MARGIN_M:
                self.a_detour_phase = 'RETURN'
                print()
                print('*** A: STATIC OBSTACLE PASSED - RETURNING TO ROUTE ***')
                print(f"Current body clearance  : {a['min_clearance']:.2f} m")
                print()
        else:
            desired_offset = 0.0
            speed = STATIC_RETURN_SPEED_MPS
            if current_s >= self.static_obstacle_s_m + STATIC_RETURN_COMPLETE_MARGIN_M and abs(current_d) <= STATIC_RETURN_LATERAL_TOLERANCE_M:
                self.a_mode = 'FOLLOW'
                self.a_detour_phase = 'NONE'
                self.static_avoidance_completed = True
                self.a['target_index'] = max(self.a['target_index'], 4)
                # Advance before computing steering when rejoining within an
                # already-reached waypoint's acceptance circle.
                while self.a['target_index'] < A_MERGE_INDEX:
                    target = A_ROUTE[self.a['target_index']]
                    if distance_2d(a['x'], a['y'], *target) > WAYPOINT_REACHED_M:
                        break
                    self.a['target_index'] += 1
                print()
                print('*** A: STATIC OBSTACLE AVOIDANCE COMPLETE ***')
                print(f"Minimum body clearance  : {a['minimum_clearance_seen']:.2f} m")
                print()
                return (0.0, 0.0)
        target = self.static_line_target(current_s, desired_offset)
        steering = self.steer_to(a, target, max_steering=STATIC_MAX_STEERING)
        speed = self.terrain_limited_speed(a, speed)
        return (speed, steering)


    def a_control(
        self,
        now
    ):

        a = self.a


        # Vehicle A can finish before Vehicle B. Once A has reached
        # its exit, keep it stopped and do not print the exit event
        # again on every control-loop iteration.
        if a[
            "goal_reached"
        ]:

            return (
                0.0,
                0.0
            )


        if self.a_mode == "STATIC_DETOUR":

            return self.static_detour_control()


        if self.a_loading_arrived:

            if not self.a_loading_departed:

                if (
                    now
                    - self.a_loading_arrival_time
                    < A_LOADING_DWELL_S
                ):

                    return (
                        0.0,
                        0.0
                    )

                self.a_loading_departed = True

                a[
                    "target_index"
                ] = A_EXIT_INDEX

                print()
                print(
                    "*** A: LEAVING SUMMIT LOADING ZONE ***"
                )

                print()


        if (
            not self.static_avoidance_started
            and a[
                "target_index"
            ]
            in A_STATIC_ACTIVE_INDICES
            and a[
                "front_range"
            ]
            <= STATIC_DETECT_FRONT_M
        ):

            camera_ok = (
                self.camera_recent(
                    now
                )
            )

            lidar_failsafe = (
                a[
                    "front_range"
                ]
                <= STATIC_LIDAR_FAILSAFE_M
            )

            if (
                camera_ok
                or lidar_failsafe
            ):

                self.start_a_detour(
                    now
                )

                return (
                    0.0,
                    0.0
                )


        desired_speed = (
            A_CRUISE_SPEED_MPS
        )


        if (
            not self.static_avoidance_started
            and a[
                "target_index"
            ]
            in A_STATIC_ACTIVE_INDICES
            and a[
                "front_range"
            ]
            <= STATIC_APPROACH_SLOW_FRONT_M
        ):

            desired_speed = min(
                desired_speed,
                STATIC_APPROACH_SLOW_SPEED_MPS
            )


        desired_speed = (
            self.terrain_limited_speed(
                a,
                desired_speed
            )
        )


        speed, steering, distance = (
            self.waypoint_control(
                a,
                desired_speed
            )
        )


        if (
            a[
                "target_index"
            ]
            == A_LOADING_INDEX
            and distance
            <= LOADING_ZONE_RADIUS_M
        ):

            self.a_loading_arrived = True
            self.a_loading_arrival_time = now

            print()
            print(
                "*** A: SUMMIT COMMON LOADING ZONE REACHED ***"
            )

            print()

            return (
                0.0,
                0.0
            )


        if (
            distance
            <= WAYPOINT_REACHED_M
        ):

            self.advance_waypoint(
                a
            )


        if (
            a[
                "target_index"
            ]
            == A_EXIT_INDEX
        ):

            exit_target = A_ROUTE[
                A_EXIT_INDEX
            ]

            exit_distance = (
                distance_2d(
                    a[
                        "x"
                    ],
                    a[
                        "y"
                    ],
                    exit_target[
                        0
                    ],
                    exit_target[
                        1
                    ]
                )
            )

            if (
                exit_distance
                <= GOAL_REACHED_M
            ):

                a[
                    "goal_reached"
                ] = True

                print()
                print(
                    "*** A: LOADING-ZONE EXIT REACHED ***"
                )

                print()

                return (
                    0.0,
                    0.0
                )


        return (
            speed,
            steering
        )


    # ========================================================
    # B MOVING OBSTACLE + COORDINATOR
    # ========================================================

    def a_has_safe_merge_headway(
        self
    ):

        # A has priority. B may merge only after A has entered the
        # shared route and moved a safe distance beyond the merge.
        if (
            self.a[
                "target_index"
            ]
            < A_SHARED_INDEX
        ):

            return False

        a_distance_from_merge = (
            distance_2d(
                self.a[
                    "x"
                ],
                self.a[
                    "y"
                ],
                MERGE_POINT[
                    0
                ],
                MERGE_POINT[
                    1
                ]
            )
        )

        return (
            a_distance_from_merge
            >= MERGE_RELEASE_HEADWAY_M
        )


    def maybe_release_b_at_merge(
        self
    ):

        if self.b_released:

            return

        if not self.a_has_safe_merge_headway():

            return

        self.b_released = True
        self.b_waiting = False
        self.coordinator_profile_active = False
        self.coordinator_stop_active = False

        self.coordinator_release_events += 1

        print()
        print(
            "*** COORDINATOR: B MERGE PERMISSION GRANTED ***"
        )

        print(
            "A has priority and has created safe headway "
            "on the shared route."
        )

        print(
            f"A distance from merge   : "
            f"{distance_2d(self.a['x'], self.a['y'], MERGE_POINT[0], MERGE_POINT[1]):.2f} m"
        )

        print()


    def coordinator_b_speed(
        self,
        desired_speed
    ):

        self.maybe_release_b_at_merge()

        b = self.b

        distance_to_merge = (
            distance_2d(
                b[
                    "x"
                ],
                b[
                    "y"
                ],
                MERGE_POINT[
                    0
                ],
                MERGE_POINT[
                    1
                ]
            )
        )


        # Before B is allowed onto the shared route, continuously
        # shape its velocity. There is no fixed hold zone.
        if (
            not self.b_released
            and b[
                "target_index"
            ]
            >= B_PREMERGE_INDEX
        ):

            profiled_speed = desired_speed
            profile_name = "CRUISE"


            if (
                distance_to_merge
                <= MERGE_STOP_DISTANCE_M
            ):

                profiled_speed = 0.0
                profile_name = "STOP_FOR_MERGE"

                if not self.coordinator_stop_active:

                    self.coordinator_stop_active = True
                    self.coordinator_stop_events += 1

                    print()
                    print(
                        "*** COORDINATOR: B TEMPORARY MERGE STOP ***"
                    )

                    print(
                        f"Distance to merge       : "
                        f"{distance_to_merge:.2f} m"
                    )

                    print(
                        "Reason                  : "
                        "A has priority on shared route"
                    )

                    print()


            elif (
                distance_to_merge
                <= MERGE_SLOW_DISTANCE_M
            ):

                profiled_speed = min(
                    desired_speed,
                    B_MERGE_SLOW_SPEED_MPS
                )

                profile_name = "MERGE_SLOW"


            elif (
                distance_to_merge
                <= MERGE_CAUTION_DISTANCE_M
            ):

                profiled_speed = min(
                    desired_speed,
                    B_MERGE_CAUTION_SPEED_MPS
                )

                profile_name = "MERGE_CAUTION"


            if (
                profile_name != "CRUISE"
                and not self.coordinator_profile_active
            ):

                self.coordinator_profile_active = True
                self.coordinator_profile_events += 1

                print()
                print(
                    "*** COORDINATOR: B MERGE SPEED PROFILE ACTIVE ***"
                )

                print(
                    f"Profile                 : "
                    f"{profile_name}"
                )

                print(
                    f"Distance to merge       : "
                    f"{distance_to_merge:.2f} m"
                )

                print()


            self.b_waiting = (
                profiled_speed
                <= 0.01
            )

            return profiled_speed


        # A's goal flag alone does not establish that its parked footprint
        # leaves enough space for B's remaining loading approach.
        if (self.b_released and b['target_index'] >= B_LOADING_INDEX-2
                and distance_2d(b['x'], b['y'], *LOADING_ZONE) <= LOADING_APPROACH_GATE_DISTANCE_M):
            a=self.a
            pose=(a['x'], a['y'], a['yaw'], math.radians(a['roll_deg']), math.radians(a['pitch_deg']))
            self.loading_gate_clearance=loading_approach_clearance(pose, B_ROUTE[B_LOADING_INDEX-2], LOADING_ZONE)
            allowed=(a['goal_reached'] and self.loading_gate_clearance >= LOADING_APPROACH_RESERVED_MARGIN_M)
            if not allowed:
                if not self.loading_gate_hold:
                    self.loading_gate_hold_events+=1
                    print(f"*** COORDINATOR: B LOADING APPROACH HOLD; parked-A reservation clearance={self.loading_gate_clearance:.3f} m ***", flush=True)
                self.loading_gate_hold=True
                return 0.0
            if self.loading_gate_hold:
                print('*** COORDINATOR: B LOADING APPROACH RELEASED ***', flush=True)
            self.loading_gate_hold=False

        # After merge permission, both vehicles use the same route.
        # Keep a sensible coordinator headway while A is still ahead.
        if (
            self.b_released
            and b[
                "target_index"
            ]
            >= B_MERGE_INDEX
            and not self.a[
                "goal_reached"
            ]
        ):

            vehicle_spacing = (
                distance_2d(
                    self.a[
                        "x"
                    ],
                    self.a[
                        "y"
                    ],
                    b[
                        "x"
                    ],
                    b[
                        "y"
                    ]
                )
            )


            if (
                vehicle_spacing
                <= SHARED_HEADWAY_STOP_M
            ):

                return 0.0


            if (
                vehicle_spacing
                <= SHARED_HEADWAY_SLOW_M
            ):

                return min(
                    desired_speed,
                    B_MERGE_SLOW_SPEED_MPS
                )


            return min(
                desired_speed,
                B_SHARED_SPEED_MPS
            )


        return desired_speed


    def b_control(self, now):
        b = self.b
        if b['goal_reached']:
            return (0.0, 0.0)
        if self.b_dynamic_hold:
            if b['front_range'] >= B_DYNAMIC_RESUME_FRONT_M:
                if b['last_lidar_time'] != getattr(self, 'resume_scan_time', None):
                    self.b_dynamic_clear_count += 1
                    self.resume_scan_time = b['last_lidar_time']
            else:
                self.b_dynamic_clear_count = 0
            if self.b_dynamic_clear_count >= B_DYNAMIC_RESUME_CONFIRM_CYCLES:
                self.b_dynamic_hold = False
                self.b_dynamic_clear_count = 0
                self.b_dynamic_resume_events += 1
                print()
                print('*** B: MOVING OBSTACLE CLEARED - RESUMING ***')
                print(f"Front range             : {b['front_range']:.2f} m")
                print()
            else:
                return (0.0, 0.0)
        if b['front_range'] <= B_DYNAMIC_STOP_FRONT_M:
            self.b_dynamic_hold = True
            self.b_dynamic_hold_events += 1
            self.b_dynamic_clear_count = 0
            print()
            print('*** B: LOCAL LIDAR MOVING-OBSTACLE HOLD ***')
            print(f"Front range             : {b['front_range']:.2f} m")
            print('Local safety override   : STOP')
            print()
            return (0.0, 0.0)
        desired_speed = B_CRUISE_SPEED_MPS
        desired_speed = self.coordinator_b_speed(desired_speed)
        desired_speed = self.terrain_limited_speed(b, desired_speed)
        speed, steering, distance = self.waypoint_control(b, desired_speed)
        if not self.b_released and b['target_index'] == B_MERGE_INDEX and (distance <= WAYPOINT_REACHED_M):
            return (0.0, steering)
        if distance <= WAYPOINT_REACHED_M:
            self.advance_waypoint(b)
        if b['target_index'] == B_LOADING_INDEX:
            goal_distance = distance_2d(b['x'], b['y'], LOADING_ZONE[0], LOADING_ZONE[1])
            if goal_distance <= GOAL_REACHED_M:
                b['goal_reached'] = True
                print()
                print('*** B: SAME SUMMIT LOADING ZONE REACHED ***')
                print()
                return (0.0, 0.0)
        return (speed, steering)


    # ========================================================
    # SCRIPTED MOVING OBSTACLE
    # ========================================================









    # ========================================================
    # SAFETY / SHARED LOADING MONITOR
    # ========================================================

    def safety_ok(self):
        for label, state in (('A', self.a), ('B', self.b)):
            if not state.get('scan_valid', False):
                self.end_reason = f'vehicle_{label.lower()}_invalid_lidar'
                return False
            if abs(state['roll_deg']) > MAX_ABS_ROLL_DEG or abs(state['pitch_deg']) > MAX_ABS_PITCH_DEG:
                self.terrain_violation = True
                self.end_reason = f'vehicle_{label.lower()}_terrain_stop'
                return False
            if state['min_clearance'] < HARD_BODY_CLEARANCE_M:
                self.lidar_violation = True
                self.end_reason = f'vehicle_{label.lower()}_lidar_stop'
                print()
                print(f'*** {label}: 360 LIDAR HARD SAFETY STOP ***')
                print(f"Body clearance          : {state['min_clearance']:.3f} m")
                print()
                return False
        if self.invalid_scan_seen:
            self.end_reason = 'invalid_lidar_seen'
            return False
        clearance = scene_clearance(self.observed)
        if clearance is None:
            self.end_reason = 'ground_truth_missing'
            return False
        self.ground_min = min(self.ground_min, clearance)
        if self.ground_min < HARD_BODY_CLEARANCE_M:
            print(f'*** GROUND-TRUTH STOP: {self.ground_min_pair}: {self.ground_min:.6f} m < {HARD_BODY_CLEARANCE_M:.6f} m ***', flush=True)
            self.end_reason = 'ground_truth_clearance_violation'
            return False
        return True


    def loading_overlap_ok(
        self
    ):

        a_in = (
            distance_2d(
                self.a[
                    "x"
                ],
                self.a[
                    "y"
                ],
                LOADING_ZONE[
                    0
                ],
                LOADING_ZONE[
                    1
                ]
            )
            <= LOADING_ZONE_RADIUS_M
        )

        b_in = (
            distance_2d(
                self.b[
                    "x"
                ],
                self.b[
                    "y"
                ],
                LOADING_ZONE[
                    0
                ],
                LOADING_ZONE[
                    1
                ]
            )
            <= LOADING_ZONE_RADIUS_M
        )

        if (
            a_in
            and b_in
        ):

            self.loading_overlap_violation = True
            self.end_reason = (
                "loading_zone_overlap"
            )

            return False

        return True


    # ========================================================
    # MAIN
    # ========================================================

    def _run(self):
        print()
        print('============================================================')
        print(' RUSKOTUNTURI FINAL SIL DEMO v6.4 - SAME-SIDE SUMMIT MERGE')
        print('============================================================')
        print(f'Vehicle A waypoints     : {len(A_ROUTE)}')
        print(f'Vehicle B waypoints     : {len(B_ROUTE)}')
        print('Map usage               : same-side branches -> merge -> full shared climb')
        print(f'Vehicle A start         : {A_ROUTE[0]}')
        print(f'Vehicle B start         : {B_ROUTE[0]}')
        print(f'Merge point             : {MERGE_POINT}')
        print('Shared route            : validated baseline route from (-4,28) to (-40,-36)')
        print(f'Summit loading zone     : {LOADING_ZONE}')
        print('Static obstacle         : A private branch near (8,34)')
        print(f'A offset-line avoidance : {STATIC_OFFSET_M:.2f} m lateral')
        print('Moving obstacle         : B private branch crossing near (8,24)')
        print('Coordinator strategy    : dynamic B velocity profile, no fixed waiting area')
        print(f'Merge release headway   : {MERGE_RELEASE_HEADWAY_M:.2f} m')
        print(f'Hard body clearance     : {HARD_BODY_CLEARANCE_M:.2f} m')
        print('Camera fusion           : A static obstacle confirmation')
        print('Local safety            : independent 360 LiDAR on A and B')
        print()
        if not self.prepare_fresh_world():
            return self.print_result(0.0)
        self.harness = subprocess.Popen([sys.executable, str(Path(__file__).with_name('final_sil_v6_harness.py'))])
        start_time = self.sim_time
        if start_time is None:
            self.end_reason = 'clock_missing'
            return self.print_result(0.0)
        last_print = 0.0
        while True:
            now = self.sim_time
            wall_now = time.monotonic()
            if wall_now - self.clock_wall > SENSOR_TIMEOUT_S:
                self.end_reason = 'simulation_clock_stalled'
                break
            if self.watchdog.poll() is not None or self.harness.poll() is not None or (wall_now - self.harness_wall > 3.0 and now - start_time > 3.0):
                self.end_reason = 'harness_unavailable'
                break
            elapsed = now - start_time
            if elapsed > MISSION_TIMEOUT_S:
                self.end_reason = 'timeout'
                break
            with self.lock:
                a = dict(self.a)
                b = dict(self.b)
            stale = wall_now - a['last_pose_time'] > SENSOR_TIMEOUT_S or wall_now - a['last_lidar_time'] > SENSOR_TIMEOUT_S or wall_now - b['last_pose_time'] > SENSOR_TIMEOUT_S or (wall_now - b['last_lidar_time'] > SENSOR_TIMEOUT_S)
            if self.moving_pose_failures:
                self.end_reason = 'invalid_scenario_pose_update'
                break
            if now < start_time:
                self.end_reason = 'unexpected_simulation_reset'
                break
            if stale:
                self.end_reason = 'sensor_timeout'
                break
            with self.lock:
                if not self.safety_ok():
                    break
                if not self.loading_overlap_ok():
                    break
            pass  # Scenario runs in its own process.
            with self.lock:
                a_speed, a_steering = self.a_control(now)
                b_speed, b_steering = self.b_control(now)
            if self.trace_writer is not None:
                self.trace_writer.writerow([now, a['x'], a['y'], b['x'], b['y'],
                    a_speed, a_steering, b_speed, b_steering,
                    a['front_range'], b['front_range'], self.ground_min,
                    self.a_mode, self.b_dynamic_hold, self.b_released,
                    self.moving_started, self.moving_completed, self.moving_pose_failures,
                    a['yaw'], b['yaw'], a['roll_deg'], a['pitch_deg'], b['roll_deg'], b['pitch_deg'],
                    self.a_detour_phase, self.static_line_project(a['x'], a['y'])[1],
                    self.static_desired_offset_m, self.ground_min_pair])
                self.trace_handle.flush()
            self.publish(self.pub_a, a_speed, a_steering)
            self.publish(self.pub_b, b_speed, b_steering)
            with self.lock:
                done = self.a['goal_reached'] and self.b['goal_reached']
            if done:
                self.end_reason = 'final_demo_complete'
                print()
                print('*** FINAL SIL DEMO COMPLETE ***')
                print('A and B used the same final route; A exited after the loading zone and B reached it.')
                print()
                break
            if now - last_print >= PRINT_INTERVAL_S:
                with self.lock:
                    print(f"A {self.a_mode:13s}/{self.a_detour_phase} wp={self.a['target_index'] + 1:02d}/{len(A_ROUTE):02d} x={self.a['x']:6.1f} y={self.a['y']:6.1f} F={self.a['front_range']:4.1f} clr={self.a['min_clearance']:4.1f} | B wp={self.b['target_index'] + 1:02d}/{len(B_ROUTE):02d} x={self.b['x']:6.1f} y={self.b['y']:6.1f} F={self.b['front_range']:4.1f} clr={self.b['min_clearance']:4.1f} | coord={('SHARED_ROUTE' if self.b_released else 'MERGE_STOP' if self.b_waiting else 'A_PRIORITY_PROFILE')} | moving={('DONE' if self.moving_completed else 'ACTIVE' if self.moving_started else 'WAIT')}")
                last_print = now
            time.sleep(CONTROL_PERIOD_S)
        self.stop_repeatedly()
        duration = self.sim_time - start_time
        return self.print_result(duration)


    # ========================================================
    # RESULT
    # ========================================================

    def write_result(self, passed, duration):
        RESULT_FILE.parent.mkdir(parents=True, exist_ok=True)
        exists = RESULT_FILE.exists()
        row = {'schema_version': '6.4', 'failed_checks': ','.join(getattr(self, 'failed_checks', [])), 'loading_gate_hold_events': self.loading_gate_hold_events, 'loading_gate_clearance_m': self.loading_gate_clearance, 'ground_truth_min_pair': self.ground_min_pair, 'trace_file': self.trace_path, 'run_valid': self.moving_pose_failures == 0 and self.end_reason not in ('harness_unavailable', 'clock_missing', 'ground_truth_missing', 'simulation_clock_stalled', 'unexpected_simulation_reset', 'reset_failed', 'fresh_world_not_ready', 'initial_scene_mismatch'), 'encounter_stop_verified': self.encounter_stop_verified, 'encounter_resume_verified': self.encounter_resume_verified, 'ground_truth_min_clearance_m': self.ground_min, 'validation_scope': 'conservative_planar_envelopes', 'timestamp': datetime.now().isoformat(timespec='seconds'), 'result': 'PASS' if passed else 'FAIL', 'duration_s': f'{duration:.3f}', 'vehicle_a_exit_reached': self.a['goal_reached'], 'vehicle_a_loading_visited': self.a_loading_arrived, 'vehicle_b_loading_reached': self.b['goal_reached'], 'static_avoidance_completed': self.static_avoidance_completed, 'camera_used_for_control': self.camera_used_for_control, 'camera_fallback_used': self.camera_fallback_used, 'camera_target_frames': self.camera_target_frames, 'camera_max_orange_samples': self.camera_max_orange_samples, 'moving_obstacle_started': self.moving_started, 'moving_obstacle_completed': self.moving_completed, 'moving_pose_failures': self.moving_pose_failures, 'b_dynamic_hold_events': self.b_dynamic_hold_events, 'b_dynamic_resume_events': self.b_dynamic_resume_events, 'coordinator_profile_events': self.coordinator_profile_events, 'coordinator_stop_events': self.coordinator_stop_events, 'coordinator_release_events': self.coordinator_release_events, 'loading_overlap_violation': self.loading_overlap_violation, 'vehicle_a_min_clearance_m': f"{self.a['minimum_clearance_seen']:.3f}", 'vehicle_b_min_clearance_m': f"{self.b['minimum_clearance_seen']:.3f}", 'vehicle_a_max_roll_deg': f"{self.a['max_abs_roll_deg']:.3f}", 'vehicle_a_max_pitch_deg': f"{self.a['max_abs_pitch_deg']:.3f}", 'vehicle_b_max_roll_deg': f"{self.b['max_abs_roll_deg']:.3f}", 'vehicle_b_max_pitch_deg': f"{self.b['max_abs_pitch_deg']:.3f}", 'terrain_violation': self.terrain_violation, 'lidar_violation': self.lidar_violation, 'end_reason': self.end_reason}
        with RESULT_FILE.open('a', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            if not exists:
                writer.writeheader()
            writer.writerow(row)


    def print_result(self, duration):
        a_min = self.a['minimum_clearance_seen']
        b_min = self.b['minimum_clearance_seen']
        safety_margin_ok = margins_ok(a_min, b_min, self.ground_min)
        checks = {
            'observed_B_stop': self.encounter_stop_verified,
            'observed_B_resume': self.encounter_resume_verified,
            'healthy_lidar': not self.invalid_scan_seen,
            'ground_truth_available': math.isfinite(self.ground_min),
            'scenario_pose_delivery': self.moving_pose_failures == 0,
            'ground_truth_clearance': self.ground_min >= HARD_BODY_CLEARANCE_M,
            'A_exit_reached': self.a['goal_reached'],
            'A_loading_visited': self.a_loading_arrived,
            'B_loading_reached': self.b['goal_reached'],
            'static_avoidance_completed': self.static_avoidance_completed,
            'camera_used': self.camera_used_for_control,
            'no_camera_fallback': not self.camera_fallback_used,
            'moving_obstacle_started': self.moving_started,
            'moving_obstacle_completed': self.moving_completed,
            'B_hold_event': self.b_dynamic_hold_events >= 1,
            'B_resume_event': self.b_dynamic_resume_events >= 1,
            'merge_profile_event': self.coordinator_profile_events >= 1,
            'one_merge_release': self.coordinator_release_events == 1,
            'no_loading_overlap': not self.loading_overlap_violation,
            'terrain_safety': not self.terrain_violation,
            'lidar_safety': not self.lidar_violation,
            'combined_clearance': safety_margin_ok,
            'mission_complete': self.end_reason == 'final_demo_complete',
        }
        passed = all(checks.values())
        self.failed_checks = [name for name, ok in checks.items() if not ok]
        self.write_result(passed, duration)
        print('Unmet PASS conditions    : ' + (', '.join(self.failed_checks) or 'NONE'))
        print()
        print('============================================================')
        print(' RUSKOTUNTURI FINAL SIL DEMO v6.4 RESULT')
        print('============================================================')
        print(f"Vehicle A exit reached   : {('YES' if self.a['goal_reached'] else 'NO')}")
        print(f"A visited loading zone   : {('YES' if self.a_loading_arrived else 'NO')}")
        print(f"Vehicle B loading reached: {('YES' if self.b['goal_reached'] else 'NO')}")
        print(f"Static avoidance done    : {('YES' if self.static_avoidance_completed else 'NO')}")
        print(f"Camera used for control  : {('YES' if self.camera_used_for_control else 'NO')}")
        print(f"Camera fallback used     : {('YES' if self.camera_fallback_used else 'NO')}")
        print(f'Camera target frames     : {self.camera_target_frames}')
        print(f"Moving obstacle completed: {('YES' if self.moving_completed else 'NO')}")
        print(f'Moving pose failures     : {self.moving_pose_failures}')
        print(f'B dynamic hold events    : {self.b_dynamic_hold_events}')
        print(f'B dynamic resume events  : {self.b_dynamic_resume_events}')
        print(f'Coordinator profile events: {self.coordinator_profile_events}')
        print(f'Coordinator merge stops  : {self.coordinator_stop_events}')
        print(f'Coordinator merge release: {self.coordinator_release_events}')
        print(f'Loading approach holds   : {self.loading_gate_hold_events}')
        print(f"Loading-zone overlap     : {('YES' if self.loading_overlap_violation else 'NO')}")
        print(f'Vehicle A min clearance  : {a_min:.3f} m')
        print(f'Vehicle B min clearance  : {b_min:.3f} m')
        print(f'Ground-truth minimum    : {self.ground_min:.3f} m')
        print(f'Closest recorded pair   : {self.ground_min_pair}')
        print(f'Required clearance       : {HARD_BODY_CLEARANCE_M:.3f} m')
        print(f"Safety margin OK         : {('YES' if safety_margin_ok else 'NO')}")
        print(f"Vehicle A max roll       : {self.a['max_abs_roll_deg']:.3f} deg")
        print(f"Vehicle A max pitch      : {self.a['max_abs_pitch_deg']:.3f} deg")
        print(f"Vehicle B max roll       : {self.b['max_abs_roll_deg']:.3f} deg")
        print(f"Vehicle B max pitch      : {self.b['max_abs_pitch_deg']:.3f} deg")
        print(f"Terrain safety violation : {('YES' if self.terrain_violation else 'NO')}")
        print(f"360 LiDAR violation      : {('YES' if self.lidar_violation else 'NO')}")
        print(f'End reason               : {self.end_reason}')
        print(f'Duration                 : {duration:.2f} s')
        print()
        print('RESULT: ' + ('PASS' if passed else 'FAIL'))
        print('============================================================')
        return passed


    def clock_callback(self, msg):
        value = msg.sim.sec + msg.sim.nsec * 1e-9
        if value != self.sim_time:
            self.clock_wall = time.monotonic()
        self.sim_time = value

    def harness_callback(self, msg):
        try:
            status = json.loads(msg.data)
            with self.lock:
                self.encounter_stop_verified = bool(status['stop_verified'])
                self.encounter_resume_verified = bool(status['resume_verified'])
                self.moving_started = bool(status['started'])
                self.moving_completed = bool(status['completed'])
                self.moving_pose_failures = int(status['failures'])
                self.harness_wall = time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass  # Missing valid heartbeat terminates the run.

    def run(self):
        try:
            RESULT_FILE.parent.mkdir(parents=True, exist_ok=True)
            trace = RESULT_FILE.parent / ('final_sil_v6_4_trace_' + datetime.now().strftime('%Y%m%dT%H%M%S_%f') + '.csv')
            self.trace_path = trace.name
            self.trace_handle = trace.open('w', newline='', encoding='utf-8')
            self.trace_writer = csv.writer(self.trace_handle)
            self.trace_writer.writerow(['sim_time_s','a_x','a_y','b_x','b_y','a_requested_speed','a_requested_steering','b_requested_speed','b_requested_steering','a_front_range','b_front_range','minimum_ground_clearance_so_far','a_mode','b_hold','b_merge_released','obstacle_started','obstacle_completed','pose_update_failures','a_yaw_rad','b_yaw_rad','a_roll_deg','a_pitch_deg','b_roll_deg','b_pitch_deg','a_detour_phase','a_actual_offset_m','a_target_offset_m','ground_min_pair'])
            self.watchdog = subprocess.Popen([sys.executable, str(Path(__file__).with_name('final_sil_v6_watchdog.py'))])
            return self._run()
        except (Exception, KeyboardInterrupt) as error:
            self.end_reason = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'controller_exception_' + type(error).__name__
            self.print_result(0.0)
            if isinstance(error, KeyboardInterrupt):
                return False
            raise
        finally:
            for _ in range(10):
                for pub in self.emergency_pubs:
                    self.publish(pub, 0.0, 0.0)
                self.stop_both()
                time.sleep(.02)
            self.stop_repeatedly()
            time.sleep(.35)  # Let arbiter relay zero before terminating.
            for process in (self.watchdog, self.harness):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            if self.trace_handle is not None:
                self.trace_handle.close()



# ============================================================
# MAIN
# ============================================================

def main():

    controller = (
        FinalSilDemoLargeArea()
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

