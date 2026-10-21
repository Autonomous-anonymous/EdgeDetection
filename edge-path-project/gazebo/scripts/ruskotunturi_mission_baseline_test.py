#!/usr/bin/env python3

import csv
import math
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


# ============================================================
# WORLD / MODEL / TOPICS
# ============================================================

WORLD_NAME = "ruskotunturi_mission_world"
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


# ============================================================
# MISSION ROUTE
# ============================================================

# Phase 1 route:
# lower Ruskotunturi area -> upper Ruskotunturi area.
#
# The route was selected from the supplied collision mesh using
# terrain elevation / slope information. Artificial obstacles are
# added only after this baseline route has been validated.

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

START_TERRAIN_Z_M = -10.517
GOAL_TERRAIN_Z_M = 4.306

PLANNED_ELEVATION_GAIN_M = (
    GOAL_TERRAIN_Z_M
    - START_TERRAIN_Z_M
)


# ============================================================
# CONTROLLER SETTINGS
# ============================================================

WAYPOINT_REACHED_M = 2.0
GOAL_REACHED_M = 2.5

CRUISE_SPEED_MPS = 1.20
MODERATE_TURN_SPEED_MPS = 0.80
SHARP_TURN_SPEED_MPS = 0.50
TERRAIN_REDUCED_SPEED_MPS = 0.75

HEADING_GAIN = 1.55
MAX_STEERING_COMMAND = 0.40

MODERATE_TURN_DEG = 22.0
SHARP_TURN_DEG = 45.0

CONTROL_PERIOD_S = 0.05
PRINT_INTERVAL_S = 1.0

MISSION_TIMEOUT_S = 170.0
RESET_TIMEOUT_S = 12.0
SETTLE_TIME_S = 1.0

POSE_TIMEOUT_S = 1.0
LIDAR_TIMEOUT_S = 1.0
CAMERA_TIMEOUT_S = 1.5


# ============================================================
# TERRAIN / FAIL-SAFE LIMITS
# ============================================================

MAX_ABS_ROLL_DEG = 20.0
MAX_ABS_PITCH_DEG = 25.0

TERRAIN_SLOW_ROLL_DEG = 12.0
TERRAIN_SLOW_PITCH_DEG = 15.0

# Phase 1 has no artificial obstacles. This is only a final
# emergency stop for an unexpectedly very close front return.
FRONT_EMERGENCY_RANGE_M = 0.60


# ============================================================
# RESULTS
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "ruskotunturi_mission_baseline_results.csv"
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


# ============================================================
# MISSION CONTROLLER
# ============================================================

class RuskotunturiMissionBaseline:

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
                "Failed to subscribe to world pose topic: "
                + POSE_TOPIC
            )


        if not self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to LiDAR topic: "
                + LIDAR_TOPIC
            )


        if not self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.camera_callback
        ):

            raise RuntimeError(
                "Failed to subscribe to camera topic: "
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

            self.front_range_m = math.inf
            self.minimum_front_range_m = math.inf

            self.minimum_z_m = None
            self.maximum_z_m = None

            self.max_abs_roll_deg = 0.0
            self.max_abs_pitch_deg = 0.0

            # Waypoint 0 is the spawn / start point.
            self.target_waypoint_index = 1
            self.reached_waypoints = 1

            self.goal_reached = False
            self.terrain_safety_violation = False
            self.emergency_front_stop = False


    # ========================================================
    # CALLBACKS
    # ========================================================

    def pose_callback(
        self,
        msg
    ):

        selected = None


        for pose in msg.pose:

            if pose_name_matches(
                pose.name
            ):

                selected = pose
                break


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


            self.pose_received = True
            self.last_pose_time = now


    def lidar_callback(
        self,
        msg
    ):

        front_values = []


        for index, value in enumerate(
            msg.ranges
        ):

            if not math.isfinite(
                value
            ):

                continue


            if (
                value
                < msg.range_min
                or value
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


            if (
                -12.0
                <= angle_deg
                <= 12.0
            ):

                front_values.append(
                    value
                )


        front_range = (
            min(
                front_values
            )
            if front_values
            else msg.range_max
        )


        now = time.monotonic()


        with self.lock:

            self.front_range_m = front_range

            self.minimum_front_range_m = min(
                self.minimum_front_range_m,
                front_range
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
    # COMMANDS
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
    # WORLD CONTROL
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
            "Resetting Ruskotunturi mission world..."
        )


        self.stop_repeatedly()
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
                    "ERROR: sensor / pose reset timeout"
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

                    self.minimum_front_range_m = (
                        self.front_range_m
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
    # WAYPOINT FOLLOWING
    # ========================================================

    def current_target(
        self
    ):

        with self.lock:

            index = (
                self.target_waypoint_index
            )


        return WAYPOINTS[
            index
        ]


    def advance_waypoint_if_reached(
        self
    ):

        with self.lock:

            x = self.x
            y = self.y
            index = self.target_waypoint_index


        while index < len(
            WAYPOINTS
        ):

            target_x, target_y = (
                WAYPOINTS[
                    index
                ]
            )


            distance = distance_2d(
                x,
                y,
                target_x,
                target_y
            )


            is_last = (
                index
                == len(
                    WAYPOINTS
                )
                - 1
            )


            threshold = (
                GOAL_REACHED_M
                if is_last
                else WAYPOINT_REACHED_M
            )


            if distance > threshold:

                break


            if is_last:

                with self.lock:

                    self.reached_waypoints = len(
                        WAYPOINTS
                    )

                    self.goal_reached = True


                print()
                print(
                    "*** MISSION GOAL AREA REACHED ***"
                )

                return


            index += 1


            with self.lock:

                self.target_waypoint_index = (
                    index
                )

                self.reached_waypoints = max(
                    self.reached_waypoints,
                    index
                )


            print(
                f"Waypoint reached -> "
                f"next {index + 1}/{len(WAYPOINTS)}"
            )


    def steering_and_speed(
        self
    ):

        with self.lock:

            x = self.x
            y = self.y
            yaw = self.yaw_rad

            roll_deg = self.roll_deg
            pitch_deg = self.pitch_deg

            index = self.target_waypoint_index


        target_x, target_y = (
            WAYPOINTS[
                index
            ]
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
            "waypoints_reached",
            "waypoints_total",
            "goal_reached",
            "camera_frames",
            "max_abs_roll_deg",
            "max_abs_pitch_deg",
            "world_z_variation_m",
            "planned_elevation_gain_m",
            "minimum_front_lidar_range_m",
            "terrain_safety_violation",
            "emergency_front_stop",
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

                    "waypoints_reached":
                        self.reached_waypoints,

                    "waypoints_total":
                        len(
                            WAYPOINTS
                        ),

                    "goal_reached":
                        self.goal_reached,

                    "camera_frames":
                        self.camera_frames,

                    "max_abs_roll_deg":
                        f"{self.max_abs_roll_deg:.3f}",

                    "max_abs_pitch_deg":
                        f"{self.max_abs_pitch_deg:.3f}",

                    "world_z_variation_m":
                        f"{z_variation:.3f}",

                    "planned_elevation_gain_m":
                        f"{PLANNED_ELEVATION_GAIN_M:.3f}",

                    "minimum_front_lidar_range_m":
                        (
                            ""
                            if not math.isfinite(
                                self.minimum_front_range_m
                            )
                            else
                            f"{self.minimum_front_range_m:.3f}"
                        ),

                    "terrain_safety_violation":
                        self.terrain_safety_violation,

                    "emergency_front_stop":
                        self.emergency_front_stop,

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
            " RUSKOTUNTURI AUTONOMOUS MISSION - BASELINE"
        )

        print(
            "============================================================"
        )

        print(
            f"Waypoints               : "
            f"{len(WAYPOINTS)}"
        )

        print(
            f"Start terrain elevation : "
            f"{START_TERRAIN_Z_M:.2f} m"
        )

        print(
            f"Goal terrain elevation  : "
            f"{GOAL_TERRAIN_Z_M:.2f} m"
        )

        print(
            f"Planned elevation gain  : "
            f"{PLANNED_ELEVATION_GAIN_M:.2f} m"
        )

        print(
            "Artificial obstacles    : NONE (Phase 1)"
        )

        print(
            "360 LiDAR               : ACTIVE"
        )

        print(
            "Camera                   : ACTIVE, not used for control"
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

                x = self.x
                y = self.y
                z = self.z

                roll_deg = self.roll_deg
                pitch_deg = self.pitch_deg

                front_range = self.front_range_m

                target_index = (
                    self.target_waypoint_index
                )

                camera_frames = (
                    self.camera_frames
                )

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


            if not (
                pose_fresh
                and lidar_fresh
                and camera_fresh
            ):

                self.stop_vehicle()
                end_reason = "sensor_timeout"

                break


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
                self.stop_vehicle()

                end_reason = (
                    "terrain_attitude_stop"
                )

                print()
                print(
                    "*** TERRAIN SAFETY STOP ***"
                )

                print(
                    f"roll={roll_deg:.2f} deg | "
                    f"pitch={pitch_deg:.2f} deg"
                )

                print()

                break


            if (
                front_range
                <= FRONT_EMERGENCY_RANGE_M
            ):

                self.emergency_front_stop = True
                self.stop_vehicle()

                end_reason = (
                    "unexpected_front_emergency"
                )

                print()
                print(
                    "*** UNEXPECTED FRONT EMERGENCY STOP ***"
                )

                print(
                    f"Front LiDAR range: "
                    f"{front_range:.3f} m"
                )

                print()

                break


            self.advance_waypoint_if_reached()


            with self.lock:

                goal_reached = (
                    self.goal_reached
                )


            if goal_reached:

                end_reason = (
                    "goal_reached"
                )

                break


            speed, steering, heading_error_deg = (
                self.steering_and_speed()
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


                print(
                    f"WP {target_index + 1:02d}/"
                    f"{len(WAYPOINTS):02d} | "
                    f"x={x:6.1f} y={y:6.1f} "
                    f"z={z:6.1f} | "
                    f"d={target_distance:4.1f} m | "
                    f"err={heading_error_deg:4.1f} deg | "
                    f"v={speed:3.1f} | "
                    f"roll={roll_deg:5.1f} "
                    f"pitch={pitch_deg:5.1f} | "
                    f"front={front_range:4.1f} m"
                )


                last_print = now


            time.sleep(
                CONTROL_PERIOD_S
            )


        self.stop_repeatedly()


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


        passed = (
            self.goal_reached
            and self.reached_waypoints
            == len(
                WAYPOINTS
            )
            and camera_active
            and not self.terrain_safety_violation
            and not self.emergency_front_stop
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
            " RUSKOTUNTURI AUTONOMOUS MISSION RESULT"
        )

        print(
            "============================================================"
        )

        print(
            f"Waypoints reached        : "
            f"{self.reached_waypoints}/"
            f"{len(WAYPOINTS)}"
        )

        print(
            f"Goal reached             : "
            f"{'YES' if self.goal_reached else 'NO'}"
        )

        print(
            f"Planned elevation gain   : "
            f"{PLANNED_ELEVATION_GAIN_M:.3f} m"
        )

        print(
            f"World Z variation        : "
            f"{z_variation:.3f} m"
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
            f"Minimum front LiDAR      : "
            f"{self.minimum_front_range_m:.3f} m"
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
            f"Terrain safety violation : "
            f"{'YES' if self.terrain_safety_violation else 'NO'}"
        )

        print(
            f"Emergency front stop     : "
            f"{'YES' if self.emergency_front_stop else 'NO'}"
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
        RuskotunturiMissionBaseline()
    )

    success = test.run()

    raise SystemExit(
        0 if success else 1
    )


if __name__ == "__main__":

    main()
