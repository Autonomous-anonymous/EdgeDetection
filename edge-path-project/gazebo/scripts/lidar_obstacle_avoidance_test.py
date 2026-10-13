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

WORLD_NAME = "lidar_avoidance_world"

LIDAR_TOPIC = "/aeb/lidar"

ODOMETRY_TOPIC = (
    "/model/aeb_vehicle/odometry"
)

CMD_TOPIC = (
    "/model/aeb_vehicle/cmd_vel"
)


# ============================================================
# DRIVING
# ============================================================

CRUISE_SPEED_MPS = 1.5

AVOIDANCE_SPEED_MPS = 1.0

RETURN_SPEED_MPS = 1.2

GOAL_X_M = 29.0


# ============================================================
# LIDAR DETECTION
# ============================================================

OBSTACLE_DETECTION_DISTANCE_M = 7.0

FRONT_SECTOR_DEG = 10.0

SIDE_SECTOR_MIN_DEG = 20.0
SIDE_SECTOR_MAX_DEG = 70.0

MIN_VALID_RANGE_M = 0.10


# ============================================================
# AVOIDANCE
# ============================================================

AVOIDANCE_OFFSET_M = 2.6

LANE_TOLERANCE_M = 0.30

# After avoidance starts, vehicle must travel this much
# longitudinal distance before returning to the original lane.
MIN_PASS_DISTANCE_M = 10.0

LOOKAHEAD_X_M = 4.0

HEADING_GAIN = 1.8

MAX_STEERING_COMMAND = 0.40


# ============================================================
# AEB SAFETY OVERRIDE
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

EMERGENCY_DISTANCE_M = 0.70

CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# TEST / SAFETY
# ============================================================

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.40

TEST_TIMEOUT_SECONDS = 45.0

SENSOR_TIMEOUT_SECONDS = 0.75

RESET_TIMEOUT_SECONDS = 10.0

MAX_ALLOWED_ABS_Y_M = 5.0

MAX_ALLOWED_X_M = 36.0


# Known obstacle geometry is used only by the TEST HARNESS
# for result validation, not by the controller decision.
OBSTACLE_CENTER_X_M = 15.0
OBSTACLE_CENTER_Y_M = 0.0

MIN_ALLOWED_OBSTACLE_CENTER_DISTANCE_M = 2.0


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "lidar_obstacle_avoidance_results.csv"
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
# TEST
# ============================================================

class LidarObstacleAvoidanceTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()


        # ----------------------------------------------------
        # Vehicle state
        # ----------------------------------------------------

        self.x = None
        self.y = None

        self.yaw = 0.0

        self.speed = 0.0


        # ----------------------------------------------------
        # LiDAR sectors
        # ----------------------------------------------------

        self.front_distance = None

        self.left_clearance = None

        self.right_clearance = None


        # ----------------------------------------------------
        # Relative-speed estimation
        # ----------------------------------------------------

        self.previous_front_distance = None

        self.previous_lidar_time = None

        self.closing_speed = None


        # ----------------------------------------------------
        # Sensor status
        # ----------------------------------------------------

        self.odom_received = False

        self.lidar_received = False

        self.last_odom_wall_time = 0.0

        self.last_lidar_wall_time = 0.0


        # ----------------------------------------------------
        # Avoidance state machine
        # ----------------------------------------------------

        self.state = "CRUISE"

        self.avoidance_direction = None

        self.avoidance_target_y = 0.0

        self.avoidance_start_x = None

        self.obstacle_detected = False

        self.avoidance_started = False

        self.obstacle_passed = False

        self.returned_to_lane = False

        self.goal_reached = False


        # ----------------------------------------------------
        # Safety result
        # ----------------------------------------------------

        self.aeb_active = False

        self.aeb_interrupts = 0

        self.estimated_collision = False

        self.minimum_obstacle_center_distance = None

        self.minimum_front_lidar_distance = None


        # ----------------------------------------------------
        # Publisher
        # ----------------------------------------------------

        self.publisher = self.node.advertise(
            CMD_TOPIC,
            Twist
        )


        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

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
                "Failed to subscribe to LiDAR"
            )


    # ========================================================
    # ODOMETRY
    # ========================================================

    def odometry_callback(
        self,
        msg
    ):

        now = time.monotonic()


        with self.lock:

            self.x = (
                msg.pose.position.x
            )

            self.y = (
                msg.pose.position.y
            )

            self.yaw = quaternion_to_yaw(
                msg.pose.orientation
            )

            self.speed = math.hypot(
                msg.twist.linear.x,
                msg.twist.linear.y
            )

            self.odom_received = True

            self.last_odom_wall_time = (
                now
            )


    # ========================================================
    # LIDAR
    # ========================================================

    def lidar_callback(
        self,
        msg
    ):

        front_values = []

        left_values = []

        right_values = []


        for index, distance in enumerate(
            msg.ranges
        ):

            if not math.isfinite(
                distance
            ):

                continue


            if distance < max(
                MIN_VALID_RANGE_M,
                msg.range_min
            ):

                continue


            if distance > msg.range_max:

                continue


            angle_rad = (
                msg.angle_min
                + index
                * msg.angle_step
            )


            angle_deg = math.degrees(
                angle_rad
            )


            # Front
            if (
                abs(angle_deg)
                <= FRONT_SECTOR_DEG
            ):

                front_values.append(
                    distance
                )


            # Left
            if (
                SIDE_SECTOR_MIN_DEG
                <= angle_deg
                <= SIDE_SECTOR_MAX_DEG
            ):

                left_values.append(
                    distance
                )


            # Right
            if (
                -SIDE_SECTOR_MAX_DEG
                <= angle_deg
                <= -SIDE_SECTOR_MIN_DEG
            ):

                right_values.append(
                    distance
                )


        front = (
            min(front_values)
            if front_values
            else None
        )


        left = (
            min(left_values)
            if left_values
            else msg.range_max
        )


        right = (
            min(right_values)
            if right_values
            else msg.range_max
        )


        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec
            * 1e-9
        )


        wall_time = time.monotonic()


        with self.lock:

            # -----------------------------------------------
            # Closing speed from LiDAR range derivative
            # -----------------------------------------------

            if front is not None:

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


                    if (
                        dt > 0.01
                        and dt < 1.0
                    ):

                        raw_closing_speed = (
                            self.previous_front_distance
                            - front
                        ) / dt


                        raw_closing_speed = clamp(
                            raw_closing_speed,
                            -20.0,
                            20.0
                        )


                        if (
                            self.closing_speed
                            is None
                        ):

                            self.closing_speed = (
                                raw_closing_speed
                            )

                        else:

                            alpha = (
                                CLOSING_SPEED_FILTER_ALPHA
                            )


                            self.closing_speed = (
                                alpha
                                * raw_closing_speed
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


            self.front_distance = (
                front
            )

            self.left_clearance = (
                left
            )

            self.right_clearance = (
                right
            )


            if front is not None:

                if (
                    self.minimum_front_lidar_distance
                    is None

                    or front
                    < self.minimum_front_lidar_distance
                ):

                    self.minimum_front_lidar_distance = (
                        front
                    )


            self.lidar_received = True

            self.last_lidar_wall_time = (
                wall_time
            )


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


    # ========================================================
    # WORLD RESET
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

            self.x = None
            self.y = None

            self.yaw = 0.0
            self.speed = 0.0

            self.front_distance = None

            self.left_clearance = None
            self.right_clearance = None

            self.previous_front_distance = None
            self.previous_lidar_time = None

            self.closing_speed = None

            self.odom_received = False
            self.lidar_received = False

            self.last_odom_wall_time = 0.0
            self.last_lidar_wall_time = 0.0


            self.state = "CRUISE"

            self.avoidance_direction = None

            self.avoidance_target_y = 0.0

            self.avoidance_start_x = None

            self.obstacle_detected = False

            self.avoidance_started = False

            self.obstacle_passed = False

            self.returned_to_lane = False

            self.goal_reached = False


            self.aeb_active = False

            self.aeb_interrupts = 0

            self.estimated_collision = False

            self.minimum_obstacle_center_distance = None

            self.minimum_front_lidar_distance = None


    def reset_world(self):

        print(
            "Resetting LiDAR avoidance world..."
        )


        self.stop_repeatedly()

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

                ready = (
                    self.odom_received
                    and self.lidar_received
                )

                x = self.x
                y = self.y


            position_ok = (
                x is not None
                and y is not None

                and abs(x) < 0.75
                and abs(y) < 0.75
            )


            if ready and position_ok:

                print(
                    f"Reset OK | "
                    f"x={x:.2f} m | "
                    f"y={y:.2f} m"
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

    def update_aeb(self):

        with self.lock:

            distance = (
                self.front_distance
            )

            closing_speed = (
                self.closing_speed
            )

            currently_active = (
                self.aeb_active
            )


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

                self.aeb_active = True

                self.aeb_interrupts += 1


            print()
            print(
                "*** AEB SAFETY OVERRIDE ***"
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

                    self.aeb_active = False


        with self.lock:

            return self.aeb_active


    # ========================================================
    # STEERING
    # ========================================================

    def steering_to_lane(
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


        steering = clamp(
            HEADING_GAIN
            * heading_error,

            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND
        )


        return steering


    # ========================================================
    # AVOIDANCE STATE MACHINE
    # ========================================================

    def update_state_machine(self):

        with self.lock:

            x = self.x
            y = self.y

            front = (
                self.front_distance
            )

            left = (
                self.left_clearance
            )

            right = (
                self.right_clearance
            )

            state = self.state


        # ----------------------------------------------------
        # CRUISE -> obstacle detected
        # ----------------------------------------------------

        if state == "CRUISE":

            if (
                front is not None

                and front
                <= OBSTACLE_DETECTION_DISTANCE_M
            ):

                self.obstacle_detected = True

                self.avoidance_started = True

                self.avoidance_start_x = x


                if (
                    left is None
                    and right is None
                ):

                    direction = "LEFT"


                elif right is None:

                    direction = "LEFT"


                elif left is None:

                    direction = "RIGHT"


                elif left >= right:

                    direction = "LEFT"


                else:

                    direction = "RIGHT"


                self.avoidance_direction = (
                    direction
                )


                if direction == "LEFT":

                    self.avoidance_target_y = (
                        AVOIDANCE_OFFSET_M
                    )

                else:

                    self.avoidance_target_y = (
                        -AVOIDANCE_OFFSET_M
                    )


                self.state = "MOVE_OUT"


                print()
                print(
                    "*** OBSTACLE DETECTED ***"
                )

                print(
                    f"Front distance : "
                    f"{front:.2f} m"
                )

                print(
                    f"Left clearance : "
                    f"{left:.2f} m"
                )

                print(
                    f"Right clearance: "
                    f"{right:.2f} m"
                )

                print(
                    f"Avoidance side : "
                    f"{direction}"
                )

                print()


        # ----------------------------------------------------
        # MOVE_OUT -> side lane reached
        # ----------------------------------------------------

        elif state == "MOVE_OUT":

            if (
                abs(
                    y
                    - self.avoidance_target_y
                )
                <= LANE_TOLERANCE_M
            ):

                self.state = "PASS_OBSTACLE"


                print()
                print(
                    "*** AVOIDANCE LANE REACHED ***"
                )

                print(
                    f"y={y:.2f} m"
                )

                print()


        # ----------------------------------------------------
        # PASS -> enough longitudinal distance
        # ----------------------------------------------------

        elif state == "PASS_OBSTACLE":

            if (
                self.avoidance_start_x
                is not None

                and x
                >= (
                    self.avoidance_start_x
                    + MIN_PASS_DISTANCE_M
                )
            ):

                self.obstacle_passed = True

                self.state = "RETURN"


                print()
                print(
                    "*** OBSTACLE PASSED ***"
                )

                print(
                    f"x={x:.2f} m"
                )

                print(
                    "Returning to original lane."
                )

                print()


        # ----------------------------------------------------
        # RETURN -> center lane reached
        # ----------------------------------------------------

        elif state == "RETURN":

            if (
                abs(y)
                <= LANE_TOLERANCE_M
            ):

                self.returned_to_lane = True

                self.state = "FINAL"


                print()
                print(
                    "*** ORIGINAL LANE REACHED ***"
                )

                print(
                    f"x={x:.2f} m | "
                    f"y={y:.2f} m"
                )

                print()


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
            "obstacle_detected",
            "avoidance_started",
            "avoidance_direction",
            "obstacle_passed",
            "returned_to_lane",
            "goal_reached",
            "aeb_interrupts",
            "minimum_front_lidar_distance_m",
            "minimum_obstacle_center_distance_m",
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


            writer.writerow(
                {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),

                    "obstacle_detected":
                        self.obstacle_detected,

                    "avoidance_started":
                        self.avoidance_started,

                    "avoidance_direction":
                        self.avoidance_direction,

                    "obstacle_passed":
                        self.obstacle_passed,

                    "returned_to_lane":
                        self.returned_to_lane,

                    "goal_reached":
                        self.goal_reached,

                    "aeb_interrupts":
                        self.aeb_interrupts,

                    "minimum_front_lidar_distance_m":
                        (
                            ""
                            if self.minimum_front_lidar_distance
                            is None
                            else
                            f"{self.minimum_front_lidar_distance:.3f}"
                        ),

                    "minimum_obstacle_center_distance_m":
                        (
                            ""
                            if self.minimum_obstacle_center_distance
                            is None
                            else
                            f"{self.minimum_obstacle_center_distance:.3f}"
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
            " GAZEBO LIDAR OBSTACLE AVOIDANCE TEST"
        )
        print(
            "============================================================"
        )

        print(
            "Controller uses LiDAR to detect the obstacle"
        )

        print(
            "and choose the clearer avoidance side."
        )

        print(
            "Relative-TTC AEB remains active as safety override."
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

                    x = self.x
                    y = self.y

                    front = (
                        self.front_distance
                    )

                    left = (
                        self.left_clearance
                    )

                    right = (
                        self.right_clearance
                    )

                    current_state = (
                        self.state
                    )

                    sensor_ready = (
                        self.odom_received
                        and self.lidar_received
                    )

                    sensor_fresh = (
                        sensor_ready

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


                if not sensor_fresh:

                    self.stop_vehicle()

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # ------------------------------------------------
                # Test-harness obstacle distance
                # ------------------------------------------------

                obstacle_center_distance = (
                    distance_2d(
                        x,
                        y,
                        OBSTACLE_CENTER_X_M,
                        OBSTACLE_CENTER_Y_M
                    )
                )


                if (
                    self.minimum_obstacle_center_distance
                    is None

                    or obstacle_center_distance
                    < self.minimum_obstacle_center_distance
                ):

                    self.minimum_obstacle_center_distance = (
                        obstacle_center_distance
                    )


                if (
                    obstacle_center_distance
                    < MIN_ALLOWED_OBSTACLE_CENTER_DISTANCE_M
                ):

                    self.estimated_collision = True


                    print()
                    print(
                        "*** UNSAFE OBSTACLE PROXIMITY ***"
                    )

                    print(
                        f"Vehicle = "
                        f"({x:.2f}, {y:.2f})"
                    )

                    print(
                        f"Obstacle center distance = "
                        f"{obstacle_center_distance:.2f} m"
                    )

                    print()


                    end_reason = (
                        "unsafe_obstacle_proximity"
                    )

                    break


                # ------------------------------------------------
                # Route deviation protection
                # ------------------------------------------------

                if (
                    abs(y)
                    > MAX_ALLOWED_ABS_Y_M

                    or x
                    > MAX_ALLOWED_X_M
                ):

                    end_reason = (
                        "route_deviation"
                    )

                    print()
                    print(
                        "*** ROUTE DEVIATION ***"
                    )

                    print(
                        f"x={x:.2f} | "
                        f"y={y:.2f}"
                    )

                    print()

                    break


                # ------------------------------------------------
                # Goal
                # ------------------------------------------------

                if (
                    x >= GOAL_X_M

                    and abs(y)
                    <= 0.75
                ):

                    self.goal_reached = True

                    end_reason = (
                        "goal_reached"
                    )

                    break


                # ------------------------------------------------
                # State machine
                # ------------------------------------------------

                self.update_state_machine()


                with self.lock:

                    current_state = (
                        self.state
                    )


                # ------------------------------------------------
                # Safety override
                # ------------------------------------------------

                if self.update_aeb():

                    self.stop_vehicle()

                    drive_state = (
                        "AEB"
                    )


                else:

                    # --------------------------------------------
                    # Controller target
                    # --------------------------------------------

                    if current_state == "CRUISE":

                        target_y = 0.0

                        speed_command = (
                            CRUISE_SPEED_MPS
                        )


                    elif current_state == "MOVE_OUT":

                        target_y = (
                            self.avoidance_target_y
                        )

                        speed_command = (
                            AVOIDANCE_SPEED_MPS
                        )


                    elif current_state == "PASS_OBSTACLE":

                        target_y = (
                            self.avoidance_target_y
                        )

                        speed_command = (
                            AVOIDANCE_SPEED_MPS
                        )


                    elif current_state == "RETURN":

                        target_y = 0.0

                        speed_command = (
                            RETURN_SPEED_MPS
                        )


                    else:

                        target_y = 0.0

                        speed_command = (
                            CRUISE_SPEED_MPS
                        )


                    steering = (
                        self.steering_to_lane(
                            target_y
                        )
                    )


                    self.publish_command(
                        speed_command,
                        steering
                    )


                    drive_state = (
                        current_state
                    )


                # ------------------------------------------------
                # Status
                # ------------------------------------------------

                if (
                    now - last_print_time
                    >= PRINT_INTERVAL_SECONDS
                ):

                    if front is None:

                        front_text = "---"

                    else:

                        front_text = (
                            f"{front:.2f}"
                        )


                    if left is None:

                        left_text = "---"

                    else:

                        left_text = (
                            f"{left:.2f}"
                        )


                    if right is None:

                        right_text = "---"

                    else:

                        right_text = (
                            f"{right:.2f}"
                        )


                    print(
                        f"x={x:5.1f} "
                        f"y={y:5.2f} | "
                        f"{drive_state:13s} | "
                        f"front={front_text:>5s} | "
                        f"left={left_text:>5s} | "
                        f"right={right_text:>5s}"
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


        self.stop_repeatedly()


        duration = (
            time.monotonic()
            - start_time
        )


        passed = (
            self.obstacle_detected

            and self.avoidance_started

            and self.obstacle_passed

            and self.returned_to_lane

            and self.goal_reached

            and not self.estimated_collision

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
            " LIDAR OBSTACLE AVOIDANCE RESULT"
        )
        print(
            "============================================================"
        )

        print(
            f"Obstacle detected         : "
            f"{'YES' if self.obstacle_detected else 'NO'}"
        )

        print(
            f"Avoidance started         : "
            f"{'YES' if self.avoidance_started else 'NO'}"
        )

        print(
            f"Avoidance direction       : "
            f"{self.avoidance_direction}"
        )

        print(
            f"Obstacle passed           : "
            f"{'YES' if self.obstacle_passed else 'NO'}"
        )

        print(
            f"Returned to original lane : "
            f"{'YES' if self.returned_to_lane else 'NO'}"
        )

        print(
            f"Goal reached              : "
            f"{'YES' if self.goal_reached else 'NO'}"
        )

        print(
            f"AEB interrupts            : "
            f"{self.aeb_interrupts}"
        )


        if (
            self.minimum_front_lidar_distance
            is not None
        ):

            print(
                f"Minimum front LiDAR dist. : "
                f"{self.minimum_front_lidar_distance:.3f} m"
            )


        if (
            self.minimum_obstacle_center_distance
            is not None
        ):

            print(
                f"Minimum obstacle distance : "
                f"{self.minimum_obstacle_center_distance:.3f} m"
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

    test = LidarObstacleAvoidanceTest()

    success = test.run()


    if success:

        raise SystemExit(0)


    raise SystemExit(1)


if __name__ == "__main__":

    main()
