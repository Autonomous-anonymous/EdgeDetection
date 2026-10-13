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

WORLD_NAME = "lidar_avoidance_batch_world"

LIDAR_TOPIC = "/aeb/lidar"
ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
CMD_TOPIC = "/model/aeb_vehicle/cmd_vel"


# ============================================================
# SCENARIOS
# ============================================================

PARK_LEFT = (-15.0, 5.0, 0.75)
PARK_RIGHT = (-15.0, -5.0, 0.75)

LEFT_BLOCK_POSITION = (11.5, 3.0, 0.75)
RIGHT_BLOCK_POSITION = (11.5, -3.0, 0.75)


SCENARIOS = [
    {
        "name": "both_sides_open",
        "left_blocker": PARK_LEFT,
        "right_blocker": PARK_RIGHT,
        "expected_direction": "LEFT",
    },
    {
        "name": "left_side_blocked",
        "left_blocker": LEFT_BLOCK_POSITION,
        "right_blocker": PARK_RIGHT,
        "expected_direction": "RIGHT",
    },
    {
        "name": "right_side_blocked",
        "left_blocker": PARK_LEFT,
        "right_blocker": RIGHT_BLOCK_POSITION,
        "expected_direction": "LEFT",
    },
]


# ============================================================
# DRIVING
# ============================================================

CRUISE_SPEED_MPS = 1.5
AVOIDANCE_SPEED_MPS = 1.0
RETURN_SPEED_MPS = 1.2

GOAL_X_M = 29.0


# ============================================================
# LIDAR
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

MIN_PASS_DISTANCE_M = 10.0

LOOKAHEAD_X_M = 4.0

HEADING_GAIN = 1.8

MAX_STEERING_COMMAND = 0.40


# ============================================================
# AEB
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

EMERGENCY_DISTANCE_M = 0.70

CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# TEST
# ============================================================

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.50

SINGLE_TEST_TIMEOUT_SECONDS = 45.0

RESET_TIMEOUT_SECONDS = 10.0

SENSOR_TIMEOUT_SECONDS = 0.75

MAX_ALLOWED_ABS_Y_M = 5.0

MAX_ALLOWED_X_M = 36.0


# ============================================================
# TEST HARNESS OBSTACLE POSITIONS
# ============================================================

MAIN_OBSTACLE = (15.0, 0.0)

MIN_ALLOWED_OBSTACLE_CENTER_DISTANCE_M = 2.0


# ============================================================
# RESULTS
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "lidar_obstacle_avoidance_batch_results.csv"
)


# ============================================================
# HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def wrap_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def distance_2d(x1, y1, x2, y2):

    return math.hypot(
        x2 - x1,
        y2 - y1
    )


# ============================================================
# BATCH TEST
# ============================================================

class LidarAvoidanceBatchTest:

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
                "Failed to subscribe to LiDAR"
            )


        self.reset_internal_state()


    # ========================================================
    # STATE RESET
    # ========================================================

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

            self.minimum_front_lidar_distance = None
            self.minimum_obstacle_distance = None


    # ========================================================
    # ODOMETRY
    # ========================================================

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


    # ========================================================
    # LIDAR
    # ========================================================

    def lidar_callback(self, msg):

        front_values = []
        left_values = []
        right_values = []


        for index, distance in enumerate(
            msg.ranges
        ):

            if not math.isfinite(distance):
                continue

            if distance < max(
                MIN_VALID_RANGE_M,
                msg.range_min
            ):
                continue

            if distance > msg.range_max:
                continue


            angle = (
                msg.angle_min
                + index * msg.angle_step
            )

            angle_deg = math.degrees(
                angle
            )


            if (
                abs(angle_deg)
                <= FRONT_SECTOR_DEG
            ):

                front_values.append(
                    distance
                )


            if (
                SIDE_SECTOR_MIN_DEG
                <= angle_deg
                <= SIDE_SECTOR_MAX_DEG
            ):

                left_values.append(
                    distance
                )


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
            + msg.header.stamp.nsec * 1e-9
        )


        now = time.monotonic()


        with self.lock:

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


                        if self.closing_speed is None:

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
                                + (1.0 - alpha)
                                * self.closing_speed
                            )


                self.previous_front_distance = (
                    front
                )

                self.previous_lidar_time = (
                    sim_time
                )


            self.front_distance = front
            self.left_clearance = left
            self.right_clearance = right


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
            self.last_lidar_wall_time = now


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
    # GENERIC GAZEBO SERVICE
    # ========================================================

    def run_gz_service(
        self,
        service,
        reqtype,
        request
    ):

        command = [
            "gz",
            "service",

            "-s",
            service,

            "--reqtype",
            reqtype,

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


        if (
            result.returncode != 0
            or "data: true"
            not in output
        ):

            print(
                "Gazebo service failed:"
            )

            print(
                " ".join(command)
            )

            print(
                output
            )

            return False


        return True


    # ========================================================
    # WORLD CONTROL
    # ========================================================

    def reset_world(self):

        print()
        print(
            "Resetting world..."
        )


        self.stop_repeatedly()

        self.reset_internal_state()


        if not self.run_gz_service(
            f"/world/{WORLD_NAME}/control",
            "gz.msgs.WorldControl",
            "reset: {all: true}"
        ):

            return False


        if not self.run_gz_service(
            f"/world/{WORLD_NAME}/control",
            "gz.msgs.WorldControl",
            "pause: false"
        ):

            return False


        start = time.monotonic()


        while True:

            now = time.monotonic()


            if (
                now - start
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
                    f"x={x:.2f} m | "
                    f"y={y:.2f} m"
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # BLOCKER POSITIONS
    # ========================================================

    def set_model_pose(
        self,
        model_name,
        position
    ):

        x, y, z = position


        request = (
            f'name: "{model_name}", '
            f'position: {{'
            f'x: {x}, '
            f'y: {y}, '
            f'z: {z}'
            f'}}'
        )


        return self.run_gz_service(
            f"/world/{WORLD_NAME}/set_pose",
            "gz.msgs.Pose",
            request
        )


    def configure_scenario(
        self,
        scenario
    ):

        if not self.set_model_pose(
            "left_blocker",
            scenario["left_blocker"]
        ):

            return False


        if not self.set_model_pose(
            "right_blocker",
            scenario["right_blocker"]
        ):

            return False


        # Let Gazebo and LiDAR receive the new geometry.
        time.sleep(
            0.75
        )


        with self.lock:

            self.previous_front_distance = None
            self.previous_lidar_time = None
            self.closing_speed = None


        return True


    # ========================================================
    # AEB
    # ========================================================

    def update_aeb(self):

        with self.lock:

            distance = self.front_distance
            closing_speed = self.closing_speed
            currently_active = self.aeb_active


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
            x + LOOKAHEAD_X_M
        )


        desired_heading = math.atan2(
            target_y - y,
            target_x - x
        )


        heading_error = wrap_angle(
            desired_heading - yaw
        )


        return clamp(
            HEADING_GAIN
            * heading_error,
            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND
        )


    # ========================================================
    # AVOIDANCE STATE MACHINE
    # ========================================================

    def update_state_machine(self):

        with self.lock:

            x = self.x
            y = self.y

            front = self.front_distance
            left = self.left_clearance
            right = self.right_clearance

            state = self.state


        if state == "CRUISE":

            if (
                front is not None
                and front
                <= OBSTACLE_DETECTION_DISTANCE_M
            ):

                self.obstacle_detected = True
                self.avoidance_started = True

                self.avoidance_start_x = x


                if left >= right:

                    direction = "LEFT"

                    target_y = (
                        AVOIDANCE_OFFSET_M
                    )

                else:

                    direction = "RIGHT"

                    target_y = (
                        -AVOIDANCE_OFFSET_M
                    )


                self.avoidance_direction = (
                    direction
                )

                self.avoidance_target_y = (
                    target_y
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


        elif state == "MOVE_OUT":

            if (
                abs(
                    y
                    - self.avoidance_target_y
                )
                <= LANE_TOLERANCE_M
            ):

                self.state = (
                    "PASS_OBSTACLE"
                )


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


        elif state == "RETURN":

            if (
                abs(y)
                <= LANE_TOLERANCE_M
            ):

                self.returned_to_lane = True

                self.state = "FINAL"


    # ========================================================
    # ACTIVE OBSTACLES
    # ========================================================

    def active_obstacles(
        self,
        scenario
    ):

        obstacles = [
            MAIN_OBSTACLE
        ]


        left = scenario[
            "left_blocker"
        ]

        right = scenario[
            "right_blocker"
        ]


        # Only count blockers that are placed ahead
        # in the active test area.
        if left[0] > 0.0:

            obstacles.append(
                (left[0], left[1])
            )


        if right[0] > 0.0:

            obstacles.append(
                (right[0], right[1])
            )


        return obstacles


    # ========================================================
    # SINGLE SCENARIO
    # ========================================================

    def run_scenario(
        self,
        scenario
    ):

        print()
        print(
            "============================================================"
        )

        print(
            f" SCENARIO: {scenario['name']}"
        )

        print(
            "============================================================"
        )

        print(
            f"Expected direction: "
            f"{scenario['expected_direction']}"
        )


        if not self.reset_world():

            return self.failed_result(
                scenario,
                "reset_failed"
            )


        if not self.configure_scenario(
            scenario
        ):

            return self.failed_result(
                scenario,
                "scenario_configuration_failed"
            )


        start_time = time.monotonic()

        last_print_time = 0.0

        end_reason = None

        active_obstacles = (
            self.active_obstacles(
                scenario
            )
        )


        try:

            while True:

                now = time.monotonic()


                if (
                    now - start_time
                    > SINGLE_TEST_TIMEOUT_SECONDS
                ):

                    end_reason = "timeout"

                    break


                with self.lock:

                    x = self.x
                    y = self.y

                    front = self.front_distance
                    left = self.left_clearance
                    right = self.right_clearance

                    state = self.state

                    sensor_fresh = (
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


                if not sensor_fresh:

                    self.stop_vehicle()

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # --------------------------------------------
                # Obstacle safety check
                # --------------------------------------------

                nearest_obstacle = min(
                    distance_2d(
                        x,
                        y,
                        obstacle_x,
                        obstacle_y
                    )

                    for obstacle_x, obstacle_y
                    in active_obstacles
                )


                if (
                    self.minimum_obstacle_distance
                    is None

                    or nearest_obstacle
                    < self.minimum_obstacle_distance
                ):

                    self.minimum_obstacle_distance = (
                        nearest_obstacle
                    )


                if (
                    nearest_obstacle
                    < MIN_ALLOWED_OBSTACLE_CENTER_DISTANCE_M
                ):

                    self.estimated_collision = True

                    end_reason = (
                        "unsafe_obstacle_proximity"
                    )

                    break


                # --------------------------------------------
                # Route protection
                # --------------------------------------------

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


                # --------------------------------------------
                # Goal
                # --------------------------------------------

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


                # --------------------------------------------
                # State machine
                # --------------------------------------------

                self.update_state_machine()


                with self.lock:

                    state = self.state


                # --------------------------------------------
                # Safety override
                # --------------------------------------------

                if self.update_aeb():

                    self.stop_vehicle()

                    drive_state = "AEB"


                else:

                    if state == "CRUISE":

                        target_y = 0.0
                        speed = CRUISE_SPEED_MPS


                    elif state == "MOVE_OUT":

                        target_y = (
                            self.avoidance_target_y
                        )

                        speed = AVOIDANCE_SPEED_MPS


                    elif state == "PASS_OBSTACLE":

                        target_y = (
                            self.avoidance_target_y
                        )

                        speed = AVOIDANCE_SPEED_MPS


                    elif state == "RETURN":

                        target_y = 0.0
                        speed = RETURN_SPEED_MPS


                    else:

                        target_y = 0.0
                        speed = CRUISE_SPEED_MPS


                    steering = (
                        self.steering_to_lane(
                            target_y
                        )
                    )


                    self.publish_command(
                        speed,
                        steering
                    )


                    drive_state = state


                # --------------------------------------------
                # Status
                # --------------------------------------------

                if (
                    now - last_print_time
                    >= PRINT_INTERVAL_SECONDS
                ):

                    front_text = (
                        "---"
                        if front is None
                        else f"{front:.2f}"
                    )

                    left_text = (
                        "---"
                        if left is None
                        else f"{left:.2f}"
                    )

                    right_text = (
                        "---"
                        if right is None
                        else f"{right:.2f}"
                    )


                    print(
                        f"x={x:5.1f} "
                        f"y={y:5.2f} | "
                        f"{drive_state:13s} | "
                        f"front={front_text:>5s} | "
                        f"left={left_text:>5s} | "
                        f"right={right_text:>5s}"
                    )


                    last_print_time = now


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


        direction_correct = (
            self.avoidance_direction
            == scenario[
                "expected_direction"
            ]
        )


        passed = (
            self.obstacle_detected
            and self.avoidance_started
            and direction_correct
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
            f"Result: {result}"
        )

        print(
            f"Expected direction : "
            f"{scenario['expected_direction']}"
        )

        print(
            f"Actual direction   : "
            f"{self.avoidance_direction}"
        )

        print(
            f"Obstacle passed    : "
            f"{self.obstacle_passed}"
        )

        print(
            f"Returned to lane   : "
            f"{self.returned_to_lane}"
        )

        print(
            f"Goal reached       : "
            f"{self.goal_reached}"
        )

        print(
            f"AEB interrupts     : "
            f"{self.aeb_interrupts}"
        )

        print(
            f"Collision estimate : "
            f"{self.estimated_collision}"
        )

        print(
            f"End reason         : "
            f"{end_reason}"
        )


        return {
            "scenario":
                scenario["name"],

            "expected_direction":
                scenario["expected_direction"],

            "actual_direction":
                self.avoidance_direction,

            "obstacle_detected":
                self.obstacle_detected,

            "avoidance_started":
                self.avoidance_started,

            "obstacle_passed":
                self.obstacle_passed,

            "returned_to_lane":
                self.returned_to_lane,

            "goal_reached":
                self.goal_reached,

            "aeb_interrupts":
                self.aeb_interrupts,

            "minimum_front_lidar_distance_m":
                self.minimum_front_lidar_distance,

            "minimum_obstacle_distance_m":
                self.minimum_obstacle_distance,

            "estimated_collision":
                self.estimated_collision,

            "end_reason":
                end_reason,

            "duration_s":
                duration,

            "result":
                result
        }


    def failed_result(
        self,
        scenario,
        reason
    ):

        return {
            "scenario":
                scenario["name"],

            "expected_direction":
                scenario["expected_direction"],

            "actual_direction":
                None,

            "obstacle_detected":
                False,

            "avoidance_started":
                False,

            "obstacle_passed":
                False,

            "returned_to_lane":
                False,

            "goal_reached":
                False,

            "aeb_interrupts":
                0,

            "minimum_front_lidar_distance_m":
                None,

            "minimum_obstacle_distance_m":
                None,

            "estimated_collision":
                False,

            "end_reason":
                reason,

            "duration_s":
                0.0,

            "result":
                "FAIL"
        }


    # ========================================================
    # CSV
    # ========================================================

    def save_results(
        self,
        results
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
            "scenario",
            "expected_direction",
            "actual_direction",
            "obstacle_detected",
            "avoidance_started",
            "obstacle_passed",
            "returned_to_lane",
            "goal_reached",
            "aeb_interrupts",
            "minimum_front_lidar_distance_m",
            "minimum_obstacle_distance_m",
            "estimated_collision",
            "end_reason",
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


            for result in results:

                writer.writerow(
                    {
                        "timestamp":
                            datetime.now().isoformat(
                                timespec="seconds"
                            ),

                        "scenario":
                            result["scenario"],

                        "expected_direction":
                            result["expected_direction"],

                        "actual_direction":
                            result["actual_direction"],

                        "obstacle_detected":
                            result["obstacle_detected"],

                        "avoidance_started":
                            result["avoidance_started"],

                        "obstacle_passed":
                            result["obstacle_passed"],

                        "returned_to_lane":
                            result["returned_to_lane"],

                        "goal_reached":
                            result["goal_reached"],

                        "aeb_interrupts":
                            result["aeb_interrupts"],

                        "minimum_front_lidar_distance_m":
                            (
                                ""
                                if result[
                                    "minimum_front_lidar_distance_m"
                                ]
                                is None
                                else
                                f"{result['minimum_front_lidar_distance_m']:.3f}"
                            ),

                        "minimum_obstacle_distance_m":
                            (
                                ""
                                if result[
                                    "minimum_obstacle_distance_m"
                                ]
                                is None
                                else
                                f"{result['minimum_obstacle_distance_m']:.3f}"
                            ),

                        "estimated_collision":
                            result["estimated_collision"],

                        "end_reason":
                            result["end_reason"],

                        "duration_s":
                            f"{result['duration_s']:.3f}",

                        "result":
                            result["result"]
                    }
                )


    # ========================================================
    # SUMMARY
    # ========================================================

    def print_summary(
        self,
        results
    ):

        print()
        print(
            "======================================================================"
        )

        print(
            " LIDAR OBSTACLE AVOIDANCE BATCH SUMMARY"
        )

        print(
            "======================================================================"
        )

        print(
            "Scenario              Expected  Actual  AEB  Collision  Goal  Result"
        )

        print(
            "----------------------------------------------------------------------"
        )


        for result in results:

            actual = (
                result["actual_direction"]
                if result["actual_direction"]
                is not None
                else "---"
            )


            print(
                f"{result['scenario']:<21s} "
                f"{result['expected_direction']:<8s} "
                f"{actual:<7s} "
                f"{result['aeb_interrupts']:<4d} "
                f"{'YES' if result['estimated_collision'] else 'NO':<10s} "
                f"{'YES' if result['goal_reached'] else 'NO':<5s} "
                f"{result['result']}"
            )


        print(
            "======================================================================"
        )


        passed = sum(
            1
            for result in results
            if result["result"] == "PASS"
        )


        print(
            f"Passed: "
            f"{passed}/{len(results)}"
        )


        if (
            passed
            == len(results)
        ):

            print(
                "OVERALL RESULT: PASS"
            )

        else:

            print(
                "OVERALL RESULT: FAIL"
            )


        print(
            f"CSV: {RESULTS_FILE}"
        )

        print(
            "======================================================================"
        )


    # ========================================================
    # RUN BATCH
    # ========================================================

    def run(self):

        print()
        print(
            "======================================================================"
        )

        print(
            " GAZEBO LIDAR OBSTACLE AVOIDANCE BATCH TEST"
        )

        print(
            "======================================================================"
        )

        print(
            "Cases:"
        )

        print(
            "1. Both sides open -> LEFT"
        )

        print(
            "2. Left side blocked -> RIGHT"
        )

        print(
            "3. Right side blocked -> LEFT"
        )

        print()


        results = []


        try:

            for scenario in SCENARIOS:

                result = (
                    self.run_scenario(
                        scenario
                    )
                )

                results.append(
                    result
                )


        except KeyboardInterrupt:

            print()
            print(
                "Batch interrupted by user."
            )


        finally:

            self.stop_repeatedly()


        if results:

            self.save_results(
                results
            )

            self.print_summary(
                results
            )


        print()
        print(
            "Returning world to initial state..."
        )

        self.reset_world()

        self.stop_repeatedly()


        return (
            len(results)
            == len(SCENARIOS)

            and all(
                result["result"]
                == "PASS"

                for result in results
            )
        )


# ============================================================
# MAIN
# ============================================================

def main():

    test = (
        LidarAvoidanceBatchTest()
    )

    success = test.run()


    if success:

        raise SystemExit(0)


    raise SystemExit(1)


if __name__ == "__main__":

    main()
