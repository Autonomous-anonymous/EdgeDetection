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
# GAZEBO TOPICS / WORLD
# ============================================================

WORLD_NAME = "lidar_avoidance_advanced_world"
LIDAR_TOPIC = "/aeb/lidar"
ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
CMD_TOPIC = "/model/aeb_vehicle/cmd_vel"


# ============================================================
# SCENARIO MODEL POSITIONS
# ============================================================

PARKING = {
    "small_obstacle": (-15.0, 8.0, 0.60),
    "large_obstacle": (-15.0, 5.0, 0.75),
    "obstacle_one": (-15.0, 2.0, 0.75),
    "obstacle_two": (-15.0, -2.0, 0.75),
    "left_blocker": (-15.0, -5.0, 0.75),
    "right_blocker": (-15.0, -8.0, 0.75),
}

SCENARIOS = [
    {
        "name": "small_far",
        "positions": {
            "small_obstacle": (24.0, 0.0, 0.60),
        },
        "active_obstacles": [
            {
                "name": "small_obstacle",
                "x": 24.0,
                "y": 0.0,
                "length": 1.2,
                "width": 1.2,
            }
        ],
        "expected_outcome": "GOAL",
        "expected_avoidances": 1,
        "goal_x": 40.0,
    },
    {
        "name": "large_close",
        "positions": {
            "large_obstacle": (16.0, 0.0, 0.75),
        },
        "active_obstacles": [
            {
                "name": "large_obstacle",
                "x": 16.0,
                "y": 0.0,
                "length": 3.0,
                "width": 3.0,
            }
        ],
        "expected_outcome": "GOAL",
        "expected_avoidances": 1,
        "goal_x": 36.0,
    },
    {
        "name": "two_obstacles",
        "positions": {
            "obstacle_one": (14.0, 0.0, 0.75),
            "obstacle_two": (42.0, 0.0, 0.75),
        },
        "active_obstacles": [
            {
                "name": "obstacle_one",
                "x": 14.0,
                "y": 0.0,
                "length": 2.0,
                "width": 2.0,
            },
            {
                "name": "obstacle_two",
                "x": 42.0,
                "y": 0.0,
                "length": 2.0,
                "width": 2.0,
            },
        ],
        "expected_outcome": "GOAL",
        "expected_avoidances": 2,
        # The green goal zone is centered at x=58 m and is 5 m long.
        # Finishing at x=56 m means the car has entered the goal zone
        # after completing both avoidance manoeuvres, without wasting time
        # driving beyond the visible goal marker.
        "goal_x": 56.0,
    },
    {
        "name": "blocked_both_sides",
        "positions": {
            "obstacle_one": (15.0, 0.0, 0.75),
            "left_blocker": (11.5, 3.0, 0.75),
            "right_blocker": (11.5, -3.0, 0.75),
        },
        "active_obstacles": [
            {
                "name": "obstacle_one",
                "x": 15.0,
                "y": 0.0,
                "length": 2.0,
                "width": 2.0,
            },
            {
                "name": "left_blocker",
                "x": 11.5,
                "y": 3.0,
                "length": 2.0,
                "width": 1.8,
            },
            {
                "name": "right_blocker",
                "x": 11.5,
                "y": -3.0,
                "length": 2.0,
                "width": 1.8,
            },
        ],
        "expected_outcome": "AEB_STOP",
        "expected_avoidances": 0,
        "goal_x": None,
    },
]


# ============================================================
# DRIVING / AVOIDANCE
# ============================================================

# Faster than the previous version, but avoidance is still slower than cruise.
CRUISE_SPEED_MPS = 2.0
AVOIDANCE_SPEED_MPS = 1.4
RETURN_SPEED_MPS = 1.8

# Detect earlier because the car now travels faster and uses a larger sidestep.
OBSTACLE_DETECTION_DISTANCE_M = 11.0
FRONT_SECTOR_DEG = 10.0
SIDE_SECTOR_MIN_DEG = 20.0
SIDE_SECTOR_MAX_DEG = 70.0
MIN_VALID_RANGE_M = 0.10
MIN_SIDE_CLEARANCE_FOR_AVOIDANCE_M = 8.0

# The previous 2.8 m lateral offset was too small for the 3 m wide obstacle.
AVOIDANCE_OFFSET_M = 4.0
LANE_TOLERANCE_M = 0.40
PASS_DISTANCE_AFTER_DETECTION_M = 14.0
LOOKAHEAD_X_M = 4.0
HEADING_GAIN = 2.0
MAX_STEERING_COMMAND = 0.50


# ============================================================
# BODY SAFETY ENVELOPE
# ============================================================

# Approximate vehicle body footprint from aeb_vehicle/model.sdf.
VEHICLE_LENGTH_M = 2.6
VEHICLE_WIDTH_M = 1.2

# PASS requires at least this much free space between the vehicle body
# rectangle and every active obstacle rectangle, in every XY direction.
MIN_REQUIRED_BODY_CLEARANCE_M = 1.0
COLLISION_EPSILON_M = 0.02


# ============================================================
# AEB / TEST SETTINGS
# ============================================================

TTC_THRESHOLD_SECONDS = 1.7
MIN_CLOSING_SPEED_MPS = 0.10
EMERGENCY_DISTANCE_M = 0.70
CLOSING_SPEED_FILTER_ALPHA = 0.35
STOP_SPEED_THRESHOLD_MPS = 0.05
STOP_STABLE_TIME_SECONDS = 0.75

CONTROL_PERIOD_SECONDS = 0.05
PRINT_INTERVAL_SECONDS = 0.50
SINGLE_TEST_TIMEOUT_SECONDS = 70.0
RESET_TIMEOUT_SECONDS = 10.0
SENSOR_TIMEOUT_SECONDS = 0.75
MAX_ALLOWED_X_M = 74.0
MAX_ALLOWED_ABS_Y_M = 6.0

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"
RESULTS_FILE = RESULTS_DIR / "lidar_obstacle_avoidance_advanced_batch_results.csv"


# ============================================================
# GEOMETRY HELPERS
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
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rectangle_corners(cx, cy, length, width, yaw=0.0):
    hx = length / 2.0
    hy = width / 2.0
    local = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]
    c = math.cos(yaw)
    s = math.sin(yaw)

    return [
        (cx + lx * c - ly * s, cy + lx * s + ly * c)
        for lx, ly in local
    ]


def polygon_axes(points):
    axes = []
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        dx = x2 - x1
        dy = y2 - y1
        edge_length = math.hypot(dx, dy)
        if edge_length > 1e-12:
            axes.append((-dy / edge_length, dx / edge_length))
    return axes


def project_polygon(points, axis):
    ax, ay = axis
    values = [x * ax + y * ay for x, y in points]
    return min(values), max(values)


def polygons_overlap(poly_a, poly_b):
    for axis in polygon_axes(poly_a) + polygon_axes(poly_b):
        min_a, max_a = project_polygon(poly_a, axis)
        min_b, max_b = project_polygon(poly_b, axis)
        if max_a < min_b or max_b < min_a:
            return False
    return True


def point_segment_distance(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    length_sq = abx * abx + aby * aby

    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = clamp(
        ((px - ax) * abx + (py - ay) * aby) / length_sq,
        0.0,
        1.0,
    )
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return math.hypot(px - closest_x, py - closest_y)


def polygon_distance(poly_a, poly_b):
    if polygons_overlap(poly_a, poly_b):
        return 0.0

    minimum = math.inf

    for px, py in poly_a:
        for index in range(len(poly_b)):
            ax, ay = poly_b[index]
            bx, by = poly_b[(index + 1) % len(poly_b)]
            minimum = min(
                minimum,
                point_segment_distance(px, py, ax, ay, bx, by),
            )

    for px, py in poly_b:
        for index in range(len(poly_a)):
            ax, ay = poly_a[index]
            bx, by = poly_a[(index + 1) % len(poly_a)]
            minimum = min(
                minimum,
                point_segment_distance(px, py, ax, ay, bx, by),
            )

    return minimum


def body_clearance(vehicle_x, vehicle_y, vehicle_yaw, obstacle):
    vehicle_poly = rectangle_corners(
        vehicle_x,
        vehicle_y,
        VEHICLE_LENGTH_M,
        VEHICLE_WIDTH_M,
        vehicle_yaw,
    )
    obstacle_poly = rectangle_corners(
        obstacle["x"],
        obstacle["y"],
        obstacle["length"],
        obstacle["width"],
        0.0,
    )
    return polygon_distance(vehicle_poly, obstacle_poly)


# ============================================================
# ADVANCED BATCH TEST
# ============================================================


class AdvancedLidarAvoidanceBatchTest:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()
        self.publisher = self.node.advertise(CMD_TOPIC, Twist)

        if not self.node.subscribe(
            Odometry,
            ODOMETRY_TOPIC,
            self.odometry_callback,
        ):
            raise RuntimeError("Failed to subscribe to odometry")

        if not self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback,
        ):
            raise RuntimeError("Failed to subscribe to LiDAR")

        self.reset_internal_state()

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
            self.avoidance_target_y = 0.0
            self.avoidance_start_x = None
            self.avoidance_directions = []
            self.avoidance_count = 0
            self.completed_avoidances = 0
            self.no_path_detected = False

            self.goal_reached = False
            self.aeb_active = False
            self.aeb_interrupts = 0

            self.minimum_front_lidar_distance = None
            self.minimum_body_clearance = None
            self.closest_obstacle_name = None
            self.safety_margin_violation = False
            self.estimated_collision = False

    # --------------------------------------------------------
    # Sensor callbacks
    # --------------------------------------------------------

    def odometry_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.yaw = quaternion_to_yaw(msg.pose.orientation)
            self.speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
            self.odom_received = True
            self.last_odom_wall_time = now

    def lidar_callback(self, msg):
        front_values = []
        left_values = []
        right_values = []

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < max(MIN_VALID_RANGE_M, msg.range_min):
                continue
            if distance > msg.range_max:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_step)

            if abs(angle_deg) <= FRONT_SECTOR_DEG:
                front_values.append(distance)
            if SIDE_SECTOR_MIN_DEG <= angle_deg <= SIDE_SECTOR_MAX_DEG:
                left_values.append(distance)
            if -SIDE_SECTOR_MAX_DEG <= angle_deg <= -SIDE_SECTOR_MIN_DEG:
                right_values.append(distance)

        front = min(front_values) if front_values else None
        left = min(left_values) if left_values else msg.range_max
        right = min(right_values) if right_values else msg.range_max
        sim_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        wall_time = time.monotonic()

        with self.lock:
            if front is not None:
                if (
                    self.previous_front_distance is not None
                    and self.previous_lidar_time is not None
                ):
                    dt = sim_time - self.previous_lidar_time
                    if 0.01 < dt < 1.0:
                        raw_closing_speed = (
                            self.previous_front_distance - front
                        ) / dt
                        raw_closing_speed = clamp(raw_closing_speed, -20.0, 20.0)

                        if self.closing_speed is None:
                            self.closing_speed = raw_closing_speed
                        else:
                            alpha = CLOSING_SPEED_FILTER_ALPHA
                            self.closing_speed = (
                                alpha * raw_closing_speed
                                + (1.0 - alpha) * self.closing_speed
                            )

                self.previous_front_distance = front
                self.previous_lidar_time = sim_time
            else:
                self.previous_front_distance = None
                self.previous_lidar_time = None
                self.closing_speed = None

            self.front_distance = front
            self.left_clearance = left
            self.right_clearance = right

            if front is not None and (
                self.minimum_front_lidar_distance is None
                or front < self.minimum_front_lidar_distance
            ):
                self.minimum_front_lidar_distance = front

            self.lidar_received = True
            self.last_lidar_wall_time = wall_time

    # --------------------------------------------------------
    # Gazebo commands / services
    # --------------------------------------------------------

    def publish_command(self, speed, steering):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steering
        self.publisher.publish(msg)

    def stop_vehicle(self):
        self.publish_command(0.0, 0.0)

    def stop_repeatedly(self):
        for _ in range(10):
            self.stop_vehicle()
            time.sleep(0.02)

    def run_service(self, service, reqtype, request):
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
            request,
        ]

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=8.0,
        )
        output = result.stdout + result.stderr

        if result.returncode != 0 or "data: true" not in output:
            print("Gazebo service failed:")
            print(output)
            return False

        return True

    def set_model_pose(self, model_name, position):
        x, y, z = position
        request = (
            f'name: "{model_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{w: 1.0}}'
        )
        return self.run_service(
            f"/world/{WORLD_NAME}/set_pose",
            "gz.msgs.Pose",
            request,
        )

    def reset_world(self):
        print()
        print("Resetting advanced LiDAR world...")
        self.stop_repeatedly()
        self.reset_internal_state()

        if not self.run_service(
            f"/world/{WORLD_NAME}/control",
            "gz.msgs.WorldControl",
            "reset: {all: true}",
        ):
            return False

        if not self.run_service(
            f"/world/{WORLD_NAME}/control",
            "gz.msgs.WorldControl",
            "pause: false",
        ):
            return False

        start = time.monotonic()
        while time.monotonic() - start <= RESET_TIMEOUT_SECONDS:
            with self.lock:
                ready = self.odom_received and self.lidar_received
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
                print(f"Reset OK | x={x:.2f} | y={y:.2f}")
                return True

            time.sleep(0.05)

        print("ERROR: reset timeout")
        return False

    def configure_scenario(self, scenario):
        positions = dict(PARKING)
        positions.update(scenario["positions"])

        for model_name, position in positions.items():
            if not self.set_model_pose(model_name, position):
                return False

        time.sleep(0.75)

        with self.lock:
            self.previous_front_distance = None
            self.previous_lidar_time = None
            self.closing_speed = None

        return True

    # --------------------------------------------------------
    # Controller
    # --------------------------------------------------------

    def update_aeb(self):
        with self.lock:
            distance = self.front_distance
            closing_speed = self.closing_speed

        if (
            distance is not None
            and closing_speed is not None
            and closing_speed > MIN_CLOSING_SPEED_MPS
        ):
            ttc = distance / closing_speed
        else:
            ttc = math.inf

        trigger = (
            distance is not None
            and distance <= EMERGENCY_DISTANCE_M
        ) or (
            math.isfinite(ttc)
            and ttc <= TTC_THRESHOLD_SECONDS
        )

        if trigger and not self.aeb_active:
            self.aeb_active = True
            self.aeb_interrupts += 1
            print()
            print("*** AEB SAFETY OVERRIDE ***")
            if distance is not None:
                print(f"Distance: {distance:.2f} m")
            if math.isfinite(ttc):
                print(f"TTC: {ttc:.2f} s")
            print()

        return self.aeb_active

    def steering_to_lane(self, target_y):
        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw

        desired_heading = math.atan2(target_y - y, LOOKAHEAD_X_M)
        heading_error = wrap_angle(desired_heading - yaw)
        return clamp(
            HEADING_GAIN * heading_error,
            -MAX_STEERING_COMMAND,
            MAX_STEERING_COMMAND,
        )

    def update_state_machine(self):
        with self.lock:
            x = self.x
            y = self.y
            front = self.front_distance
            left = self.left_clearance
            right = self.right_clearance
            state = self.state

        if state == "CRUISE":
            if front is None or front > OBSTACLE_DETECTION_DISTANCE_M:
                return

            left_open = left >= MIN_SIDE_CLEARANCE_FOR_AVOIDANCE_M
            right_open = right >= MIN_SIDE_CLEARANCE_FOR_AVOIDANCE_M

            print()
            print("*** OBSTACLE DETECTED ***")
            print(f"Front distance : {front:.2f} m")
            print(f"Left clearance : {left:.2f} m")
            print(f"Right clearance: {right:.2f} m")

            if not left_open and not right_open:
                self.no_path_detected = True
                self.state = "NO_PATH"
                print("Safe avoidance side: NONE")
                print("AEB fallback armed.")
                print()
                return

            if left_open and not right_open:
                direction = "LEFT"
            elif right_open and not left_open:
                direction = "RIGHT"
            elif left >= right:
                direction = "LEFT"
            else:
                direction = "RIGHT"

            self.avoidance_start_x = x
            self.avoidance_target_y = (
                AVOIDANCE_OFFSET_M if direction == "LEFT" else -AVOIDANCE_OFFSET_M
            )
            self.avoidance_directions.append(direction)
            self.avoidance_count += 1
            self.state = "MOVE_OUT"

            print(f"Safe avoidance side: {direction}")
            print(f"Avoidance number: {self.avoidance_count}")
            print()

        elif state == "MOVE_OUT":
            if abs(y - self.avoidance_target_y) <= LANE_TOLERANCE_M:
                self.state = "PASS_OBSTACLE"
                print()
                print("*** AVOIDANCE LANE REACHED ***")
                print(f"y={y:.2f} m")
                print()

        elif state == "PASS_OBSTACLE":
            if (
                self.avoidance_start_x is not None
                and x >= self.avoidance_start_x + PASS_DISTANCE_AFTER_DETECTION_M
            ):
                self.state = "RETURN"
                print()
                print("*** OBSTACLE PASSED ***")
                print(f"x={x:.2f} m")
                print()

        elif state == "RETURN":
            if abs(y) <= LANE_TOLERANCE_M:
                self.completed_avoidances += 1
                self.state = "CRUISE"
                print()
                print("*** ORIGINAL LANE REACHED ***")
                print(f"Completed avoidances: {self.completed_avoidances}")
                print()

    # --------------------------------------------------------
    # Body-to-obstacle safety clearance
    # --------------------------------------------------------

    def update_body_clearance(self, scenario, x, y, yaw):
        current_minimum = math.inf
        current_name = None

        for obstacle in scenario["active_obstacles"]:
            clearance = body_clearance(x, y, yaw, obstacle)
            if clearance < current_minimum:
                current_minimum = clearance
                current_name = obstacle["name"]

        if (
            self.minimum_body_clearance is None
            or current_minimum < self.minimum_body_clearance
        ):
            self.minimum_body_clearance = current_minimum
            self.closest_obstacle_name = current_name

        if current_minimum <= COLLISION_EPSILON_M:
            self.estimated_collision = True

        if current_minimum < MIN_REQUIRED_BODY_CLEARANCE_M:
            self.safety_margin_violation = True

        return current_minimum, current_name

    # --------------------------------------------------------
    # One scenario
    # --------------------------------------------------------

    def run_scenario(self, scenario):
        print()
        print("============================================================")
        print(f" SCENARIO: {scenario['name']}")
        print("============================================================")
        print(f"Expected outcome: {scenario['expected_outcome']}")
        print(f"Expected avoidances: {scenario['expected_avoidances']}")
        print(f"Required body clearance: {MIN_REQUIRED_BODY_CLEARANCE_M:.2f} m")

        if not self.reset_world():
            return self.failed_result(scenario, "reset_failed")

        if not self.configure_scenario(scenario):
            return self.failed_result(scenario, "configuration_failed")

        start_time = time.monotonic()
        last_print = 0.0
        stopped_since = None
        end_reason = None

        while True:
            now = time.monotonic()

            if now - start_time > SINGLE_TEST_TIMEOUT_SECONDS:
                end_reason = "timeout"
                break

            with self.lock:
                x = self.x
                y = self.y
                yaw = self.yaw
                speed = self.speed
                front = self.front_distance
                left = self.left_clearance
                right = self.right_clearance

                sensors_fresh = (
                    self.odom_received
                    and self.lidar_received
                    and now - self.last_odom_wall_time <= SENSOR_TIMEOUT_SECONDS
                    and now - self.last_lidar_wall_time <= SENSOR_TIMEOUT_SECONDS
                )

            if not sensors_fresh:
                self.stop_vehicle()
                time.sleep(CONTROL_PERIOD_SECONDS)
                continue

            current_clearance, nearest_name = self.update_body_clearance(
                scenario,
                x,
                y,
                yaw,
            )

            if self.estimated_collision:
                print()
                print("*** COLLISION ENVELOPE REACHED ***")
                print(f"Obstacle: {nearest_name}")
                print(f"Body clearance: {current_clearance:.3f} m")
                print()
                end_reason = "estimated_collision"
                break

            if self.safety_margin_violation:
                print()
                print("*** BODY CLEARANCE SAFETY MARGIN VIOLATION ***")
                print(f"Obstacle: {nearest_name}")
                print(f"Body clearance: {current_clearance:.3f} m")
                print(f"Required: {MIN_REQUIRED_BODY_CLEARANCE_M:.3f} m")
                print()
                end_reason = "body_clearance_violation"
                break

            if x > MAX_ALLOWED_X_M or abs(y) > MAX_ALLOWED_ABS_Y_M:
                end_reason = "route_deviation"
                break

            self.update_state_machine()
            state = self.state
            aeb_active = self.update_aeb()

            if aeb_active:
                self.stop_vehicle()
                drive_state = "AEB"

                if speed <= STOP_SPEED_THRESHOLD_MPS:
                    if stopped_since is None:
                        stopped_since = now
                    elif now - stopped_since >= STOP_STABLE_TIME_SECONDS:
                        end_reason = (
                            "expected_aeb_stop"
                            if scenario["expected_outcome"] == "AEB_STOP"
                            else "unexpected_aeb_stop"
                        )
                        break
                else:
                    stopped_since = None

            else:
                stopped_since = None

                if state == "CRUISE":
                    target_y = 0.0
                    command_speed = CRUISE_SPEED_MPS
                elif state in ("MOVE_OUT", "PASS_OBSTACLE"):
                    target_y = self.avoidance_target_y
                    command_speed = AVOIDANCE_SPEED_MPS
                elif state == "RETURN":
                    target_y = 0.0
                    command_speed = RETURN_SPEED_MPS
                elif state == "NO_PATH":
                    target_y = 0.0
                    command_speed = CRUISE_SPEED_MPS
                else:
                    target_y = 0.0
                    command_speed = CRUISE_SPEED_MPS

                steering = self.steering_to_lane(target_y)
                self.publish_command(command_speed, steering)
                drive_state = state

            if (
                scenario["expected_outcome"] == "GOAL"
                and scenario["goal_x"] is not None
                and x >= scenario["goal_x"]
                and abs(y) <= 0.75
                and self.state == "CRUISE"
                and self.completed_avoidances >= scenario["expected_avoidances"]
            ):
                self.goal_reached = True
                end_reason = "goal_reached"
                break

            if now - last_print >= PRINT_INTERVAL_SECONDS:
                front_text = "---" if front is None else f"{front:.2f}"
                left_text = "---" if left is None else f"{left:.2f}"
                right_text = "---" if right is None else f"{right:.2f}"

                print(
                    f"x={x:5.1f} "
                    f"y={y:5.2f} | "
                    f"{drive_state:13s} | "
                    f"front={front_text:>5s} | "
                    f"left={left_text:>5s} | "
                    f"right={right_text:>5s} | "
                    f"bodyClr={current_clearance:4.2f} | "
                    f"avoid={self.completed_avoidances}"
                )
                last_print = now

            time.sleep(CONTROL_PERIOD_SECONDS)

        self.stop_repeatedly()
        duration = time.monotonic() - start_time

        clearance_ok = (
            self.minimum_body_clearance is not None
            and self.minimum_body_clearance >= MIN_REQUIRED_BODY_CLEARANCE_M
            and not self.safety_margin_violation
        )

        if scenario["expected_outcome"] == "GOAL":
            passed = (
                self.goal_reached
                and self.completed_avoidances == scenario["expected_avoidances"]
                and self.aeb_interrupts == 0
                and clearance_ok
                and not self.estimated_collision
                and end_reason == "goal_reached"
            )
        else:
            passed = (
                self.no_path_detected
                and self.avoidance_count == 0
                and self.aeb_interrupts >= 1
                and clearance_ok
                and not self.estimated_collision
                and end_reason == "expected_aeb_stop"
            )

        result = "PASS" if passed else "FAIL"

        print()
        print(f"Scenario result        : {result}")
        print(f"Avoidance directions   : {self.avoidance_directions}")
        print(f"Avoidances started     : {self.avoidance_count}")
        print(f"Avoidances completed   : {self.completed_avoidances}")
        print(f"No-path detected       : {self.no_path_detected}")
        print(f"AEB interrupts         : {self.aeb_interrupts}")
        print(f"Goal reached           : {self.goal_reached}")

        if self.minimum_body_clearance is not None:
            print(f"Minimum body clearance : {self.minimum_body_clearance:.3f} m")
            print(f"Required body clearance: {MIN_REQUIRED_BODY_CLEARANCE_M:.3f} m")
            print(f"Safety margin OK       : {'YES' if clearance_ok else 'NO'}")

        print(f"Collision estimate     : {self.estimated_collision}")
        print(f"End reason             : {end_reason}")
        print(f"Duration               : {duration:.2f} s")

        return {
            "scenario": scenario["name"],
            "expected_outcome": scenario["expected_outcome"],
            "expected_avoidances": scenario["expected_avoidances"],
            "avoidance_count": self.avoidance_count,
            "completed_avoidances": self.completed_avoidances,
            "directions": ",".join(self.avoidance_directions),
            "no_path_detected": self.no_path_detected,
            "aeb_interrupts": self.aeb_interrupts,
            "goal_reached": self.goal_reached,
            "minimum_front_lidar_distance_m": self.minimum_front_lidar_distance,
            "minimum_body_clearance_m": self.minimum_body_clearance,
            "required_body_clearance_m": MIN_REQUIRED_BODY_CLEARANCE_M,
            "safety_margin_ok": clearance_ok,
            "closest_obstacle": self.closest_obstacle_name,
            "estimated_collision": self.estimated_collision,
            "end_reason": end_reason,
            "duration_s": duration,
            "result": result,
        }

    def failed_result(self, scenario, reason):
        return {
            "scenario": scenario["name"],
            "expected_outcome": scenario["expected_outcome"],
            "expected_avoidances": scenario["expected_avoidances"],
            "avoidance_count": 0,
            "completed_avoidances": 0,
            "directions": "",
            "no_path_detected": False,
            "aeb_interrupts": 0,
            "goal_reached": False,
            "minimum_front_lidar_distance_m": None,
            "minimum_body_clearance_m": None,
            "required_body_clearance_m": MIN_REQUIRED_BODY_CLEARANCE_M,
            "safety_margin_ok": False,
            "closest_obstacle": "",
            "estimated_collision": False,
            "end_reason": reason,
            "duration_s": 0.0,
            "result": "FAIL",
        }

    # --------------------------------------------------------
    # CSV + summary
    # --------------------------------------------------------

    def save_results(self, results):
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        file_exists = RESULTS_FILE.exists()

        fields = [
            "timestamp",
            "scenario",
            "expected_outcome",
            "expected_avoidances",
            "avoidance_count",
            "completed_avoidances",
            "directions",
            "no_path_detected",
            "aeb_interrupts",
            "goal_reached",
            "minimum_front_lidar_distance_m",
            "minimum_body_clearance_m",
            "required_body_clearance_m",
            "safety_margin_ok",
            "closest_obstacle",
            "estimated_collision",
            "end_reason",
            "duration_s",
            "result",
        ]

        with RESULTS_FILE.open("a", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            if not file_exists:
                writer.writeheader()

            for result in results:
                row = dict(result)
                row["timestamp"] = datetime.now().isoformat(timespec="seconds")
                row["minimum_front_lidar_distance_m"] = (
                    ""
                    if result["minimum_front_lidar_distance_m"] is None
                    else f"{result['minimum_front_lidar_distance_m']:.3f}"
                )
                row["minimum_body_clearance_m"] = (
                    ""
                    if result["minimum_body_clearance_m"] is None
                    else f"{result['minimum_body_clearance_m']:.3f}"
                )
                row["required_body_clearance_m"] = (
                    f"{result['required_body_clearance_m']:.3f}"
                )
                row["duration_s"] = f"{result['duration_s']:.3f}"
                writer.writerow(row)

    def print_summary(self, results):
        print()
        print("====================================================================================")
        print(" ADVANCED LIDAR OBSTACLE AVOIDANCE BATCH SUMMARY")
        print("====================================================================================")
        print("Scenario             Outcome    Avoid exp/got   AEB   Goal   Min clr   Margin   Result")
        print("------------------------------------------------------------------------------------")

        for result in results:
            avoid_text = (
                f"{result['expected_avoidances']}/{result['completed_avoidances']}"
            )
            clearance_text = (
                "---"
                if result["minimum_body_clearance_m"] is None
                else f"{result['minimum_body_clearance_m']:.2f}m"
            )

            print(
                f"{result['scenario']:<20s} "
                f"{result['expected_outcome']:<10s} "
                f"{avoid_text:<15s} "
                f"{result['aeb_interrupts']:<5d} "
                f"{'YES' if result['goal_reached'] else 'NO':<6s} "
                f"{clearance_text:<9s} "
                f"{'YES' if result['safety_margin_ok'] else 'NO':<8s} "
                f"{result['result']}"
            )

        print("====================================================================================")
        passed_count = sum(result["result"] == "PASS" for result in results)
        print(f"Passed: {passed_count}/{len(results)}")
        print(
            "OVERALL RESULT: PASS"
            if passed_count == len(results)
            else "OVERALL RESULT: FAIL"
        )
        print(f"Required body clearance: {MIN_REQUIRED_BODY_CLEARANCE_M:.2f} m")
        print(f"CSV: {RESULTS_FILE}")
        print("====================================================================================")

    def run(self):
        print()
        print("====================================================================================")
        print(" ADVANCED GAZEBO LIDAR OBSTACLE AVOIDANCE TEST")
        print("====================================================================================")
        print(f"Cruise speed    : {CRUISE_SPEED_MPS:.1f} m/s")
        print(f"Avoidance speed : {AVOIDANCE_SPEED_MPS:.1f} m/s")
        print(f"Return speed    : {RETURN_SPEED_MPS:.1f} m/s")
        print(f"Body clearance  : >= {MIN_REQUIRED_BODY_CLEARANCE_M:.1f} m")
        print()

        results = []

        try:
            for scenario in SCENARIOS:
                results.append(self.run_scenario(scenario))
        except KeyboardInterrupt:
            print()
            print("Batch interrupted.")
        finally:
            self.stop_repeatedly()

        if results:
            self.save_results(results)
            self.print_summary(results)

        print()
        print("Returning world to initial state...")
        self.reset_world()
        self.stop_repeatedly()

        return (
            len(results) == len(SCENARIOS)
            and all(result["result"] == "PASS" for result in results)
        )


def main():
    test = AdvancedLidarAvoidanceBatchTest()
    success = test.run()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
