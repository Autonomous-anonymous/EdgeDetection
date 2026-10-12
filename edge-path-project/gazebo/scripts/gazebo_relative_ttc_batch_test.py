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

WORLD_NAME = "moving_obstacle_world"

LIDAR_TOPIC = "/aeb/lidar"
EGO_ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
EGO_CMD_TOPIC = "/model/aeb_vehicle/cmd_vel"
OBSTACLE_CMD_TOPIC = "/model/moving_obstacle/cmd_vel"


# Testit:
#
# expect_aeb = True:
# ego lähestyy hitaampaa estettä -> AEB:n pitää laueta.
#
# expect_aeb = False:
# sulkeutumisnopeutta ei käytännössä ole -> AEB EI saa laueta.
#
SCENARIOS = [
    {
        "name": "ego3_obstacle1",
        "ego_speed": 3.0,
        "obstacle_speed": 1.0,
        "expect_aeb": True,
    },
    {
        "name": "ego4_obstacle1",
        "ego_speed": 4.0,
        "obstacle_speed": 1.0,
        "expect_aeb": True,
    },
    {
        "name": "ego5_obstacle2",
        "ego_speed": 5.0,
        "obstacle_speed": 2.0,
        "expect_aeb": True,
    },
    {
        "name": "ego5_obstacle3",
        "ego_speed": 5.0,
        "obstacle_speed": 3.0,
        "expect_aeb": True,
    },
    {
        "name": "ego4_obstacle4",
        "ego_speed": 4.0,
        "obstacle_speed": 4.0,
        "expect_aeb": False,
    },
]


TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 10.0

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.30

SENSOR_TIMEOUT_SECONDS = 0.50

SINGLE_TEST_TIMEOUT_SECONDS = 30.0

# Ei-AEB-tapauksessa seurataan tätä aikaa.
NO_AEB_OBSERVATION_SECONDS = 10.0

STOP_SPEED_THRESHOLD_MPS = 0.05

STOP_STABLE_TIME_SECONDS = 0.75

RESET_TIMEOUT_SECONDS = 10.0

RESET_POSITION_TOLERANCE_M = 0.75

LIDAR_TO_FRONT_BUMPER_M = 0.30

MIN_SAFE_BUMPER_GAP_M = 0.20

# LiDAR-etäisyysderivaatan alipäästösuodatus.
CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "gazebo_relative_ttc_batch_results.csv"
)


# ============================================================
# BATCH TEST
# ============================================================

class RelativeTTCBatchTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()

        # ----------------------------------------------------
        # Sensor state
        # ----------------------------------------------------

        self.front_distance_m = None

        self.ego_speed_mps = 0.0

        self.ego_position_x_m = None

        self.lidar_received = False

        self.odometry_received = False

        self.last_lidar_wall_time = 0.0

        self.last_odometry_wall_time = 0.0


        # ----------------------------------------------------
        # Relative-speed estimation
        # ----------------------------------------------------

        self.previous_lidar_distance_m = None

        self.previous_lidar_sim_time_s = None

        self.raw_closing_speed_mps = None

        self.filtered_closing_speed_mps = None


        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.ego_publisher = self.node.advertise(
            EGO_CMD_TOPIC,
            Twist
        )

        self.obstacle_publisher = self.node.advertise(
            OBSTACLE_CMD_TOPIC,
            Twist
        )


        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        lidar_ok = self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback
        )

        odom_ok = self.node.subscribe(
            Odometry,
            EGO_ODOMETRY_TOPIC,
            self.odometry_callback
        )


        if not lidar_ok:

            raise RuntimeError(
                f"Failed to subscribe to {LIDAR_TOPIC}"
            )


        if not odom_ok:

            raise RuntimeError(
                f"Failed to subscribe to {EGO_ODOMETRY_TOPIC}"
            )


    # ========================================================
    # LIDAR
    # ========================================================

    def lidar_callback(
        self,
        msg: LaserScan
    ):

        half_angle_rad = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        valid_ranges = []


        for index, distance in enumerate(
            msg.ranges
        ):

            angle = (
                msg.angle_min
                + index * msg.angle_step
            )


            if abs(angle) > half_angle_rad:
                continue


            if not math.isfinite(distance):
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


        wall_time = time.monotonic()


        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec * 1e-9
        )


        with self.lock:

            if distance is not None:

                previous_distance = (
                    self.previous_lidar_distance_m
                )

                previous_time = (
                    self.previous_lidar_sim_time_s
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

                        # Positiivinen = etäisyys pienenee.
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


                        self.raw_closing_speed_mps = (
                            raw_closing_speed
                        )


                        if (
                            self.filtered_closing_speed_mps
                            is None
                        ):

                            self.filtered_closing_speed_mps = (
                                raw_closing_speed
                            )

                        else:

                            alpha = (
                                CLOSING_SPEED_FILTER_ALPHA
                            )

                            self.filtered_closing_speed_mps = (
                                alpha
                                * raw_closing_speed
                                + (
                                    1.0
                                    - alpha
                                )
                                * self.filtered_closing_speed_mps
                            )


                self.previous_lidar_distance_m = (
                    distance
                )

                self.previous_lidar_sim_time_s = (
                    sim_time
                )


            self.front_distance_m = (
                distance
            )

            self.lidar_received = True

            self.last_lidar_wall_time = (
                wall_time
            )


    # ========================================================
    # ODOMETRY
    # ========================================================

    def odometry_callback(
        self,
        msg: Odometry
    ):

        wall_time = time.monotonic()


        with self.lock:

            self.ego_speed_mps = abs(
                msg.twist.linear.x
            )

            self.ego_position_x_m = (
                msg.pose.position.x
            )

            self.odometry_received = True

            self.last_odometry_wall_time = (
                wall_time
            )


    # ========================================================
    # COMMANDS
    # ========================================================

    def publish_twist(
        self,
        publisher,
        linear_speed
    ):

        msg = Twist()

        msg.linear.x = linear_speed

        msg.angular.z = 0.0

        publisher.publish(
            msg
        )


    def command_ego(
        self,
        speed
    ):

        self.publish_twist(
            self.ego_publisher,
            speed
        )


    def command_obstacle(
        self,
        speed
    ):

        self.publish_twist(
            self.obstacle_publisher,
            speed
        )


    def stop_both(self):

        for _ in range(10):

            self.command_ego(
                0.0
            )

            self.command_obstacle(
                0.0
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
                "World-control command failed:"
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


    def reset_sensor_state(self):

        with self.lock:

            self.front_distance_m = None

            self.ego_speed_mps = 0.0

            self.ego_position_x_m = None

            self.lidar_received = False

            self.odometry_received = False

            self.last_lidar_wall_time = 0.0

            self.last_odometry_wall_time = 0.0

            self.previous_lidar_distance_m = None

            self.previous_lidar_sim_time_s = None

            self.raw_closing_speed_mps = None

            self.filtered_closing_speed_mps = None


    def reset_world(self):

        print()
        print(
            "Resetting Gazebo world..."
        )


        self.stop_both()

        self.reset_sensor_state()


        if not self.call_world_control(
            "reset: {all: true}"
        ):

            return False


        if not self.call_world_control(
            "pause: false"
        ):

            return False


        wait_start = time.monotonic()


        while True:

            now = time.monotonic()


            if (
                now - wait_start
                > RESET_TIMEOUT_SECONDS
            ):

                print(
                    "ERROR: world reset timeout"
                )

                return False


            with self.lock:

                lidar_received = (
                    self.lidar_received
                )

                odometry_received = (
                    self.odometry_received
                )

                x = (
                    self.ego_position_x_m
                )

                speed = (
                    self.ego_speed_mps
                )

                distance = (
                    self.front_distance_m
                )

                lidar_age = (
                    now
                    - self.last_lidar_wall_time
                )

                odom_age = (
                    now
                    - self.last_odometry_wall_time
                )


            sensors_ready = (
                lidar_received
                and odometry_received
                and lidar_age
                <= SENSOR_TIMEOUT_SECONDS
                and odom_age
                <= SENSOR_TIMEOUT_SECONDS
            )


            position_ready = (
                x is not None
                and abs(x)
                <= RESET_POSITION_TOLERANCE_M
            )


            stopped = (
                speed
                <= STOP_SPEED_THRESHOLD_MPS
            )


            if (
                sensors_ready
                and position_ready
                and stopped
                and distance is not None
            ):

                print(
                    f"Reset OK | "
                    f"x={x:.3f} m | "
                    f"speed={speed:.3f} m/s | "
                    f"distance={distance:.3f} m"
                )

                time.sleep(
                    0.50
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # SINGLE SCENARIO
    # ========================================================

    def run_scenario(
        self,
        scenario
    ):

        name = scenario["name"]

        ego_target = scenario["ego_speed"]

        obstacle_target = scenario["obstacle_speed"]

        expect_aeb = scenario["expect_aeb"]

        expected_closing_speed = (
            ego_target
            - obstacle_target
        )


        print()
        print(
            "============================================================"
        )
        print(
            f" SCENARIO: {name}"
        )
        print(
            "============================================================"
        )

        print(
            f"Ego target speed      : "
            f"{ego_target:.2f} m/s"
        )

        print(
            f"Obstacle target speed : "
            f"{obstacle_target:.2f} m/s"
        )

        print(
            f"Expected closing speed: "
            f"{expected_closing_speed:.2f} m/s"
        )

        print(
            f"Expected AEB          : "
            f"{'YES' if expect_aeb else 'NO'}"
        )


        if not self.reset_world():

            return {
                "scenario":
                    name,

                "ego_target_speed_mps":
                    ego_target,

                "obstacle_target_speed_mps":
                    obstacle_target,

                "expected_closing_speed_mps":
                    expected_closing_speed,

                "expected_aeb":
                    expect_aeb,

                "aeb_triggered":
                    False,

                "trigger_distance_m":
                    None,

                "trigger_ego_speed_mps":
                    None,

                "trigger_closing_speed_mps":
                    None,

                "trigger_ttc_s":
                    None,

                "minimum_lidar_distance_m":
                    None,

                "minimum_bumper_gap_m":
                    None,

                "estimated_collision":
                    False,

                "end_reason":
                    "reset_failed",

                "result":
                    "FAIL"
            }


        test_start = time.monotonic()

        observation_start = None

        stable_stop_start = None

        last_print_time = 0.0

        aeb_triggered = False

        trigger_distance = None

        trigger_ego_speed = None

        trigger_closing_speed = None

        trigger_ttc = None

        minimum_distance = None

        estimated_collision = False

        end_reason = None


        while True:

            now = time.monotonic()


            if (
                now - test_start
                > SINGLE_TEST_TIMEOUT_SECONDS
            ):

                end_reason = (
                    "timeout"
                )

                break


            with self.lock:

                distance = (
                    self.front_distance_m
                )

                ego_speed = (
                    self.ego_speed_mps
                )

                closing_speed = (
                    self.filtered_closing_speed_mps
                )

                lidar_age = (
                    now
                    - self.last_lidar_wall_time
                )

                odom_age = (
                    now
                    - self.last_odometry_wall_time
                )

                sensors_ready = (
                    self.lidar_received
                    and self.odometry_received
                    and lidar_age
                    <= SENSOR_TIMEOUT_SECONDS
                    and odom_age
                    <= SENSOR_TIMEOUT_SECONDS
                )


            if not sensors_ready:

                self.command_ego(
                    0.0
                )

                self.command_obstacle(
                    obstacle_target
                )

                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )

                continue


            # ------------------------------------------------
            # Minimum distance
            # ------------------------------------------------

            if distance is not None:

                if (
                    minimum_distance is None
                    or distance
                    < minimum_distance
                ):

                    minimum_distance = (
                        distance
                    )


            # ------------------------------------------------
            # Bumper gap / collision proxy
            # ------------------------------------------------

            if distance is None:

                bumper_gap = None

            else:

                bumper_gap = (
                    distance
                    - LIDAR_TO_FRONT_BUMPER_M
                )


            if (
                bumper_gap is not None
                and bumper_gap <= 0.0
            ):

                estimated_collision = True

                end_reason = (
                    "estimated_collision"
                )

                break


            # ------------------------------------------------
            # Relative TTC
            # ------------------------------------------------

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


            # ------------------------------------------------
            # Trigger
            # ------------------------------------------------

            if (
                not aeb_triggered
                and math.isfinite(ttc)
                and ttc
                <= TTC_THRESHOLD_SECONDS
            ):

                aeb_triggered = True

                trigger_distance = distance

                trigger_ego_speed = (
                    ego_speed
                )

                trigger_closing_speed = (
                    closing_speed
                )

                trigger_ttc = ttc


                print()
                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )
                print(
                    " RELATIVE TTC AEB TRIGGERED"
                )
                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )

                print(
                    f"Distance      : "
                    f"{trigger_distance:.3f} m"
                )

                print(
                    f"Ego speed     : "
                    f"{trigger_ego_speed:.3f} m/s"
                )

                print(
                    f"Closing speed : "
                    f"{trigger_closing_speed:.3f} m/s"
                )

                print(
                    f"TTC           : "
                    f"{trigger_ttc:.3f} s"
                )

                print()


            # ------------------------------------------------
            # Commands
            # ------------------------------------------------

            self.command_obstacle(
                obstacle_target
            )


            if aeb_triggered:

                self.command_ego(
                    0.0
                )

                state = "AEB BRAKE"

            else:

                self.command_ego(
                    ego_target
                )

                state = "FOLLOW"


            # ------------------------------------------------
            # Expected-AEB case
            # ------------------------------------------------

            if expect_aeb:

                if aeb_triggered:

                    if (
                        ego_speed
                        <= STOP_SPEED_THRESHOLD_MPS
                    ):

                        if stable_stop_start is None:

                            stable_stop_start = now

                        elif (
                            now
                            - stable_stop_start
                            >= STOP_STABLE_TIME_SECONDS
                        ):

                            end_reason = (
                                "ego_stopped"
                            )

                            break

                    else:

                        stable_stop_start = None


            # ------------------------------------------------
            # Expected-NO-AEB case
            # ------------------------------------------------

            else:

                # Aloitetaan 10 s tarkkailu vasta kun
                # ego-auto on lähellä tavoitenopeutta.
                if (
                    ego_speed
                    >= ego_target * 0.90
                ):

                    if observation_start is None:

                        observation_start = (
                            now
                        )


                if (
                    observation_start is not None
                    and now
                    - observation_start
                    >= NO_AEB_OBSERVATION_SECONDS
                ):

                    end_reason = (
                        "no_aeb_observation_complete"
                    )

                    break


            # ------------------------------------------------
            # Status
            # ------------------------------------------------

            if (
                now - last_print_time
                >= PRINT_INTERVAL_SECONDS
            ):

                if distance is None:

                    distance_text = "none"

                else:

                    distance_text = (
                        f"{distance:.2f} m"
                    )


                if closing_speed is None:

                    closing_text = "none"

                else:

                    closing_text = (
                        f"{closing_speed:.2f}"
                    )


                if math.isfinite(ttc):

                    ttc_text = (
                        f"{ttc:.2f} s"
                    )

                else:

                    ttc_text = "inf"


                print(
                    f"{state:10s} | "
                    f"ego={ego_speed:5.2f} m/s | "
                    f"distance={distance_text:>8s} | "
                    f"closing={closing_text:>5s} m/s | "
                    f"TTC={ttc_text:>7s}"
                )


                last_print_time = now


            time.sleep(
                CONTROL_PERIOD_SECONDS
            )


        # ====================================================
        # STOP
        # ====================================================

        self.stop_both()

        time.sleep(
            0.30
        )


        if minimum_distance is None:

            minimum_bumper_gap = None

        else:

            minimum_bumper_gap = (
                minimum_distance
                - LIDAR_TO_FRONT_BUMPER_M
            )


        safe_gap = (
            minimum_bumper_gap is not None
            and minimum_bumper_gap
            > MIN_SAFE_BUMPER_GAP_M
        )


        # ====================================================
        # PASS / FAIL
        # ====================================================

        if expect_aeb:

            passed = (
                aeb_triggered
                and trigger_ttc is not None
                and trigger_ttc
                <= TTC_THRESHOLD_SECONDS
                and safe_gap
                and not estimated_collision
                and end_reason
                == "ego_stopped"
            )

        else:

            passed = (
                not aeb_triggered
                and safe_gap
                and not estimated_collision
                and end_reason
                == "no_aeb_observation_complete"
            )


        result = (
            "PASS"
            if passed
            else "FAIL"
        )


        print()
        print(
            f"Result for {name}:"
        )

        print(
            f"  Expected AEB           : "
            f"{'YES' if expect_aeb else 'NO'}"
        )

        print(
            f"  AEB triggered          : "
            f"{'YES' if aeb_triggered else 'NO'}"
        )


        if trigger_ttc is not None:

            print(
                f"  Trigger TTC            : "
                f"{trigger_ttc:.3f} s"
            )


        if trigger_closing_speed is not None:

            print(
                f"  Trigger closing speed  : "
                f"{trigger_closing_speed:.3f} m/s"
            )


        if minimum_bumper_gap is not None:

            print(
                f"  Minimum bumper gap     : "
                f"{minimum_bumper_gap:.3f} m"
            )


        print(
            f"  Estimated collision    : "
            f"{'YES' if estimated_collision else 'NO'}"
        )

        print(
            f"  End reason             : "
            f"{end_reason}"
        )

        print(
            f"  RESULT                 : "
            f"{result}"
        )


        return {
            "scenario":
                name,

            "ego_target_speed_mps":
                ego_target,

            "obstacle_target_speed_mps":
                obstacle_target,

            "expected_closing_speed_mps":
                expected_closing_speed,

            "expected_aeb":
                expect_aeb,

            "aeb_triggered":
                aeb_triggered,

            "trigger_distance_m":
                trigger_distance,

            "trigger_ego_speed_mps":
                trigger_ego_speed,

            "trigger_closing_speed_mps":
                trigger_closing_speed,

            "trigger_ttc_s":
                trigger_ttc,

            "minimum_lidar_distance_m":
                minimum_distance,

            "minimum_bumper_gap_m":
                minimum_bumper_gap,

            "estimated_collision":
                estimated_collision,

            "end_reason":
                end_reason,

            "result":
                result
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


        batch_id = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )


        file_exists = (
            RESULTS_FILE.exists()
        )


        fields = [
            "batch_id",
            "timestamp",
            "scenario",
            "ego_target_speed_mps",
            "obstacle_target_speed_mps",
            "expected_closing_speed_mps",
            "expected_aeb",
            "aeb_triggered",
            "trigger_distance_m",
            "trigger_ego_speed_mps",
            "trigger_closing_speed_mps",
            "trigger_ttc_s",
            "minimum_lidar_distance_m",
            "minimum_bumper_gap_m",
            "estimated_collision",
            "end_reason",
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
                        "batch_id":
                            batch_id,

                        "timestamp":
                            datetime.now().isoformat(
                                timespec="seconds"
                            ),

                        "scenario":
                            result["scenario"],

                        "ego_target_speed_mps":
                            f"{result['ego_target_speed_mps']:.3f}",

                        "obstacle_target_speed_mps":
                            f"{result['obstacle_target_speed_mps']:.3f}",

                        "expected_closing_speed_mps":
                            f"{result['expected_closing_speed_mps']:.3f}",

                        "expected_aeb":
                            result["expected_aeb"],

                        "aeb_triggered":
                            result["aeb_triggered"],

                        "trigger_distance_m":
                            (
                                ""
                                if result["trigger_distance_m"]
                                is None
                                else
                                f"{result['trigger_distance_m']:.3f}"
                            ),

                        "trigger_ego_speed_mps":
                            (
                                ""
                                if result["trigger_ego_speed_mps"]
                                is None
                                else
                                f"{result['trigger_ego_speed_mps']:.3f}"
                            ),

                        "trigger_closing_speed_mps":
                            (
                                ""
                                if result["trigger_closing_speed_mps"]
                                is None
                                else
                                f"{result['trigger_closing_speed_mps']:.3f}"
                            ),

                        "trigger_ttc_s":
                            (
                                ""
                                if result["trigger_ttc_s"]
                                is None
                                else
                                f"{result['trigger_ttc_s']:.3f}"
                            ),

                        "minimum_lidar_distance_m":
                            (
                                ""
                                if result["minimum_lidar_distance_m"]
                                is None
                                else
                                f"{result['minimum_lidar_distance_m']:.3f}"
                            ),

                        "minimum_bumper_gap_m":
                            (
                                ""
                                if result["minimum_bumper_gap_m"]
                                is None
                                else
                                f"{result['minimum_bumper_gap_m']:.3f}"
                            ),

                        "estimated_collision":
                            result["estimated_collision"],

                        "end_reason":
                            result["end_reason"],

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
            "=========================================================================="
        )
        print(
            " GAZEBO RELATIVE TTC BATCH TEST SUMMARY"
        )
        print(
            "=========================================================================="
        )

        print(
            "Ego   Lead   Exp.close   Meas.close   AEB exp/got   Min gap   Result"
        )

        print(
            "--------------------------------------------------------------------------"
        )


        for result in results:

            ego = (
                result["ego_target_speed_mps"]
            )

            lead = (
                result["obstacle_target_speed_mps"]
            )

            expected_closing = (
                result["expected_closing_speed_mps"]
            )

            measured_closing = (
                result["trigger_closing_speed_mps"]
            )


            if measured_closing is None:

                measured_text = "---"

            else:

                measured_text = (
                    f"{measured_closing:.2f}"
                )


            expected_aeb_text = (
                "YES"
                if result["expected_aeb"]
                else "NO"
            )

            actual_aeb_text = (
                "YES"
                if result["aeb_triggered"]
                else "NO"
            )


            gap = (
                result["minimum_bumper_gap_m"]
            )


            if gap is None:

                gap_text = "---"

            else:

                gap_text = (
                    f"{gap:.2f}m"
                )


            print(
                f"{ego:3.1f}   "
                f"{lead:4.1f}      "
                f"{expected_closing:5.2f}        "
                f"{measured_text:>5s}       "
                f"{expected_aeb_text:>3s}/{actual_aeb_text:<3s}      "
                f"{gap_text:>6s}    "
                f"{result['result']}"
            )


        print(
            "=========================================================================="
        )


        passed_count = sum(
            1
            for result in results
            if result["result"] == "PASS"
        )

        total_count = len(
            results
        )


        print(
            f"Passed: {passed_count}/{total_count}"
        )


        if passed_count == total_count:

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
            "=========================================================================="
        )


    # ========================================================
    # RUN BATCH
    # ========================================================

    def run(self):

        print()
        print(
            "=========================================================================="
        )
        print(
            " AUTOMATIC GAZEBO MOVING-OBSTACLE RELATIVE TTC BATCH TEST"
        )
        print(
            "=========================================================================="
        )

        print(
            f"TTC threshold: "
            f"{TTC_THRESHOLD_SECONDS:.2f} s"
        )

        print()


        results = []


        try:

            for scenario in SCENARIOS:

                result = self.run_scenario(
                    scenario
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

            self.stop_both()


        if results:

            self.save_results(
                results
            )

            self.print_summary(
                results
            )


        print()
        print(
            "Returning world to start position..."
        )

        self.reset_world()

        self.stop_both()


        all_passed = (
            len(results)
            == len(SCENARIOS)
            and all(
                result["result"] == "PASS"
                for result in results
            )
        )


        return all_passed


# ============================================================
# MAIN
# ============================================================

def main():

    test = RelativeTTCBatchTest()

    success = test.run()

    if success:

        raise SystemExit(0)

    raise SystemExit(1)


if __name__ == "__main__":

    main()
