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

EGO_ODOMETRY_TOPIC = (
    "/model/aeb_vehicle/odometry"
)

EGO_CMD_TOPIC = (
    "/model/aeb_vehicle/cmd_vel"
)

OBSTACLE_CMD_TOPIC = (
    "/model/moving_obstacle/cmd_vel"
)


# Ego ajaa 5 m/s.
EGO_TARGET_SPEED_MPS = 5.0

# Edellä oleva este liikkuu samaan suuntaan 2 m/s.
OBSTACLE_TARGET_SPEED_MPS = 2.0

# CARLA-vaiheessa hyväksi todettu TTC-raja.
TTC_THRESHOLD_SECONDS = 1.7

MIN_CLOSING_SPEED_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 10.0

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.25

SENSOR_TIMEOUT_SECONDS = 0.50

TEST_TIMEOUT_SECONDS = 30.0

STOP_SPEED_THRESHOLD_MPS = 0.05

STOP_STABLE_TIME_SECONDS = 0.75

RESET_TIMEOUT_SECONDS = 10.0

RESET_POSITION_TOLERANCE_M = 0.75

LIDAR_TO_FRONT_BUMPER_M = 0.30

MIN_SAFE_BUMPER_GAP_M = 0.20


# LiDARin etäisyysderivaatan suodatus.
#
# 1.0 = ei suodatusta
# pienempi = tasaisempi tulos.
CLOSING_SPEED_FILTER_ALPHA = 0.35


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = (
    SCRIPT_DIR.parent
    / "results"
)

RESULTS_FILE = (
    RESULTS_DIR
    / "gazebo_relative_ttc_results.csv"
)


# ============================================================
# TEST
# ============================================================

class RelativeTTCTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()


        # Sensor state
        self.front_distance_m = None

        self.ego_speed_mps = 0.0

        self.ego_position_x_m = None


        self.lidar_received = False

        self.odometry_received = False


        self.last_lidar_wall_time = 0.0

        self.last_odometry_wall_time = 0.0


        # LiDAR range-rate calculation
        self.previous_lidar_distance_m = None

        self.previous_lidar_sim_time_s = None

        self.raw_closing_speed_mps = None

        self.filtered_closing_speed_mps = None


        # Result state
        self.aeb_triggered = False

        self.trigger_distance_m = None

        self.trigger_ego_speed_mps = None

        self.trigger_closing_speed_mps = None

        self.trigger_ttc_s = None


        self.minimum_distance_m = None

        self.estimated_collision = False


        # Publishers
        self.ego_publisher = (
            self.node.advertise(
                EGO_CMD_TOPIC,
                Twist
            )
        )

        self.obstacle_publisher = (
            self.node.advertise(
                OBSTACLE_CMD_TOPIC,
                Twist
            )
        )


        # Subscribers
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
                "Failed to subscribe to "
                f"{LIDAR_TOPIC}"
            )


        if not odom_ok:

            raise RuntimeError(
                "Failed to subscribe to "
                f"{EGO_ODOMETRY_TOPIC}"
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


        if not valid_ranges:

            distance = None

        else:

            distance = min(
                valid_ranges
            )


        wall_time = time.monotonic()


        # Käytetään Gazebon simulaatioaikaa
        # range-rate laskennassa.
        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nsec
            * 1e-9
        )


        with self.lock:

            if distance is not None:

                if (
                    self.minimum_distance_m
                    is None
                    or distance
                    < self.minimum_distance_m
                ):

                    self.minimum_distance_m = (
                        distance
                    )


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

                        # Positiivinen arvo tarkoittaa,
                        # että este lähestyy ego-autoa.
                        raw_closing_speed = (
                            previous_distance
                            - distance
                        ) / dt


                        # Rajataan yksittäiset epärealistiset
                        # sensoripiikit.
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


    def reset_world(self):

        print(
            "Resetting world..."
        )


        self.stop_both()


        with self.lock:

            self.front_distance_m = None

            self.ego_speed_mps = 0.0

            self.ego_position_x_m = None

            self.lidar_received = False

            self.odometry_received = False


            self.previous_lidar_distance_m = (
                None
            )

            self.previous_lidar_sim_time_s = (
                None
            )

            self.raw_closing_speed_mps = (
                None
            )

            self.filtered_closing_speed_mps = (
                None
            )


            self.aeb_triggered = False

            self.trigger_distance_m = None

            self.trigger_ego_speed_mps = None

            self.trigger_closing_speed_mps = (
                None
            )

            self.trigger_ttc_s = None

            self.minimum_distance_m = None

            self.estimated_collision = False


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
                    "ERROR: reset timeout"
                )

                return False


            with self.lock:

                sensors_ready = (
                    self.lidar_received
                    and self.odometry_received
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


            position_ready = (
                x is not None
                and abs(x)
                <= RESET_POSITION_TOLERANCE_M
            )


            vehicle_stopped = (
                speed
                <= STOP_SPEED_THRESHOLD_MPS
            )


            if (
                sensors_ready
                and position_ready
                and vehicle_stopped
                and distance is not None
            ):

                print(
                    f"Reset OK | "
                    f"x={x:.3f} m | "
                    f"distance={distance:.3f} m"
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # SAVE RESULT
    # ========================================================

    def save_result(
        self,
        result,
        minimum_bumper_gap,
        final_speed,
        end_reason
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
            "ego_target_speed_mps",
            "obstacle_target_speed_mps",
            "expected_relative_speed_mps",
            "ttc_threshold_s",
            "aeb_triggered",
            "trigger_distance_m",
            "trigger_ego_speed_mps",
            "trigger_closing_speed_mps",
            "trigger_ttc_s",
            "minimum_lidar_distance_m",
            "minimum_bumper_gap_m",
            "final_ego_speed_mps",
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


            writer.writerow(
                {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),

                    "ego_target_speed_mps":
                        f"{EGO_TARGET_SPEED_MPS:.3f}",

                    "obstacle_target_speed_mps":
                        f"{OBSTACLE_TARGET_SPEED_MPS:.3f}",

                    "expected_relative_speed_mps":
                        (
                            f"{EGO_TARGET_SPEED_MPS - OBSTACLE_TARGET_SPEED_MPS:.3f}"
                        ),

                    "ttc_threshold_s":
                        f"{TTC_THRESHOLD_SECONDS:.3f}",

                    "aeb_triggered":
                        self.aeb_triggered,

                    "trigger_distance_m":
                        (
                            ""
                            if self.trigger_distance_m
                            is None
                            else
                            f"{self.trigger_distance_m:.3f}"
                        ),

                    "trigger_ego_speed_mps":
                        (
                            ""
                            if self.trigger_ego_speed_mps
                            is None
                            else
                            f"{self.trigger_ego_speed_mps:.3f}"
                        ),

                    "trigger_closing_speed_mps":
                        (
                            ""
                            if self.trigger_closing_speed_mps
                            is None
                            else
                            f"{self.trigger_closing_speed_mps:.3f}"
                        ),

                    "trigger_ttc_s":
                        (
                            ""
                            if self.trigger_ttc_s
                            is None
                            else
                            f"{self.trigger_ttc_s:.3f}"
                        ),

                    "minimum_lidar_distance_m":
                        (
                            ""
                            if self.minimum_distance_m
                            is None
                            else
                            f"{self.minimum_distance_m:.3f}"
                        ),

                    "minimum_bumper_gap_m":
                        (
                            ""
                            if minimum_bumper_gap
                            is None
                            else
                            f"{minimum_bumper_gap:.3f}"
                        ),

                    "final_ego_speed_mps":
                        f"{final_speed:.3f}",

                    "estimated_collision":
                        self.estimated_collision,

                    "end_reason":
                        end_reason,

                    "result":
                        result
                }
            )


    # ========================================================
    # RUN TEST
    # ========================================================

    def run(self):

        print()
        print(
            "============================================================"
        )
        print(
            " GAZEBO MOVING-OBSTACLE RELATIVE TTC TEST"
        )
        print(
            "============================================================"
        )

        print(
            f"Ego target speed      : "
            f"{EGO_TARGET_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Obstacle target speed : "
            f"{OBSTACLE_TARGET_SPEED_MPS:.2f} m/s"
        )

        print(
            f"Expected closing speed: "
            f"{EGO_TARGET_SPEED_MPS - OBSTACLE_TARGET_SPEED_MPS:.2f} m/s"
        )

        print(
            f"TTC threshold         : "
            f"{TTC_THRESHOLD_SECONDS:.2f} s"
        )

        print()


        if not self.reset_world():

            print(
                "RESULT: FAIL"
            )

            return False


        print()
        print(
            "Starting both vehicles..."
        )
        print()


        start_time = time.monotonic()

        stable_stop_start = None

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

                    aeb_triggered = (
                        self.aeb_triggered
                    )


                if not sensors_ready:

                    self.command_ego(
                        0.0
                    )

                    self.command_obstacle(
                        OBSTACLE_TARGET_SPEED_MPS
                    )

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # --------------------------------------------
                # Bumper gap / collision proxy
                # --------------------------------------------

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

                    with self.lock:

                        self.estimated_collision = (
                            True
                        )

                    end_reason = (
                        "estimated_collision"
                    )

                    break


                # --------------------------------------------
                # Relative TTC
                # --------------------------------------------

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


                # --------------------------------------------
                # AEB trigger
                # --------------------------------------------

                if (
                    not aeb_triggered
                    and math.isfinite(ttc)
                    and ttc
                    <= TTC_THRESHOLD_SECONDS
                ):

                    with self.lock:

                        self.aeb_triggered = True

                        self.trigger_distance_m = (
                            distance
                        )

                        self.trigger_ego_speed_mps = (
                            ego_speed
                        )

                        self.trigger_closing_speed_mps = (
                            closing_speed
                        )

                        self.trigger_ttc_s = (
                            ttc
                        )


                    aeb_triggered = True


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
                        f"{distance:.3f} m"
                    )

                    print(
                        f"Ego speed     : "
                        f"{ego_speed:.3f} m/s"
                    )

                    print(
                        f"Closing speed : "
                        f"{closing_speed:.3f} m/s"
                    )

                    print(
                        f"TTC           : "
                        f"{ttc:.3f} s"
                    )

                    print()


                # --------------------------------------------
                # Commands
                # --------------------------------------------

                # Lead obstacle jatkaa koko ajan 2 m/s.
                self.command_obstacle(
                    OBSTACLE_TARGET_SPEED_MPS
                )


                if aeb_triggered:

                    self.command_ego(
                        0.0
                    )

                    state = "AEB BRAKE"

                else:

                    self.command_ego(
                        EGO_TARGET_SPEED_MPS
                    )

                    state = "FOLLOW"


                # --------------------------------------------
                # Stable stop
                # --------------------------------------------

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


                # --------------------------------------------
                # Status print
                # --------------------------------------------

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


        except KeyboardInterrupt:

            end_reason = (
                "user_interrupt"
            )


        self.stop_both()

        time.sleep(
            0.25
        )


        with self.lock:

            final_speed = (
                self.ego_speed_mps
            )

            minimum_distance = (
                self.minimum_distance_m
            )


        if minimum_distance is None:

            minimum_bumper_gap = None

        else:

            minimum_bumper_gap = (
                minimum_distance
                - LIDAR_TO_FRONT_BUMPER_M
            )


        stopped = (
            final_speed
            <= STOP_SPEED_THRESHOLD_MPS
        )


        safe_gap = (
            minimum_bumper_gap is not None
            and minimum_bumper_gap
            > MIN_SAFE_BUMPER_GAP_M
        )


        passed = (
            self.aeb_triggered
            and self.trigger_ttc_s
            is not None
            and self.trigger_ttc_s
            <= TTC_THRESHOLD_SECONDS
            and stopped
            and safe_gap
            and not self.estimated_collision
            and end_reason
            == "ego_stopped"
        )


        result = (
            "PASS"
            if passed
            else "FAIL"
        )


        # ====================================================
        # RESULT
        # ====================================================

        print()
        print(
            "============================================================"
        )
        print(
            " RELATIVE TTC TEST RESULT"
        )
        print(
            "============================================================"
        )

        print(
            f"AEB triggered          : "
            f"{'YES' if self.aeb_triggered else 'NO'}"
        )


        if self.trigger_distance_m is not None:

            print(
                f"Trigger distance       : "
                f"{self.trigger_distance_m:.3f} m"
            )


        if self.trigger_ego_speed_mps is not None:

            print(
                f"Trigger ego speed      : "
                f"{self.trigger_ego_speed_mps:.3f} m/s"
            )


        if (
            self.trigger_closing_speed_mps
            is not None
        ):

            print(
                f"Measured closing speed : "
                f"{self.trigger_closing_speed_mps:.3f} m/s"
            )


        print(
            f"Expected closing speed : "
            f"{EGO_TARGET_SPEED_MPS - OBSTACLE_TARGET_SPEED_MPS:.3f} m/s"
        )


        if self.trigger_ttc_s is not None:

            print(
                f"Trigger TTC            : "
                f"{self.trigger_ttc_s:.3f} s"
            )


        if minimum_distance is not None:

            print(
                f"Minimum LiDAR distance : "
                f"{minimum_distance:.3f} m"
            )


        if minimum_bumper_gap is not None:

            print(
                f"Minimum bumper gap     : "
                f"{minimum_bumper_gap:.3f} m"
            )


        print(
            f"Final ego speed        : "
            f"{final_speed:.3f} m/s"
        )

        print(
            f"Estimated collision    : "
            f"{'YES' if self.estimated_collision else 'NO'}"
        )

        print(
            f"End reason             : "
            f"{end_reason}"
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
            minimum_bumper_gap,
            final_speed,
            end_reason
        )


        print(
            f"CSV: {RESULTS_FILE}"
        )


        return passed


# ============================================================
# MAIN
# ============================================================

def main():

    test = RelativeTTCTest()

    success = test.run()

    if success:

        raise SystemExit(0)

    raise SystemExit(1)


if __name__ == "__main__":

    main()
