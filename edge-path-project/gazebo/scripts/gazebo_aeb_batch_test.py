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

WORLD_NAME = "aeb_world"

LIDAR_TOPIC = "/aeb/lidar"
ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
CMD_VEL_TOPIC = "/model/aeb_vehicle/cmd_vel"

TEST_SPEEDS_MPS = [
    1.0,
    2.0,
    3.0,
    4.0,
    5.0,
]

TTC_THRESHOLD_SECONDS = 1.7

MIN_SPEED_FOR_TTC_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 10.0

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.25

SENSOR_TIMEOUT_SECONDS = 0.50

SINGLE_TEST_TIMEOUT_SECONDS = 30.0

RESET_TIMEOUT_SECONDS = 10.0

RESET_POSITION_TOLERANCE_M = 0.75

STOP_SPEED_THRESHOLD_MPS = 0.05

STOP_STABLE_TIME_SECONDS = 0.75

# LiDAR on noin 0.30 m auton etupuskurin takana.
LIDAR_TO_FRONT_BUMPER_M = 0.30

# PASS vaatii vähintään tämän verran tilaa
# puskurin ja esteen väliin.
MIN_FINAL_BUMPER_GAP_M = 0.20


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "gazebo_aeb_batch_results.csv"
)


# ============================================================
# BATCH TEST
# ============================================================

class GazeboAEBBatchTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()

        self.front_distance_m = None
        self.speed_mps = 0.0
        self.position_x_m = None

        self.lidar_received = False
        self.odometry_received = False

        self.last_lidar_time = 0.0
        self.last_odometry_time = 0.0

        self.publisher = self.node.advertise(
            CMD_VEL_TOPIC,
            Twist
        )

        lidar_ok = self.node.subscribe(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback
        )

        odometry_ok = self.node.subscribe(
            Odometry,
            ODOMETRY_TOPIC,
            self.odometry_callback
        )

        if not lidar_ok:

            raise RuntimeError(
                f"Failed to subscribe to {LIDAR_TOPIC}"
            )

        if not odometry_ok:

            raise RuntimeError(
                f"Failed to subscribe to {ODOMETRY_TOPIC}"
            )


    # ========================================================
    # SENSOR CALLBACKS
    # ========================================================

    def lidar_callback(self, msg: LaserScan):

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

            front_distance = min(
                valid_ranges
            )

        else:

            front_distance = None


        now = time.monotonic()


        with self.lock:

            self.front_distance_m = front_distance

            self.lidar_received = True

            self.last_lidar_time = now


    def odometry_callback(
        self,
        msg: Odometry
    ):

        speed = abs(
            msg.twist.linear.x
        )

        position_x = (
            msg.pose.position.x
        )

        now = time.monotonic()


        with self.lock:

            self.speed_mps = speed

            self.position_x_m = position_x

            self.odometry_received = True

            self.last_odometry_time = now


    # ========================================================
    # VEHICLE COMMAND
    # ========================================================

    def publish_speed(
        self,
        speed_mps
    ):

        msg = Twist()

        msg.linear.x = speed_mps
        msg.angular.z = 0.0

        self.publisher.publish(
            msg
        )


    def stop_vehicle(self):

        self.publish_speed(
            0.0
        )


    def send_repeated_stop(self):

        for _ in range(10):

            self.stop_vehicle()

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
                "Gazebo world-control command failed:"
            )

            print(
                output
            )

            return False


        if "data: true" not in output:

            print(
                "Gazebo world-control returned "
                "an unexpected response:"
            )

            print(
                output
            )

            return False


        return True


    def reset_world(self):

        print()
        print(
            "Resetting Gazebo world..."
        )


        self.send_repeated_stop()


        with self.lock:

            self.front_distance_m = None
            self.speed_mps = 0.0
            self.position_x_m = None

            self.lidar_received = False
            self.odometry_received = False

            self.last_lidar_time = 0.0
            self.last_odometry_time = 0.0


        reset_ok = self.call_world_control(
            "reset: {all: true}"
        )


        if not reset_ok:

            return False


        # Varmistetaan, että simulaatio jatkuu resetin jälkeen.
        play_ok = self.call_world_control(
            "pause: false"
        )


        if not play_ok:

            return False


        start_wait = time.monotonic()


        while True:

            now = time.monotonic()


            if (
                now - start_wait
                > RESET_TIMEOUT_SECONDS
            ):

                print(
                    "ERROR: Vehicle did not return "
                    "to the start position."
                )

                return False


            with self.lock:

                lidar_received = (
                    self.lidar_received
                )

                odometry_received = (
                    self.odometry_received
                )

                distance = (
                    self.front_distance_m
                )

                speed = (
                    self.speed_mps
                )

                position_x = (
                    self.position_x_m
                )

                lidar_age = (
                    now
                    - self.last_lidar_time
                )

                odometry_age = (
                    now
                    - self.last_odometry_time
                )


            sensors_ready = (
                lidar_received
                and odometry_received
                and lidar_age
                <= SENSOR_TIMEOUT_SECONDS
                and odometry_age
                <= SENSOR_TIMEOUT_SECONDS
            )


            position_ready = (
                position_x is not None
                and abs(position_x)
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
                    f"x={position_x:.3f} m | "
                    f"speed={speed:.3f} m/s | "
                    f"obstacle={distance:.3f} m"
                )

                time.sleep(
                    0.50
                )

                return True


            time.sleep(
                0.05
            )


    # ========================================================
    # SINGLE TEST
    # ========================================================

    def run_single_test(
        self,
        target_speed
    ):

        print()
        print(
            "========================================"
        )
        print(
            f" TEST SPEED: {target_speed:.1f} m/s"
        )
        print(
            "========================================"
        )


        if not self.reset_world():

            return {
                "target_speed_mps":
                    target_speed,

                "aeb_triggered":
                    False,

                "trigger_distance_m":
                    None,

                "trigger_speed_mps":
                    None,

                "trigger_ttc_s":
                    None,

                "final_lidar_distance_m":
                    None,

                "final_bumper_gap_m":
                    None,

                "final_speed_mps":
                    None,

                "estimated_collision":
                    False,

                "end_reason":
                    "reset_failed",

                "result":
                    "FAIL"
            }


        test_start = time.monotonic()

        last_print_time = 0.0

        aeb_triggered = False

        trigger_distance = None
        trigger_speed = None
        trigger_ttc = None

        stable_stop_start = None

        estimated_collision = False

        end_reason = None


        while True:

            now = time.monotonic()


            if (
                now - test_start
                > SINGLE_TEST_TIMEOUT_SECONDS
            ):

                end_reason = "timeout"

                self.send_repeated_stop()

                break


            with self.lock:

                distance = (
                    self.front_distance_m
                )

                speed = (
                    self.speed_mps
                )

                lidar_received = (
                    self.lidar_received
                )

                odometry_received = (
                    self.odometry_received
                )

                lidar_age = (
                    now
                    - self.last_lidar_time
                )

                odometry_age = (
                    now
                    - self.last_odometry_time
                )


            sensors_ready = (
                lidar_received
                and odometry_received
                and lidar_age
                <= SENSOR_TIMEOUT_SECONDS
                and odometry_age
                <= SENSOR_TIMEOUT_SECONDS
            )


            if not sensors_ready:

                self.stop_vehicle()

                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )

                continue


            # ------------------------------------------------
            # Bumper gap
            # ------------------------------------------------

            if distance is None:

                bumper_gap = None

            else:

                bumper_gap = (
                    distance
                    - LIDAR_TO_FRONT_BUMPER_M
                )


            # ------------------------------------------------
            # Collision proxy
            # ------------------------------------------------

            if (
                bumper_gap is not None
                and bumper_gap <= 0.0
            ):

                estimated_collision = True

                end_reason = (
                    "estimated_collision"
                )

                self.send_repeated_stop()

                break


            # ------------------------------------------------
            # TTC
            # ------------------------------------------------

            if (
                distance is not None
                and speed
                >= MIN_SPEED_FOR_TTC_MPS
            ):

                ttc = (
                    distance
                    / speed
                )

            else:

                ttc = math.inf


            # ------------------------------------------------
            # AEB trigger
            # ------------------------------------------------

            if (
                not aeb_triggered
                and math.isfinite(ttc)
                and ttc
                <= TTC_THRESHOLD_SECONDS
            ):

                aeb_triggered = True

                trigger_distance = distance
                trigger_speed = speed
                trigger_ttc = ttc


                print()
                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )
                print(
                    " AEB TRIGGERED"
                )
                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )

                print(
                    f"Target   : "
                    f"{target_speed:.3f} m/s"
                )

                print(
                    f"Distance : "
                    f"{trigger_distance:.3f} m"
                )

                print(
                    f"Speed    : "
                    f"{trigger_speed:.3f} m/s"
                )

                print(
                    f"TTC      : "
                    f"{trigger_ttc:.3f} s"
                )

                print()


            # ------------------------------------------------
            # Vehicle command
            # ------------------------------------------------

            if aeb_triggered:

                self.stop_vehicle()

                state = "AEB BRAKE"

            else:

                self.publish_speed(
                    target_speed
                )

                state = "DRIVE"


            # ------------------------------------------------
            # Stable stop detection
            # ------------------------------------------------

            if aeb_triggered:

                if (
                    speed
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
                            "vehicle_stopped"
                        )

                        self.send_repeated_stop()

                        break

                else:

                    stable_stop_start = None


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


                if math.isfinite(ttc):

                    ttc_text = (
                        f"{ttc:.2f} s"
                    )

                else:

                    ttc_text = "inf"


                print(
                    f"{state:10s} | "
                    f"target={target_speed:4.1f} | "
                    f"speed={speed:5.2f} m/s | "
                    f"distance={distance_text:>8s} | "
                    f"TTC={ttc_text:>7s}"
                )


                last_print_time = now


            time.sleep(
                CONTROL_PERIOD_SECONDS
            )


        # ====================================================
        # FINAL VALUES
        # ====================================================

        time.sleep(
            0.20
        )


        with self.lock:

            final_distance = (
                self.front_distance_m
            )

            final_speed = (
                self.speed_mps
            )


        if final_distance is None:

            final_bumper_gap = None

        else:

            final_bumper_gap = (
                final_distance
                - LIDAR_TO_FRONT_BUMPER_M
            )


        stopped = (
            final_speed
            <= STOP_SPEED_THRESHOLD_MPS
        )


        safe_gap = (
            final_bumper_gap is not None
            and final_bumper_gap
            > MIN_FINAL_BUMPER_GAP_M
        )


        passed = (
            aeb_triggered
            and trigger_ttc is not None
            and trigger_ttc
            <= TTC_THRESHOLD_SECONDS
            and stopped
            and safe_gap
            and not estimated_collision
            and end_reason
            == "vehicle_stopped"
        )


        result = (
            "PASS"
            if passed
            else "FAIL"
        )


        print()
        print(
            f"Result for {target_speed:.1f} m/s:"
        )

        print(
            f"  AEB triggered    : "
            f"{'YES' if aeb_triggered else 'NO'}"
        )


        if trigger_ttc is not None:

            print(
                f"  Trigger TTC      : "
                f"{trigger_ttc:.3f} s"
            )


        if final_distance is not None:

            print(
                f"  Final distance   : "
                f"{final_distance:.3f} m"
            )


        if final_bumper_gap is not None:

            print(
                f"  Final bumper gap : "
                f"{final_bumper_gap:.3f} m"
            )


        print(
            f"  Final speed      : "
            f"{final_speed:.3f} m/s"
        )

        print(
            f"  Collision        : "
            f"{'YES' if estimated_collision else 'NO'}"
        )

        print(
            f"  RESULT           : "
            f"{result}"
        )


        return {
            "target_speed_mps":
                target_speed,

            "aeb_triggered":
                aeb_triggered,

            "trigger_distance_m":
                trigger_distance,

            "trigger_speed_mps":
                trigger_speed,

            "trigger_ttc_s":
                trigger_ttc,

            "final_lidar_distance_m":
                final_distance,

            "final_bumper_gap_m":
                final_bumper_gap,

            "final_speed_mps":
                final_speed,

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


        batch_id = (
            datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
        )


        file_exists = (
            RESULTS_FILE.exists()
        )


        fieldnames = [
            "batch_id",
            "timestamp",
            "target_speed_mps",
            "ttc_threshold_s",
            "aeb_triggered",
            "trigger_distance_m",
            "trigger_speed_mps",
            "trigger_ttc_s",
            "final_lidar_distance_m",
            "final_bumper_gap_m",
            "final_speed_mps",
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
                fieldnames=fieldnames
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

                        "target_speed_mps":
                            (
                                f"{result['target_speed_mps']:.3f}"
                            ),

                        "ttc_threshold_s":
                            (
                                f"{TTC_THRESHOLD_SECONDS:.3f}"
                            ),

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

                        "trigger_speed_mps":
                            (
                                ""
                                if result["trigger_speed_mps"]
                                is None
                                else
                                f"{result['trigger_speed_mps']:.3f}"
                            ),

                        "trigger_ttc_s":
                            (
                                ""
                                if result["trigger_ttc_s"]
                                is None
                                else
                                f"{result['trigger_ttc_s']:.3f}"
                            ),

                        "final_lidar_distance_m":
                            (
                                ""
                                if result["final_lidar_distance_m"]
                                is None
                                else
                                f"{result['final_lidar_distance_m']:.3f}"
                            ),

                        "final_bumper_gap_m":
                            (
                                ""
                                if result["final_bumper_gap_m"]
                                is None
                                else
                                f"{result['final_bumper_gap_m']:.3f}"
                            ),

                        "final_speed_mps":
                            (
                                ""
                                if result["final_speed_mps"]
                                is None
                                else
                                f"{result['final_speed_mps']:.3f}"
                            ),

                        "estimated_collision":
                            result[
                                "estimated_collision"
                            ],

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
            "=============================================================="
        )
        print(
            " GAZEBO TTC AEB BATCH TEST SUMMARY"
        )
        print(
            "=============================================================="
        )

        print(
            "Speed   Trigger TTC   Final gap   Collision   Result"
        )

        print(
            "--------------------------------------------------------------"
        )


        for result in results:

            speed = (
                result[
                    "target_speed_mps"
                ]
            )

            trigger_ttc = (
                result[
                    "trigger_ttc_s"
                ]
            )

            final_gap = (
                result[
                    "final_bumper_gap_m"
                ]
            )


            if trigger_ttc is None:

                ttc_text = "   ---"

            else:

                ttc_text = (
                    f"{trigger_ttc:6.3f}s"
                )


            if final_gap is None:

                gap_text = "   ---"

            else:

                gap_text = (
                    f"{final_gap:6.3f}m"
                )


            collision_text = (
                "YES"
                if result[
                    "estimated_collision"
                ]
                else "NO"
            )


            print(
                f"{speed:4.1f}   "
                f"{ttc_text:>11s}   "
                f"{gap_text:>9s}   "
                f"{collision_text:^9s}   "
                f"{result['result']}"
            )


        print(
            "=============================================================="
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
            "=============================================================="
        )


    # ========================================================
    # RUN BATCH
    # ========================================================

    def run(self):

        print()
        print(
            "=============================================================="
        )
        print(
            " AUTOMATIC GAZEBO TTC AEB BATCH TEST"
        )
        print(
            "=============================================================="
        )

        print(
            "Test speeds:"
        )

        for speed in TEST_SPEEDS_MPS:

            print(
                f"  {speed:.1f} m/s"
            )

        print()

        print(
            f"TTC threshold: "
            f"{TTC_THRESHOLD_SECONDS:.2f} s"
        )

        print()


        results = []


        try:

            for target_speed in TEST_SPEEDS_MPS:

                result = self.run_single_test(
                    target_speed
                )

                results.append(
                    result
                )


        except KeyboardInterrupt:

            print()
            print(
                "Batch test interrupted by user."
            )

            self.send_repeated_stop()


        finally:

            self.send_repeated_stop()


        if results:

            self.save_results(
                results
            )

            self.print_summary(
                results
            )


        # Palautetaan auto lopuksi lähtöpisteeseen.
        print()
        print(
            "Returning vehicle to start position..."
        )

        self.reset_world()

        self.send_repeated_stop()


        all_passed = (
            len(results)
            == len(TEST_SPEEDS_MPS)
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

    test = GazeboAEBBatchTest()

    success = test.run()

    if success:

        raise SystemExit(0)

    raise SystemExit(1)


if __name__ == "__main__":

    main()
