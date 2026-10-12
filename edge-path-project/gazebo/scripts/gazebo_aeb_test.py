import csv
import math
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

LIDAR_TOPIC = "/aeb/lidar"
ODOMETRY_TOPIC = "/model/aeb_vehicle/odometry"
CMD_VEL_TOPIC = "/model/aeb_vehicle/cmd_vel"

TARGET_SPEED_MPS = 2.0

TTC_THRESHOLD_SECONDS = 1.7

MIN_SPEED_FOR_TTC_MPS = 0.10

FRONT_HALF_ANGLE_DEG = 10.0

CONTROL_PERIOD_SECONDS = 0.05

PRINT_INTERVAL_SECONDS = 0.25

SENSOR_TIMEOUT_SECONDS = 0.50

TEST_TIMEOUT_SECONDS = 30.0

STOP_SPEED_THRESHOLD_MPS = 0.05

STOP_STABLE_TIME_SECONDS = 0.75

# LiDAR on noin 0.30 m auton etupuskurin takana.
LIDAR_TO_FRONT_BUMPER_M = 0.30

# PASS vaatii tämän verran vähintään tilaa puskurin
# ja esteen väliin pysähtymisen jälkeen.
MIN_FINAL_BUMPER_GAP_M = 0.20


# ============================================================
# RESULT FILE
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

RESULTS_DIR = SCRIPT_DIR.parent / "results"

RESULTS_FILE = (
    RESULTS_DIR
    / "gazebo_aeb_results.csv"
)


# ============================================================
# TEST
# ============================================================

class GazeboAEBTest:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()

        self.front_distance_m = None
        self.speed_mps = 0.0

        self.lidar_received = False
        self.odometry_received = False

        self.last_lidar_time = 0.0
        self.last_odometry_time = 0.0

        self.aeb_triggered = False

        self.trigger_distance_m = None
        self.trigger_speed_mps = None
        self.trigger_ttc_s = None
        self.trigger_time = None

        self.estimated_collision = False

        self.last_print_time = 0.0

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
                "Failed to subscribe to "
                f"{LIDAR_TOPIC}"
            )

        if not odometry_ok:

            raise RuntimeError(
                "Failed to subscribe to "
                f"{ODOMETRY_TOPIC}"
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

            self.front_distance_m = (
                front_distance
            )

            self.lidar_received = True

            self.last_lidar_time = now


    def odometry_callback(
        self,
        msg: Odometry
    ):

        speed = abs(
            msg.twist.linear.x
        )

        now = time.monotonic()


        with self.lock:

            self.speed_mps = speed

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


    # ========================================================
    # CSV
    # ========================================================

    def save_result(
        self,
        result,
        final_distance,
        final_bumper_gap,
        final_speed
    ):

        RESULTS_DIR.mkdir(
            parents=True,
            exist_ok=True
        )

        file_exists = (
            RESULTS_FILE.exists()
        )


        with RESULTS_FILE.open(
            "a",
            newline="",
            encoding="utf-8"
        ) as csv_file:

            fieldnames = [
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
                "result"
            ]

            writer = csv.DictWriter(
                csv_file,
                fieldnames=fieldnames
            )


            if not file_exists:

                writer.writeheader()


            writer.writerow(
                {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),

                    "target_speed_mps":
                        f"{TARGET_SPEED_MPS:.3f}",

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

                    "trigger_speed_mps":
                        (
                            ""
                            if self.trigger_speed_mps
                            is None
                            else
                            f"{self.trigger_speed_mps:.3f}"
                        ),

                    "trigger_ttc_s":
                        (
                            ""
                            if self.trigger_ttc_s
                            is None
                            else
                            f"{self.trigger_ttc_s:.3f}"
                        ),

                    "final_lidar_distance_m":
                        (
                            ""
                            if final_distance
                            is None
                            else
                            f"{final_distance:.3f}"
                        ),

                    "final_bumper_gap_m":
                        (
                            ""
                            if final_bumper_gap
                            is None
                            else
                            f"{final_bumper_gap:.3f}"
                        ),

                    "final_speed_mps":
                        f"{final_speed:.3f}",

                    "estimated_collision":
                        self.estimated_collision,

                    "result":
                        result
                }
            )


    # ========================================================
    # FINAL RESULT
    # ========================================================

    def finish_test(
        self,
        reason
    ):

        for _ in range(10):

            self.stop_vehicle()

            time.sleep(
                0.02
            )


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
            self.aeb_triggered
            and stopped
            and safe_gap
            and not self.estimated_collision
            and reason == "vehicle_stopped"
        )


        result = (
            "PASS"
            if passed
            else "FAIL"
        )


        print()
        print(
            "========================================"
        )
        print(
            " GAZEBO AEB TEST RESULT"
        )
        print(
            "========================================"
        )

        print(
            f"AEB triggered       : "
            f"{'YES' if self.aeb_triggered else 'NO'}"
        )


        if self.trigger_distance_m is not None:

            print(
                f"Trigger distance    : "
                f"{self.trigger_distance_m:.3f} m"
            )


        if self.trigger_speed_mps is not None:

            print(
                f"Trigger speed       : "
                f"{self.trigger_speed_mps:.3f} m/s"
            )


        if self.trigger_ttc_s is not None:

            print(
                f"Trigger TTC         : "
                f"{self.trigger_ttc_s:.3f} s"
            )


        if final_distance is not None:

            print(
                f"Final LiDAR distance: "
                f"{final_distance:.3f} m"
            )


        if final_bumper_gap is not None:

            print(
                f"Final bumper gap    : "
                f"{final_bumper_gap:.3f} m"
            )


        print(
            f"Final speed         : "
            f"{final_speed:.3f} m/s"
        )

        print(
            f"Estimated collision : "
            f"{'YES' if self.estimated_collision else 'NO'}"
        )

        print(
            f"End reason          : "
            f"{reason}"
        )

        print()

        print(
            f"RESULT: {result}"
        )

        print(
            "========================================"
        )

        print()

        print(
            f"CSV: {RESULTS_FILE}"
        )


        self.save_result(
            result,
            final_distance,
            final_bumper_gap,
            final_speed
        )


        return passed


    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def run(self):

        print()
        print(
            "========================================"
        )
        print(
            " AUTOMATIC GAZEBO TTC AEB TEST"
        )
        print(
            "========================================"
        )

        print(
            f"Target speed : "
            f"{TARGET_SPEED_MPS:.2f} m/s"
        )

        print(
            f"TTC threshold: "
            f"{TTC_THRESHOLD_SECONDS:.2f} s"
        )

        print(
            "Waiting for sensors..."
        )

        print()


        start_time = time.monotonic()

        stable_stop_start = None


        try:

            while True:

                now = time.monotonic()


                if (
                    now - start_time
                    > TEST_TIMEOUT_SECONDS
                ):

                    self.stop_vehicle()

                    return self.finish_test(
                        "timeout"
                    )


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

                    aeb_triggered = (
                        self.aeb_triggered
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
                # Estimate bumper gap
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

                    self.estimated_collision = True

                    self.stop_vehicle()

                    return self.finish_test(
                        "estimated_collision"
                    )


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
                # Trigger AEB
                # ------------------------------------------------

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

                        self.trigger_speed_mps = (
                            speed
                        )

                        self.trigger_ttc_s = (
                            ttc
                        )

                        self.trigger_time = now


                    aeb_triggered = True


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
                        f"Distance : "
                        f"{distance:.3f} m"
                    )

                    print(
                        f"Speed    : "
                        f"{speed:.3f} m/s"
                    )

                    print(
                        f"TTC      : "
                        f"{ttc:.3f} s"
                    )

                    print()


                # ------------------------------------------------
                # Command
                # ------------------------------------------------

                if aeb_triggered:

                    self.stop_vehicle()

                    state = "AEB BRAKE"

                else:

                    self.publish_speed(
                        TARGET_SPEED_MPS
                    )

                    state = "DRIVE"


                # ------------------------------------------------
                # Detect stable stop
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

                            return self.finish_test(
                                "vehicle_stopped"
                            )

                    else:

                        stable_stop_start = None


                # ------------------------------------------------
                # Status print
                # ------------------------------------------------

                if (
                    now - self.last_print_time
                    >= PRINT_INTERVAL_SECONDS
                ):

                    if distance is None:

                        distance_text = (
                            "none"
                        )

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
                        f"speed={speed:5.2f} m/s | "
                        f"distance={distance_text:>8s} | "
                        f"TTC={ttc_text:>7s}"
                    )


                    self.last_print_time = now


                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )


        except KeyboardInterrupt:

            print()
            print(
                "Test interrupted by user."
            )

            self.stop_vehicle()

            return self.finish_test(
                "user_interrupt"
            )


# ============================================================
# MAIN
# ============================================================

def main():

    test = GazeboAEBTest()

    success = test.run()

    if success:

        raise SystemExit(0)

    raise SystemExit(1)


if __name__ == "__main__":

    main()
