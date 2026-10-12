import math
import threading
import time

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

# Normaali ajonopeus testissä.
TARGET_SPEED_MPS = 2.0

# Sama TTC-raja kuin aiemmassa AEB-testissä.
TTC_THRESHOLD_SECONDS = 1.7

# Alle tämän nopeuden TTC:tä ei lasketa.
MIN_SPEED_FOR_TTC_MPS = 0.10

# Hätäraja riippumatta TTC:stä.
EMERGENCY_DISTANCE_M = 0.80

# Tarkastellaan vain auton edessä olevaa sektoria.
FRONT_HALF_ANGLE_DEG = 10.0

# Controllerin päivitystaajuus.
CONTROL_PERIOD_SECONDS = 0.05

# Kuinka usein tilanne tulostetaan.
PRINT_INTERVAL_SECONDS = 0.25

# Jos sensoridataa ei ole tullut hetkeen, pysäytetään auto.
SENSOR_TIMEOUT_SECONDS = 0.50


# ============================================================
# AEB CONTROLLER
# ============================================================

class AEBController:

    def __init__(self):

        self.node = Node()

        self.lock = threading.Lock()

        self.front_distance_m = None
        self.speed_mps = 0.0

        self.lidar_received = False
        self.odometry_received = False

        self.last_lidar_time = 0.0
        self.last_odometry_time = 0.0

        self.aeb_latched = False

        self.last_print_time = 0.0

        self.cmd_publisher = self.node.advertise(
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
                f"LiDAR-topicin tilaaminen epäonnistui: "
                f"{LIDAR_TOPIC}"
            )

        if not odometry_ok:
            raise RuntimeError(
                f"Odometry-topicin tilaaminen epäonnistui: "
                f"{ODOMETRY_TOPIC}"
            )


    # ========================================================
    # LIDAR
    # ========================================================

    def lidar_callback(self, msg: LaserScan):

        front_half_angle_rad = math.radians(
            FRONT_HALF_ANGLE_DEG
        )

        valid_ranges = []

        for index, distance in enumerate(msg.ranges):

            angle = (
                msg.angle_min
                + index * msg.angle_step
            )

            if abs(angle) > front_half_angle_rad:
                continue

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            valid_ranges.append(distance)

        if valid_ranges:
            front_distance = min(valid_ranges)
        else:
            front_distance = None

        now = time.monotonic()

        with self.lock:

            self.front_distance_m = front_distance
            self.lidar_received = True
            self.last_lidar_time = now


    # ========================================================
    # ODOMETRY
    # ========================================================

    def odometry_callback(self, msg: Odometry):

        # Ackermann-auton etenemisnopeus.
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

    def publish_speed(self, speed_mps):

        command = Twist()

        command.linear.x = speed_mps
        command.angular.z = 0.0

        self.cmd_publisher.publish(
            command
        )


    def publish_stop(self):

        self.publish_speed(
            0.0
        )


    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def run(self):

        print()
        print("========================================")
        print(" GAZEBO TTC AEB CONTROLLER")
        print("========================================")
        print()
        print(
            f"Target speed : "
            f"{TARGET_SPEED_MPS:.2f} m/s"
        )
        print(
            f"TTC threshold: "
            f"{TTC_THRESHOLD_SECONDS:.2f} s"
        )
        print(
            f"Emergency distance: "
            f"{EMERGENCY_DISTANCE_M:.2f} m"
        )
        print(
            f"LiDAR sector: "
            f"±{FRONT_HALF_ANGLE_DEG:.1f} deg"
        )
        print()
        print("Ctrl+C lopettaa.")
        print()

        try:

            while True:

                now = time.monotonic()

                with self.lock:

                    front_distance = self.front_distance_m
                    speed = self.speed_mps

                    lidar_received = self.lidar_received
                    odometry_received = self.odometry_received

                    last_lidar_time = self.last_lidar_time
                    last_odometry_time = (
                        self.last_odometry_time
                    )

                    aeb_latched = self.aeb_latched


                # ============================================
                # SENSOR CHECK
                # ============================================

                sensors_ready = (
                    lidar_received
                    and odometry_received
                )

                sensors_fresh = (
                    sensors_ready
                    and (
                        now - last_lidar_time
                        <= SENSOR_TIMEOUT_SECONDS
                    )
                    and (
                        now - last_odometry_time
                        <= SENSOR_TIMEOUT_SECONDS
                    )
                )


                if not sensors_fresh:

                    self.publish_stop()

                    if (
                        now - self.last_print_time
                        >= PRINT_INTERVAL_SECONDS
                    ):

                        print(
                            "WAITING: sensor data "
                            "missing or stale -> STOP"
                        )

                        self.last_print_time = now

                    time.sleep(
                        CONTROL_PERIOD_SECONDS
                    )

                    continue


                # ============================================
                # TTC CALCULATION
                # ============================================

                ttc = math.inf

                if (
                    front_distance is not None
                    and speed >= MIN_SPEED_FOR_TTC_MPS
                ):

                    ttc = (
                        front_distance
                        / speed
                    )


                # ============================================
                # AEB TRIGGER
                # ============================================

                trigger_reason = None

                if not aeb_latched:

                    if (
                        front_distance is not None
                        and front_distance
                        <= EMERGENCY_DISTANCE_M
                    ):

                        trigger_reason = (
                            "emergency distance"
                        )

                    elif (
                        front_distance is not None
                        and speed
                        >= MIN_SPEED_FOR_TTC_MPS
                        and ttc
                        <= TTC_THRESHOLD_SECONDS
                    ):

                        trigger_reason = (
                            "TTC threshold"
                        )


                    if trigger_reason is not None:

                        with self.lock:
                            self.aeb_latched = True

                        aeb_latched = True

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
                            f"Reason   : "
                            f"{trigger_reason}"
                        )
                        print(
                            f"Distance : "
                            f"{front_distance:.2f} m"
                        )
                        print(
                            f"Speed    : "
                            f"{speed:.2f} m/s"
                        )

                        if math.isfinite(ttc):

                            print(
                                f"TTC      : "
                                f"{ttc:.3f} s"
                            )

                        print()


                # ============================================
                # CONTROL OUTPUT
                # ============================================

                if aeb_latched:

                    command_speed = 0.0
                    state = "AEB BRAKE"

                else:

                    command_speed = (
                        TARGET_SPEED_MPS
                    )

                    state = "DRIVE"


                self.publish_speed(
                    command_speed
                )


                # ============================================
                # STATUS OUTPUT
                # ============================================

                if (
                    now - self.last_print_time
                    >= PRINT_INTERVAL_SECONDS
                ):

                    if front_distance is None:

                        distance_text = (
                            "no obstacle"
                        )

                    else:

                        distance_text = (
                            f"{front_distance:.2f} m"
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
                        f"distance={distance_text:>11s} | "
                        f"TTC={ttc_text:>7s} | "
                        f"cmd={command_speed:.2f} m/s"
                    )

                    self.last_print_time = now


                time.sleep(
                    CONTROL_PERIOD_SECONDS
                )


        except KeyboardInterrupt:

            print()
            print(
                "Controller stopped by user."
            )

        finally:

            # Lähetetään useita STOP-komentoja,
            # jotta auto varmasti pysähtyy myös
            # controllerin sulkeutuessa.

            print(
                "Sending final STOP command..."
            )

            for _ in range(5):

                self.publish_stop()

                time.sleep(
                    0.02
                )

            print(
                "Done."
            )


# ============================================================
# MAIN
# ============================================================

def main():

    controller = AEBController()

    controller.run()


if __name__ == "__main__":

    main()
