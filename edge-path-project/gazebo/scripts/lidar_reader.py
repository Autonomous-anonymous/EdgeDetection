import math
import time

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node


LIDAR_TOPIC = "/aeb/lidar"

# Tarkastellaan vain kapeaa aluetta suoraan auton edessä.
FRONT_HALF_ANGLE_DEG = 10.0

# Tulostetaan korkeintaan kaksi kertaa sekunnissa.
PRINT_INTERVAL_SECONDS = 0.5


last_print_time = 0.0


def lidar_callback(msg: LaserScan):

    global last_print_time

    current_time = time.time()

    if (
        current_time
        - last_print_time
        < PRINT_INTERVAL_SECONDS
    ):
        return

    last_print_time = current_time


    if len(msg.ranges) == 0:

        print(
            "LiDAR: ei mittauksia"
        )

        return


    front_half_angle_rad = math.radians(
        FRONT_HALF_ANGLE_DEG
    )


    valid_front_ranges = []


    for index, distance in enumerate(
        msg.ranges
    ):

        angle = (
            msg.angle_min
            + index
            * msg.angle_step
        )


        # Tarkastellaan vain suoraan auton edessä
        # olevaa ±10 asteen sektoria.
        if (
            abs(angle)
            > front_half_angle_rad
        ):

            continue


        # Hylätään inf / nan ja sensorin
        # mittausalueen ulkopuoliset arvot.
        if (
            not math.isfinite(
                distance
            )
        ):

            continue


        if (
            distance
            < msg.range_min

            or

            distance
            > msg.range_max
        ):

            continue


        valid_front_ranges.append(
            distance
        )


    if not valid_front_ranges:

        print(
            "Front obstacle: ei havaintoa"
        )

        return


    minimum_distance = min(
        valid_front_ranges
    )


    print(
        f"Front obstacle distance: "
        f"{minimum_distance:.2f} m"
    )


def main():

    print(
        "Gazebo LiDAR reader"
    )

    print(
        f"Topic: {LIDAR_TOPIC}"
    )

    print(
        f"Front sector: "
        f"±{FRONT_HALF_ANGLE_DEG:.1f} deg"
    )

    print(
        "Ctrl+C lopettaa."
    )

    print()


    node = Node()


    subscribed = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback
    )


    if not subscribed:

        print(
            f"Virhe: topicin "
            f"{LIDAR_TOPIC} "
            f"tilaaminen epäonnistui."
        )

        return


    try:

        while True:

            time.sleep(
                0.01
            )


    except KeyboardInterrupt:

        print()

        print(
            "LiDAR reader lopetettu."
        )


if __name__ == "__main__":

    main()
