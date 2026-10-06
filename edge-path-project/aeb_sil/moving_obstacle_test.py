import carla
import csv
import json
import math
import time

from datetime import datetime
from pathlib import Path

import zmq


# =========================================================
# TEST CONFIGURATION
# =========================================================

EGO_TARGET_SPEED_KMH = 50.0

OBSTACLE_SPEEDS_KMH = [
    0.0,
    20.0,
    30.0,
    40.0
]

FIXED_DELTA_SECONDS = 0.05

TTC_TRIGGER_S = 1.7

SAFETY_MARGIN_M = 1.0

STOP_SPEED_KMH = 0.5

TEST_TIMEOUT_SECONDS = 20.0


# =========================================================
# TEST START DISTANCE CONFIGURATION
# =========================================================

# Lisätila TTC-laukaisupisteen eteen.
START_DISTANCE_BUFFER_M = 10.0

# Pienin sallittu ajoneuvojen keskipiste-etäisyys.
MIN_CENTER_DISTANCE_M = 15.0


# =========================================================
# RESULTS PATH
# =========================================================

# Tämä tiedosto sijaitsee:
#
# edge-path-project/aeb_sil/moving_obstacle_test.py
#
# parent       = aeb_sil
# parent.parent = edge-path-project

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parent
    .parent
)

RESULTS_DIR = (
    PROJECT_ROOT
    / "results"
)

RESULTS_FILE = (
    RESULTS_DIR
    / "aeb_relative_ttc_results.csv"
)


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = (
    "tcp://127.0.0.1:5555"
)

CONTROL_ADDRESS = (
    "tcp://127.0.0.1:5556"
)


# =========================================================
# SPEED
# =========================================================

def get_speed_kmh(vehicle):

    velocity = (
        vehicle.get_velocity()
    )

    speed_ms = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )

    return (
        speed_ms
        * 3.6
    )


# =========================================================
# SET VEHICLE SPEED
# =========================================================

def set_vehicle_speed(
    vehicle,
    speed_kmh
):

    speed_ms = (
        speed_kmh
        / 3.6
    )

    transform = (
        vehicle.get_transform()
    )

    forward = (
        transform.get_forward_vector()
    )

    velocity = carla.Vector3D(
        x=forward.x * speed_ms,
        y=forward.y * speed_ms,
        z=forward.z * speed_ms
    )

    vehicle.set_target_velocity(
        velocity
    )


# =========================================================
# YAW DIFFERENCE
# =========================================================

def get_yaw_difference(
    yaw1,
    yaw2
):

    difference = abs(
        yaw1 - yaw2
    )

    if difference > 180.0:

        difference = (
            360.0
            - difference
        )

    return difference


# =========================================================
# INITIAL TEST DISTANCE
# =========================================================

def get_initial_center_distance(
    obstacle_speed_kmh
):

    # ---------------------------------------------
    # Suhteellinen nopeus
    # ---------------------------------------------

    relative_speed_kmh = (
        EGO_TARGET_SPEED_KMH
        - obstacle_speed_kmh
    )

    relative_speed_ms = (
        relative_speed_kmh
        / 3.6
    )

    # ---------------------------------------------
    # Arvio TTC-controllerin laukaisuetäisyydestä
    # ---------------------------------------------

    expected_trigger_gap = (
        relative_speed_ms
        * TTC_TRIGGER_S
    )

    # ---------------------------------------------
    # Testi aloitetaan hieman ennen
    # arvioitua TTC-laukaisupistettä.
    # ---------------------------------------------

    center_distance = max(
        MIN_CENTER_DISTANCE_M,
        (
            expected_trigger_gap
            + START_DISTANCE_BUFFER_M
        )
    )

    return center_distance


# =========================================================
# FIND STRAIGHT ROAD
# =========================================================

def find_test_locations(
    world,
    distance_m
):

    carla_map = (
        world.get_map()
    )

    spawn_points = (
        carla_map.get_spawn_points()
    )

    for spawn_transform in spawn_points:

        start_wp = (
            carla_map.get_waypoint(

                spawn_transform.location,

                project_to_road=True,

                lane_type=(
                    carla.LaneType.Driving
                )
            )
        )

        if start_wp is None:
            continue

        if start_wp.is_junction:
            continue

        start_transform = (
            start_wp.transform
        )

        forward = (
            start_transform
            .get_forward_vector()
        )

        valid = True
        target_wp = None

        # -------------------------------------------------
        # Tarkistetaan myös tietä esteen alkupisteen jälkeen.
        #
        # Tämä auttaa erityisesti 40 km/h moving-obstacle
        # -testissä, koska este jatkaa liikkumista eteenpäin.
        # -------------------------------------------------

        check_distances = [
            10.0,
            20.0,
            30.0,
            40.0,
            distance_m
        ]

        for check_distance in (
            check_distances
        ):

            expected_location = (
                carla.Location(

                    x=(
                        start_transform.location.x
                        + forward.x
                        * check_distance
                    ),

                    y=(
                        start_transform.location.y
                        + forward.y
                        * check_distance
                    ),

                    z=(
                        start_transform.location.z
                    )
                )
            )

            check_wp = (
                carla_map.get_waypoint(

                    expected_location,

                    project_to_road=True,

                    lane_type=(
                        carla.LaneType.Driving
                    )
                )
            )

            if check_wp is None:

                valid = False
                break

            if check_wp.is_junction:

                valid = False
                break

            projection_error = (
                check_wp
                .transform
                .location
                .distance(
                    expected_location
                )
            )

            if projection_error > 2.0:

                valid = False
                break

            yaw_error = (
                get_yaw_difference(

                    start_transform
                    .rotation
                    .yaw,

                    check_wp
                    .transform
                    .rotation
                    .yaw
                )
            )

            if yaw_error > 5.0:

                valid = False
                break

            # Target waypointiksi asetetaan vain
            # varsinaista haluttua distance_m-etäisyyttä
            # vastaava waypoint.
            if abs(
                check_distance
                - distance_m
            ) < 0.01:

                target_wp = (
                    check_wp
                )

        if not valid:
            continue

        # distance_m ei välttämättä ole listassa
        # täsmälleen liukulukutarkkuuden vuoksi.
        if target_wp is None:

            expected_target_location = (
                carla.Location(

                    x=(
                        start_transform.location.x
                        + forward.x
                        * distance_m
                    ),

                    y=(
                        start_transform.location.y
                        + forward.y
                        * distance_m
                    ),

                    z=(
                        start_transform.location.z
                    )
                )
            )

            target_wp = (
                carla_map.get_waypoint(

                    expected_target_location,

                    project_to_road=True,

                    lane_type=(
                        carla.LaneType.Driving
                    )
                )
            )

        if target_wp is None:
            continue

        actual_distance = (
            start_transform
            .location
            .distance(
                target_wp
                .transform
                .location
            )
        )

        if abs(
            actual_distance
            - distance_m
        ) > 3.0:

            continue

        # -------------------------------------------------
        # Ego transform
        # -------------------------------------------------

        ego_transform = (
            carla.Transform(

                carla.Location(

                    x=(
                        start_transform
                        .location
                        .x
                    ),

                    y=(
                        start_transform
                        .location
                        .y
                    ),

                    z=(
                        start_transform
                        .location
                        .z
                        + 0.3
                    )
                ),

                start_transform.rotation
            )
        )

        # -------------------------------------------------
        # Obstacle transform
        # -------------------------------------------------

        obstacle_transform = (
            carla.Transform(

                carla.Location(

                    x=(
                        target_wp
                        .transform
                        .location
                        .x
                    ),

                    y=(
                        target_wp
                        .transform
                        .location
                        .y
                    ),

                    z=(
                        target_wp
                        .transform
                        .location
                        .z
                        + 0.3
                    )
                ),

                target_wp
                .transform
                .rotation
            )
        )

        return (
            ego_transform,
            obstacle_transform
        )

    return None, None


# =========================================================
# BUMPER-TO-BUMPER GAP
# =========================================================

def get_obstacle_gap(
    ego_vehicle,
    obstacle_vehicle
):

    ego_transform = (
        ego_vehicle.get_transform()
    )

    ego_location = (
        ego_vehicle.get_location()
    )

    obstacle_location = (
        obstacle_vehicle.get_location()
    )

    forward = (
        ego_transform
        .get_forward_vector()
    )

    delta_x = (
        obstacle_location.x
        - ego_location.x
    )

    delta_y = (
        obstacle_location.y
        - ego_location.y
    )

    delta_z = (
        obstacle_location.z
        - ego_location.z
    )

    # Projektoidaan ajoneuvojen välinen
    # etäisyys ego-auton pitkittäissuuntaan.
    longitudinal_distance = (

        delta_x * forward.x
        + delta_y * forward.y
        + delta_z * forward.z
    )

    ego_half_length = (
        ego_vehicle
        .bounding_box
        .extent
        .x
    )

    obstacle_half_length = (
        obstacle_vehicle
        .bounding_box
        .extent
        .x
    )

    gap = (
        longitudinal_distance
        - ego_half_length
        - obstacle_half_length
    )

    return max(
        0.0,
        gap
    )


# =========================================================
# DRAIN OLD ZEROMQ COMMANDS
# =========================================================

def drain_control_socket(
    control_socket
):

    while True:

        try:

            control_socket.recv_string(
                flags=zmq.NOBLOCK
            )

        except zmq.Again:

            break


# =========================================================
# GET CONTROLLER COMMAND
# =========================================================

def get_control_command(
    sensor_socket,
    control_socket,
    poller,
    sensor_data
):

    expected_test = (
        sensor_data["test_id"]
    )

    expected_sequence = (
        sensor_data["sequence"]
    )

    start_time = (
        time.perf_counter()
    )

    # PUB/SUB voi hukata yksittäisen viestin,
    # joten lähetetään tarvittaessa uudelleen.
    for _ in range(3):

        sensor_socket.send_string(
            json.dumps(
                sensor_data
            )
        )

        events = dict(
            poller.poll(
                200
            )
        )

        if (
            control_socket
            not in events
        ):

            continue

        while True:

            try:

                message = (
                    control_socket
                    .recv_string(
                        flags=zmq.NOBLOCK
                    )
                )

                command = (
                    json.loads(
                        message
                    )
                )

                if (
                    command.get(
                        "test_id"
                    )
                    == expected_test
                    and
                    command.get(
                        "sequence"
                    )
                    == expected_sequence
                ):

                    round_trip_ms = (
                        time.perf_counter()
                        - start_time
                    ) * 1000.0

                    return (
                        command,
                        round_trip_ms
                    )

            except zmq.Again:

                break

    return None, None


# =========================================================
# SAVE CSV
# =========================================================

def save_result(result):

    # Varmistetaan, että projektin results-kansio
    # on olemassa.
    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    file_exists = (
        RESULTS_FILE.exists()
    )

    fieldnames = [

        "timestamp",

        "test_id",

        "ego_target_speed_kmh",

        "obstacle_target_speed_kmh",

        "ego_trigger_speed_kmh",

        "obstacle_trigger_speed_kmh",

        "relative_trigger_speed_kmh",

        "trigger_gap_m",

        "trigger_ttc_s",

        "ttc_threshold_s",

        "controller_processing_ms",

        "controller_round_trip_ms",

        "stopping_time_s",

        "ego_stopping_distance_m",

        "minimum_gap_m",

        "final_gap_m",

        "safety_margin_m",

        "collision",

        "result"
    ]

    with RESULTS_FILE.open(
        "a",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames
        )

        if not file_exists:

            writer.writeheader()

        writer.writerow(
            result
        )


# =========================================================
# SINGLE TEST
# =========================================================

def run_test(
    world,
    blueprint_library,
    ego_transform,
    obstacle_transform,
    obstacle_target_speed,
    sensor_socket,
    control_socket,
    poller
):

    ego_vehicle = None
    obstacle_vehicle = None
    collision_sensor = None

    collision_state = {
        "detected": False
    }

    test_id = (
        f"ego50_obs"
        f"{int(obstacle_target_speed)}"
    )

    sequence = 0

    simulation_time = 0.0

    brake_active = False

    trigger_time = None
    trigger_position = None

    ego_trigger_speed = None

    obstacle_trigger_speed = None

    relative_trigger_speed = None

    trigger_gap = None

    trigger_ttc = None

    ttc_threshold = None

    controller_processing_ms = None

    controller_round_trip_ms = None

    stopping_time = None

    stopping_distance = None

    minimum_gap = None

    final_gap = None

    print()
    print(
        "=" * 85
    )

    print(
        f"RELATIVE TTC TESTI | "
        f"Ego 50 km/h | "
        f"Obstacle "
        f"{obstacle_target_speed:.0f} km/h"
    )

    print(
        "=" * 85
    )

    try:

        # =================================================
        # VEHICLES
        # =================================================

        vehicle_bp = (
            blueprint_library.find(
                "vehicle.tesla.model3"
            )
        )

        ego_vehicle = (
            world.try_spawn_actor(
                vehicle_bp,
                ego_transform
            )
        )

        obstacle_vehicle = (
            world.try_spawn_actor(
                vehicle_bp,
                obstacle_transform
            )
        )

        if ego_vehicle is None:

            raise RuntimeError(
                "Ego-ajoneuvon "
                "luonti epäonnistui."
            )

        if obstacle_vehicle is None:

            raise RuntimeError(
                "Esteajoneuvon "
                "luonti epäonnistui."
            )

        # =================================================
        # COLLISION SENSOR
        # =================================================

        collision_bp = (
            blueprint_library.find(
                "sensor.other.collision"
            )
        )

        collision_sensor = (
            world.spawn_actor(
                collision_bp,
                carla.Transform(),
                attach_to=ego_vehicle
            )
        )

        def collision_callback(event):

            collision_state[
                "detected"
            ] = True

        collision_sensor.listen(
            collision_callback
        )

        # =================================================
        # ACTOR INITIALIZATION
        # =================================================

        for _ in range(5):

            world.tick()

        initial_gap = (
            get_obstacle_gap(
                ego_vehicle,
                obstacle_vehicle
            )
        )

        minimum_gap = (
            initial_gap
        )

        print(
            f"Initial gap: "
            f"{initial_gap:.2f} m"
        )

        print(
            f"Ego target: "
            f"{EGO_TARGET_SPEED_KMH:.1f} km/h"
        )

        print(
            f"Obstacle target: "
            f"{obstacle_target_speed:.1f} km/h"
        )

        print(
            f"Initial relative speed: "
            f"{EGO_TARGET_SPEED_KMH - obstacle_target_speed:.1f} km/h"
        )

        print()

        drain_control_socket(
            control_socket
        )

        vehicle_has_moved = False

        # =================================================
        # TEST LOOP
        # =================================================

        while (
            simulation_time
            < TEST_TIMEOUT_SECONDS
        ):

            # =============================================
            # MOVING OBSTACLE
            # =============================================

            set_vehicle_speed(
                obstacle_vehicle,
                obstacle_target_speed
            )

            obstacle_vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    brake=0.0,
                    steer=0.0
                )
            )

            # =============================================
            # EGO VEHICLE
            # =============================================

            if not brake_active:

                set_vehicle_speed(
                    ego_vehicle,
                    EGO_TARGET_SPEED_KMH
                )

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=0.0,
                        steer=0.0
                    )
                )

            else:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

            # =============================================
            # SIMULATION STEP
            # =============================================

            world.tick()

            simulation_time += (
                FIXED_DELTA_SECONDS
            )

            time.sleep(
                0.001
            )

            # =============================================
            # MEASUREMENTS
            # =============================================

            ego_speed = (
                get_speed_kmh(
                    ego_vehicle
                )
            )

            obstacle_speed = (
                get_speed_kmh(
                    obstacle_vehicle
                )
            )

            gap = (
                get_obstacle_gap(
                    ego_vehicle,
                    obstacle_vehicle
                )
            )

            if (
                minimum_gap is None
                or gap < minimum_gap
            ):

                minimum_gap = gap

            if ego_speed > 1.0:

                vehicle_has_moved = True

            # =============================================
            # SENSOR DATA -> CONTROLLER
            # =============================================

            sequence += 1

            sensor_data = {

                "test_id":
                    test_id,

                "sequence":
                    sequence,

                "ego_speed_kmh":
                    ego_speed,

                "obstacle_speed_kmh":
                    obstacle_speed,

                "obstacle_distance_m":
                    gap
            }

            (
                command,
                round_trip_ms
            ) = get_control_command(

                sensor_socket,

                control_socket,

                poller,

                sensor_data
            )

            if command is None:

                raise RuntimeError(
                    "Controllerilta "
                    "ei saatu vastausta."
                )

            controller_brake = float(
                command.get(
                    "brake",
                    0.0
                )
            )

            controller_aeb = bool(
                command.get(
                    "aeb_active",
                    False
                )
            )

            current_ttc = (
                command.get(
                    "ttc_s"
                )
            )

            # =============================================
            # FIRST AEB TRIGGER
            # =============================================

            if (
                controller_aeb
                and not brake_active
            ):

                brake_active = True

                trigger_time = (
                    simulation_time
                )

                trigger_position = (
                    ego_vehicle.get_location()
                )

                ego_trigger_speed = (
                    ego_speed
                )

                obstacle_trigger_speed = (
                    obstacle_speed
                )

                relative_trigger_speed = (
                    command.get(
                        "relative_speed_kmh"
                    )
                )

                trigger_gap = (
                    gap
                )

                trigger_ttc = (
                    current_ttc
                )

                ttc_threshold = (
                    command.get(
                        "ttc_threshold_s"
                    )
                )

                controller_processing_ms = (
                    command.get(
                        "controller_processing_ms"
                    )
                )

                controller_round_trip_ms = (
                    round_trip_ms
                )

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=controller_brake,
                        steer=0.0
                    )
                )

                print()
                print(
                    "*** RELATIVE TTC "
                    "AEB TRIGGER ***"
                )

                print(
                    f"Ego speed: "
                    f"{ego_trigger_speed:.2f} km/h"
                )

                print(
                    f"Obstacle speed: "
                    f"{obstacle_trigger_speed:.2f} km/h"
                )

                print(
                    f"Relative speed: "
                    f"{relative_trigger_speed:.2f} km/h"
                )

                print(
                    f"Trigger gap: "
                    f"{trigger_gap:.2f} m"
                )

                print(
                    f"Trigger TTC: "
                    f"{trigger_ttc:.3f} s"
                )

                print(
                    f"TTC threshold: "
                    f"{ttc_threshold:.3f} s"
                )

                print(
                    f"Controller processing: "
                    f"{controller_processing_ms:.3f} ms"
                )

                print(
                    f"Controller round trip: "
                    f"{controller_round_trip_ms:.3f} ms"
                )

                print()

            # =============================================
            # TERMINAL OUTPUT
            # =============================================

            if current_ttc is None:

                ttc_text = "---"

            else:

                ttc_text = (
                    f"{current_ttc:.2f}"
                )

            state = (
                "AEB ACTIVE"
                if brake_active
                else "FOLLOWING"
            )

            print(
                f"{state:10} | "
                f"Ego: "
                f"{ego_speed:5.1f} | "
                f"Obs: "
                f"{obstacle_speed:5.1f} | "
                f"Gap: "
                f"{gap:6.2f} | "
                f"TTC: "
                f"{ttc_text:>5}",
                end="\r"
            )

            # =============================================
            # EGO STOPPED
            # =============================================

            if (
                vehicle_has_moved
                and brake_active
                and ego_speed
                <= STOP_SPEED_KMH
            ):

                stopping_time = (
                    simulation_time
                    - trigger_time
                )

                stop_position = (
                    ego_vehicle.get_location()
                )

                stopping_distance = (
                    trigger_position.distance(
                        stop_position
                    )
                )

                final_gap = (
                    get_obstacle_gap(
                        ego_vehicle,
                        obstacle_vehicle
                    )
                )

                # Annetaan collision callbackille
                # pari simulaatioaskelta.
                for _ in range(2):

                    world.tick()

                    time.sleep(
                        0.001
                    )

                break

        # =================================================
        # RESULT
        # =================================================

        safe_margin_ok = (
            minimum_gap is not None
            and minimum_gap
            >= SAFETY_MARGIN_M
        )

        if (
            brake_active
            and not collision_state[
                "detected"
            ]
            and safe_margin_ok
        ):

            result_string = (
                "PASS"
            )

        else:

            result_string = (
                "FAIL"
            )

        print()
        print()

        print(
            f"Minimum gap: "
            f"{minimum_gap:.2f} m"
        )

        if final_gap is not None:

            print(
                f"Final gap: "
                f"{final_gap:.2f} m"
            )

        print(
            f"Collision: "
            f"{'KYLLÄ' if collision_state['detected'] else 'EI'}"
        )

        print(
            f"Result: "
            f"{result_string}"
        )

        # =================================================
        # RESULT DICTIONARY
        # =================================================

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "test_id":
                test_id,

            "ego_target_speed_kmh":
                EGO_TARGET_SPEED_KMH,

            "obstacle_target_speed_kmh":
                obstacle_target_speed,

            "ego_trigger_speed_kmh":
                round(
                    ego_trigger_speed,
                    3
                )
                if ego_trigger_speed
                is not None
                else "",

            "obstacle_trigger_speed_kmh":
                round(
                    obstacle_trigger_speed,
                    3
                )
                if obstacle_trigger_speed
                is not None
                else "",

            "relative_trigger_speed_kmh":
                round(
                    relative_trigger_speed,
                    3
                )
                if relative_trigger_speed
                is not None
                else "",

            "trigger_gap_m":
                round(
                    trigger_gap,
                    3
                )
                if trigger_gap
                is not None
                else "",

            "trigger_ttc_s":
                round(
                    trigger_ttc,
                    4
                )
                if trigger_ttc
                is not None
                else "",

            "ttc_threshold_s":
                round(
                    ttc_threshold,
                    3
                )
                if ttc_threshold
                is not None
                else "",

            "controller_processing_ms":
                round(
                    controller_processing_ms,
                    3
                )
                if controller_processing_ms
                is not None
                else "",

            "controller_round_trip_ms":
                round(
                    controller_round_trip_ms,
                    3
                )
                if controller_round_trip_ms
                is not None
                else "",

            "stopping_time_s":
                round(
                    stopping_time,
                    3
                )
                if stopping_time
                is not None
                else "",

            "ego_stopping_distance_m":
                round(
                    stopping_distance,
                    3
                )
                if stopping_distance
                is not None
                else "",

            "minimum_gap_m":
                round(
                    minimum_gap,
                    3
                ),

            "final_gap_m":
                round(
                    final_gap,
                    3
                )
                if final_gap
                is not None
                else "",

            "safety_margin_m":
                SAFETY_MARGIN_M,

            "collision":
                collision_state[
                    "detected"
                ],

            "result":
                result_string
        }

        return result

    finally:

        # =================================================
        # CLEANUP
        # =================================================

        if collision_sensor is not None:

            collision_sensor.stop()
            collision_sensor.destroy()

        if ego_vehicle is not None:

            ego_vehicle.destroy()

        if obstacle_vehicle is not None:

            obstacle_vehicle.destroy()

        for _ in range(3):

            world.tick()


# =========================================================
# MAIN
# =========================================================

def main():

    world = None
    original_settings = None

    context = None

    sensor_socket = None
    control_socket = None

    results = []

    try:

        # =================================================
        # ZEROMQ
        # =================================================

        context = (
            zmq.Context()
        )

        sensor_socket = (
            context.socket(
                zmq.PUB
            )
        )

        sensor_socket.bind(
            SENSOR_ADDRESS
        )

        control_socket = (
            context.socket(
                zmq.SUB
            )
        )

        control_socket.connect(
            CONTROL_ADDRESS
        )

        control_socket.setsockopt_string(
            zmq.SUBSCRIBE,
            ""
        )

        poller = (
            zmq.Poller()
        )

        poller.register(
            control_socket,
            zmq.POLLIN
        )

        # =================================================
        # CARLA
        # =================================================

        print(
            "Yhdistetään CARLAan..."
        )

        client = carla.Client(
            "localhost",
            2000
        )

        client.set_timeout(
            10.0
        )

        world = (
            client.get_world()
        )

        print(
            "Yhteys CARLAan onnistui."
        )

        print(
            f"Kartta: "
            f"{world.get_map().name}"
        )

        # =================================================
        # SYNCHRONOUS MODE
        # =================================================

        original_settings = (
            world.get_settings()
        )

        settings = (
            world.get_settings()
        )

        settings.synchronous_mode = (
            True
        )

        settings.fixed_delta_seconds = (
            FIXED_DELTA_SECONDS
        )

        world.apply_settings(
            settings
        )

        world.set_weather(
            carla.WeatherParameters.ClearNoon
        )

        print(
            f"Synchronous mode: "
            f"{1 / FIXED_DELTA_SECONDS:.1f} Hz"
        )

        print(
            "Odotetaan controller-yhteyttä..."
        )

        time.sleep(
            1.0
        )

        blueprint_library = (
            world.get_blueprint_library()
        )

        # =================================================
        # TEST SERIES
        # =================================================

        for obstacle_speed in (
            OBSTACLE_SPEEDS_KMH
        ):

            # ---------------------------------------------
            # Jokaiselle obstacle-nopeudelle
            # lasketaan sopiva lähtöetäisyys.
            # ---------------------------------------------

            center_distance = (
                get_initial_center_distance(
                    obstacle_speed
                )
            )

            print()
            print(
                f"Haetaan testipaikkaa | "
                f"Obstacle "
                f"{obstacle_speed:.0f} km/h | "
                f"Start center distance "
                f"{center_distance:.2f} m"
            )

            (
                ego_transform,
                obstacle_transform
            ) = find_test_locations(
                world,
                center_distance
            )

            if ego_transform is None:

                print(
                    "Sopivaa testipaikkaa "
                    "ei löytynyt."
                )

                continue

            # ---------------------------------------------
            # Testi
            # ---------------------------------------------

            result = run_test(
                world,
                blueprint_library,
                ego_transform,
                obstacle_transform,
                obstacle_speed,
                sensor_socket,
                control_socket,
                poller
            )

            results.append(
                result
            )

            save_result(
                result
            )

        # =================================================
        # SUMMARY
        # =================================================

        print()
        print()

        print(
            "=" * 110
        )

        print(
            "RELATIVE-SPEED TTC "
            "AEB TESTISARJA"
        )

        print(
            "=" * 110
        )

        print(
            "Ego | Obstacle | Relative | "
            "Trigger gap | TTC | "
            "Min gap | Result"
        )

        print(
            "-" * 110
        )

        for result in results:

            print(
                f"{result['ego_target_speed_kmh']:4.0f} | "
                f"{result['obstacle_target_speed_kmh']:8.0f} | "
                f"{str(result['relative_trigger_speed_kmh']):>8} | "
                f"{str(result['trigger_gap_m']):>11} | "
                f"{str(result['trigger_ttc_s']):>5} | "
                f"{str(result['minimum_gap_m']):>7} | "
                f"{result['result']}"
            )

        print(
            "=" * 110
        )

        print()
        print(
            "Tulokset tallennettu:"
        )

        print(
            RESULTS_FILE
        )

    except KeyboardInterrupt:

        print()
        print(
            "Testisarja keskeytetty."
        )

    except Exception as error:

        print()
        print(
            f"Virhe: "
            f"{error}"
        )

    finally:

        # =================================================
        # RESTORE CARLA SETTINGS
        # =================================================

        if (
            world is not None
            and original_settings is not None
        ):

            print()
            print(
                "Palautetaan CARLAn asetukset..."
            )

            world.apply_settings(
                original_settings
            )

        # =================================================
        # ZEROMQ CLEANUP
        # =================================================

        if sensor_socket is not None:

            sensor_socket.close()

        if control_socket is not None:

            control_socket.close()

        if context is not None:

            context.term()

        print(
            "Valmis."
        )


if __name__ == "__main__":
    main()