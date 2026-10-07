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

FIXED_DELTA_SECONDS = 0.05

ROUTE_LENGTH_M = 160.0
ROUTE_STEP_M = 2.0

LOOKAHEAD_POINTS = 4

# Vehicle A lähtee reitin alusta.
VEHICLE_A_SPEED_KMH = 30.0

# Vehicle B lähtee A:n edeltä ja ajaa hitaammin.
VEHICLE_B_SPEED_KMH = 20.0

VEHICLE_B_START_DISTANCE_M = 36.0

# Reitin loppuosa toimii yhteisenä loading zonena.
LOADING_ZONE_LENGTH_M = 25.0

# B:n tavoite on reitin lopussa.
# A:n tavoite on hieman ennen B:tä,
# jotta molemmat mahtuvat loading zonelle.
VEHICLE_A_GOAL_OFFSET_FROM_B_M = 18.0

GOAL_TOLERANCE_M = 4.0

# A:n safety interrupt voidaan vapauttaa,
# kun A on pysähtynyt ja reittipohjainen
# bumper-to-bumper gap on jälleen riittävä.
SAFETY_RELEASE_GAP_M = 10.0

SAFETY_MARGIN_M = 1.0

STOP_SPEED_KMH = 0.5

TEST_TIMEOUT_SECONDS = 60.0


# =========================================================
# RESULTS
# =========================================================

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
    / "two_vehicle_coordination_results.csv"
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
# GENERAL UTILITIES
# =========================================================

def clamp(
    value,
    minimum,
    maximum
):

    return max(
        minimum,
        min(
            maximum,
            value
        )
    )


def normalize_angle(
    angle_deg
):

    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


def get_speed_kmh(
    vehicle
):

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
# ROUTE GENERATION
# =========================================================

def yaw_difference(
    yaw1,
    yaw2
):

    difference = (
        yaw2
        - yaw1
    )

    return abs(
        normalize_angle(
            difference
        )
    )


def build_route(
    start_waypoint,
    route_length_m,
    step_m
):

    route = [
        start_waypoint
    ]

    current_waypoint = (
        start_waypoint
    )

    travelled_distance = 0.0

    while (
        travelled_distance
        < route_length_m
    ):

        next_waypoints = (
            current_waypoint.next(
                step_m
            )
        )

        if not next_waypoints:
            break

        current_yaw = (
            current_waypoint
            .transform
            .rotation
            .yaw
        )

        # Jos risteyksessä on useita vaihtoehtoja,
        # valitaan suorin jatkosuunta.
        next_waypoint = min(

            next_waypoints,

            key=lambda waypoint:
                yaw_difference(

                    current_yaw,

                    waypoint
                    .transform
                    .rotation
                    .yaw
                )
        )

        route.append(
            next_waypoint
        )

        current_waypoint = (
            next_waypoint
        )

        travelled_distance += (
            step_m
        )

    return route


def make_spawn_transform(
    waypoint
):

    transform = (
        waypoint.transform
    )

    return carla.Transform(

        carla.Location(

            x=(
                transform.location.x
            ),

            y=(
                transform.location.y
            ),

            z=(
                transform.location.z
                + 0.3
            )
        ),

        transform.rotation
    )


def find_route(
    world
):

    carla_map = (
        world.get_map()
    )

    spawn_points = (
        carla_map.get_spawn_points()
    )

    required_points = int(
        ROUTE_LENGTH_M
        / ROUTE_STEP_M
    )

    for spawn_transform in (
        spawn_points
    ):

        start_waypoint = (
            carla_map.get_waypoint(

                spawn_transform.location,

                project_to_road=True,

                lane_type=(
                    carla.LaneType.Driving
                )
            )
        )

        if start_waypoint is None:
            continue

        route = build_route(
            start_waypoint,
            ROUTE_LENGTH_M,
            ROUTE_STEP_M
        )

        if (
            len(route)
            >= required_points
        ):

            return route

    return None


# =========================================================
# DEBUG DRAWING
# =========================================================

def draw_route_and_zone(
    world,
    route,
    a_start_index,
    b_start_index,
    a_goal_index,
    b_goal_index
):

    # -----------------------------------------------------
    # Route
    # -----------------------------------------------------

    for index in range(
        len(route) - 1
    ):

        start = (
            route[index]
            .transform
            .location
        )

        end = (
            route[index + 1]
            .transform
            .location
        )

        world.debug.draw_line(

            carla.Location(
                x=start.x,
                y=start.y,
                z=start.z + 0.5
            ),

            carla.Location(
                x=end.x,
                y=end.y,
                z=end.z + 0.5
            ),

            thickness=0.07,

            color=carla.Color(
                0,
                100,
                255
            ),

            life_time=90.0
        )

    # -----------------------------------------------------
    # Start A
    # -----------------------------------------------------

    a_start = (
        route[a_start_index]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=a_start.x,
            y=a_start.y,
            z=a_start.z + 1.0
        ),

        size=0.35,

        color=carla.Color(
            0,
            255,
            255
        ),

        life_time=90.0
    )

    # -----------------------------------------------------
    # Start B
    # -----------------------------------------------------

    b_start = (
        route[b_start_index]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=b_start.x,
            y=b_start.y,
            z=b_start.z + 1.0
        ),

        size=0.35,

        color=carla.Color(
            255,
            150,
            0
        ),

        life_time=90.0
    )

    # -----------------------------------------------------
    # Loading zone
    # -----------------------------------------------------

    zone_points = int(
        LOADING_ZONE_LENGTH_M
        / ROUTE_STEP_M
    )

    zone_start_index = max(
        0,
        len(route)
        - zone_points
        - 1
    )

    for index in range(
        zone_start_index,
        len(route) - 1
    ):

        start = (
            route[index]
            .transform
            .location
        )

        end = (
            route[index + 1]
            .transform
            .location
        )

        world.debug.draw_line(

            carla.Location(
                x=start.x,
                y=start.y,
                z=start.z + 0.7
            ),

            carla.Location(
                x=end.x,
                y=end.y,
                z=end.z + 0.7
            ),

            thickness=0.15,

            color=carla.Color(
                255,
                0,
                255
            ),

            life_time=90.0
        )

    # -----------------------------------------------------
    # Vehicle A goal
    # -----------------------------------------------------

    a_goal = (
        route[a_goal_index]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=a_goal.x,
            y=a_goal.y,
            z=a_goal.z + 1.0
        ),

        size=0.4,

        color=carla.Color(
            0,
            255,
            255
        ),

        life_time=90.0
    )

    # -----------------------------------------------------
    # Vehicle B goal
    # -----------------------------------------------------

    b_goal = (
        route[b_goal_index]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=b_goal.x,
            y=b_goal.y,
            z=b_goal.z + 1.0
        ),

        size=0.4,

        color=carla.Color(
            0,
            255,
            0
        ),

        life_time=90.0
    )


# =========================================================
# ROUTE TRACKING
# =========================================================

def update_route_index(
    vehicle,
    route,
    current_index
):

    vehicle_location = (
        vehicle.get_location()
    )

    search_end = min(
        current_index + 15,
        len(route)
    )

    best_index = (
        current_index
    )

    best_distance = float(
        "inf"
    )

    for index in range(
        current_index,
        search_end
    ):

        waypoint_location = (
            route[index]
            .transform
            .location
        )

        distance = (
            vehicle_location.distance(
                waypoint_location
            )
        )

        if (
            distance
            < best_distance
        ):

            best_distance = (
                distance
            )

            best_index = (
                index
            )

    return best_index


def calculate_steering(
    vehicle,
    route,
    route_index,
    goal_index
):

    target_index = min(
        route_index
        + LOOKAHEAD_POINTS,
        goal_index
    )

    target_location = (
        route[target_index]
        .transform
        .location
    )

    transform = (
        vehicle.get_transform()
    )

    vehicle_location = (
        transform.location
    )

    dx = (
        target_location.x
        - vehicle_location.x
    )

    dy = (
        target_location.y
        - vehicle_location.y
    )

    target_yaw = math.degrees(
        math.atan2(
            dy,
            dx
        )
    )

    vehicle_yaw = (
        transform.rotation.yaw
    )

    heading_error = (
        normalize_angle(
            target_yaw
            - vehicle_yaw
        )
    )

    steer = (
        heading_error
        / 35.0
    )

    return clamp(
        steer,
        -1.0,
        1.0
    )


# =========================================================
# GOAL SPEED CONTROL
# =========================================================

def get_goal_distance(
    vehicle,
    route,
    goal_index
):

    goal_location = (
        route[goal_index]
        .transform
        .location
    )

    return (
        vehicle
        .get_location()
        .distance(
            goal_location
        )
    )


def get_target_speed_near_goal(
    cruise_speed_kmh,
    goal_distance_m
):

    if (
        goal_distance_m
        > 20.0
    ):

        return (
            cruise_speed_kmh
        )

    if (
        goal_distance_m
        > GOAL_TOLERANCE_M
    ):

        factor = (
            goal_distance_m
            / 20.0
        )

        factor = clamp(
            factor,
            0.25,
            1.0
        )

        return (
            cruise_speed_kmh
            * factor
        )

    return 0.0


def calculate_drive_control(
    speed_kmh,
    target_speed_kmh,
    steer
):

    speed_error = (
        target_speed_kmh
        - speed_kmh
    )

    throttle = 0.0
    brake = 0.0

    if (
        speed_error
        > 0.0
    ):

        throttle = clamp(
            speed_error
            / 15.0,
            0.0,
            0.55
        )

    else:

        brake = clamp(
            (-speed_error)
            / 10.0,
            0.0,
            0.6
        )

    return carla.VehicleControl(
        throttle=throttle,
        brake=brake,
        steer=steer
    )


# =========================================================
# ROUTE-BASED INTER-VEHICLE GAP
# =========================================================

def get_route_gap(
    vehicle_a,
    vehicle_b,
    route_index_a,
    route_index_b
):

    # Tässä testissä molemmat ajavat täsmälleen
    # samaa waypoint-reittiä.
    #
    # Sen vuoksi etäisyys lasketaan reitin etenemästä
    # eikä A:n paikallisesta XYZ-koordinaatistosta.
    #
    # Tämä toimii myös silloin, kun tie kaartuu.

    index_difference = (
        route_index_b
        - route_index_a
    )

    if (
        index_difference
        <= 0
    ):

        return 0.0

    center_distance = (
        index_difference
        * ROUTE_STEP_M
    )

    half_length_a = (
        vehicle_a
        .bounding_box
        .extent
        .x
    )

    half_length_b = (
        vehicle_b
        .bounding_box
        .extent
        .x
    )

    bumper_gap = (
        center_distance
        - half_length_a
        - half_length_b
    )

    return max(
        0.0,
        bumper_gap
    )


# =========================================================
# ZEROMQ
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


def get_control_command(
    sensor_socket,
    control_socket,
    poller,
    sensor_data
):

    expected_test = (
        sensor_data[
            "test_id"
        ]
    )

    expected_sequence = (
        sensor_data[
            "sequence"
        ]
    )

    start_time = (
        time.perf_counter()
    )

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
# RESULTS
# =========================================================

def save_result(
    result
):

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    file_exists = (
        RESULTS_FILE.exists()
    )

    fieldnames = [

        "timestamp",

        "route_length_m",

        "vehicle_a_speed_kmh",

        "vehicle_b_speed_kmh",

        "vehicle_b_start_distance_m",

        "safety_interrupt_count",

        "first_trigger_speed_a_kmh",

        "first_trigger_speed_b_kmh",

        "first_trigger_relative_speed_kmh",

        "first_trigger_gap_m",

        "first_trigger_ttc_s",

        "minimum_gap_m",

        "vehicle_a_goal_reached",

        "vehicle_b_goal_reached",

        "vehicle_a_final_goal_distance_m",

        "vehicle_b_final_goal_distance_m",

        "collision_a",

        "collision_b",

        "controller_processing_ms",

        "controller_round_trip_ms",

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

        if (
            not file_exists
        ):

            writer.writeheader()

        writer.writerow(
            result
        )


# =========================================================
# MAIN
# =========================================================

def main():

    world = None
    original_settings = None

    vehicle_a = None
    vehicle_b = None

    collision_sensor_a = None
    collision_sensor_b = None

    context = None
    sensor_socket = None
    control_socket = None


    collision_state = {

        "A": False,

        "B": False
    }


    test_id = (
        "two_vehicle_coordination"
    )


    sequence = 0

    simulation_time = 0.0


    route_index_a = 0

    route_index_b = 0


    vehicle_a_goal_reached = False

    vehicle_b_goal_reached = False


    safety_latched = False

    safety_interrupt_count = 0


    minimum_gap = None


    first_trigger_speed_a = None

    first_trigger_speed_b = None

    first_trigger_relative_speed = None

    first_trigger_gap = None

    first_trigger_ttc = None

    first_controller_processing_ms = None

    first_controller_round_trip_ms = None


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

        settings.synchronous_mode = True

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


        # =================================================
        # ROUTE
        # =================================================

        print()

        print(
            "Etsitään yhteistä reittiä..."
        )


        route = (
            find_route(
                world
            )
        )


        if (
            route
            is None
        ):

            raise RuntimeError(
                "Sopivaa reittiä ei löytynyt."
            )


        route_length = (
            (len(route) - 1)
            * ROUTE_STEP_M
        )


        # -------------------------------------------------
        # START POINTS
        # -------------------------------------------------

        a_start_index = 0


        b_start_index = int(
            VEHICLE_B_START_DISTANCE_M
            / ROUTE_STEP_M
        )

        b_start_index = min(
            b_start_index,
            len(route) - 20
        )


        # -------------------------------------------------
        # COMMON LOADING ZONE
        # -------------------------------------------------

        b_goal_index = (
            len(route) - 1
        )


        a_goal_offset_points = int(
            VEHICLE_A_GOAL_OFFSET_FROM_B_M
            / ROUTE_STEP_M
        )


        a_goal_index = max(
            0,
            b_goal_index
            - a_goal_offset_points
        )


        route_index_a = (
            a_start_index
        )

        route_index_b = (
            b_start_index
        )


        print(
            f"Reitti löytyi: "
            f"{route_length:.1f} m"
        )

        print(
            f"Route points: "
            f"{len(route)}"
        )

        print(
            f"Vehicle A start: "
            f"{a_start_index * ROUTE_STEP_M:.1f} m"
        )

        print(
            f"Vehicle B start: "
            f"{b_start_index * ROUTE_STEP_M:.1f} m"
        )

        print(
            f"Vehicle A goal: "
            f"{a_goal_index * ROUTE_STEP_M:.1f} m"
        )

        print(
            f"Vehicle B goal: "
            f"{b_goal_index * ROUTE_STEP_M:.1f} m"
        )


        draw_route_and_zone(

            world,

            route,

            a_start_index,

            b_start_index,

            a_goal_index,

            b_goal_index
        )


        # =================================================
        # VEHICLES
        # =================================================

        blueprint_library = (
            world.get_blueprint_library()
        )


        vehicle_bp_a = (
            blueprint_library.find(
                "vehicle.tesla.model3"
            )
        )


        vehicle_bp_b = (
            blueprint_library.find(
                "vehicle.tesla.model3"
            )
        )


        if (
            vehicle_bp_a.has_attribute(
                "color"
            )
        ):

            vehicle_bp_a.set_attribute(
                "color",
                "0,0,255"
            )


        if (
            vehicle_bp_b.has_attribute(
                "color"
            )
        ):

            vehicle_bp_b.set_attribute(
                "color",
                "255,0,0"
            )


        transform_a = (
            make_spawn_transform(
                route[
                    a_start_index
                ]
            )
        )


        transform_b = (
            make_spawn_transform(
                route[
                    b_start_index
                ]
            )
        )


        vehicle_a = (
            world.try_spawn_actor(
                vehicle_bp_a,
                transform_a
            )
        )


        if (
            vehicle_a
            is None
        ):

            raise RuntimeError(
                "Vehicle A:n luonti epäonnistui."
            )


        vehicle_b = (
            world.try_spawn_actor(
                vehicle_bp_b,
                transform_b
            )
        )


        if (
            vehicle_b
            is None
        ):

            raise RuntimeError(
                "Vehicle B:n luonti epäonnistui."
            )


        # =================================================
        # COLLISION SENSORS
        # =================================================

        collision_bp = (
            blueprint_library.find(
                "sensor.other.collision"
            )
        )


        collision_sensor_a = (
            world.spawn_actor(
                collision_bp,
                carla.Transform(),
                attach_to=vehicle_a
            )
        )


        collision_sensor_b = (
            world.spawn_actor(
                collision_bp,
                carla.Transform(),
                attach_to=vehicle_b
            )
        )


        def collision_callback_a(
            event
        ):

            collision_state[
                "A"
            ] = True


        def collision_callback_b(
            event
        ):

            collision_state[
                "B"
            ] = True


        collision_sensor_a.listen(
            collision_callback_a
        )


        collision_sensor_b.listen(
            collision_callback_b
        )


        # =================================================
        # INITIALIZATION
        # =================================================

        for _ in range(5):

            world.tick()


        time.sleep(
            1.0
        )


        drain_control_socket(
            control_socket
        )


        print()

        print(
            "=" * 90
        )

        print(
            "TWO-VEHICLE COORDINATION + TTC SAFETY TEST"
        )

        print(
            "=" * 90
        )


        print(
            f"Vehicle A cruise speed: "
            f"{VEHICLE_A_SPEED_KMH:.1f} km/h"
        )

        print(
            f"Vehicle B cruise speed: "
            f"{VEHICLE_B_SPEED_KMH:.1f} km/h"
        )

        print()

        print(
            "A ja B lähtevät eri pisteistä."
        )

        print(
            "Molemmat ajavat samaan loading zoneen."
        )

        print(
            "Vehicle A on nopeampi ja saavuttaa B:tä."
        )

        print(
            "Relative-TTC toimii A:n safety layerina."
        )

        print(
            "Ajoneuvojen väli lasketaan "
            "reitin etenemän perusteella."
        )

        print()


        # =================================================
        # TEST LOOP
        # =================================================

        while (
            simulation_time
            < TEST_TIMEOUT_SECONDS
        ):

            # -------------------------------------------------
            # SIMULATION STEP
            # -------------------------------------------------

            world.tick()

            simulation_time += (
                FIXED_DELTA_SECONDS
            )

            time.sleep(
                0.001
            )


            # =================================================
            # VEHICLE STATES
            # =================================================

            speed_a = (
                get_speed_kmh(
                    vehicle_a
                )
            )


            speed_b = (
                get_speed_kmh(
                    vehicle_b
                )
            )


            route_index_a = (
                update_route_index(
                    vehicle_a,
                    route,
                    route_index_a
                )
            )


            route_index_b = (
                update_route_index(
                    vehicle_b,
                    route,
                    route_index_b
                )
            )


            goal_distance_a = (
                get_goal_distance(
                    vehicle_a,
                    route,
                    a_goal_index
                )
            )


            goal_distance_b = (
                get_goal_distance(
                    vehicle_b,
                    route,
                    b_goal_index
                )
            )


            # =================================================
            # GOAL DETECTION
            # =================================================

            if (
                not vehicle_a_goal_reached

                and

                goal_distance_a
                <= GOAL_TOLERANCE_M

                and

                route_index_a
                >= a_goal_index - 4
            ):

                vehicle_a_goal_reached = True

                safety_latched = False


                print()
                print()

                print(
                    "*** VEHICLE A REACHED "
                    "LOADING ZONE ***"
                )

                print(
                    f"Goal distance A: "
                    f"{goal_distance_a:.2f} m"
                )


            if (
                not vehicle_b_goal_reached

                and

                goal_distance_b
                <= GOAL_TOLERANCE_M

                and

                route_index_b
                >= b_goal_index - 4
            ):

                vehicle_b_goal_reached = True


                print()
                print()

                print(
                    "*** VEHICLE B REACHED "
                    "LOADING ZONE ***"
                )

                print(
                    f"Goal distance B: "
                    f"{goal_distance_b:.2f} m"
                )


            # =================================================
            # ROUTE-BASED VEHICLE GAP
            # =================================================

            gap = (
                get_route_gap(

                    vehicle_a,

                    vehicle_b,

                    route_index_a,

                    route_index_b
                )
            )


            if (
                minimum_gap
                is None

                or

                gap
                < minimum_gap
            ):

                minimum_gap = (
                    gap
                )


            # =================================================
            # TTC SAFETY CONTROLLER FOR VEHICLE A
            # =================================================

            sequence += 1


            sensor_data = {

                "test_id":
                    test_id,

                "sequence":
                    sequence,

                "ego_speed_kmh":
                    speed_a,

                "obstacle_speed_kmh":
                    speed_b,

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


            if (
                command
                is None
            ):

                raise RuntimeError(
                    "TTC-controllerilta "
                    "ei saatu vastausta."
                )


            aeb_active = bool(
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


            relative_speed = (
                command.get(
                    "relative_speed_kmh"
                )
            )


            # =================================================
            # NEW SAFETY INTERRUPT
            # =================================================

            if (
                aeb_active

                and not safety_latched

                and not vehicle_a_goal_reached
            ):

                safety_latched = True

                safety_interrupt_count += 1


                print()
                print()

                print(
                    "*** VEHICLE A SAFETY INTERRUPT ***"
                )

                print(
                    f"Event: "
                    f"{safety_interrupt_count}"
                )

                print(
                    f"Vehicle A speed: "
                    f"{speed_a:.2f} km/h"
                )

                print(
                    f"Vehicle B speed: "
                    f"{speed_b:.2f} km/h"
                )

                print(
                    f"Relative speed: "
                    f"{relative_speed:.2f} km/h"
                )

                print(
                    f"Route gap: "
                    f"{gap:.2f} m"
                )

                print(
                    f"TTC: "
                    f"{current_ttc:.3f} s"
                )


                if (
                    safety_interrupt_count
                    == 1
                ):

                    first_trigger_speed_a = (
                        speed_a
                    )

                    first_trigger_speed_b = (
                        speed_b
                    )

                    first_trigger_relative_speed = (
                        relative_speed
                    )

                    first_trigger_gap = (
                        gap
                    )

                    first_trigger_ttc = (
                        current_ttc
                    )

                    first_controller_processing_ms = (
                        command.get(
                            "controller_processing_ms"
                        )
                    )

                    first_controller_round_trip_ms = (
                        round_trip_ms
                    )


            # =================================================
            # SAFETY RELEASE
            # =================================================

            if (
                safety_latched

                and not aeb_active

                and speed_a
                <= STOP_SPEED_KMH

                and gap
                >= SAFETY_RELEASE_GAP_M

                and not vehicle_a_goal_reached
            ):

                safety_latched = False


                print()
                print()

                print(
                    "*** VEHICLE A SAFETY RELEASED ***"
                )

                print(
                    f"Safe route gap: "
                    f"{gap:.2f} m"
                )

                print(
                    "Vehicle A jatkaa "
                    "kohti loading zonea."
                )


            # =================================================
            # STEERING
            # =================================================

            steer_a = (
                calculate_steering(

                    vehicle_a,

                    route,

                    route_index_a,

                    a_goal_index
                )
            )


            steer_b = (
                calculate_steering(

                    vehicle_b,

                    route,

                    route_index_b,

                    b_goal_index
                )
            )


            # =================================================
            # VEHICLE B CONTROL
            # =================================================

            if (
                vehicle_b_goal_reached
            ):

                vehicle_b.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

            else:

                target_speed_b = (
                    get_target_speed_near_goal(

                        VEHICLE_B_SPEED_KMH,

                        goal_distance_b
                    )
                )


                control_b = (
                    calculate_drive_control(

                        speed_b,

                        target_speed_b,

                        steer_b
                    )
                )


                vehicle_b.apply_control(
                    control_b
                )


            # =================================================
            # VEHICLE A CONTROL
            # =================================================

            if (
                vehicle_a_goal_reached
            ):

                vehicle_a.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

            elif (
                safety_latched
            ):

                # Safety controller ohittaa
                # navigation-controllerin.
                vehicle_a.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=steer_a
                    )
                )

            else:

                target_speed_a = (
                    get_target_speed_near_goal(

                        VEHICLE_A_SPEED_KMH,

                        goal_distance_a
                    )
                )


                control_a = (
                    calculate_drive_control(

                        speed_a,

                        target_speed_a,

                        steer_a
                    )
                )


                vehicle_a.apply_control(
                    control_a
                )


            # =================================================
            # COLLISION FAILSAFE
            # =================================================

            if (
                collision_state[
                    "A"
                ]

                or

                collision_state[
                    "B"
                ]
            ):

                vehicle_a.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )


                vehicle_b.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )


                print()
                print()

                print(
                    "*** COLLISION DETECTED ***"
                )

                break


            # =================================================
            # BOTH VEHICLES AT LOADING ZONE
            # =================================================

            if (
                vehicle_a_goal_reached

                and

                vehicle_b_goal_reached
            ):

                for _ in range(3):

                    world.tick()

                    time.sleep(
                        0.001
                    )


                print()
                print()

                print(
                    "*** BOTH VEHICLES "
                    "REACHED LOADING ZONE ***"
                )

                break


            # =================================================
            # TERMINAL OUTPUT
            # =================================================

            ttc_text = (
                "---"
                if current_ttc
                is None
                else
                f"{current_ttc:.2f}"
            )


            if (
                vehicle_a_goal_reached
            ):

                state_a = (
                    "GOAL"
                )

            elif (
                safety_latched
            ):

                state_a = (
                    "HOLD"
                )

            else:

                state_a = (
                    "NAV"
                )


            state_b = (

                "GOAL"

                if vehicle_b_goal_reached

                else "NAV"
            )


            print(

                f"A:{state_a:4} "
                f"{speed_a:5.1f} km/h "
                f"[{route_index_a:3}/{a_goal_index:3}] | "

                f"B:{state_b:4} "
                f"{speed_b:5.1f} km/h "
                f"[{route_index_b:3}/{b_goal_index:3}] | "

                f"Route gap: "
                f"{gap:5.1f} m | "

                f"TTC: "
                f"{ttc_text:>5} s",

                end="\r"
            )


        # =================================================
        # FINAL RESULT
        # =================================================

        final_goal_distance_a = (
            get_goal_distance(
                vehicle_a,
                route,
                a_goal_index
            )
        )


        final_goal_distance_b = (
            get_goal_distance(
                vehicle_b,
                route,
                b_goal_index
            )
        )


        safety_ok = (
            minimum_gap
            is not None

            and

            minimum_gap
            >= SAFETY_MARGIN_M
        )


        collision_free = (

            not collision_state[
                "A"
            ]

            and

            not collision_state[
                "B"
            ]
        )


        success = (

            safety_interrupt_count
            >= 1

            and

            vehicle_a_goal_reached

            and

            vehicle_b_goal_reached

            and

            safety_ok

            and

            collision_free
        )


        result_string = (

            "PASS"

            if success

            else "FAIL"
        )


        print()
        print()

        print(
            "=" * 90
        )

        print(
            "TEST RESULT"
        )

        print(
            "=" * 90
        )


        print(
            f"Safety interrupts: "
            f"{safety_interrupt_count}"
        )

        print(
            f"Minimum route gap: "
            f"{minimum_gap}"
        )

        print(
            f"Vehicle A goal reached: "
            f"{vehicle_a_goal_reached}"
        )

        print(
            f"Vehicle B goal reached: "
            f"{vehicle_b_goal_reached}"
        )

        print(
            f"Vehicle A collision: "
            f"{collision_state['A']}"
        )

        print(
            f"Vehicle B collision: "
            f"{collision_state['B']}"
        )

        print(
            f"Vehicle A final goal distance: "
            f"{final_goal_distance_a:.2f} m"
        )

        print(
            f"Vehicle B final goal distance: "
            f"{final_goal_distance_b:.2f} m"
        )

        print()

        print(
            f"RESULT: "
            f"{result_string}"
        )


        # =================================================
        # CSV
        # =================================================

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "route_length_m":
                round(
                    route_length,
                    3
                ),

            "vehicle_a_speed_kmh":
                VEHICLE_A_SPEED_KMH,

            "vehicle_b_speed_kmh":
                VEHICLE_B_SPEED_KMH,

            "vehicle_b_start_distance_m":
                (
                    b_start_index
                    * ROUTE_STEP_M
                ),

            "safety_interrupt_count":
                safety_interrupt_count,

            "first_trigger_speed_a_kmh":
                (
                    round(
                        first_trigger_speed_a,
                        3
                    )
                    if first_trigger_speed_a
                    is not None
                    else ""
                ),

            "first_trigger_speed_b_kmh":
                (
                    round(
                        first_trigger_speed_b,
                        3
                    )
                    if first_trigger_speed_b
                    is not None
                    else ""
                ),

            "first_trigger_relative_speed_kmh":
                (
                    round(
                        first_trigger_relative_speed,
                        3
                    )
                    if first_trigger_relative_speed
                    is not None
                    else ""
                ),

            "first_trigger_gap_m":
                (
                    round(
                        first_trigger_gap,
                        3
                    )
                    if first_trigger_gap
                    is not None
                    else ""
                ),

            "first_trigger_ttc_s":
                (
                    round(
                        first_trigger_ttc,
                        4
                    )
                    if first_trigger_ttc
                    is not None
                    else ""
                ),

            "minimum_gap_m":
                (
                    round(
                        minimum_gap,
                        3
                    )
                    if minimum_gap
                    is not None
                    else ""
                ),

            "vehicle_a_goal_reached":
                vehicle_a_goal_reached,

            "vehicle_b_goal_reached":
                vehicle_b_goal_reached,

            "vehicle_a_final_goal_distance_m":
                round(
                    final_goal_distance_a,
                    3
                ),

            "vehicle_b_final_goal_distance_m":
                round(
                    final_goal_distance_b,
                    3
                ),

            "collision_a":
                collision_state[
                    "A"
                ],

            "collision_b":
                collision_state[
                    "B"
                ],

            "controller_processing_ms":
                (
                    round(
                        first_controller_processing_ms,
                        3
                    )
                    if first_controller_processing_ms
                    is not None
                    else ""
                ),

            "controller_round_trip_ms":
                (
                    round(
                        first_controller_round_trip_ms,
                        3
                    )
                    if first_controller_round_trip_ms
                    is not None
                    else ""
                ),

            "result":
                result_string
        }


        save_result(
            result
        )


        print()

        print(
            "Tulokset:"
        )

        print(
            RESULTS_FILE
        )


    except KeyboardInterrupt:

        print()
        print(
            "Testi keskeytetty."
        )


    except Exception as error:

        print()
        print(
            f"Virhe: "
            f"{error}"
        )


    finally:

        # =================================================
        # CLEANUP
        # =================================================

        if (
            collision_sensor_a
            is not None
        ):

            collision_sensor_a.stop()

            collision_sensor_a.destroy()


        if (
            collision_sensor_b
            is not None
        ):

            collision_sensor_b.stop()

            collision_sensor_b.destroy()


        if (
            vehicle_a
            is not None
        ):

            vehicle_a.destroy()


        if (
            vehicle_b
            is not None
        ):

            vehicle_b.destroy()


        if (
            world
            is not None

            and

            original_settings
            is not None
        ):

            print()
            print(
                "Palautetaan CARLAn asetukset..."
            )

            world.apply_settings(
                original_settings
            )


        if (
            sensor_socket
            is not None
        ):

            sensor_socket.close()


        if (
            control_socket
            is not None
        ):

            control_socket.close()


        if (
            context
            is not None
        ):

            context.term()


        print(
            "Valmis."
        )


if __name__ == "__main__":

    main()