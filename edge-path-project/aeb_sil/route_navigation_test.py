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

TARGET_SPEED_KMH = 30.0

FIXED_DELTA_SECONDS = 0.05

ROUTE_LENGTH_M = 120.0

ROUTE_STEP_M = 2.0

LOOKAHEAD_POINTS = 4

OBSTACLE_ROUTE_DISTANCE_M = 60.0

SAFETY_MARGIN_M = 1.0

STOP_SPEED_KMH = 0.5

GOAL_TOLERANCE_M = 5.0

TEST_TIMEOUT_SECONDS = 45.0

# Kun safety interrupt on pysäyttänyt auton,
# odotetaan hetki ennen esteen poistamista.
OBSTACLE_CLEAR_WAIT_S = 1.0


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
    / "route_navigation_results.csv"
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

    difference = (
        normalize_angle(
            difference
        )
    )

    return abs(
        difference
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

        # Jos edessä on useita vaihtoehtoja,
        # valitaan tässä ensimmäisessä versiossa
        # suorin jatkosuunta.
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

            ego_transform = (
                carla.Transform(

                    carla.Location(

                        x=(
                            start_waypoint
                            .transform
                            .location
                            .x
                        ),

                        y=(
                            start_waypoint
                            .transform
                            .location
                            .y
                        ),

                        z=(
                            start_waypoint
                            .transform
                            .location
                            .z
                            + 0.3
                        )
                    ),

                    start_waypoint
                    .transform
                    .rotation
                )
            )

            return (
                ego_transform,
                route
            )

    return None, None


# =========================================================
# DEBUG DRAWING
# =========================================================

def draw_route(
    world,
    route
):

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

        start = carla.Location(
            x=start.x,
            y=start.y,
            z=start.z + 0.5
        )

        end = carla.Location(
            x=end.x,
            y=end.y,
            z=end.z + 0.5
        )

        world.debug.draw_line(
            start,
            end,
            thickness=0.08,
            color=carla.Color(
                0,
                100,
                255
            ),
            life_time=60.0
        )

    goal = (
        route[-1]
        .transform
        .location
    )

    goal = carla.Location(
        x=goal.x,
        y=goal.y,
        z=goal.z + 1.0
    )

    world.debug.draw_point(
        goal,
        size=0.35,
        color=carla.Color(
            0,
            255,
            0
        ),
        life_time=60.0
    )


# =========================================================
# ROUTE TRACKING
# =========================================================

def update_route_index(
    vehicle,
    route,
    current_index
):

    location = (
        vehicle.get_location()
    )

    search_end = min(
        current_index + 12,
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

        route_location = (
            route[index]
            .transform
            .location
        )

        distance = (
            location.distance(
                route_location
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
    route_index
):

    target_index = min(
        route_index
        + LOOKAHEAD_POINTS,
        len(route) - 1
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

    # Noin 35 asteen heading error
    # vastaa täyttä steering-komentoa.
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
# LONGITUDINAL CONTROL
# =========================================================

def calculate_drive_control(
    speed_kmh,
    steer
):

    speed_error = (
        TARGET_SPEED_KMH
        - speed_kmh
    )

    throttle = 0.0
    brake = 0.0

    if (
        speed_error
        > 0.0
    ):

        throttle = clamp(
            speed_error / 15.0,
            0.0,
            0.55
        )

    else:

        brake = clamp(
            (-speed_error) / 10.0,
            0.0,
            0.4
        )

    return carla.VehicleControl(
        throttle=throttle,
        brake=brake,
        steer=steer
    )


# =========================================================
# OBSTACLE DISTANCE
# =========================================================

def get_obstacle_gap(
    ego_vehicle,
    obstacle_vehicle
):

    if (
        obstacle_vehicle
        is None
    ):

        return None

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

    right = (
        ego_transform
        .get_right_vector()
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

    longitudinal_distance = (

        delta_x * forward.x
        + delta_y * forward.y
        + delta_z * forward.z
    )

    lateral_distance = abs(

        delta_x * right.x
        + delta_y * right.y
        + delta_z * right.z
    )

    # Esteen pitää olla ego-auton edessä.
    if (
        longitudinal_distance
        <= 0.0
    ):

        return None

    # Esteen pitää olla suunnilleen samalla kaistalla.
    if (
        lateral_distance
        > 3.0
    ):

        return None

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
        sensor_data["test_id"]
    )

    expected_sequence = (
        sensor_data["sequence"]
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

        "target_speed_kmh",

        "aeb_triggered",

        "trigger_speed_kmh",

        "trigger_gap_m",

        "trigger_ttc_s",

        "minimum_gap_m",

        "safety_latched",

        "obstacle_cleared",

        "goal_reached",

        "final_goal_distance_m",

        "collision",

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

    ego_vehicle = None

    obstacle_vehicle = None

    collision_sensor = None

    context = None

    sensor_socket = None

    control_socket = None


    collision_state = {
        "detected": False
    }


    test_id = (
        "route_navigation"
    )


    sequence = 0

    simulation_time = 0.0

    route_index = 0


    aeb_triggered = False

    # Safety interrupt lukitaan ensimmäisestä
    # TTC-triggeristä lähtien.
    safety_latched = False


    trigger_speed = None

    trigger_gap = None

    trigger_ttc = None


    controller_processing_ms = None

    controller_round_trip_ms = None


    minimum_gap = None


    obstacle_cleared = False

    stopped_time = None


    goal_reached = False


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


        # =================================================
        # ROUTE
        # =================================================

        print()

        print(
            "Etsitään ajettavaa reittiä..."
        )

        (
            ego_transform,
            route
        ) = find_route(
            world
        )

        if (
            ego_transform
            is None
        ):

            raise RuntimeError(
                "Sopivaa reittiä ei löytynyt."
            )

        route_length = (
            (len(route) - 1)
            * ROUTE_STEP_M
        )

        print(
            f"Reitti löytyi: "
            f"{route_length:.1f} m"
        )

        print(
            f"Route points: "
            f"{len(route)}"
        )

        draw_route(
            world,
            route
        )


        # =================================================
        # VEHICLES
        # =================================================

        blueprint_library = (
            world
            .get_blueprint_library()
        )

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

        if (
            ego_vehicle
            is None
        ):

            raise RuntimeError(
                "Ego-ajoneuvon luonti epäonnistui."
            )


        obstacle_index = int(
            OBSTACLE_ROUTE_DISTANCE_M
            / ROUTE_STEP_M
        )

        obstacle_index = min(
            obstacle_index,
            len(route) - 10
        )

        obstacle_waypoint = (
            route[
                obstacle_index
            ]
        )

        obstacle_transform = (
            obstacle_waypoint
            .transform
        )

        obstacle_transform = (
            carla.Transform(

                carla.Location(

                    x=(
                        obstacle_transform
                        .location
                        .x
                    ),

                    y=(
                        obstacle_transform
                        .location
                        .y
                    ),

                    z=(
                        obstacle_transform
                        .location
                        .z
                        + 0.3
                    )
                ),

                obstacle_transform.rotation
            )
        )

        obstacle_vehicle = (
            world.try_spawn_actor(
                vehicle_bp,
                obstacle_transform
            )
        )

        if (
            obstacle_vehicle
            is None
        ):

            raise RuntimeError(
                "Esteajoneuvon luonti epäonnistui."
            )

        obstacle_vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0
            )
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

        def collision_callback(
            event
        ):

            collision_state[
                "detected"
            ] = True

        collision_sensor.listen(
            collision_callback
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


        goal_location = (
            route[-1]
            .transform
            .location
        )


        print()

        print(
            "=" * 80
        )

        print(
            "ROUTE NAVIGATION + TTC AEB SIL TEST"
        )

        print(
            "=" * 80
        )

        print(
            f"Target speed: "
            f"{TARGET_SPEED_KMH:.1f} km/h"
        )

        print(
            f"Route length: "
            f"{route_length:.1f} m"
        )

        print(
            f"Obstacle at route distance: "
            f"{OBSTACLE_ROUTE_DISTANCE_M:.1f} m"
        )

        print()

        print(
            "Ajetaan Start -> Goal."
        )

        print(
            "AEB saa keskeyttää ajon."
        )

        print(
            "Safety interrupt pysyy lukittuna "
            "kunnes este poistetaan."
        )

        print(
            "Sen jälkeen navigointi jatkuu."
        )

        print()


        # =================================================
        # TEST LOOP
        # =================================================

        while (
            simulation_time
            < TEST_TIMEOUT_SECONDS
        ):

            # ---------------------------------------------
            # SIMULATION STEP
            # ---------------------------------------------

            world.tick()

            simulation_time += (
                FIXED_DELTA_SECONDS
            )

            time.sleep(
                0.001
            )


            # ---------------------------------------------
            # VEHICLE STATE
            # ---------------------------------------------

            speed_kmh = (
                get_speed_kmh(
                    ego_vehicle
                )
            )


            route_index = (
                update_route_index(
                    ego_vehicle,
                    route,
                    route_index
                )
            )


            steer = (
                calculate_steering(
                    ego_vehicle,
                    route,
                    route_index
                )
            )


            # ---------------------------------------------
            # OBSTACLE STATE
            # ---------------------------------------------

            obstacle_gap = (
                get_obstacle_gap(
                    ego_vehicle,
                    obstacle_vehicle
                )
            )


            if (
                obstacle_gap
                is not None
            ):

                if (
                    minimum_gap
                    is None
                    or obstacle_gap
                    < minimum_gap
                ):

                    minimum_gap = (
                        obstacle_gap
                    )


            obstacle_speed = 0.0

            if (
                obstacle_vehicle
                is not None
            ):

                obstacle_speed = (
                    get_speed_kmh(
                        obstacle_vehicle
                    )
                )


            # =============================================
            # SENSOR -> TTC CONTROLLER
            # =============================================

            sequence += 1


            sensor_data = {

                "test_id":
                    test_id,

                "sequence":
                    sequence,

                "ego_speed_kmh":
                    speed_kmh,

                "obstacle_speed_kmh":
                    obstacle_speed,

                "obstacle_distance_m":
                    obstacle_gap
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


            # =============================================
            # FIRST AEB TRIGGER
            # =============================================

            if (
                aeb_active
                and not aeb_triggered
            ):

                aeb_triggered = True

                # TÄRKEÄ:
                # safety interrupt lukitaan,
                # vaikka TTC myöhemmin kasvaisi.
                safety_latched = True


                trigger_speed = (
                    speed_kmh
                )

                trigger_gap = (
                    obstacle_gap
                )

                trigger_ttc = (
                    current_ttc
                )


                controller_processing_ms = (
                    command.get(
                        "controller_processing_ms"
                    )
                )

                controller_round_trip_ms = (
                    round_trip_ms
                )


                print()
                print()

                print(
                    "*** TTC AEB SAFETY INTERRUPT ***"
                )

                print(
                    f"Speed: "
                    f"{trigger_speed:.2f} km/h"
                )

                print(
                    f"Gap: "
                    f"{trigger_gap:.2f} m"
                )

                print(
                    f"TTC: "
                    f"{trigger_ttc:.3f} s"
                )

                print(
                    "Safety interrupt latched."
                )

                print()


            # =============================================
            # VEHICLE CONTROL
            # =============================================

            if (
                safety_latched
                and not obstacle_cleared
            ):

                # Safety layer ohittaa
                # normaalin navigation-controllerin.
                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=steer
                    )
                )

            else:

                drive_control = (
                    calculate_drive_control(
                        speed_kmh,
                        steer
                    )
                )

                ego_vehicle.apply_control(
                    drive_control
                )


            # =============================================
            # OBSTACLE CLEARING
            # =============================================

            if (
                aeb_triggered
                and safety_latched
                and not obstacle_cleared
                and speed_kmh
                <= STOP_SPEED_KMH
            ):

                if (
                    stopped_time
                    is None
                ):

                    stopped_time = (
                        simulation_time
                    )

                    print()
                    print()

                    print(
                        "Ajoneuvo pysähtyi "
                        "turvallisesti."
                    )

                    print(
                        "Safety interrupt pysyy "
                        "lukittuna."
                    )

                    print(
                        "Odotetaan esteen "
                        "poistamista..."
                    )


                if (
                    simulation_time
                    - stopped_time
                    >= OBSTACLE_CLEAR_WAIT_S
                ):

                    if (
                        obstacle_vehicle
                        is not None
                    ):

                        obstacle_vehicle.destroy()

                        obstacle_vehicle = None


                    obstacle_cleared = True


                    # Safety layer vapautetaan
                    # vasta kun este on poistettu.
                    safety_latched = False


                    print()
                    print()

                    print(
                        "Este poistettu."
                    )

                    print(
                        "Safety interrupt vapautettu."
                    )

                    print(
                        "Navigointi jatkuu "
                        "kohti Goal-pistettä."
                    )

                    print()


            # =============================================
            # GOAL
            # =============================================

            goal_distance = (
                ego_vehicle
                .get_location()
                .distance(
                    goal_location
                )
            )


            if (
                goal_distance
                <= GOAL_TOLERANCE_M
                and route_index
                >= len(route) - 6
            ):

                goal_reached = True


                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )


                print()
                print()

                print(
                    "*** GOAL REACHED ***"
                )

                print(
                    f"Goal distance: "
                    f"{goal_distance:.2f} m"
                )

                break


            # =============================================
            # COLLISION FAILSAFE
            # =============================================

            if (
                collision_state[
                    "detected"
                ]
            ):

                ego_vehicle.apply_control(
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


            # =============================================
            # TERMINAL STATE
            # =============================================

            gap_text = (
                "---"
                if obstacle_gap
                is None
                else
                f"{obstacle_gap:.1f}"
            )


            ttc_text = (
                "---"
                if current_ttc
                is None
                else
                f"{current_ttc:.2f}"
            )


            if (
                safety_latched
                and not obstacle_cleared
            ):

                state = (
                    "HOLD"
                )

            elif (
                aeb_active
            ):

                state = (
                    "AEB"
                )

            else:

                state = (
                    "NAV"
                )


            print(
                f"{state:4} | "
                f"Speed: "
                f"{speed_kmh:5.1f} km/h | "
                f"Route: "
                f"{route_index:3}/{len(route):3} | "
                f"Gap: "
                f"{gap_text:>5} m | "
                f"TTC: "
                f"{ttc_text:>5} s | "
                f"Goal: "
                f"{goal_distance:6.1f} m",
                end="\r"
            )


        # =================================================
        # RESULT
        # =================================================

        final_goal_distance = (
            ego_vehicle
            .get_location()
            .distance(
                goal_location
            )
        )


        safety_ok = (
            minimum_gap
            is not None
            and minimum_gap
            >= SAFETY_MARGIN_M
        )


        success = (

            aeb_triggered

            and obstacle_cleared

            and goal_reached

            and safety_ok

            and not collision_state[
                "detected"
            ]
        )


        result_string = (
            "PASS"
            if success
            else "FAIL"
        )


        print()
        print()

        print(
            "=" * 80
        )

        print(
            "TEST RESULT"
        )

        print(
            "=" * 80
        )


        print(
            f"AEB triggered: "
            f"{aeb_triggered}"
        )

        print(
            f"Safety latched at end: "
            f"{safety_latched}"
        )

        print(
            f"Obstacle cleared: "
            f"{obstacle_cleared}"
        )

        print(
            f"Goal reached: "
            f"{goal_reached}"
        )

        print(
            f"Minimum gap: "
            f"{minimum_gap}"
        )

        print(
            f"Collision: "
            f"{collision_state['detected']}"
        )

        print(
            f"Final goal distance: "
            f"{final_goal_distance:.2f} m"
        )

        print()

        print(
            f"RESULT: "
            f"{result_string}"
        )


        # =================================================
        # CSV RESULT
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

            "target_speed_kmh":
                TARGET_SPEED_KMH,

            "aeb_triggered":
                aeb_triggered,

            "trigger_speed_kmh":
                (
                    round(
                        trigger_speed,
                        3
                    )
                    if trigger_speed
                    is not None
                    else ""
                ),

            "trigger_gap_m":
                (
                    round(
                        trigger_gap,
                        3
                    )
                    if trigger_gap
                    is not None
                    else ""
                ),

            "trigger_ttc_s":
                (
                    round(
                        trigger_ttc,
                        4
                    )
                    if trigger_ttc
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

            "safety_latched":
                safety_latched,

            "obstacle_cleared":
                obstacle_cleared,

            "goal_reached":
                goal_reached,

            "final_goal_distance_m":
                round(
                    final_goal_distance,
                    3
                ),

            "collision":
                collision_state[
                    "detected"
                ],

            "controller_processing_ms":
                (
                    round(
                        controller_processing_ms,
                        3
                    )
                    if controller_processing_ms
                    is not None
                    else ""
                ),

            "controller_round_trip_ms":
                (
                    round(
                        controller_round_trip_ms,
                        3
                    )
                    if controller_round_trip_ms
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
            collision_sensor
            is not None
        ):

            collision_sensor.stop()

            collision_sensor.destroy()


        if (
            obstacle_vehicle
            is not None
        ):

            obstacle_vehicle.destroy()


        if (
            ego_vehicle
            is not None
        ):

            ego_vehicle.destroy()


        if (
            world
            is not None
            and original_settings
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