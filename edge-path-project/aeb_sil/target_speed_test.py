import carla
import csv
import json
import math
import os
import time
from datetime import datetime

import zmq


# =========================================================
# TEST CONFIGURATION
# =========================================================

TARGET_SPEEDS_KMH = [
    20.0,
    30.0,
    40.0,
    50.0
]

FIXED_DELTA_SECONDS = 0.05

# Autojen keskipisteiden lähtöetäisyys.
OBSTACLE_CENTER_DISTANCE_M = 50.0

AEB_TRIGGER_DISTANCE_M = 10.0

SAFETY_MARGIN_M = 1.0

STOP_SPEED_KMH = 0.5

TEST_TIMEOUT_SECONDS = 15.0

RESULTS_FILE = (
    "results/aeb_sil_target_speed_results.csv"
)


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = "tcp://127.0.0.1:5555"
CONTROL_ADDRESS = "tcp://127.0.0.1:5556"


# =========================================================
# SPEED
# =========================================================

def get_speed_kmh(vehicle):

    velocity = vehicle.get_velocity()

    speed_ms = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )

    return speed_ms * 3.6


# =========================================================
# SET EXACT TEST SPEED
# =========================================================

def set_target_speed(
    vehicle,
    target_speed_kmh
):

    speed_ms = (
        target_speed_kmh / 3.6
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
            360.0 - difference
        )

    return difference


# =========================================================
# FIND STRAIGHT TEST AREA
# =========================================================

def find_test_locations(
    world,
    distance_m
):

    carla_map = world.get_map()

    spawn_points = (
        carla_map.get_spawn_points()
    )

    for spawn_transform in spawn_points:

        start_wp = carla_map.get_waypoint(
            spawn_transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if start_wp is None:
            continue

        if start_wp.is_junction:
            continue

        start_transform = (
            start_wp.transform
        )

        forward = (
            start_transform.get_forward_vector()
        )

        valid = True
        target_wp = None

        check_distances = [
            10.0,
            20.0,
            30.0,
            40.0,
            distance_m
        ]

        for check_distance in check_distances:

            expected_location = carla.Location(
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
                z=start_transform.location.z
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
                check_wp.transform.location.distance(
                    expected_location
                )
            )

            if projection_error > 2.0:
                valid = False
                break

            yaw_error = get_yaw_difference(
                start_transform.rotation.yaw,
                check_wp.transform.rotation.yaw
            )

            if yaw_error > 5.0:
                valid = False
                break

            target_wp = check_wp

        if not valid:
            continue

        if target_wp is None:
            continue

        actual_distance = (
            start_transform.location.distance(
                target_wp.transform.location
            )
        )

        if abs(
            actual_distance - distance_m
        ) > 3.0:
            continue

        ego_transform = carla.Transform(

            carla.Location(
                x=start_transform.location.x,
                y=start_transform.location.y,
                z=(
                    start_transform.location.z
                    + 0.3
                )
            ),

            start_transform.rotation
        )

        obstacle_transform = carla.Transform(

            carla.Location(
                x=target_wp.transform.location.x,
                y=target_wp.transform.location.y,
                z=(
                    target_wp.transform.location.z
                    + 0.3
                )
            ),

            target_wp.transform.rotation
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
        ego_transform.get_forward_vector()
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

    center_distance = (
        delta_x * forward.x
        + delta_y * forward.y
        + delta_z * forward.z
    )

    ego_half_length = (
        ego_vehicle.bounding_box.extent.x
    )

    obstacle_half_length = (
        obstacle_vehicle.bounding_box.extent.x
    )

    gap = (
        center_distance
        - ego_half_length
        - obstacle_half_length
    )

    return max(
        0.0,
        gap
    )


# =========================================================
# DRAIN OLD CONTROL COMMANDS
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
# SEND SENSOR DATA AND WAIT FOR MATCHING RESPONSE
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

    # PUB/SUB voi kadottaa yksittäisen viestin,
    # joten yritetään tarvittaessa uudelleen.
    for _ in range(3):

        sensor_socket.send_string(
            json.dumps(sensor_data)
        )

        events = dict(
            poller.poll(200)
        )

        if (
            control_socket
            not in events
        ):
            continue

        while True:

            try:

                message = (
                    control_socket.recv_string(
                        flags=zmq.NOBLOCK
                    )
                )

                command = json.loads(
                    message
                )

                if (
                    command.get("test_id")
                    == expected_test
                    and command.get("sequence")
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

    os.makedirs(
        "results",
        exist_ok=True
    )

    file_exists = os.path.isfile(
        RESULTS_FILE
    )

    fieldnames = [
        "timestamp",
        "test_id",
        "target_speed_kmh",
        "trigger_speed_kmh",
        "speed_error_kmh",
        "trigger_gap_m",
        "controller_processing_ms",
        "controller_round_trip_ms",
        "stopping_time_s",
        "stopping_distance_m",
        "final_gap_m",
        "safety_margin_m",
        "collision",
        "result"
    ]

    with open(
        RESULTS_FILE,
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

        writer.writerow(result)


# =========================================================
# SINGLE TEST
# =========================================================

def run_test(
    world,
    blueprint_library,
    ego_transform,
    obstacle_transform,
    target_speed,
    sensor_socket,
    control_socket,
    poller
):

    ego_vehicle = None
    obstacle_vehicle = None
    collision_sensor = None

    collision_state = {
        "detected": False,
        "actor": None
    }

    test_id = (
        f"{int(target_speed)}_kmh"
    )

    brake_active = False

    trigger_speed = None
    trigger_gap = None
    trigger_time = None
    trigger_position = None

    controller_processing_ms = None
    controller_round_trip_ms = None

    stopping_time = None
    stopping_distance = None
    final_gap = None

    sequence = 0
    simulation_time = 0.0

    print()
    print("=" * 75)
    print(
        f"SIL AEB TESTI: "
        f"{target_speed:.0f} km/h"
    )
    print("=" * 75)

    try:

        # -------------------------------------------------
        # VEHICLES
        # -------------------------------------------------

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

        if ego_vehicle is None:
            raise RuntimeError(
                "Ego-ajoneuvon luonti epäonnistui."
            )

        obstacle_vehicle = (
            world.try_spawn_actor(
                vehicle_bp,
                obstacle_transform
            )
        )

        if obstacle_vehicle is None:
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

        # -------------------------------------------------
        # COLLISION SENSOR
        # -------------------------------------------------

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

            collision_state["detected"] = True

            if event.other_actor is not None:
                collision_state["actor"] = (
                    event.other_actor.type_id
                )

        collision_sensor.listen(
            collision_callback
        )

        # Vakautetaan actorit.
        for _ in range(5):
            world.tick()

        initial_gap = get_obstacle_gap(
            ego_vehicle,
            obstacle_vehicle
        )

        print(
            f"Target speed: "
            f"{target_speed:.1f} km/h"
        )

        print(
            f"Initial gap: "
            f"{initial_gap:.2f} m"
        )

        print(
            f"AEB trigger: "
            f"{AEB_TRIGGER_DISTANCE_M:.1f} m"
        )

        print(
            f"Safety margin: "
            f"{SAFETY_MARGIN_M:.1f} m"
        )

        print()

        drain_control_socket(
            control_socket
        )

        vehicle_has_moved = False

        # -------------------------------------------------
        # TEST LOOP
        # -------------------------------------------------

        while (
            simulation_time
            < TEST_TIMEOUT_SECONDS
        ):

            # =============================================
            # BEFORE AEB:
            # force exact baseline test speed
            # =============================================

            if not brake_active:

                set_target_speed(
                    ego_vehicle,
                    target_speed
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

            time.sleep(0.001)

            speed_kmh = get_speed_kmh(
                ego_vehicle
            )

            gap_m = get_obstacle_gap(
                ego_vehicle,
                obstacle_vehicle
            )

            if speed_kmh > 1.0:
                vehicle_has_moved = True

            # =============================================
            # SENSOR -> CONTROLLER
            # =============================================

            sequence += 1

            sensor_data = {
                "test_id": test_id,
                "sequence": sequence,
                "speed_kmh": speed_kmh,
                "obstacle_distance_m": gap_m
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

                print()
                print(
                    "Controllerilta ei saatu vastausta."
                )

                return {
                    "timestamp":
                        datetime.now().isoformat(
                            timespec="seconds"
                        ),
                    "test_id": test_id,
                    "target_speed_kmh": target_speed,
                    "trigger_speed_kmh": "",
                    "speed_error_kmh": "",
                    "trigger_gap_m": "",
                    "controller_processing_ms": "",
                    "controller_round_trip_ms": "",
                    "stopping_time_s": "",
                    "stopping_distance_m": "",
                    "final_gap_m": "",
                    "safety_margin_m": SAFETY_MARGIN_M,
                    "collision":
                        collision_state["detected"],
                    "result": "FAIL"
                }

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

            # =============================================
            # FIRST AEB TRIGGER
            # =============================================

            if (
                controller_aeb
                and not brake_active
            ):

                brake_active = True

                trigger_speed = speed_kmh
                trigger_gap = gap_m

                trigger_time = (
                    simulation_time
                )

                trigger_position = (
                    ego_vehicle.get_location()
                )

                controller_processing_ms = (
                    command.get(
                        "controller_processing_ms"
                    )
                )

                controller_round_trip_ms = (
                    round_trip_ms
                )

                # Käytetään controllerin komentoa heti.
                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=controller_brake,
                        steer=0.0
                    )
                )

                print()
                print(
                    "*** AEB TRIGGER ***"
                )

                print(
                    f"Target speed: "
                    f"{target_speed:.2f} km/h"
                )

                print(
                    f"Trigger speed: "
                    f"{trigger_speed:.2f} km/h"
                )

                print(
                    f"Trigger gap: "
                    f"{trigger_gap:.2f} m"
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
            # OUTPUT
            # =============================================

            state = (
                "AEB ACTIVE"
                if brake_active
                else "TEST SPEED"
            )

            print(
                f"{test_id:7} | "
                f"{state:10} | "
                f"Speed: {speed_kmh:5.1f} km/h | "
                f"Gap: {gap_m:6.2f} m",
                end="\r"
            )

            # =============================================
            # STOPPED
            # =============================================

            if (
                vehicle_has_moved
                and brake_active
                and speed_kmh
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

                final_gap = get_obstacle_gap(
                    ego_vehicle,
                    obstacle_vehicle
                )

                # Muutama tick collision-eventille.
                for _ in range(2):

                    world.tick()
                    time.sleep(0.001)

                break

        # -------------------------------------------------
        # RESULT
        # -------------------------------------------------

        if trigger_speed is None:

            result_string = "FAIL"

        else:

            safe_margin_ok = (
                final_gap is not None
                and final_gap
                >= SAFETY_MARGIN_M
            )

            if (
                not collision_state["detected"]
                and safe_margin_ok
            ):
                result_string = "PASS"
            else:
                result_string = "FAIL"

        speed_error = None

        if trigger_speed is not None:

            speed_error = (
                trigger_speed
                - target_speed
            )

        print()
        print()

        print(
            f"Tulos: {result_string}"
        )

        if stopping_distance is not None:

            print(
                f"Pysähtymismatka: "
                f"{stopping_distance:.2f} m"
            )

        if stopping_time is not None:

            print(
                f"Pysähtymisaika: "
                f"{stopping_time:.2f} s"
            )

        if final_gap is not None:

            print(
                f"Loppuetäisyys: "
                f"{final_gap:.2f} m"
            )

        print(
            f"Törmäys: "
            f"{'KYLLÄ' if collision_state['detected'] else 'EI'}"
        )

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "test_id":
                test_id,

            "target_speed_kmh":
                target_speed,

            "trigger_speed_kmh":
                round(
                    trigger_speed,
                    3
                )
                if trigger_speed is not None
                else "",

            "speed_error_kmh":
                round(
                    speed_error,
                    3
                )
                if speed_error is not None
                else "",

            "trigger_gap_m":
                round(
                    trigger_gap,
                    3
                )
                if trigger_gap is not None
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
                if stopping_time is not None
                else "",

            "stopping_distance_m":
                round(
                    stopping_distance,
                    3
                )
                if stopping_distance is not None
                else "",

            "final_gap_m":
                round(
                    final_gap,
                    3
                )
                if final_gap is not None
                else "",

            "safety_margin_m":
                SAFETY_MARGIN_M,

            "collision":
                collision_state["detected"],

            "result":
                result_string
        }

        return result

    finally:

        if collision_sensor is not None:
            collision_sensor.stop()
            collision_sensor.destroy()

        if ego_vehicle is not None:
            ego_vehicle.destroy()

        if obstacle_vehicle is not None:
            obstacle_vehicle.destroy()

        # Annetaan CARLAn poistaa actorit.
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

        context = zmq.Context()

        sensor_socket = context.socket(
            zmq.PUB
        )

        sensor_socket.bind(
            SENSOR_ADDRESS
        )

        control_socket = context.socket(
            zmq.SUB
        )

        control_socket.connect(
            CONTROL_ADDRESS
        )

        control_socket.setsockopt_string(
            zmq.SUBSCRIBE,
            ""
        )

        poller = zmq.Poller()

        poller.register(
            control_socket,
            zmq.POLLIN
        )

        print(
            "ZeroMQ käynnistetty."
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

        world = client.get_world()

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

        # ZeroMQ slow-joiner protection.
        print(
            "Odotetaan controller-yhteyttä..."
        )

        time.sleep(1.0)

        blueprint_library = (
            world.get_blueprint_library()
        )

        # =================================================
        # TEST LOCATION
        # =================================================

        (
            ego_transform,
            obstacle_transform
        ) = find_test_locations(
            world,
            OBSTACLE_CENTER_DISTANCE_M
        )

        if ego_transform is None:

            print(
                "Sopivaa testipaikkaa "
                "ei löytynyt."
            )

            return

        print(
            "Testipaikka löytyi."
        )

        # =================================================
        # RUN TESTS
        # =================================================

        for target_speed in TARGET_SPEEDS_KMH:

            result = run_test(
                world,
                blueprint_library,
                ego_transform,
                obstacle_transform,
                target_speed,
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
        print("=" * 95)
        print(
            "SIL AEB TARGET-SPEED TESTISARJA"
        )
        print("=" * 95)

        print(
            "Target | Trigger | Gap     | "
            "Stop dist | Final gap | Result"
        )

        print("-" * 95)

        for result in results:

            print(
                f"{result['target_speed_kmh']:6.1f} | "
                f"{str(result['trigger_speed_kmh']):>7} | "
                f"{str(result['trigger_gap_m']):>7} | "
                f"{str(result['stopping_distance_m']):>9} | "
                f"{str(result['final_gap_m']):>9} | "
                f"{result['result']}"
            )

        print("=" * 95)

        print()
        print(
            f"Tulokset: {RESULTS_FILE}"
        )

    except KeyboardInterrupt:

        print()
        print(
            "Testisarja keskeytetty."
        )

    except Exception as error:

        print()
        print(
            f"Virhe: {error}"
        )

    finally:

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