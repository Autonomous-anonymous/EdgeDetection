import carla
import csv
import math
import os
import threading
import time
from datetime import datetime


# =========================================================
# SIMULAATION ASETUKSET
# =========================================================

FIXED_DELTA_SECONDS = 0.05  # 20 simulaatioaskelta / sekunti

OBSTACLE_START_DISTANCE = 35.0
SENSOR_RANGE = 40.0

AEB_TRIGGER_DISTANCE = 10.0
STOP_SPEED_KMH = 0.5

TEST_TIMEOUT_SECONDS = 25.0

RESULTS_FILE = "results/aeb_batch_results.csv"


# =========================================================
# TESTISARJA
#
# Sama 10 m AEB-raja, mutta eri kaasuarvot.
# Näin nähdään, miten ajonopeus vaikuttaa tulokseen.
# =========================================================

TEST_CASES = [
    {
        "name": "test_01",
        "throttle": 0.25
    },
    {
        "name": "test_02",
        "throttle": 0.35
    },
    {
        "name": "test_03",
        "throttle": 0.45
    },
    {
        "name": "test_04",
        "throttle": 0.55
    },
    {
        "name": "test_05",
        "throttle": 0.65
    }
]


# =========================================================
# SENSORIDATA
# =========================================================

latest_obstacle_distance = None
latest_obstacle_name = None
latest_obstacle_callback_time = None

collision_detected = False
collision_with = None

data_lock = threading.Lock()


# =========================================================
# SENSORIEN CALLBACK-FUNKTIOT
# =========================================================

def obstacle_callback(event):

    global latest_obstacle_distance
    global latest_obstacle_name
    global latest_obstacle_callback_time

    with data_lock:

        latest_obstacle_distance = event.distance
        latest_obstacle_callback_time = time.perf_counter()

        if event.other_actor is not None:
            latest_obstacle_name = event.other_actor.type_id
        else:
            latest_obstacle_name = "unknown"


def collision_callback(event):

    global collision_detected
    global collision_with

    with data_lock:

        collision_detected = True

        if event.other_actor is not None:
            collision_with = event.other_actor.type_id
        else:
            collision_with = "unknown"


# =========================================================
# SENSORITILAN NOLLAUS TESTIEN VÄLISSÄ
# =========================================================

def reset_sensor_data():

    global latest_obstacle_distance
    global latest_obstacle_name
    global latest_obstacle_callback_time

    global collision_detected
    global collision_with

    with data_lock:

        latest_obstacle_distance = None
        latest_obstacle_name = None
        latest_obstacle_callback_time = None

        collision_detected = False
        collision_with = None


# =========================================================
# AJONEUVON NOPEUS
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
# TESTIPAIKAN ETSINTÄ
# =========================================================

def find_test_locations(world, distance):

    carla_map = world.get_map()

    spawn_points = carla_map.get_spawn_points()

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

        next_waypoints = start_wp.next(distance)

        if not next_waypoints:
            continue

        obstacle_wp = next_waypoints[0]

        if obstacle_wp.is_junction:
            continue

        # Tarkistetaan että tien alku ja loppu
        # ovat suunnilleen samaan suuntaan.
        yaw1 = start_wp.transform.rotation.yaw
        yaw2 = obstacle_wp.transform.rotation.yaw

        yaw_difference = abs(yaw1 - yaw2)

        if yaw_difference > 180:
            yaw_difference = 360 - yaw_difference

        if yaw_difference > 5:
            continue

        ego_transform = start_wp.transform

        ego_transform.location.z += 0.3

        obstacle_transform = obstacle_wp.transform

        obstacle_transform.location.z += 0.3

        return ego_transform, obstacle_transform

    return None, None


# =========================================================
# TULOSTEN TALLENNUS
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
        "test_name",
        "result",
        "throttle",
        "aeb_trigger_distance_m",
        "actual_trigger_distance_m",
        "speed_at_trigger_kmh",
        "controller_processing_latency_ms",
        "stopping_time_s",
        "stopping_distance_m",
        "final_obstacle_distance_m",
        "collision"
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
# YHDEN TESTIN SUORITUS
# =========================================================

def run_test(
    world,
    blueprint_library,
    ego_transform,
    obstacle_transform,
    test_case
):

    reset_sensor_data()

    ego_vehicle = None
    obstacle_vehicle = None

    obstacle_sensor = None
    collision_sensor = None

    brake_triggered = False

    trigger_time = None
    trigger_position = None
    trigger_speed = None

    actual_trigger_distance = None

    controller_latency_ms = None

    stopping_time = None
    stopping_distance = None
    final_obstacle_distance = None

    test_result = "FAIL"

    throttle = test_case["throttle"]
    test_name = test_case["name"]

    print()
    print("=" * 65)
    print(f"ALOITETAAN {test_name}")
    print("=" * 65)

    print(
        f"Throttle: {throttle:.2f}"
    )

    print(
        f"AEB-raja: "
        f"{AEB_TRIGGER_DISTANCE:.1f} m"
    )

    try:

        # -------------------------------------------------
        # LUODAAN AUTOT
        # -------------------------------------------------

        vehicle_bp = blueprint_library.find(
            "vehicle.tesla.model3"
        )

        ego_vehicle = world.try_spawn_actor(
            vehicle_bp,
            ego_transform
        )

        if ego_vehicle is None:
            print("Ego-autoa ei voitu luoda.")
            return None

        obstacle_vehicle = world.try_spawn_actor(
            vehicle_bp,
            obstacle_transform
        )

        if obstacle_vehicle is None:
            print("Esteautoa ei voitu luoda.")
            return None

        # Esteauto pysyy paikallaan
        obstacle_vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0
            )
        )

        # -------------------------------------------------
        # OBSTACLE SENSOR
        # -------------------------------------------------

        obstacle_bp = blueprint_library.find(
            "sensor.other.obstacle"
        )

        obstacle_bp.set_attribute(
            "distance",
            str(SENSOR_RANGE)
        )

        obstacle_bp.set_attribute(
            "hit_radius",
            "0.8"
        )

        obstacle_bp.set_attribute(
            "only_dynamics",
            "false"
        )

        obstacle_sensor = world.spawn_actor(
            obstacle_bp,
            carla.Transform(
                carla.Location(
                    x=2.0,
                    y=0.0,
                    z=1.0
                )
            ),
            attach_to=ego_vehicle
        )

        obstacle_sensor.listen(
            obstacle_callback
        )

        # -------------------------------------------------
        # COLLISION SENSOR
        # -------------------------------------------------

        collision_bp = blueprint_library.find(
            "sensor.other.collision"
        )

        collision_sensor = world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=ego_vehicle
        )

        collision_sensor.listen(
            collision_callback
        )

        # -------------------------------------------------
        # ANNETAAN SIMULAATION ALUSTUA
        # -------------------------------------------------

        for _ in range(10):
            world.tick()

        # -------------------------------------------------
        # TESTIN ALKU
        # -------------------------------------------------

        simulation_time = 0.0

        while simulation_time < TEST_TIMEOUT_SECONDS:

            # -------------------------------------------------
            # NORMAALIAJO
            # -------------------------------------------------

            if not brake_triggered:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=throttle,
                        brake=0.0,
                        steer=0.0
                    )
                )

            else:

                # -------------------------------------------------
                # AEB ACTIVE
                # -------------------------------------------------

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

            # -------------------------------------------------
            # EDETÄÄN TÄSMÄLLEEN YKSI SIMULAATIOASKEL
            # -------------------------------------------------

            world.tick()

            simulation_time += FIXED_DELTA_SECONDS

            speed_kmh = get_speed_kmh(
                ego_vehicle
            )

            # -------------------------------------------------
            # SENSORIDATA
            # -------------------------------------------------

            with data_lock:

                obstacle_distance = (
                    latest_obstacle_distance
                )

                obstacle_name = (
                    latest_obstacle_name
                )

                callback_time = (
                    latest_obstacle_callback_time
                )

                collision_now = (
                    collision_detected
                )

            # -------------------------------------------------
            # AEB-LAUKAISU
            # -------------------------------------------------

            if (
                not brake_triggered
                and obstacle_distance is not None
                and obstacle_distance
                <= AEB_TRIGGER_DISTANCE
            ):

                controller_time = (
                    time.perf_counter()
                )

                brake_triggered = True

                trigger_time = simulation_time

                trigger_speed = speed_kmh

                actual_trigger_distance = (
                    obstacle_distance
                )

                trigger_position = (
                    ego_vehicle.get_location()
                )

                if callback_time is not None:

                    controller_latency_ms = (
                        controller_time
                        - callback_time
                    ) * 1000.0

                print()
                print()
                print(
                    "!!! AEB TRIGGERED !!!"
                )

                print(
                    f"Este: {obstacle_name}"
                )

                print(
                    f"Etäisyys: "
                    f"{actual_trigger_distance:.2f} m"
                )

                print(
                    f"Nopeus: "
                    f"{trigger_speed:.2f} km/h"
                )

                if controller_latency_ms is not None:

                    print(
                        "Controller processing latency: "
                        f"{controller_latency_ms:.2f} ms"
                    )

            # -------------------------------------------------
            # STATUS
            # -------------------------------------------------

            if obstacle_distance is None:
                distance_text = "no detection"
            else:
                distance_text = (
                    f"{obstacle_distance:.2f} m"
                )

            if brake_triggered:

                print(
                    f"{test_name} | "
                    f"AEB ACTIVE | "
                    f"{speed_kmh:5.1f} km/h | "
                    f"{distance_text}",
                    end="\r"
                )

            else:

                print(
                    f"{test_name} | "
                    f"Speed: {speed_kmh:5.1f} km/h | "
                    f"Obstacle: {distance_text}",
                    end="\r"
                )

            # -------------------------------------------------
            # PYSÄHTYMISEN TUNNISTUS
            # -------------------------------------------------

            if (
                brake_triggered
                and speed_kmh <= STOP_SPEED_KMH
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

                final_obstacle_distance = (
                    obstacle_distance
                )

                # Muutama ylimääräinen tick,
                # jotta collision callback ehtii varmasti.
                for _ in range(3):
                    world.tick()

                with data_lock:
                    collision_now = collision_detected

                if collision_now:
                    test_result = "FAIL"
                else:
                    test_result = "PASS"

                print()
                print()
                print(
                    "Ajoneuvo pysähtyi."
                )

                print(
                    f"Pysähtymisaika: "
                    f"{stopping_time:.2f} s"
                )

                print(
                    f"Pysähtymismatka: "
                    f"{stopping_distance:.2f} m"
                )

                if final_obstacle_distance is not None:

                    print(
                        f"Loppuetäisyys: "
                        f"{final_obstacle_distance:.2f} m"
                    )

                print(
                    f"Törmäys: "
                    f"{'KYLLÄ' if collision_now else 'EI'}"
                )

                print(
                    f"TULOS: {test_result}"
                )

                break

            # -------------------------------------------------
            # JOS TÖRMÄYS TAPAHTUU ENNEN PYSÄHDYSTÄ
            # -------------------------------------------------

            if collision_now:

                print()
                print()
                print(
                    "TÖRMÄYS HAVAITTU!"
                )

                test_result = "FAIL"

                # Jarrutetaan joka tapauksessa auto pysähdyksiin
                brake_triggered = True

        # -------------------------------------------------
        # TIMEOUT
        # -------------------------------------------------

        if simulation_time >= TEST_TIMEOUT_SECONDS:

            print()
            print(
                "TEST TIMEOUT."
            )

            test_result = "FAIL"

        # -------------------------------------------------
        # RESULT-DICT
        # -------------------------------------------------

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "test_name":
                test_name,

            "result":
                test_result,

            "throttle":
                throttle,

            "aeb_trigger_distance_m":
                AEB_TRIGGER_DISTANCE,

            "actual_trigger_distance_m":
                round(
                    actual_trigger_distance,
                    3
                )
                if actual_trigger_distance is not None
                else "",

            "speed_at_trigger_kmh":
                round(
                    trigger_speed,
                    3
                )
                if trigger_speed is not None
                else "",

            "controller_processing_latency_ms":
                round(
                    controller_latency_ms,
                    3
                )
                if controller_latency_ms is not None
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

            "final_obstacle_distance_m":
                round(
                    final_obstacle_distance,
                    3
                )
                if final_obstacle_distance is not None
                else "",

            "collision":
                collision_detected
        }

        save_result(result)

        return result

    finally:

        # -------------------------------------------------
        # SIIVOUS TESTIEN VÄLISSÄ
        # -------------------------------------------------

        if obstacle_sensor is not None:

            obstacle_sensor.stop()
            obstacle_sensor.destroy()

        if collision_sensor is not None:

            collision_sensor.stop()
            collision_sensor.destroy()

        if ego_vehicle is not None:

            ego_vehicle.destroy()

        if obstacle_vehicle is not None:

            obstacle_vehicle.destroy()

        # Muutama tick jotta actorien poisto ehtii päivittyä
        for _ in range(3):
            world.tick()


# =========================================================
# PÄÄOHJELMA
# =========================================================

def main():

    client = None
    world = None

    original_settings = None

    results = []

    try:

        # -------------------------------------------------
        # CARLA-YHTEYS
        # -------------------------------------------------

        print("Yhdistetään CARLAan...")

        client = carla.Client(
            "localhost",
            2000
        )

        client.set_timeout(10.0)

        world = client.get_world()

        print("Yhteys onnistui.")
        print(
            f"Kartta: {world.get_map().name}"
        )

        # -------------------------------------------------
        # TALLENNETAAN VANHAT ASETUKSET
        # -------------------------------------------------

        original_settings = (
            world.get_settings()
        )

        # -------------------------------------------------
        # SYNCHRONOUS MODE
        # -------------------------------------------------

        settings = world.get_settings()

        settings.synchronous_mode = True

        settings.fixed_delta_seconds = (
            FIXED_DELTA_SECONDS
        )

        world.apply_settings(
            settings
        )

        print()
        print(
            "Synchronous mode käytössä."
        )

        print(
            f"Fixed timestep: "
            f"{FIXED_DELTA_SECONDS:.3f} s"
        )

        print(
            f"Simulation frequency: "
            f"{1 / FIXED_DELTA_SECONDS:.1f} Hz"
        )

        # Kiinteä sää parantaa toistettavuutta
        world.set_weather(
            carla.WeatherParameters.ClearNoon
        )

        blueprint_library = (
            world.get_blueprint_library()
        )

        # -------------------------------------------------
        # ETSITÄÄN YKSI TESTIPAIKKA
        # -------------------------------------------------

        print()
        print(
            "Etsitään testipaikkaa..."
        )

        ego_transform, obstacle_transform = (
            find_test_locations(
                world,
                OBSTACLE_START_DISTANCE
            )
        )

        if ego_transform is None:

            print(
                "Sopivaa testipaikkaa ei löytynyt."
            )

            return

        print(
            "Testipaikka löytyi."
        )

        print()
        print(
            f"Suoritetaan "
            f"{len(TEST_CASES)} testiä..."
        )

        # -------------------------------------------------
        # TESTISARJA
        # -------------------------------------------------

        for test_case in TEST_CASES:

            result = run_test(
                world,
                blueprint_library,
                ego_transform,
                obstacle_transform,
                test_case
            )

            if result is not None:
                results.append(result)

        # -------------------------------------------------
        # YHTEENVETO
        # -------------------------------------------------

        print()
        print()
        print("=" * 75)
        print("AEB TESTISARJAN YHTEENVETO")
        print("=" * 75)

        for result in results:

            print(
                f"{result['test_name']} | "
                f"Throttle {result['throttle']:.2f} | "
                f"Trigger speed "
                f"{result['speed_at_trigger_kmh']} km/h | "
                f"Stopping distance "
                f"{result['stopping_distance_m']} m | "
                f"{result['result']}"
            )

        print("=" * 75)

        pass_count = sum(
            1
            for result in results
            if result["result"] == "PASS"
        )

        fail_count = (
            len(results)
            - pass_count
        )

        print(
            f"PASS: {pass_count}"
        )

        print(
            f"FAIL: {fail_count}"
        )

        print()
        print(
            f"Tulokset tallennettu: "
            f"{RESULTS_FILE}"
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

        # -------------------------------------------------
        # PALAUTETAAN CARLAN ALKUPERÄISET ASETUKSET
        # -------------------------------------------------

        if (
            world is not None
            and original_settings is not None
        ):

            print()
            print(
                "Palautetaan CARLAn "
                "alkuperäiset asetukset..."
            )

            world.apply_settings(
                original_settings
            )

            print(
                "Asetukset palautettu."
            )


if __name__ == "__main__":
    main()