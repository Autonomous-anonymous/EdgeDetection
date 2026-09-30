import carla
import csv
import math
import os
import threading
import time
from datetime import datetime


# =========================================================
# TESTIN ASETUKSET
# =========================================================

OBSTACLE_START_DISTANCE = 35.0   # Este noin 35 m ego-auton edessä
AEB_TRIGGER_DISTANCE = 10.0      # Täysi jarrutus alle 10 m
SENSOR_RANGE = 40.0              # Obstacle sensorin kantama

THROTTLE = 0.45                  # Normaaliajon kaasupyyntö
STOP_SPEED_KMH = 0.5             # Tämän alle tulkitaan pysähtyneeksi

TEST_TIMEOUT = 25.0              # Maksimi testiaika sekunteina

RESULTS_FILE = "results/aeb_results.csv"


# =========================================================
# SENSORIDATA
# =========================================================

latest_obstacle_distance = None
latest_obstacle_name = None
latest_obstacle_timestamp = None

collision_detected = False
collision_with = None

data_lock = threading.Lock()


# =========================================================
# OBSTACLE SENSOR
# =========================================================

def obstacle_callback(event):

    global latest_obstacle_distance
    global latest_obstacle_name
    global latest_obstacle_timestamp

    with data_lock:

        latest_obstacle_distance = event.distance
        latest_obstacle_timestamp = time.perf_counter()

        if event.other_actor is not None:
            latest_obstacle_name = event.other_actor.type_id
        else:
            latest_obstacle_name = "unknown"


# =========================================================
# COLLISION SENSOR
# =========================================================

def collision_callback(event):

    global collision_detected
    global collision_with

    collision_detected = True

    if event.other_actor is not None:
        collision_with = event.other_actor.type_id
    else:
        collision_with = "unknown"


# =========================================================
# NOPEUS
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
# ETSITÄÄN SUORA TESTIPAIKKA
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

        # Tarkistetaan että tie on melko suora
        yaw1 = start_wp.transform.rotation.yaw
        yaw2 = obstacle_wp.transform.rotation.yaw

        yaw_difference = abs(yaw1 - yaw2)

        if yaw_difference > 180:
            yaw_difference = 360 - yaw_difference

        if yaw_difference > 5:
            continue

        # Nostetaan spawn-pisteitä hieman tien pinnasta
        ego_transform = start_wp.transform

        ego_transform.location.z += 0.3

        obstacle_transform = obstacle_wp.transform

        obstacle_transform.location.z += 0.3

        return ego_transform, obstacle_transform

    return None, None


# =========================================================
# CSV-TULOSTEN TALLENNUS
# =========================================================

def save_result(result):

    os.makedirs("results", exist_ok=True)

    file_exists = os.path.isfile(
        RESULTS_FILE
    )

    fieldnames = [
        "timestamp",
        "result",
        "trigger_distance_m",
        "speed_at_trigger_kmh",
        "controller_latency_ms",
        "stopping_time_s",
        "stopping_distance_m",
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
# PÄÄOHJELMA
# =========================================================

def main():

    global latest_obstacle_distance
    global latest_obstacle_name
    global latest_obstacle_timestamp
    global collision_detected
    global collision_with

    ego_vehicle = None
    obstacle_vehicle = None

    obstacle_sensor = None
    collision_sensor = None

    brake_triggered = False

    trigger_position = None
    trigger_time = None
    trigger_speed = None

    controller_latency_ms = None

    stopped_time = None
    stopping_distance = None

    test_result = "FAIL"

    try:

        # =================================================
        # 1. CARLA-YHTEYS
        # =================================================

        print("Yhdistetään CARLAan...")

        client = carla.Client(
            "localhost",
            2000
        )

        client.set_timeout(10.0)

        world = client.get_world()

        print("Yhteys onnistui.")
        print(f"Kartta: {world.get_map().name}")

        blueprint_library = (
            world.get_blueprint_library()
        )

        # =================================================
        # 2. ETSITÄÄN TESTIPAIKKA
        # =================================================

        print("Etsitään suoraa testipaikkaa...")

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

        print("Testipaikka löytyi.")

        # =================================================
        # 3. LUODAAN EGO-AUTO
        # =================================================

        vehicle_bp = blueprint_library.find(
            "vehicle.tesla.model3"
        )

        ego_vehicle = world.try_spawn_actor(
            vehicle_bp,
            ego_transform
        )

        if ego_vehicle is None:

            print(
                "Ego-autoa ei voitu luoda."
            )

            return

        print(
            f"Ego-auto luotu. ID: "
            f"{ego_vehicle.id}"
        )

        # =================================================
        # 4. LUODAAN ESTE-AUTO
        # =================================================

        obstacle_vehicle = world.try_spawn_actor(
            vehicle_bp,
            obstacle_transform
        )

        if obstacle_vehicle is None:

            print(
                "Esteautoa ei voitu luoda."
            )

            return

        print(
            f"Esteauto luotu. ID: "
            f"{obstacle_vehicle.id}"
        )

        # Esteauto pysyy paikallaan
        obstacle_vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0
            )
        )

        # =================================================
        # 5. OBSTACLE SENSOR
        # =================================================

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

        obstacle_transform_sensor = carla.Transform(
            carla.Location(
                x=2.0,
                y=0.0,
                z=1.0
            )
        )

        obstacle_sensor = world.spawn_actor(
            obstacle_bp,
            obstacle_transform_sensor,
            attach_to=ego_vehicle
        )

        obstacle_sensor.listen(
            obstacle_callback
        )

        print(
            f"Obstacle sensor luotu. ID: "
            f"{obstacle_sensor.id}"
        )

        # =================================================
        # 6. COLLISION SENSOR
        # =================================================

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

        print(
            f"Collision sensor luotu. ID: "
            f"{collision_sensor.id}"
        )

        # =================================================
        # 7. TESTI ALKAA
        # =================================================

        print()
        print("=" * 55)
        print("AEB SIL TEST")
        print("=" * 55)

        print(
            f"Esteen alkuetäisyys: "
            f"{OBSTACLE_START_DISTANCE:.1f} m"
        )

        print(
            f"AEB-raja: "
            f"{AEB_TRIGGER_DISTANCE:.1f} m"
        )

        print(
            f"Kaasu: {THROTTLE:.2f}"
        )

        print("=" * 55)
        print()

        test_start = time.perf_counter()

        # =================================================
        # 8. TESTISILMUKKA
        # =================================================

        while True:

            current_time = time.perf_counter()

            elapsed = (
                current_time
                - test_start
            )

            speed_kmh = get_speed_kmh(
                ego_vehicle
            )

            with data_lock:

                obstacle_distance = (
                    latest_obstacle_distance
                )

                obstacle_name = (
                    latest_obstacle_name
                )

                sensor_timestamp = (
                    latest_obstacle_timestamp
                )

            # =============================================
            # TESTIN TIMEOUT
            # =============================================

            if elapsed > TEST_TIMEOUT:

                print()
                print(
                    "TEST TIMEOUT."
                )

                break

            # =============================================
            # NORMAALIAJO ENNEN AEB:TÄ
            # =============================================

            if not brake_triggered:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=THROTTLE,
                        brake=0.0,
                        steer=0.0
                    )
                )

                # =========================================
                # AEB-TRIGGER
                # =========================================

                if (
                    obstacle_distance is not None
                    and obstacle_distance
                    <= AEB_TRIGGER_DISTANCE
                ):

                    brake_triggered = True

                    trigger_time = (
                        time.perf_counter()
                    )

                    trigger_speed = speed_kmh

                    trigger_position = (
                        ego_vehicle
                        .get_location()
                    )

                    if sensor_timestamp is not None:

                        controller_latency_ms = (
                            trigger_time
                            - sensor_timestamp
                        ) * 1000

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
                        f"{obstacle_distance:.2f} m"
                    )

                    print(
                        f"Nopeus: "
                        f"{trigger_speed:.2f} km/h"
                    )

                    if controller_latency_ms is not None:

                        print(
                            "Controller latency: "
                            f"{controller_latency_ms:.2f} ms"
                        )

                    print()

            # =============================================
            # HÄTÄJARRUTUS
            # =============================================

            if brake_triggered:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

                print(
                    f"AEB ACTIVE | "
                    f"Speed: {speed_kmh:5.1f} km/h | "
                    f"Distance: "
                    f"{obstacle_distance if obstacle_distance is not None else -1:5.1f} m",
                    end="\r"
                )

                # =========================================
                # AUTO PYSÄHTYNYT
                # =========================================

                if speed_kmh <= STOP_SPEED_KMH:

                    stopped_time = (
                        time.perf_counter()
                    )

                    stop_position = (
                        ego_vehicle
                        .get_location()
                    )

                    stopping_distance = (
                        trigger_position.distance(
                            stop_position
                        )
                    )

                    stopping_time = (
                        stopped_time
                        - trigger_time
                    )

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

                    if collision_detected:

                        print(
                            "TÖRMÄYS HAVAITTU!"
                        )

                        print(
                            f"Törmäyskohde: "
                            f"{collision_with}"
                        )

                        test_result = "FAIL"

                    else:

                        print(
                            "Ei törmäystä."
                        )

                        test_result = "PASS"

                    break

            # =============================================
            # TERMINAALIN STATUS
            # =============================================

            else:

                if obstacle_distance is None:

                    distance_text = (
                        "ei havaintoa"
                    )

                else:

                    distance_text = (
                        f"{obstacle_distance:.1f} m"
                    )

                print(
                    f"Speed: "
                    f"{speed_kmh:5.1f} km/h | "
                    f"Obstacle: {distance_text}",
                    end="\r"
                )

            time.sleep(0.02)

        # =================================================
        # 9. TESTITULOKSET
        # =================================================

        if trigger_time is not None:
            stopping_time_value = (
                stopped_time - trigger_time
                if stopped_time is not None
                else None
            )
        else:
            stopping_time_value = None

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "result":
                test_result,

            "trigger_distance_m":
                AEB_TRIGGER_DISTANCE,

            "speed_at_trigger_kmh":
                round(trigger_speed, 3)
                if trigger_speed is not None
                else "",

            "controller_latency_ms":
                round(controller_latency_ms, 3)
                if controller_latency_ms is not None
                else "",

            "stopping_time_s":
                round(stopping_time_value, 3)
                if stopping_time_value is not None
                else "",

            "stopping_distance_m":
                round(stopping_distance, 3)
                if stopping_distance is not None
                else "",

            "collision":
                collision_detected
        }

        save_result(
            result
        )

        print()
        print("=" * 55)
        print(
            f"TEST RESULT: {test_result}"
        )
        print("=" * 55)

        print(
            f"Tulos tallennettu: "
            f"{RESULTS_FILE}"
        )

    except KeyboardInterrupt:

        print()
        print("Testi keskeytetty.")

    except Exception as error:

        print()
        print(
            f"Virhe: {error}"
        )

    finally:

        # =================================================
        # 10. SIIVOUS
        # =================================================

        print()
        print(
            "Siivotaan CARLA-objektit..."
        )

        if obstacle_sensor is not None:

            obstacle_sensor.stop()
            obstacle_sensor.destroy()

            print(
                "Obstacle sensor poistettu."
            )

        if collision_sensor is not None:

            collision_sensor.stop()
            collision_sensor.destroy()

            print(
                "Collision sensor poistettu."
            )

        if ego_vehicle is not None:

            ego_vehicle.destroy()

            print(
                "Ego-auto poistettu."
            )

        if obstacle_vehicle is not None:

            obstacle_vehicle.destroy()

            print(
                "Esteauto poistettu."
            )

        print("Valmis.")


if __name__ == "__main__":
    main()