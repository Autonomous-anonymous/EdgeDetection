import carla
import json
import math
import threading
import time
import zmq


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = "tcp://127.0.0.1:5555"
CONTROL_ADDRESS = "tcp://127.0.0.1:5556"


# =========================================================
# TESTIASETUKSET
# =========================================================

# Ajoneuvojen keskipisteiden tavoite-etäisyys testin alussa.
OBSTACLE_CENTER_DISTANCE_M = 35.0

NORMAL_THROTTLE = 0.40

FIXED_DELTA_SECONDS = 0.05

STOP_SPEED_KMH = 0.5


# =========================================================
# COLLISION DATA
# =========================================================

collision_detected = False
collision_with = None

data_lock = threading.Lock()


# =========================================================
# COLLISION CALLBACK
# =========================================================

def collision_callback(event):

    global collision_detected
    global collision_with

    with data_lock:

        collision_detected = True

        if event.other_actor is not None:
            collision_with = event.other_actor.type_id
        else:
            collision_with = "unknown"

    print()
    print()
    print("TÖRMÄYS HAVAITTU!")

    if collision_with is not None:
        print(
            f"Törmäyskohde: {collision_with}"
        )


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
# KULMAERO
# =========================================================

def yaw_difference(yaw1, yaw2):

    difference = abs(yaw1 - yaw2)

    if difference > 180.0:
        difference = 360.0 - difference

    return difference


# =========================================================
# TESTIPAIKAN ETSINTÄ
# =========================================================

def find_test_locations(world, distance_m):

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

        start_transform = start_wp.transform

        forward = start_transform.get_forward_vector()

        valid = True

        # -------------------------------------------------
        # Tarkistetaan tie useasta pisteestä.
        #
        # Tässä EI käytetä Waypoint.next(distance),
        # vaan muodostetaan geometrisesti piste suoraan
        # auton eteen ja projisoidaan se tielle.
        # -------------------------------------------------

        target_wp = None

        for check_distance in [
            10.0,
            20.0,
            30.0,
            distance_m
        ]:

            expected_location = carla.Location(
                x=(
                    start_transform.location.x
                    + forward.x * check_distance
                ),
                y=(
                    start_transform.location.y
                    + forward.y * check_distance
                ),
                z=start_transform.location.z
            )

            check_wp = carla_map.get_waypoint(
                expected_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )

            if check_wp is None:
                valid = False
                break

            if check_wp.is_junction:
                valid = False
                break

            # Projektoidun pisteen pitää olla lähellä
            # geometrisesti odotettua pistettä.
            projection_error = (
                check_wp.transform.location.distance(
                    expected_location
                )
            )

            if projection_error > 2.0:
                valid = False
                break

            angle_error = yaw_difference(
                start_transform.rotation.yaw,
                check_wp.transform.rotation.yaw
            )

            if angle_error > 5.0:
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

        # Etäisyyden täytyy olla lähellä tavoitetta.
        if abs(
            actual_distance - distance_m
        ) > 3.0:
            continue

        ego_transform = carla.Transform(
            carla.Location(
                x=start_transform.location.x,
                y=start_transform.location.y,
                z=start_transform.location.z + 0.3
            ),
            start_transform.rotation
        )

        obstacle_transform = carla.Transform(
            carla.Location(
                x=target_wp.transform.location.x,
                y=target_wp.transform.location.y,
                z=target_wp.transform.location.z + 0.3
            ),
            target_wp.transform.rotation
        )

        return (
            ego_transform,
            obstacle_transform
        )

    return None, None


# =========================================================
# LONGITUDINAALINEN ETÄISYYS
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

    # Projektoidaan ajoneuvojen välinen vektori
    # ego-auton pituusakselille.
    longitudinal_center_distance = (
        delta_x * forward.x
        + delta_y * forward.y
        + delta_z * forward.z
    )

    # CARLAn bounding box extent.x on
    # ajoneuvon puolikas pituus.
    ego_half_length = (
        ego_vehicle.bounding_box.extent.x
    )

    obstacle_half_length = (
        obstacle_vehicle.bounding_box.extent.x
    )

    # Bumper-to-bumper-etäisyys.
    gap = (
        longitudinal_center_distance
        - ego_half_length
        - obstacle_half_length
    )

    return max(
        0.0,
        gap
    )


# =========================================================
# MAIN
# =========================================================

def main():

    global collision_detected
    global collision_with

    ego_vehicle = None
    obstacle_vehicle = None

    collision_sensor = None

    context = None
    sensor_socket = None
    control_socket = None

    world = None
    original_settings = None

    try:

        # =================================================
        # NOLLATAAN COLLISION-TILA
        # =================================================

        with data_lock:

            collision_detected = False
            collision_with = None

        # =================================================
        # ZEROMQ
        # =================================================

        context = zmq.Context()

        # Simulator -> Controller
        sensor_socket = context.socket(
            zmq.PUB
        )

        sensor_socket.bind(
            SENSOR_ADDRESS
        )

        # Controller -> Simulator
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

        print(
            "ZeroMQ käynnistetty."
        )

        print(
            f"Sensor output: "
            f"{SENSOR_ADDRESS}"
        )

        print(
            f"Control input: "
            f"{CONTROL_ADDRESS}"
        )

        print()

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

        blueprint_library = (
            world.get_blueprint_library()
        )

        # =================================================
        # TESTIPAIKKA
        # =================================================

        print()
        print(
            "Etsitään suoraa testipaikkaa..."
        )

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
        # AJONEUVOT
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

        if ego_vehicle is None:

            print(
                "Ego-ajoneuvoa "
                "ei voitu luoda."
            )

            return

        obstacle_vehicle = (
            world.try_spawn_actor(
                vehicle_bp,
                obstacle_transform
            )
        )

        if obstacle_vehicle is None:

            print(
                "Esteajoneuvoa "
                "ei voitu luoda."
            )

            return

        # Esteauto paikalleen.
        obstacle_vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0
            )
        )

        # Muutama tick actorien vakauttamiseksi.
        for _ in range(5):
            world.tick()

        print(
            "Ajoneuvot luotu."
        )

        print(
            f"Ego actor ID: "
            f"{ego_vehicle.id}"
        )

        print(
            f"Obstacle actor ID: "
            f"{obstacle_vehicle.id}"
        )

        center_distance = (
            ego_vehicle.get_location().distance(
                obstacle_vehicle.get_location()
            )
        )

        initial_gap = get_obstacle_gap(
            ego_vehicle,
            obstacle_vehicle
        )

        print(
            f"Actorien keskipiste-etäisyys: "
            f"{center_distance:.2f} m"
        )

        print(
            f"Alkuperäinen bumper gap: "
            f"{initial_gap:.2f} m"
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

        collision_sensor.listen(
            collision_callback
        )

        print(
            "Collision sensor luotu."
        )

        # =================================================
        # ZEROMQ CONNECTION
        # =================================================

        print()
        print(
            "Odotetaan ZeroMQ-yhteyksien "
            "muodostumista..."
        )

        time.sleep(
            1.0
        )

        print()
        print(
            "SIL-testi käynnissä."
        )

        print(
            "Ctrl+C lopettaa."
        )

        print()

        brake_command = 0.0
        aeb_active = False

        vehicle_has_moved = False

        # =================================================
        # TESTISILMUKKA
        # =================================================

        while True:

            # ---------------------------------------------
            # CARLA SIMULATION STEP
            # ---------------------------------------------

            world.tick()

            # ---------------------------------------------
            # VIRTUAL SENSOR DATA
            # ---------------------------------------------

            speed_kmh = get_speed_kmh(
                ego_vehicle
            )

            distance_m = get_obstacle_gap(
                ego_vehicle,
                obstacle_vehicle
            )

            if speed_kmh > 1.0:
                vehicle_has_moved = True

            with data_lock:

                collision_now = (
                    collision_detected
                )

            # =============================================
            # SIMULATOR -> CONTROLLER
            # =============================================

            sensor_data = {

                "speed_kmh":
                    speed_kmh,

                "obstacle_distance_m":
                    distance_m
            }

            sensor_socket.send_string(
                json.dumps(sensor_data)
            )

            # =============================================
            # CONTROLLER -> SIMULATOR
            # =============================================

            try:

                # Luetaan kaikki jonossa olevat viestit,
                # jotta käytetään uusinta komentoa.
                while True:

                    message = (
                        control_socket.recv_string(
                            flags=zmq.NOBLOCK
                        )
                    )

                    command = json.loads(
                        message
                    )

                    brake_command = (
                        float(
                            command.get(
                                "brake",
                                0.0
                            )
                        )
                    )

                    aeb_active = (
                        bool(
                            command.get(
                                "aeb_active",
                                False
                            )
                        )
                    )

            except zmq.Again:
                pass

            # =============================================
            # VEHICLE CONTROL
            # =============================================

            if aeb_active:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=brake_command,
                        steer=0.0
                    )
                )

            else:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=NORMAL_THROTTLE,
                        brake=0.0,
                        steer=0.0
                    )
                )

            # =============================================
            # OUTPUT
            # =============================================

            if aeb_active:

                state_text = (
                    "AEB ACTIVE"
                )

            else:

                state_text = (
                    "DRIVING"
                )

            print(
                f"{state_text:10} | "
                f"Speed: "
                f"{speed_kmh:5.1f} km/h | "
                f"Gap: "
                f"{distance_m:6.2f} m | "
                f"Brake: "
                f"{brake_command:.1f}",
                end="\r"
            )

            # =============================================
            # STOP CONDITION
            # =============================================

            if (
                vehicle_has_moved
                and aeb_active
                and speed_kmh
                <= STOP_SPEED_KMH
            ):

                print()
                print()

                print(
                    "Ajoneuvo pysähtyi "
                    "AEB:n avulla."
                )

                print(
                    f"Loppuetäisyys esteeseen: "
                    f"{distance_m:.2f} m"
                )

                print(
                    f"Törmäys: "
                    f"{'KYLLÄ' if collision_now else 'EI'}"
                )

                break

            # =============================================
            # COLLISION FAIL SAFE
            # =============================================

            if collision_now:

                ego_vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

    except KeyboardInterrupt:

        print()
        print(
            "Simulaatio keskeytetty."
        )

    except Exception as error:

        print()
        print(
            f"Virhe: {error}"
        )

    finally:

        # =================================================
        # CLEANUP
        # =================================================

        print()
        print(
            "Siivotaan CARLA-actorit..."
        )

        if collision_sensor is not None:

            collision_sensor.stop()
            collision_sensor.destroy()

        if ego_vehicle is not None:

            ego_vehicle.destroy()

        if obstacle_vehicle is not None:

            obstacle_vehicle.destroy()

        if (
            world is not None
            and original_settings is not None
        ):

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