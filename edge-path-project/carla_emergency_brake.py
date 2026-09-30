import carla
import time
import threading


# -------------------------------------------------
# ASETUKSET
# -------------------------------------------------

OBSTACLE_SENSOR_DISTANCE = 20.0   # Sensori näkee 20 metriin
EMERGENCY_BRAKE_DISTANCE = 8.0   # Hätäjarrutus alle 8 metrissä

latest_obstacle_distance = None
latest_obstacle_name = None

data_lock = threading.Lock()


# -------------------------------------------------
# OBSTACLE SENSOR CALLBACK
# -------------------------------------------------

def obstacle_callback(event):
    """
    Tätä funktiota kutsutaan aina, kun obstacle sensor
    havaitsee jotain auton edessä.
    """

    global latest_obstacle_distance
    global latest_obstacle_name

    with data_lock:
        latest_obstacle_distance = event.distance

        if event.other_actor is not None:
            latest_obstacle_name = event.other_actor.type_id
        else:
            latest_obstacle_name = "unknown"


# -------------------------------------------------
# PÄÄOHJELMA
# -------------------------------------------------

def main():

    vehicle = None
    obstacle_sensor = None

    emergency_brake_active = False

    try:

        # -------------------------------------------------
        # 1. YHDISTETÄÄN CARLAAN
        # -------------------------------------------------

        print("Yhdistetään CARLAan...")

        client = carla.Client(
            "localhost",
            2000
        )

        client.set_timeout(10.0)

        world = client.get_world()

        print("Yhteys onnistui!")
        print(f"Kartta: {world.get_map().name}")

        blueprint_library = (
            world.get_blueprint_library()
        )

        # -------------------------------------------------
        # 2. LUODAAN AJONEUVO
        # -------------------------------------------------

        vehicle_bp = blueprint_library.find(
            "vehicle.tesla.model3"
        )

        spawn_points = (
            world
            .get_map()
            .get_spawn_points()
        )

        for spawn_point in spawn_points:

            vehicle = world.try_spawn_actor(
                vehicle_bp,
                spawn_point
            )

            if vehicle is not None:
                break

        if vehicle is None:

            print(
                "Ajoneuvoa ei voitu luoda."
            )

            return

        print(
            f"Ajoneuvo luotu. ID: {vehicle.id}"
        )

        # -------------------------------------------------
        # 3. LUODAAN OBSTACLE SENSOR
        # -------------------------------------------------

        obstacle_bp = blueprint_library.find(
            "sensor.other.obstacle"
        )

        # Kuinka kauas sensori näkee
        obstacle_bp.set_attribute(
            "distance",
            str(OBSTACLE_SENSOR_DISTANCE)
        )

        # Sensorin tunnistusalueen leveys
        obstacle_bp.set_attribute(
            "hit_radius",
            "0.8"
        )

        # Havaitaan myös staattiset objektit
        obstacle_bp.set_attribute(
            "only_dynamics",
            "false"
        )

        # -------------------------------------------------
        # 4. SENSORIN SIJAINTI
        # -------------------------------------------------

        obstacle_transform = carla.Transform(

            carla.Location(
                x=2.0,
                y=0.0,
                z=1.0
            )
        )

        # -------------------------------------------------
        # 5. KIINNITETÄÄN SENSORI AUTOON
        # -------------------------------------------------

        obstacle_sensor = world.spawn_actor(
            obstacle_bp,
            obstacle_transform,
            attach_to=vehicle
        )

        obstacle_sensor.listen(
            obstacle_callback
        )

        print(
            f"Obstacle sensor luotu. ID: "
            f"{obstacle_sensor.id}"
        )

        print(
            f"Sensorin kantama: "
            f"{OBSTACLE_SENSOR_DISTANCE:.1f} m"
        )

        print(
            f"Hätäjarrutuksen raja: "
            f"{EMERGENCY_BRAKE_DISTANCE:.1f} m"
        )

        # -------------------------------------------------
        # 6. AUTOPILOT
        # -------------------------------------------------

        vehicle.set_autopilot(True)

        print()
        print("Autopilot käynnistetty.")
        print("Emergency Brake Controller aktiivinen.")
        print("Paina Ctrl+C lopettaaksesi.")
        print()

        # -------------------------------------------------
        # 7. PÄÄSILMUKKA
        # -------------------------------------------------

        while True:

            # Ajoneuvon nopeus
            velocity = vehicle.get_velocity()

            speed_ms = (
                velocity.x ** 2
                + velocity.y ** 2
                + velocity.z ** 2
            ) ** 0.5

            speed_kmh = speed_ms * 3.6

            # -------------------------------------------------
            # LUETAAN ESTEEN ETÄISYYS
            # -------------------------------------------------

            with data_lock:

                obstacle_distance = (
                    latest_obstacle_distance
                )

                obstacle_name = (
                    latest_obstacle_name
                )

            # -------------------------------------------------
            # 8. EMERGENCY BRAKING
            # -------------------------------------------------

            if (
                obstacle_distance is not None
                and obstacle_distance
                <= EMERGENCY_BRAKE_DISTANCE
                and not emergency_brake_active
            ):

                print()
                print(
                    "!!! EMERGENCY BRAKE !!!"
                )

                print(
                    f"Este: {obstacle_name}"
                )

                print(
                    f"Etäisyys: "
                    f"{obstacle_distance:.2f} m"
                )

                print(
                    f"Nopeus ennen jarrutusta: "
                    f"{speed_kmh:.1f} km/h"
                )

                # Autopilot pois päältä,
                # jotta meidän controller saa hallinnan
                vehicle.set_autopilot(False)

                # Täysi jarrutus
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

                emergency_brake_active = True

            # -------------------------------------------------
            # PIDETÄÄN JARRU PÄÄLLÄ
            # -------------------------------------------------

            if emergency_brake_active:

                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

                print(
                    f"HÄTÄJARRUTUS | "
                    f"Nopeus: {speed_kmh:.1f} km/h",
                    end="\r"
                )

            # -------------------------------------------------
            # NORMAALIAJO
            # -------------------------------------------------

            elif obstacle_distance is not None:

                print(
                    f"Nopeus: {speed_kmh:5.1f} km/h | "
                    f"Este: {obstacle_distance:5.1f} m",
                    end="\r"
                )

            else:

                print(
                    f"Nopeus: {speed_kmh:5.1f} km/h | "
                    f"Ei estettä",
                    end="\r"
                )

            time.sleep(0.05)

    # -------------------------------------------------
    # CTRL+C
    # -------------------------------------------------

    except KeyboardInterrupt:

        print()
        print("Ohjelma lopetetaan.")

    except Exception as error:

        print()
        print(f"Virhe: {error}")

    # -------------------------------------------------
    # SIIVOUS
    # -------------------------------------------------

    finally:

        print()
        print("Siivotaan CARLA-objektit...")

        if obstacle_sensor is not None:

            obstacle_sensor.stop()
            obstacle_sensor.destroy()

            print(
                "Obstacle sensor poistettu."
            )

        if vehicle is not None:

            vehicle.destroy()

            print(
                "Ajoneuvo poistettu."
            )

        print("Valmis.")


if __name__ == "__main__":
    main()