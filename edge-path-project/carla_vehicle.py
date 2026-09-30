import carla
import random
import time


def main():
    vehicle = None

    try:
        # -------------------------------------------------
        # 1. YHDISTETÄÄN CARLAAN
        # -------------------------------------------------

        print("Yhdistetään CARLAan...")

        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)

        world = client.get_world()

        print("Yhteys onnistui!")
        print(f"Kartta: {world.get_map().name}")

        # -------------------------------------------------
        # 2. HAETAAN AJONEUVOT
        # -------------------------------------------------

        blueprint_library = world.get_blueprint_library()

        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            print("Ajoneuvoja ei löytynyt.")
            return

        # Valitaan satunnainen henkilöauto
        suitable_vehicles = []

        for blueprint in vehicle_blueprints:

            if blueprint.has_attribute("number_of_wheels"):

                wheels = int(
                    blueprint.get_attribute(
                        "number_of_wheels"
                    )
                )

                if wheels == 4:
                    suitable_vehicles.append(blueprint)

        if not suitable_vehicles:
            print("Nelipyöräisiä ajoneuvoja ei löytynyt.")
            return

        vehicle_bp = random.choice(
            suitable_vehicles
        )

        print(
            f"Valittu ajoneuvo: {vehicle_bp.id}"
        )

        # -------------------------------------------------
        # 3. HAETAAN SPAWN POINT
        # -------------------------------------------------

        spawn_points = (
            world.get_map().get_spawn_points()
        )

        if not spawn_points:
            print("Spawn-pisteitä ei löytynyt.")
            return

        random.shuffle(spawn_points)

        # -------------------------------------------------
        # 4. YRITETÄÄN LUODA AJONEUVO
        # -------------------------------------------------

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
            f"Ajoneuvo luotu! ID: {vehicle.id}"
        )

        # -------------------------------------------------
        # 5. AUTOPILOT PÄÄLLE
        # -------------------------------------------------

        vehicle.set_autopilot(True)

        print("Autopilot käynnistetty.")
        print("Auto ajaa nyt CARLAssa.")
        print("Paina Ctrl+C lopettaaksesi.")

        # -------------------------------------------------
        # 6. PIDETÄÄN OHJELMA KÄYNNISSÄ
        # -------------------------------------------------

        while True:

            velocity = vehicle.get_velocity()

            speed = (
                3.6
                * (
                    velocity.x ** 2
                    + velocity.y ** 2
                    + velocity.z ** 2
                ) ** 0.5
            )

            print(
                f"Nopeus: {speed:.1f} km/h",
                end="\r"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nOhjelma lopetetaan.")

    except Exception as error:
        print(f"\nVirhe: {error}")

    finally:

        # -------------------------------------------------
        # 7. POISTETAAN AUTO LOPUKSI
        # -------------------------------------------------

        if vehicle is not None:

            print("\nPoistetaan ajoneuvo...")

            vehicle.destroy()

            print("Ajoneuvo poistettu.")


if __name__ == "__main__":
    main()