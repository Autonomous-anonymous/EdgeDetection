import carla
import cv2
import numpy as np
import queue
import time


# Kamerakuvien siirtoon CARLAsta pääsilmukalle
image_queue = queue.Queue(maxsize=2)


def camera_callback(image):
    """
    Muuttaa CARLAn kamerakuvan OpenCV:n käyttämään muotoon.
    CARLA antaa kuvan BGRA-muodossa.
    OpenCV käyttää BGR-muotoa.
    """

    array = np.frombuffer(
        image.raw_data,
        dtype=np.uint8
    )

    array = array.reshape(
        (image.height, image.width, 4)
    )

    # Poistetaan alpha-kanava
    frame = array[:, :, :3].copy()

    # Jos jonossa on vanha kuva, poistetaan se.
    # Näin kameranäkymä pysyy reaaliaikaisena.
    if image_queue.full():
        try:
            image_queue.get_nowait()
        except queue.Empty:
            pass

    try:
        image_queue.put_nowait(frame)
    except queue.Full:
        pass


def main():

    vehicle = None
    camera = None

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
        # 2. VALITAAN AUTO
        # -------------------------------------------------

        # Käytetään aina samaa autoa, jotta testit
        # ovat helpommin toistettavia.
        vehicle_bp = blueprint_library.find(
            "vehicle.tesla.model3"
        )

        print(
            f"Ajoneuvo: {vehicle_bp.id}"
        )

        # -------------------------------------------------
        # 3. LUODAAN AJONEUVO
        # -------------------------------------------------

        spawn_points = (
            world
            .get_map()
            .get_spawn_points()
        )

        if not spawn_points:
            print("Spawn-pisteitä ei löytynyt.")
            return

        # Käydään pisteitä läpi kunnes vapaa löytyy
        for spawn_point in spawn_points:

            vehicle = world.try_spawn_actor(
                vehicle_bp,
                spawn_point
            )

            if vehicle is not None:
                break

        if vehicle is None:
            print("Ajoneuvoa ei voitu luoda.")
            return

        print(
            f"Ajoneuvo luotu. ID: {vehicle.id}"
        )

        # -------------------------------------------------
        # 4. LUODAAN RGB-KAMERA
        # -------------------------------------------------

        camera_bp = blueprint_library.find(
            "sensor.camera.rgb"
        )

        # Kameran resoluutio
        camera_bp.set_attribute(
            "image_size_x",
            "960"
        )

        camera_bp.set_attribute(
            "image_size_y",
            "540"
        )

        # Kameran näkökenttä
        camera_bp.set_attribute(
            "fov",
            "90"
        )

        # Kamera tuottaa kuvan noin 20 kertaa sekunnissa
        camera_bp.set_attribute(
            "sensor_tick",
            "0.05"
        )

        # -------------------------------------------------
        # 5. KAMERAN SIJAINTI AUTOSSA
        # -------------------------------------------------

        camera_transform = carla.Transform(

            # Kamera hieman auton etuosan yläpuolella
            carla.Location(
                x=1.5,
                y=0.0,
                z=2.2
            ),

            # Katsotaan hieman alaspäin
            carla.Rotation(
                pitch=-5.0
            )
        )

        # -------------------------------------------------
        # 6. KIINNITETÄÄN KAMERA AUTOON
        # -------------------------------------------------

        camera = world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )

        print(
            f"Etukamera luotu. ID: {camera.id}"
        )

        # Kun kamera saa uuden kuvan,
        # camera_callback suoritetaan.
        camera.listen(
            camera_callback
        )

        # -------------------------------------------------
        # 7. AUTOPILOT
        # -------------------------------------------------

        vehicle.set_autopilot(True)

        print("Autopilot käynnistetty.")
        print("Etukamera aktiivinen.")
        print("Paina Q kameraikkunassa lopettaaksesi.")

        # Annetaan CARLAlle hetki aikaa
        time.sleep(1)

        # -------------------------------------------------
        # 8. PÄÄSILMUKKA
        # -------------------------------------------------

        while True:

            try:

                # Odotetaan kamerakuvaa
                frame = image_queue.get(
                    timeout=2.0
                )

            except queue.Empty:

                print(
                    "\nKameralta ei saatu kuvaa."
                )

                continue

            # -------------------------------------------------
            # 9. AJONEUVON NOPEUS
            # -------------------------------------------------

            velocity = vehicle.get_velocity()

            speed = (
                3.6
                * (
                    velocity.x ** 2
                    + velocity.y ** 2
                    + velocity.z ** 2
                ) ** 0.5
            )

            # -------------------------------------------------
            # 10. NÄYTETÄÄN NOPEUS KAMERAKUVASSA
            # -------------------------------------------------

            cv2.putText(
                frame,
                f"Speed: {speed:.1f} km/h",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2
            )

            # -------------------------------------------------
            # 11. NÄYTETÄÄN ETUKAMERAN KUVA
            # -------------------------------------------------

            cv2.imshow(
                "CARLA Front Camera",
                frame
            )

            # Näytetään nopeus myös terminaalissa
            print(
                f"Nopeus: {speed:.1f} km/h",
                end="\r"
            )

            # Q lopettaa ohjelman
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:

        print(
            "\nOhjelma lopetetaan."
        )

    except Exception as error:

        print(
            f"\nVirhe: {error}"
        )

    finally:

        # -------------------------------------------------
        # 12. SIIVOTAAN SENSORIT JA AUTO
        # -------------------------------------------------

        print(
            "\nSiivotaan CARLA-objektit..."
        )

        if camera is not None:

            camera.stop()
            camera.destroy()

            print(
                "Kamera poistettu."
            )

        if vehicle is not None:

            vehicle.destroy()

            print(
                "Ajoneuvo poistettu."
            )

        cv2.destroyAllWindows()

        print("Valmis.")


if __name__ == "__main__":
    main()