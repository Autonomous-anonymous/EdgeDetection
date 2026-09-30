import carla
import cv2
import numpy as np
import math
import queue
import time
from collections import deque


# Kamerakuvien jono
image_queue = queue.Queue(maxsize=2)

# Viimeiset ohjauskulmat vakautusta varten
angle_history = deque(maxlen=10)


# -------------------------------------------------
# CARLA-KAMERAN CALLBACK
# -------------------------------------------------

def camera_callback(image):

    array = np.frombuffer(
        image.raw_data,
        dtype=np.uint8
    )

    array = array.reshape(
        (image.height, image.width, 4)
    )

    # CARLA = BGRA, OpenCV = BGR
    frame = array[:, :, :3].copy()

    # Poistetaan vanha frame tarvittaessa
    if image_queue.full():
        try:
            image_queue.get_nowait()
        except queue.Empty:
            pass

    try:
        image_queue.put_nowait(frame)
    except queue.Full:
        pass


# -------------------------------------------------
# MUODOSTETAAN YKSI KAISTAVIIVA
# -------------------------------------------------

def make_coordinates(image, line_parameters):

    slope, intercept = line_parameters

    height = image.shape[0]
    width = image.shape[1]

    y1 = height
    y2 = int(height * 0.60)

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    # Estetään koordinaatteja karkaamasta
    # täysin kuvan ulkopuolelle
    x1 = max(-width, min(width * 2, x1))
    x2 = max(-width, min(width * 2, x2))

    return np.array([
        x1,
        y1,
        x2,
        y2
    ])


# -------------------------------------------------
# VASEN JA OIKEA KAISTAVIIVA
# -------------------------------------------------

def average_lane_lines(image, lines):

    left_fit = []
    right_fit = []

    if lines is None:
        return None, None

    width = image.shape[1]

    for line in lines:

        x1, y1, x2, y2 = line.reshape(4)

        # Pystysuora viiva
        if x2 == x1:
            continue

        slope, intercept = np.polyfit(
            (x1, x2),
            (y1, y2),
            1
        )

        # Ohitetaan lähes vaakasuorat viivat
        if abs(slope) < 0.5:
            continue

        midpoint_x = (x1 + x2) / 2

        # Vasemman puolen viiva
        if slope < 0 and midpoint_x < width * 0.65:
            left_fit.append(
                (slope, intercept)
            )

        # Oikean puolen viiva
        elif slope > 0 and midpoint_x > width * 0.35:
            right_fit.append(
                (slope, intercept)
            )

    left_line = None
    right_line = None

    if left_fit:

        left_average = np.average(
            left_fit,
            axis=0
        )

        left_line = make_coordinates(
            image,
            left_average
        )

    if right_fit:

        right_average = np.average(
            right_fit,
            axis=0
        )

        right_line = make_coordinates(
            image,
            right_average
        )

    return left_line, right_line


# -------------------------------------------------
# KONENÄKÖ
# -------------------------------------------------

def process_frame(frame):

    # 1. Harmaasävy
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    # 2. Gaussian blur
    blurred = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    # 3. Canny edge detection
    edges = cv2.Canny(
        blurred,
        50,
        150
    )

    height, width = edges.shape

    # -------------------------------------------------
    # 4. REGION OF INTEREST
    # -------------------------------------------------

    polygon = np.array([[
        (0, height),
        (width, height),
        (int(width * 0.60), int(height * 0.58)),
        (int(width * 0.40), int(height * 0.58))
    ]], np.int32)

    mask = np.zeros_like(edges)

    cv2.fillPoly(
        mask,
        polygon,
        255
    )

    cropped_edges = cv2.bitwise_and(
        edges,
        mask
    )

    # -------------------------------------------------
    # 5. HOUGH LINE TRANSFORM
    # -------------------------------------------------

    lines = cv2.HoughLinesP(
        cropped_edges,
        rho=2,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=30,
        maxLineGap=100
    )

    # -------------------------------------------------
    # 6. VASEN + OIKEA KAISTAVIIVA
    # -------------------------------------------------

    left_line, right_line = average_lane_lines(
        frame,
        lines
    )

    line_image = np.zeros_like(frame)

    # Vasen vihreällä
    if left_line is not None:

        x1, y1, x2, y2 = left_line

        cv2.line(
            line_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            8
        )

    # Oikea vihreällä
    if right_line is not None:

        x1, y1, x2, y2 = right_line

        cv2.line(
            line_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            8
        )

    direction = "UNKNOWN"
    smoothed_angle = None

    # -------------------------------------------------
    # 7. PATH PREDICTION
    # -------------------------------------------------

    if (
        left_line is not None
        and right_line is not None
    ):

        # Kaistojen keskipiste
        target_x = int(
            (
                left_line[2]
                + right_line[2]
            ) / 2
        )

        target_y = int(
            (
                left_line[3]
                + right_line[3]
            ) / 2
        )

        # Kameran/auton oletettu keskikohta
        car_x = width // 2
        car_y = height

        # Punainen ennustettu ajolinja
        cv2.line(
            line_image,
            (car_x, car_y),
            (target_x, target_y),
            (0, 0, 255),
            8
        )

        cv2.circle(
            line_image,
            (target_x, target_y),
            10,
            (0, 0, 255),
            -1
        )

        # -------------------------------------------------
        # 8. OHJAUSKULMAN ARVIO
        # -------------------------------------------------

        dx = target_x - car_x
        dy = car_y - target_y

        steering_angle = math.degrees(
            math.atan2(
                dx,
                dy
            )
        )

        # Vakautus
        angle_history.append(
            steering_angle
        )

        smoothed_angle = (
            sum(angle_history)
            / len(angle_history)
        )

        # -------------------------------------------------
        # 9. SUUNTA
        # -------------------------------------------------

        if smoothed_angle < -3:
            direction = "LEFT"

        elif smoothed_angle > 3:
            direction = "RIGHT"

        else:
            direction = "STRAIGHT"

        cv2.putText(
            line_image,
            f"Direction: {direction}",
            (30, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2
        )

        cv2.putText(
            line_image,
            f"Path angle: {smoothed_angle:.1f} deg",
            (30, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2
        )

    else:

        cv2.putText(
            line_image,
            "Lane detection unavailable",
            (30, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    # -------------------------------------------------
    # 10. ALKUPERÄINEN KUVA + TULOKSET
    # -------------------------------------------------

    result = cv2.addWeighted(
        frame,
        0.8,
        line_image,
        1,
        1
    )

    return result, cropped_edges


# -------------------------------------------------
# PÄÄOHJELMA
# -------------------------------------------------

def main():

    vehicle = None
    camera = None

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

        print("Yhteys onnistui!")
        print(
            f"Kartta: {world.get_map().name}"
        )

        blueprint_library = (
            world.get_blueprint_library()
        )

        # -------------------------------------------------
        # AJONEUVO
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
        # RGB-KAMERA
        # -------------------------------------------------

        camera_bp = blueprint_library.find(
            "sensor.camera.rgb"
        )

        camera_bp.set_attribute(
            "image_size_x",
            "960"
        )

        camera_bp.set_attribute(
            "image_size_y",
            "540"
        )

        camera_bp.set_attribute(
            "fov",
            "90"
        )

        camera_bp.set_attribute(
            "sensor_tick",
            "0.05"
        )

        camera_transform = carla.Transform(

            carla.Location(
                x=1.5,
                y=0.0,
                z=2.2
            ),

            carla.Rotation(
                pitch=-5.0
            )
        )

        camera = world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )

        camera.listen(
            camera_callback
        )

        print(
            f"Etukamera luotu. ID: {camera.id}"
        )

        # -------------------------------------------------
        # AUTOPILOT
        # -------------------------------------------------

        vehicle.set_autopilot(True)

        print("Autopilot käynnistetty.")
        print("Live path prediction käynnissä.")
        print("Paina Q lopettaaksesi.")

        time.sleep(1)

        # -------------------------------------------------
        # PÄÄSILMUKKA
        # -------------------------------------------------

        while True:

            try:

                frame = image_queue.get(
                    timeout=2.0
                )

            except queue.Empty:

                print(
                    "Kamerakuvaa ei saatu."
                )

                continue

            # OpenCV-analyysi
            result, edges = process_frame(
                frame
            )

            # -------------------------------------------------
            # AJONEUVON NOPEUS
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

            cv2.putText(
                result,
                f"Speed: {speed:.1f} km/h",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2
            )

            # -------------------------------------------------
            # NÄYTETÄÄN TULOKSET
            # -------------------------------------------------

            cv2.imshow(
                "CARLA Path Prediction",
                result
            )

            cv2.imshow(
                "CARLA Edges",
                edges
            )

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