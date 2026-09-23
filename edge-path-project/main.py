import cv2
import numpy as np
import math
import os
from collections import deque


# Tallennetaan viimeiset 10 ohjauskulmaa
angle_history = deque(maxlen=10)


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters

    y1 = image.shape[0]
    y2 = int(y1 * 0.60)

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return np.array([x1, y1, x2, y2])


def average_lane_lines(image, lines):
    left_fit = []
    right_fit = []

    if lines is None:
        return None, None

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)

        # Ohitetaan pystysuorat viivat
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

        # Vasen kaistaviiva
        if slope < 0:
            left_fit.append((slope, intercept))

        # Oikea kaistaviiva
        else:
            right_fit.append((slope, intercept))

    left_line = None
    right_line = None

    if left_fit:
        left_average = np.average(left_fit, axis=0)
        left_line = make_coordinates(
            image,
            left_average
        )

    if right_fit:
        right_average = np.average(right_fit, axis=0)
        right_line = make_coordinates(
            image,
            right_average
        )

    return left_line, right_line


# -------------------------------------------------
# VIDEO
# -------------------------------------------------

video_path = "videos/solidWhiteRight.mp4"

cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Videota ei voitu avata.")
    exit()


# -------------------------------------------------
# OUTPUT-VIDEON ASETUKSET
# -------------------------------------------------

os.makedirs("output", exist_ok=True)

output_path = "output/path_prediction.mp4"

# Haetaan alkuperäisen videon tiedot
fps = cap.get(cv2.CAP_PROP_FPS)

width = int(
    cap.get(cv2.CAP_PROP_FRAME_WIDTH)
)

height = int(
    cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
)

# Jos FPS-arvoa ei saada videosta
if fps <= 0:
    fps = 25

# MP4-koodekki
fourcc = cv2.VideoWriter_fourcc(*"mp4v")

writer = cv2.VideoWriter(
    output_path,
    fourcc,
    fps,
    (width, height)
)

if not writer.isOpened():
    print("Output-videota ei voitu luoda.")
    cap.release()
    exit()


# -------------------------------------------------
# VIDEON KÄSITTELY
# -------------------------------------------------

while True:

    ret, frame = cap.read()

    if not ret:
        break

    # -------------------------------------------------
    # 1. HARMAASÄVY
    # -------------------------------------------------

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    # -------------------------------------------------
    # 2. PEHMENNYS
    # -------------------------------------------------

    blurred = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    # -------------------------------------------------
    # 3. EDGE DETECTION
    # -------------------------------------------------

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
        (int(width * 0.55), int(height * 0.60)),
        (int(width * 0.45), int(height * 0.60))
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
        threshold=50,
        minLineLength=40,
        maxLineGap=100
    )

    # -------------------------------------------------
    # 6. VASEN JA OIKEA KAISTAVIIVA
    # -------------------------------------------------

    left_line, right_line = average_lane_lines(
        frame,
        lines
    )

    line_image = np.zeros_like(frame)

    if left_line is not None:

        x1, y1, x2, y2 = left_line

        cv2.line(
            line_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            8
        )

    if right_line is not None:

        x1, y1, x2, y2 = right_line

        cv2.line(
            line_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            8
        )

    # -------------------------------------------------
    # 7. PATH PREDICTION
    # -------------------------------------------------

    if left_line is not None and right_line is not None:

        target_x = int(
            (left_line[2] + right_line[2]) / 2
        )

        target_y = int(
            (left_line[3] + right_line[3]) / 2
        )

        car_x = width // 2
        car_y = height

        # Ennustettu ajolinja
        cv2.line(
            line_image,
            (car_x, car_y),
            (target_x, target_y),
            (0, 0, 255),
            8
        )

        # Kohdepiste
        cv2.circle(
            line_image,
            (target_x, target_y),
            10,
            (0, 0, 255),
            -1
        )

        # -------------------------------------------------
        # 8. OHJAUSKULMA
        # -------------------------------------------------

        dx = target_x - car_x
        dy = car_y - target_y

        steering_angle = math.degrees(
            math.atan2(dx, dy)
        )

        # -------------------------------------------------
        # 9. OHJAUSKULMAN VAKAUTUS
        # -------------------------------------------------

        angle_history.append(
            steering_angle
        )

        smoothed_angle = (
            sum(angle_history)
            / len(angle_history)
        )

        # -------------------------------------------------
        # 10. AJOSUUNTA
        # -------------------------------------------------

        if smoothed_angle < -3:
            direction = "LEFT"

        elif smoothed_angle > 3:
            direction = "RIGHT"

        else:
            direction = "STRAIGHT"

        # Näytetään suunta
        cv2.putText(
            line_image,
            f"Direction: {direction}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2
        )

        # Näytetään ohjauskulma
        cv2.putText(
            line_image,
            f"Steering angle: {smoothed_angle:.1f} deg",
            (50, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2
        )

    # -------------------------------------------------
    # 11. YHDISTETÄÄN TULOKSET
    # -------------------------------------------------

    result = cv2.addWeighted(
        frame,
        0.8,
        line_image,
        1,
        1
    )

    # -------------------------------------------------
    # 12. TALLENNETAAN FRAME OUTPUT-VIDEOON
    # -------------------------------------------------

    writer.write(result)

    # Näytetään myös ruudulla
    cv2.imshow(
        "Path prediction",
        result
    )

    # Q lopettaa
    if cv2.waitKey(25) & 0xFF == ord("q"):
        break


# -------------------------------------------------
# LOPETUS
# -------------------------------------------------

cap.release()
writer.release()

cv2.destroyAllWindows()

print("Valmis!")
print(f"Video tallennettu: {output_path}")