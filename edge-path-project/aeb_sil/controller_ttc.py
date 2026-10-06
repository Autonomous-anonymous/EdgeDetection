import json
import time
import zmq


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = "tcp://127.0.0.1:5555"
CONTROL_ADDRESS = "tcp://127.0.0.1:5556"


# =========================================================
# TTC CONTROLLER CONFIGURATION
# =========================================================

TTC_TRIGGER_S = 1.7

# Alle tämän suhteellisen nopeuden TTC:tä ei lasketa.
MIN_RELATIVE_SPEED_MS = 0.1


# =========================================================
# MAIN
# =========================================================

def main():

    context = zmq.Context()

    # Simulator -> controller
    sensor_socket = context.socket(
        zmq.SUB
    )

    sensor_socket.connect(
        SENSOR_ADDRESS
    )

    sensor_socket.setsockopt_string(
        zmq.SUBSCRIBE,
        ""
    )

    # Controller -> simulator
    control_socket = context.socket(
        zmq.PUB
    )

    control_socket.bind(
        CONTROL_ADDRESS
    )

    print("TTC AEB controller käynnissä")
    print(
        f"Sensor data: "
        f"{SENSOR_ADDRESS}"
    )
    print(
        f"Control output: "
        f"{CONTROL_ADDRESS}"
    )
    print(
        f"TTC trigger: "
        f"{TTC_TRIGGER_S:.2f} s"
    )
    print()

    try:

        while True:

            # =============================================
            # SENSOR DATA
            # =============================================

            message = (
                sensor_socket.recv_string()
            )

            receive_time = (
                time.perf_counter()
            )

            data = json.loads(
                message
            )

            test_id = data.get(
                "test_id"
            )

            sequence = data.get(
                "sequence"
            )

            speed_kmh = float(
                data.get(
                    "speed_kmh",
                    0.0
                )
            )

            distance_m = data.get(
                "obstacle_distance_m"
            )

            # =============================================
            # RELATIVE SPEED
            # =============================================

            # Tässä testissä este on paikallaan.
            # Siksi ego-auton nopeus =
            # suhteellinen lähestymisnopeus.

            relative_speed_ms = (
                speed_kmh / 3.6
            )

            # =============================================
            # TTC CALCULATION
            # =============================================

            ttc_s = None

            if (
                distance_m is not None
                and relative_speed_ms
                > MIN_RELATIVE_SPEED_MS
            ):

                ttc_s = (
                    distance_m
                    / relative_speed_ms
                )

            # =============================================
            # AEB DECISION
            # =============================================

            brake = 0.0
            aeb_active = False

            if (
                ttc_s is not None
                and ttc_s
                <= TTC_TRIGGER_S
            ):

                brake = 1.0
                aeb_active = True

            # =============================================
            # CONTROLLER PROCESSING TIME
            # =============================================

            processing_ms = (
                time.perf_counter()
                - receive_time
            ) * 1000.0

            # =============================================
            # RESPONSE
            # =============================================

            command = {

                "test_id":
                    test_id,

                "sequence":
                    sequence,

                "brake":
                    brake,

                "aeb_active":
                    aeb_active,

                "ttc_s":
                    ttc_s,

                "ttc_threshold_s":
                    TTC_TRIGGER_S,

                "relative_speed_ms":
                    relative_speed_ms,

                "controller_processing_ms":
                    processing_ms
            }

            control_socket.send_string(
                json.dumps(command)
            )

            # =============================================
            # TERMINAL OUTPUT
            # =============================================

            if distance_m is None:

                distance_text = (
                    "no obstacle"
                )

            else:

                distance_text = (
                    f"{distance_m:.2f} m"
                )

            if ttc_s is None:

                ttc_text = (
                    "---"
                )

            else:

                ttc_text = (
                    f"{ttc_s:.2f} s"
                )

            print(
                f"Test: "
                f"{str(test_id):>7} | "
                f"Speed: "
                f"{speed_kmh:5.1f} km/h | "
                f"Distance: "
                f"{distance_text:>10} | "
                f"TTC: "
                f"{ttc_text:>7} | "
                f"Brake: "
                f"{brake:.1f}",
                end="\r"
            )

    except KeyboardInterrupt:

        print()
        print(
            "TTC controller pysäytetty."
        )

    finally:

        sensor_socket.close()
        control_socket.close()
        context.term()


if __name__ == "__main__":
    main()