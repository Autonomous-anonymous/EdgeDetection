import json
import time

import zmq


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = "tcp://127.0.0.1:5555"
CONTROL_ADDRESS = "tcp://127.0.0.1:5556"


# =========================================================
# TTC CONFIGURATION
# =========================================================

TTC_TRIGGER_S = 1.7

# TTC:tä ei lasketa, jos suhteellinen nopeus
# on tätä pienempi.
MIN_RELATIVE_SPEED_MS = 0.1


# =========================================================
# MAIN
# =========================================================

def main():

    context = zmq.Context()

    # -----------------------------------------------------
    # Simulator -> Controller
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # Controller -> Simulator
    # -----------------------------------------------------

    control_socket = context.socket(
        zmq.PUB
    )

    control_socket.bind(
        CONTROL_ADDRESS
    )

    print(
        "Relative-speed TTC AEB controller käynnissä"
    )

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

            ego_speed_kmh = float(
                data.get(
                    "ego_speed_kmh",
                    0.0
                )
            )

            obstacle_speed_kmh = float(
                data.get(
                    "obstacle_speed_kmh",
                    0.0
                )
            )

            distance_m = data.get(
                "obstacle_distance_m"
            )

            # =============================================
            # RELATIVE SPEED
            # =============================================

            relative_speed_kmh = (
                ego_speed_kmh
                - obstacle_speed_kmh
            )

            relative_speed_ms = (
                relative_speed_kmh
                / 3.6
            )

            # =============================================
            # TTC
            # =============================================

            ttc_s = None

            # TTC lasketaan vain silloin,
            # kun ego lähestyy edellä olevaa ajoneuvoa.
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
            # PROCESSING TIME
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

                "relative_speed_kmh":
                    relative_speed_kmh,

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

                gap_text = "---"

            else:

                gap_text = (
                    f"{distance_m:.2f} m"
                )

            if ttc_s is None:

                ttc_text = "---"

            else:

                ttc_text = (
                    f"{ttc_s:.2f} s"
                )

            print(
                f"{str(test_id):>12} | "
                f"Ego: "
                f"{ego_speed_kmh:5.1f} | "
                f"Obstacle: "
                f"{obstacle_speed_kmh:5.1f} | "
                f"Relative: "
                f"{relative_speed_kmh:5.1f} | "
                f"Gap: "
                f"{gap_text:>9} | "
                f"TTC: "
                f"{ttc_text:>7} | "
                f"Brake: "
                f"{brake:.1f}",
                end="\r"
            )

    except KeyboardInterrupt:

        print()
        print(
            "Relative TTC controller pysäytetty."
        )

    finally:

        sensor_socket.close()
        control_socket.close()
        context.term()


if __name__ == "__main__":
    main()