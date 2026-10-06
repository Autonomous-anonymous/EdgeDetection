import json
import zmq


SENSOR_ADDRESS = "tcp://127.0.0.1:5555"
CONTROL_ADDRESS = "tcp://127.0.0.1:5556"

AEB_TRIGGER_DISTANCE_M = 10.0


def main():
    context = zmq.Context()

    # Vastaanottaa sensoridataa ajoneuvolta/simulaattorilta
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.connect(SENSOR_ADDRESS)
    sensor_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Lähettää controllerin päätökset takaisin
    control_socket = context.socket(zmq.PUB)
    control_socket.bind(CONTROL_ADDRESS)

    print("AEB controller käynnissä")
    print(f"Sensor data: {SENSOR_ADDRESS}")
    print(f"Control output: {CONTROL_ADDRESS}")
    print(f"AEB trigger: {AEB_TRIGGER_DISTANCE_M:.1f} m")
    print()

    try:
        while True:
            message = sensor_socket.recv_string()

            data = json.loads(message)

            speed_kmh = data["speed_kmh"]
            distance_m = data["obstacle_distance_m"]

            brake = 0.0
            aeb_active = False

            if (
                distance_m is not None
                and distance_m <= AEB_TRIGGER_DISTANCE_M
            ):
                brake = 1.0
                aeb_active = True

            command = {
                "brake": brake,
                "aeb_active": aeb_active,
            }

            control_socket.send_string(
                json.dumps(command)
            )

            if distance_m is None:
                distance_text = "no obstacle"
            else:
                distance_text = f"{distance_m:.2f} m"

            print(
                f"Speed: {speed_kmh:5.1f} km/h | "
                f"Distance: {distance_text:>12} | "
                f"Brake: {brake:.1f}",
                end="\r",
            )

    except KeyboardInterrupt:
        print()
        print("Controller pysäytetty.")

    finally:
        sensor_socket.close()
        control_socket.close()
        context.term()


if __name__ == "__main__":
    main()