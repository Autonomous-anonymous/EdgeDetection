from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))


from r2100_aeb_controller import R2100AEBController
from r2100_can_gateway import (
    R2100CANGateway,
    SENSOR_MAGIC,
    SENSOR_PACKET,
    unpack_can_frame,
)
from r2100_lidar_perception import LidarPerceptionCAN
from signal_mapping import load_signal_mapping


ALT_CONFIG = (
    HIL_DIR
    / "config"
    / "ruskotunturi_hil_alt_scaling.json"
)


class SignalCodecEndToEndTests(unittest.TestCase):
    def test_alternate_scaling_round_trip(self):
        mapping = load_signal_mapping(ALT_CONFIG)

        speed_signal = mapping.signal("vehicle_speed")
        distance_signal = mapping.signal("obstacle_distance")
        brake_signal = mapping.signal("brake_command")

        # Gateway: simulation speed -> CAN
        with patch(
            "r2100_can_gateway.socket.socket"
        ) as socket_mock:
            gateway_can = MagicMock()
            gateway_udp = MagicMock()

            socket_mock.side_effect = [
                gateway_can,
                gateway_udp,
            ]

            gateway = R2100CANGateway(
                can_interface=mapping.can.gateway_interface,
                udp_port=15000,
                speed_signal=speed_signal,
                distance_signal=distance_signal,
                brake_can_id=brake_signal.can_id,
                mapping_profile=mapping.profile,
                mapping_bitrate=mapping.can.bitrate,
                external_perception=True,
            )

            gateway_udp.recvfrom.return_value = (
                SENSOR_PACKET.pack(
                    SENSOR_MAGIC,
                    1,
                    1.2,
                    float("nan"),
                ),
                ("127.0.0.1", 15000),
            )

            gateway.handle_udp()

            speed_frame = gateway_can.send.call_args.args[0]

        speed_can_id, speed_data = unpack_can_frame(
            speed_frame
        )

        self.assertEqual(speed_can_id, 0x100)

        # scale=0.02: 1.2 / 0.02 = raw 60
        self.assertEqual(
            speed_data[:2],
            bytes([60, 0]),
        )

        # Perception: physical clearance -> CAN
        with patch(
            "r2100_lidar_perception.socket.socket"
        ) as socket_mock:
            perception_udp = MagicMock()
            perception_can = MagicMock()

            socket_mock.side_effect = [
                perception_udp,
                perception_can,
            ]

            perception = LidarPerceptionCAN(
                port=15010,
                can_interface=mapping.can.gateway_interface,
                distance_signal=distance_signal,
                mapping_profile=mapping.profile,
                mapping_bitrate=mapping.can.bitrate,
            )

            perception.send_distance_can(1.234)

            distance_frame = (
                perception_can.send.call_args.args[0]
            )

        # AEB: CAN -> physical values
        with patch(
            "r2100_aeb_controller.socket.socket"
        ) as socket_mock:
            controller_socket = MagicMock()
            socket_mock.return_value = controller_socket

            controller = R2100AEBController(
                interface=mapping.can.controller_interface,
                speed_signal=speed_signal,
                distance_signal=distance_signal,
                brake_can_id=brake_signal.can_id,
                mapping_profile=mapping.profile,
                mapping_bitrate=mapping.can.bitrate,
            )

            controller.handle_frame(
                speed_frame,
                10.0,
            )
            controller.handle_frame(
                distance_frame,
                10.0,
            )

        self.assertAlmostEqual(
            controller.speed_mps,
            1.2,
        )

        self.assertAlmostEqual(
            controller.front_distance_m,
            1.234,
        )


if __name__ == "__main__":
    unittest.main()
