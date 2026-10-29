from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping
from r2100_can_gateway import R2100CANGateway


class GatewaySignalMappingTests(unittest.TestCase):
    def test_default_mapping_matches_current_hil_protocol(self):
        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        self.assertEqual(mapping.can.bitrate, 500000)
        self.assertEqual(mapping.can.gateway_interface, "can0")
        self.assertEqual(mapping.can.controller_interface, "can1")

        self.assertEqual(
            mapping.signal("vehicle_speed").can_id,
            0x100,
        )
        self.assertEqual(
            mapping.signal("obstacle_distance").can_id,
            0x101,
        )
        self.assertEqual(
            mapping.signal("brake_command").can_id,
            0x200,
        )

    @patch("r2100_can_gateway.socket.socket")
    def test_gateway_accepts_mapping_values(self, socket_mock):
        fake_can_socket = MagicMock()
        fake_udp_socket = MagicMock()

        socket_mock.side_effect = [
            fake_can_socket,
            fake_udp_socket,
        ]

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        gateway = R2100CANGateway(
            can_interface=mapping.can.gateway_interface,
            udp_port=15000,
            speed_can_id=mapping.signal("vehicle_speed").can_id,
            distance_can_id=mapping.signal(
                "obstacle_distance"
            ).can_id,
            brake_can_id=mapping.signal("brake_command").can_id,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
            external_perception=True,
        )

        self.assertEqual(gateway.can_interface, "can0")
        self.assertEqual(gateway.speed_can_id, 0x100)
        self.assertEqual(gateway.distance_can_id, 0x101)
        self.assertEqual(gateway.brake_can_id, 0x200)
        self.assertEqual(gateway.mapping_bitrate, 500000)
        self.assertEqual(
            gateway.mapping_profile,
            "ruskotunturi_hil",
        )

        fake_can_socket.bind.assert_called_once_with(("can0",))
        fake_udp_socket.bind.assert_called_once_with(
            ("0.0.0.0", 15000)
        )


if __name__ == "__main__":
    unittest.main()
