from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping
from r2100_lidar_perception import LidarPerceptionCAN


class PerceptionSignalMappingTests(unittest.TestCase):
    def test_default_mapping_matches_perception_side(self):
        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        self.assertEqual(
            mapping.can.gateway_interface,
            "can0",
        )

        self.assertEqual(
            mapping.signal("obstacle_distance").can_id,
            0x101,
        )

    @patch("r2100_lidar_perception.socket.socket")
    def test_perception_accepts_mapping_values(self, socket_mock):
        fake_udp_socket = MagicMock()
        fake_can_socket = MagicMock()

        socket_mock.side_effect = [
            fake_udp_socket,
            fake_can_socket,
        ]

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        perception = LidarPerceptionCAN(
            port=15010,
            can_interface=mapping.can.gateway_interface,
            distance_can_id=mapping.signal(
                "obstacle_distance"
            ).can_id,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
        )

        self.assertEqual(perception.can_interface, "can0")
        self.assertEqual(perception.distance_can_id, 0x101)
        self.assertEqual(perception.mapping_bitrate, 500000)
        self.assertEqual(
            perception.mapping_profile,
            "ruskotunturi_hil",
        )

        fake_udp_socket.bind.assert_called_once_with(
            ("0.0.0.0", 15010)
        )
        fake_can_socket.bind.assert_called_once_with(("can0",))


if __name__ == "__main__":
    unittest.main()
