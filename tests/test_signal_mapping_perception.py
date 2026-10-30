from dataclasses import replace
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import MagicMock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping
from r2100_lidar_perception import (
    CAN_FRAME_FORMAT,
    LidarPerceptionCAN,
)


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
            distance_signal=mapping.signal(
                "obstacle_distance"
            ),
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


    @patch("r2100_lidar_perception.socket.socket")
    def test_perception_distance_encoding_uses_mapping_scale(
        self,
        socket_mock,
    ):
        fake_udp_socket = MagicMock()
        fake_can_socket = MagicMock()

        socket_mock.side_effect = [
            fake_udp_socket,
            fake_can_socket,
        ]

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        distance_signal = replace(
            mapping.signal("obstacle_distance"),
            scale=0.5,
        )

        perception = LidarPerceptionCAN(
            port=15010,
            can_interface=mapping.can.gateway_interface,
            distance_signal=distance_signal,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
        )

        result = perception.send_distance_can(1.5)

        self.assertTrue(result)
        fake_can_socket.send.assert_called_once()

        frame = fake_can_socket.send.call_args.args[0]

        can_id, dlc, data = struct.unpack(
            CAN_FRAME_FORMAT,
            frame,
        )

        self.assertEqual(can_id, 0x101)
        self.assertEqual(dlc, 8)

        # scale=0.5 means 1.5 m -> raw value 3.
        self.assertEqual(data[:2], bytes([3, 0]))


if __name__ == "__main__":
    unittest.main()
