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
from r2100_aeb_controller import (
    R2100AEBController,
    pack_can_frame,
)


class AEBSignalMappingTests(unittest.TestCase):
    def test_default_mapping_matches_controller_side(self):
        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        self.assertEqual(
            mapping.can.controller_interface,
            "can1",
        )
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

    @patch("r2100_aeb_controller.socket.socket")
    def test_controller_accepts_mapping_values(self, socket_mock):
        fake_socket = MagicMock()
        socket_mock.return_value = fake_socket

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        controller = R2100AEBController(
            interface=mapping.can.controller_interface,
            speed_signal=mapping.signal("vehicle_speed"),
            distance_signal=mapping.signal(
                "obstacle_distance"
            ),
            brake_can_id=mapping.signal("brake_command").can_id,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
        )

        self.assertEqual(controller.interface, "can1")
        self.assertEqual(controller.speed_can_id, 0x100)
        self.assertEqual(controller.distance_can_id, 0x101)
        self.assertEqual(controller.brake_can_id, 0x200)
        self.assertEqual(controller.mapping_bitrate, 500000)

        fake_socket.bind.assert_called_once_with(("can1",))


    @patch("r2100_aeb_controller.socket.socket")
    def test_controller_decoding_uses_mapping_scales(
        self,
        socket_mock,
    ):
        fake_socket = MagicMock()
        socket_mock.return_value = fake_socket

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        speed_signal = replace(
            mapping.signal("vehicle_speed"),
            scale=0.5,
        )
        distance_signal = replace(
            mapping.signal("obstacle_distance"),
            scale=0.25,
        )

        controller = R2100AEBController(
            interface=mapping.can.controller_interface,
            speed_signal=speed_signal,
            distance_signal=distance_signal,
            brake_can_id=mapping.signal(
                "brake_command"
            ).can_id,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
        )

        speed_data = struct.pack("<H6x", 4)
        distance_data = struct.pack("<H6x", 6)

        controller.handle_frame(
            pack_can_frame(
                speed_signal.can_id,
                speed_data,
            ),
            10.0,
        )

        controller.handle_frame(
            pack_can_frame(
                distance_signal.can_id,
                distance_data,
            ),
            10.0,
        )

        # raw 4 * scale 0.5 = 2.0 m/s
        self.assertAlmostEqual(
            controller.speed_mps,
            2.0,
        )

        # raw 6 * scale 0.25 = 1.5 m
        self.assertAlmostEqual(
            controller.front_distance_m,
            1.5,
        )

        self.assertEqual(controller.last_speed_time, 10.0)
        self.assertEqual(controller.last_distance_time, 10.0)

    @patch("r2100_aeb_controller.socket.socket")
    def test_controller_uses_configured_no_value_raw(
        self,
        socket_mock,
    ):
        fake_socket = MagicMock()
        socket_mock.return_value = fake_socket

        mapping = load_signal_mapping(DEFAULT_CONFIG_PATH)

        distance_signal = replace(
            mapping.signal("obstacle_distance"),
            no_value_raw=12345,
        )

        controller = R2100AEBController(
            interface=mapping.can.controller_interface,
            speed_signal=mapping.signal("vehicle_speed"),
            distance_signal=distance_signal,
            brake_can_id=mapping.signal(
                "brake_command"
            ).can_id,
            mapping_profile=mapping.profile,
            mapping_bitrate=mapping.can.bitrate,
        )

        data = struct.pack("<H6x", 12345)

        controller.handle_frame(
            pack_can_frame(
                distance_signal.can_id,
                data,
            ),
            20.0,
        )

        self.assertIsNone(
            controller.front_distance_m
        )
        self.assertEqual(
            controller.last_distance_time,
            20.0,
        )


if __name__ == "__main__":
    unittest.main()
