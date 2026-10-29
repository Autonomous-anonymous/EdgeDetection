from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping
from r2100_aeb_controller import R2100AEBController


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
            speed_can_id=mapping.signal("vehicle_speed").can_id,
            distance_can_id=mapping.signal(
                "obstacle_distance"
            ).can_id,
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


if __name__ == "__main__":
    unittest.main()
