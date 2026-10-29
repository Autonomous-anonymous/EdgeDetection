import json
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

from signal_mapping import load_signal_mapping


VALID_CONFIG = {
    "schema_version": 1,
    "profile": "test_profile",
    "can": {
        "bitrate": 500000,
        "gateway_interface": "can0",
        "controller_interface": "can1",
    },
    "signals": {
        "vehicle_speed": {
            "can_id": "0x100",
            "direction": "simulation_to_controller",
            "source": "gazebo.vehicle_b.odometry.speed",
            "encoding": "uint16_le",
            "scale": 0.01,
            "unit": "m/s",
        },
        "obstacle_distance": {
            "can_id": "0x101",
            "direction": "perception_to_controller",
            "source": "r2100.perception.front_clearance",
            "encoding": "uint16_le",
            "scale": 0.001,
            "unit": "m",
            "no_value_raw": 65535,
        },
        "brake_command": {
            "can_id": "0x200",
            "direction": "controller_to_simulation",
            "source": "r2100.aeb.brake_command",
            "encoding": "brake_command_v1",
        },
    },
}


class SignalMappingTests(unittest.TestCase):
    def load_config(self, config):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "mapping.json"
            path.write_text(json.dumps(config))
            return load_signal_mapping(path)

    def test_valid_config_loads(self):
        mapping = self.load_config(VALID_CONFIG)

        self.assertEqual(mapping.schema_version, 1)
        self.assertEqual(mapping.profile, "test_profile")
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

    def test_missing_required_signal_is_rejected(self):
        config = json.loads(json.dumps(VALID_CONFIG))
        del config["signals"]["obstacle_distance"]

        with self.assertRaisesRegex(
            ValueError,
            "missing required signals",
        ):
            self.load_config(config)

    def test_duplicate_can_id_is_rejected(self):
        config = json.loads(json.dumps(VALID_CONFIG))

        config["signals"]["brake_command"]["can_id"] = "0x100"

        with self.assertRaisesRegex(
            ValueError,
            "duplicate CAN ID",
        ):
            self.load_config(config)

    def test_invalid_11_bit_can_id_is_rejected(self):
        config = json.loads(json.dumps(VALID_CONFIG))

        config["signals"]["vehicle_speed"]["can_id"] = "0x800"

        with self.assertRaisesRegex(
            ValueError,
            "classic 11-bit CAN ID",
        ):
            self.load_config(config)

    def test_same_can_interfaces_are_rejected(self):
        config = json.loads(json.dumps(VALID_CONFIG))

        config["can"]["controller_interface"] = "can0"

        with self.assertRaisesRegex(
            ValueError,
            "must be different",
        ):
            self.load_config(config)

    def test_unsupported_schema_version_is_rejected(self):
        config = json.loads(json.dumps(VALID_CONFIG))

        config["schema_version"] = 999

        with self.assertRaisesRegex(
            ValueError,
            "unsupported schema_version",
        ):
            self.load_config(config)


if __name__ == "__main__":
    unittest.main()
