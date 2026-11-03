import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HIL = ROOT / "edge-path-project" / "hil"
GENERATOR = HIL / "stage_signal_config.py"
INTERFACE = HIL / "config" / "ruskotunturi_aeb_interface.json"
SNAPSHOT = HIL / "config" / "ruskotunturi_hil.json"

spec = importlib.util.spec_from_file_location("stage_signal_config_snapshot", GENERATOR)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class GeneratedConfigSnapshotTests(unittest.TestCase):
    def test_checked_in_hil_snapshot_has_no_generator_drift(self):
        expected = mod.generate_from_file(INTERFACE, "hil")
        actual = json.loads(SNAPSHOT.read_text())
        self.assertEqual(actual, expected)

    def test_generated_hil_mapping_preserves_validated_baseline(self):
        data = json.loads(SNAPSHOT.read_text())
        self.assertEqual(data["profile"], "ruskotunturi_hil")
        self.assertEqual(
            data["can"],
            {
                "bitrate": 500000,
                "gateway_interface": "can0",
                "controller_interface": "can1",
            },
        )
        expected = {
            "vehicle_speed": ("0x100", "gazebo.vehicle_b.odometry.speed", 0.01),
            "obstacle_distance": ("0x101", "r2100.perception.front_clearance", 0.001),
            "brake_command": ("0x200", "r2100.aeb.brake_command", None),
        }
        for name, (can_id, source, scale) in expected.items():
            signal = data["signals"][name]
            self.assertEqual(signal["can_id"], can_id)
            self.assertEqual(signal["source"], source)
            self.assertEqual(signal.get("scale"), scale)


if __name__ == "__main__":
    unittest.main()
