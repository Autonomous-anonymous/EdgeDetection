import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "edge-path-project" / "hil" / "stage_signal_config.py"
INTERFACE_PATH = ROOT / "edge-path-project" / "hil" / "config" / "ruskotunturi_aeb_interface.json"

spec = importlib.util.spec_from_file_location("stage_signal_config", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class StageSignalConfigTests(unittest.TestCase):
    def setUp(self):
        self.interface = json.loads(INTERFACE_PATH.read_text())

    def test_hil_config_is_generated_from_canonical_interface(self):
        generated = mod.generate_stage_config(self.interface, "hil")
        self.assertEqual(generated["target"], "hil")
        self.assertEqual(generated["profile"], "ruskotunturi_hil")
        self.assertEqual(generated["can"]["bitrate"], 500000)
        self.assertEqual(generated["signals"]["vehicle_speed"]["can_id"], "0x100")
        self.assertEqual(generated["signals"]["obstacle_distance"]["can_id"], "0x101")
        self.assertEqual(generated["signals"]["brake_command"]["can_id"], "0x200")

    def test_sil_config_uses_virtual_sensor_and_actuator_bindings(self):
        generated = mod.generate_stage_config(self.interface, "sil")
        self.assertEqual(generated["profile"], "ruskotunturi_sil")
        self.assertNotIn("can", generated)
        self.assertEqual(
            generated["signals"]["vehicle_speed"]["source"],
            "/model/aeb_vehicle_360_b/odometry",
        )
        self.assertEqual(
            generated["signals"]["obstacle_distance"]["source"],
            "/aeb360b/lidar",
        )
        self.assertEqual(
            generated["signals"]["brake_command"]["source"],
            "/model/aeb_vehicle_360_b/cmd_vel",
        )

    def test_phase_change_is_automatic_not_duplicate_configuration(self):
        changed = copy.deepcopy(self.interface)
        changed["signals"]["vehicle_speed"]["bindings"]["hil"]["can_id"] = "0x155"
        changed["signals"]["vehicle_speed"]["bindings"]["sil"]["source"] = "/new/odom"
        hil = mod.generate_stage_config(changed, "hil")
        sil = mod.generate_stage_config(changed, "sil")
        self.assertEqual(hil["signals"]["vehicle_speed"]["can_id"], "0x155")
        self.assertEqual(sil["signals"]["vehicle_speed"]["source"], "/new/odom")

    def test_missing_stage_binding_fails_closed(self):
        changed = copy.deepcopy(self.interface)
        del changed["signals"]["vehicle_speed"]["bindings"]["hil"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "interface.json"
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(mod.StageConfigError, "bindings.hil"):
                mod.load_interface(path)

    def test_duplicate_hil_can_id_fails_closed(self):
        changed = copy.deepcopy(self.interface)
        changed["signals"]["brake_command"]["bindings"]["hil"]["can_id"] = "0x100"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "interface.json"
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(mod.StageConfigError, "duplicate HIL CAN ID"):
                mod.load_interface(path)

    def test_write_is_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            a = Path(directory) / "a.json"
            b = Path(directory) / "b.json"
            mod.write_stage_config(INTERFACE_PATH, "hil", a)
            mod.write_stage_config(INTERFACE_PATH, "hil", b)
            self.assertEqual(a.read_bytes(), b.read_bytes())


if __name__ == "__main__":
    unittest.main()
