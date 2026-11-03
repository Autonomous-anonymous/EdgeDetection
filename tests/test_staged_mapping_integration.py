import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "edge-path-project" / "gazebo" / "scripts" / "run_staged_scenario.py"
ADAPTER = ROOT / "edge-path-project" / "gazebo" / "scripts" / "ruskotunturi_emergency_braking_sil_stage.py"
SCENARIO = ROOT / "edge-path-project" / "gazebo" / "scenarios" / "emergency_braking.json"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class StagedMappingIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load(RUNNER, "runner")
        cls.adapter = load(ADAPTER, "adapter")
        cls.scenario = json.loads(SCENARIO.read_text())

    def test_sil_runner_receives_generated_stage_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = self.runner.generate_target_config(
                self.scenario, "sil", Path(directory)
            )
            command = self.runner.build_command(
                argparse.Namespace(target="sil", r2100_host=None),
                self.scenario,
                config_path,
            )
            self.assertIn("--stage-config", command)
            self.assertEqual(Path(command[command.index("--stage-config") + 1]), config_path)

    def test_adapter_applies_generated_topics_to_existing_backend(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.runner.generate_target_config(self.scenario, "sil", Path(directory))
            config = self.adapter.load_stage_config(path)
            backend = types.SimpleNamespace(
                ODOM_TOPIC="old-odom", LIDAR_TOPIC="old-lidar", CMD_B_TOPIC="old-cmd"
            )
            self.adapter.apply_stage_config(backend, config)
            self.assertEqual(backend.ODOM_TOPIC, "/model/aeb_vehicle_360_b/odometry")
            self.assertEqual(backend.LIDAR_TOPIC, "/aeb360b/lidar")
            self.assertEqual(backend.CMD_B_TOPIC, "/model/aeb_vehicle_360_b/cmd_vel")

    def test_hil_stage_config_is_generated_even_though_runtime_is_remote(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.runner.generate_target_config(self.scenario, "hil", Path(directory))
            data = json.loads(path.read_text())
            self.assertEqual(data["target"], "hil")
            self.assertEqual(data["signals"]["brake_command"]["can_id"], "0x200")

    def test_sil_dry_run_exercises_generation(self):
        completed = subprocess.run(
            [sys.executable, str(RUNNER), "--scenario", "emergency_braking", "--target", "sil", "--dry-run"],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("Signal config", completed.stdout)
        self.assertIn("Scenario configuration: PASS", completed.stdout)


if __name__ == "__main__":
    unittest.main()
