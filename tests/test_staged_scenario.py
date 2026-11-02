import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]

RUNNER_PATH = (
    ROOT
    / "edge-path-project"
    / "gazebo"
    / "scripts"
    / "run_staged_scenario.py"
)

SCENARIO_PATH = (
    ROOT
    / "edge-path-project"
    / "gazebo"
    / "scenarios"
    / "emergency_braking.json"
)


def load_runner_module():
    spec = importlib.util.spec_from_file_location(
        "run_staged_scenario",
        RUNNER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class TestStagedScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load_runner_module()

    def test_emergency_braking_scenario_schema(self):
        data = json.loads(SCENARIO_PATH.read_text())

        self.assertEqual(data["schema_version"], 1)
        self.assertEqual(data["name"], "emergency_braking")

        self.assertEqual(
            data["world"],
            "edge-path-project/gazebo/worlds/"
            "ruskotunturi_hil9e_aeb_world.sdf",
        )

        acceptance = data["acceptance"]

        self.assertTrue(
            acceptance["vehicle_must_move_before_braking"]
        )
        self.assertEqual(
            acceptance["valid_brake_reasons"],
            [
                "emergency_distance",
                "ttc_threshold",
            ],
        )
        self.assertEqual(
            acceptance["stop_speed_mps"],
            0.05,
        )
        self.assertEqual(
            acceptance["stop_confirm_samples"],
            3,
        )
        self.assertEqual(
            acceptance["required_clearance_m"],
            1.0,
        )

    def test_sil_command(self):
        _path, scenario = self.runner.load_scenario(
            "emergency_braking"
        )

        args = argparse.Namespace(
            target="sil",
            r2100_host=None,
        )

        command = self.runner.build_command(
            args,
            scenario,
        )

        self.assertEqual(command[0], sys.executable)
        self.assertTrue(
            command[2].endswith(
                "ruskotunturi_emergency_braking_sil.py"
            )
        )

        self.assertNotIn("--r2100-host", command)

    def test_hil_requires_r2100_host(self):
        _path, scenario = self.runner.load_scenario(
            "emergency_braking"
        )

        args = argparse.Namespace(
            target="hil",
            r2100_host=None,
        )

        with self.assertRaises(SystemExit):
            self.runner.build_command(
                args,
                scenario,
            )

    def test_hil_command(self):
        _path, scenario = self.runner.load_scenario(
            "emergency_braking"
        )

        args = argparse.Namespace(
            target="hil",
            r2100_host="192.0.2.2",
        )

        command = self.runner.build_command(
            args,
            scenario,
        )

        self.assertTrue(
            command[2].endswith(
                "ruskotunturi_hil10_b_test.py"
            )
        )

        self.assertEqual(
            command[-4:],
            [
                "--mode",
                "aeb",
                "--r2100-host",
                "192.0.2.2",
            ],
        )

    def test_sil_dry_run(self):
        completed = subprocess.run(
            [
                sys.executable,
                str(RUNNER_PATH),
                "--scenario",
                "emergency_braking",
                "--target",
                "sil",
                "--dry-run",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            completed.returncode,
            0,
            msg=completed.stdout + completed.stderr,
        )
        self.assertIn(
            "Scenario configuration: PASS",
            completed.stdout,
        )

    def test_hil_dry_run(self):
        completed = subprocess.run(
            [
                sys.executable,
                str(RUNNER_PATH),
                "--scenario",
                "emergency_braking",
                "--target",
                "hil",
                "--r2100-host",
                "192.0.2.2",
                "--dry-run",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            completed.returncode,
            0,
            msg=completed.stdout + completed.stderr,
        )
        self.assertIn(
            "Scenario configuration: PASS",
            completed.stdout,
        )


if __name__ == "__main__":
    unittest.main()
