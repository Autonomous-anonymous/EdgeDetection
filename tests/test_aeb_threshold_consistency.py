import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]

SIL_PATH = (
    ROOT
    / "edge-path-project"
    / "gazebo"
    / "scripts"
    / "ruskotunturi_emergency_braking_sil.py"
)

HIL_PATH = (
    ROOT
    / "edge-path-project"
    / "hil"
    / "r2100_aeb_controller.py"
)


def constants_from_file(path):
    tree = ast.parse(path.read_text())
    values = {}

    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue

        if len(node.targets) != 1:
            continue

        target = node.targets[0]

        if not isinstance(target, ast.Name):
            continue

        try:
            values[target.id] = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            pass

    return values


class TestAEBThresholdConsistency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sil = constants_from_file(SIL_PATH)
        cls.hil = constants_from_file(HIL_PATH)

    def test_ttc_threshold_matches(self):
        self.assertEqual(
            self.sil["TTC_THRESHOLD_S"],
            self.hil["TTC_THRESHOLD_SECONDS"],
        )

    def test_emergency_distance_matches(self):
        self.assertEqual(
            self.sil["EMERGENCY_DISTANCE_M"],
            self.hil["EMERGENCY_DISTANCE_M"],
        )

    def test_sensor_timeout_matches(self):
        self.assertEqual(
            self.sil["SENSOR_TIMEOUT_S"],
            self.hil["SENSOR_TIMEOUT_SECONDS"],
        )

    def test_control_period_matches(self):
        self.assertEqual(
            self.sil["CONTROL_PERIOD_S"],
            self.hil["CONTROL_PERIOD_SECONDS"],
        )

    def test_minimum_ttc_speed_matches(self):
        self.assertEqual(
            self.sil["MIN_SPEED_FOR_TTC_MPS"],
            self.hil["MIN_SPEED_FOR_TTC_MPS"],
        )


if __name__ == "__main__":
    unittest.main()
