from __future__ import annotations

from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "edge-path-project" / "gazebo" / "scripts"

sys.path.insert(0, str(SCRIPTS_DIR))

from hil_startup_diagnostics import StartupReadiness


def readiness(**overrides):
    values = {
        "r2100_host": "100.66.149.39",
        "pose_fresh": True,
        "odometry_fresh": True,
        "brake_reply_fresh": True,
        "brake_active": False,
        "brake_reason": 0,
        "fault": None,
        "lidar_error": None,
    }
    values.update(overrides)
    return StartupReadiness(**values)


class StartupReadinessTests(unittest.TestCase):
    def test_ready_state_passes(self):
        state = readiness()

        self.assertTrue(state.ready())
        self.assertIsNone(state.failure_reason())

    def test_missing_gazebo_data_is_identified(self):
        state = readiness(
            pose_fresh=False,
            odometry_fresh=False,
        )

        self.assertEqual(
            state.failure_reason(),
            "startup_gazebo_not_ready",
        )

    def test_missing_brake_reply_is_identified(self):
        state = readiness(
            brake_reply_fresh=False,
            brake_active=True,
            brake_reason=3,
        )

        self.assertEqual(
            state.failure_reason(),
            "startup_brake_reply_missing",
        )

    def test_r2100_sensor_timeout_is_identified(self):
        state = readiness(
            brake_active=True,
            brake_reason=3,
        )

        self.assertEqual(
            state.failure_reason(),
            "startup_r2100_sensor_timeout",
        )

    def test_udp_error_is_identified(self):
        state = readiness(
            fault="udp_error",
        )

        self.assertEqual(
            state.failure_reason(),
            "startup_udp_error",
        )

    def test_lidar_send_error_is_identified(self):
        state = readiness(
            lidar_error="network unreachable",
        )

        self.assertEqual(
            state.failure_reason(),
            "startup_lidar_send_error",
        )


    def test_report_shows_readiness_state(self):
        state = readiness(
            r2100_host="192.0.2.2",
            pose_fresh=False,
            odometry_fresh=False,
            brake_reply_fresh=False,
            brake_active=True,
            brake_reason=3,
        )

        report = state.format_report()

        self.assertIn("R2100 host           : 192.0.2.2", report)
        self.assertIn("[FAIL] Gazebo pose", report)
        self.assertIn("[FAIL] Gazebo odometry", report)
        self.assertIn("[FAIL] R2100 brake reply", report)
        self.assertIn("sensor_timeout", report)


if __name__ == "__main__":
    unittest.main()
