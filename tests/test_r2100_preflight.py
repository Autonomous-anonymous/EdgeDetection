from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

sys.path.insert(0, str(HIL_DIR))

import r2100_preflight


class PreflightCANTests(unittest.TestCase):
    @patch("r2100_preflight.subprocess.run")
    def test_can_interface_passes_when_up_active_and_bitrate_matches(
        self,
        run_mock,
    ):
        run_mock.return_value = Mock(
            returncode=0,
            stdout=(
                "5: can0: <NOARP,UP,LOWER_UP> mtu 16 state UP\n"
                "    can state ERROR-ACTIVE restart-ms 100\n"
                "          bitrate 500000 sample-point 0.875\n"
            ),
            stderr="",
        )

        result = r2100_preflight.check_can_interface(
            "can0",
            500000,
        )

        self.assertTrue(result.ok)
        self.assertIn("ERROR-ACTIVE", result.detail)
        self.assertIn("500000", result.detail)

    @patch("r2100_preflight.subprocess.run")
    def test_can_interface_fails_when_interface_is_down(
        self,
        run_mock,
    ):
        run_mock.return_value = Mock(
            returncode=0,
            stdout=(
                "5: can0: <NOARP> mtu 16 state DOWN\n"
                "    can state STOPPED\n"
                "          bitrate 500000\n"
            ),
            stderr="",
        )

        result = r2100_preflight.check_can_interface(
            "can0",
            500000,
        )

        self.assertFalse(result.ok)
        self.assertIn("not UP", result.detail)

    @patch("r2100_preflight.subprocess.run")
    def test_can_interface_fails_on_wrong_bitrate(
        self,
        run_mock,
    ):
        run_mock.return_value = Mock(
            returncode=0,
            stdout=(
                "5: can0: <NOARP,UP,LOWER_UP> mtu 16 state UP\n"
                "    can state ERROR-ACTIVE\n"
                "          bitrate 250000 sample-point 0.875\n"
            ),
            stderr="",
        )

        result = r2100_preflight.check_can_interface(
            "can0",
            500000,
        )

        self.assertFalse(result.ok)
        self.assertIn("250000", result.detail)
        self.assertIn("500000", result.detail)

    @patch("r2100_preflight.subprocess.run")
    def test_can_interface_fails_when_missing(
        self,
        run_mock,
    ):
        run_mock.return_value = Mock(
            returncode=1,
            stdout="",
            stderr='Device "can0" does not exist.',
        )

        result = r2100_preflight.check_can_interface(
            "can0",
            500000,
        )

        self.assertFalse(result.ok)
        self.assertIn("does not exist", result.detail)


if __name__ == "__main__":
    unittest.main()
