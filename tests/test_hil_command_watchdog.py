import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]

WATCHDOG_PATH = (
    ROOT
    / "edge-path-project"
    / "gazebo"
    / "scripts"
    / "hil_command_watchdog.py"
)


def load_watchdog_module():
    spec = importlib.util.spec_from_file_location(
        "hil_command_watchdog",
        WATCHDOG_PATH,
    )

    module = importlib.util.module_from_spec(spec)

    assert spec.loader is not None
    spec.loader.exec_module(module)

    return module


class TestCommandWatchdogState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_watchdog_module()

    def test_timeout_must_be_positive(self):
        with self.assertRaises(ValueError):
            self.module.CommandWatchdogState(
                timeout_s=0.0
            )

    def test_unarmed_watchdog_does_not_trip(self):
        state = self.module.CommandWatchdogState(
            timeout_s=0.30
        )

        self.assertFalse(
            state.evaluate(100.0)
        )
        self.assertFalse(state.armed)
        self.assertFalse(state.latched)

    def test_command_arms_watchdog(self):
        state = self.module.CommandWatchdogState(
            timeout_s=0.30
        )

        state.observe_command(10.0)

        self.assertTrue(state.armed)
        self.assertEqual(
            state.last_command_time,
            10.0,
        )

    def test_fresh_command_does_not_trip(self):
        state = self.module.CommandWatchdogState(
            timeout_s=0.30
        )

        state.observe_command(10.0)

        self.assertFalse(
            state.evaluate(10.299)
        )

    def test_timeout_latches_stop(self):
        state = self.module.CommandWatchdogState(
            timeout_s=0.30
        )

        state.observe_command(10.0)

        self.assertTrue(
            state.evaluate(10.30)
        )
        self.assertTrue(state.latched)

        # A later command must not release a latched fail-safe.
        state.observe_command(10.31)

        self.assertTrue(
            state.evaluate(10.32)
        )
        self.assertEqual(
            state.last_command_time,
            10.0,
        )


if __name__ == "__main__":
    unittest.main()
