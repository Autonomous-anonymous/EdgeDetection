"""Offline checks for wheel-loader bridge shutdown and stale-command watchdog."""
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

SCRIPTS = Path(__file__).resolve().parents[1] / "edge-path-project" / "gazebo" / "scripts"
sys.path.insert(0, str(SCRIPTS))
from wheelloader_cmd_bridge import Bridge  # noqa: E402


class BridgeLifecycleTests(unittest.TestCase):
    def make_bridge(self, last):
        bridge = object.__new__(Bridge)  # No Gazebo dependency for watchdog logic.
        bridge.last = last
        bridge.timeout_s = 0.5
        bridge.lock = threading.Lock()
        bridge.send = Mock()
        return bridge

    def test_stop_event_sends_final_zero_command(self):
        bridge = self.make_bridge(time.monotonic())
        stop = threading.Event()
        stop.set()
        bridge.run(stop)
        bridge.send.assert_called_once_with(0.0, 0.0)

    def test_watchdog_stops_stale_command_then_exits_cleanly(self):
        bridge = self.make_bridge(time.monotonic() - 10.0)

        class OneCycle:
            count = 0
            def wait(self, interval):
                self.count += 1
                return self.count > 1

        bridge.run(OneCycle())
        self.assertEqual(bridge.send.call_count, 2)
        for call in bridge.send.call_args_list:
            self.assertEqual(call.args, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
