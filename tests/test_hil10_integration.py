"""Deterministic controller-loop regressions; no Gazebo physics or CAN device.

Only Gazebo transport and message imports are stubbed. The existing route
helpers, HIL-10 command gate, execution loop and stop verification are real.
"""

import contextlib
import importlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

from test_hil10_safety import Clock, FakeSocket, brake
from hil10_safety import HILLink, OdometryMonitor, RunLog


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0, y=0, z=0)
        self.angular = types.SimpleNamespace(x=0, y=0, z=0)


class Publisher:
    def __init__(self):
        self.commands = []

    def publish(self, msg):
        self.commands.append((msg.linear.x, msg.angular.z))


def load_adapters():
    modules = {name: types.ModuleType(name) for name in ("gz", "gz.msgs10", "gz.transport13")}
    modules["gz.transport13"].Node = object
    for name, cls in (("twist", Twist), ("odometry", type("Odometry", (), {})),
                      ("laserscan", type("LaserScan", (), {})), ("pose_v", type("Pose_V", (), {}))):
        module = types.ModuleType("gz.msgs10." + name + "_pb2")
        setattr(module, cls.__name__, cls)
        modules[module.__name__] = module
    with patch.dict(sys.modules, modules):
        b = importlib.import_module("ruskotunturi_hil10_b_test")
        hybrid = importlib.import_module("ruskotunturi_hil10_hybrid_test")
        runtime = importlib.import_module("hil10_runtime")
    return b, hybrid, runtime


B, HYBRID, RUNTIME = load_adapters()


class Recorder:
    def __init__(self):
        self.rows = []

    def record(self, *args, **kwargs):
        self.rows.append((args, kwargs))


class LoopRig:
    """A command-following stand-in, deliberately not a vehicle physics model."""

    def __init__(self, mode, *, ignore_stop=False, dead_observer=False,
                 initial_fault=False, false_goal=False, late_sensor=False,
                 clearance_s=1.0, move_after_confirmation=False):
        self.clock = Clock()
        self.tick = 0
        self.commands = []
        self.ignore_stop = ignore_stop
        self.dead_observer = dead_observer
        self.initial_fault = initial_fault
        self.false_goal = false_goal
        self.late_sensor = late_sensor
        self.clearance_s = clearance_s
        self.move_after_confirmation = move_after_confirmation
        obj = self.obj = B.HIL10B.__new__(B.HIL10B)
        obj.mode = mode
        obj.fault_after = 0.20
        obj.link = HILLink(sock=FakeSocket(), clock=self.clock)
        obj.observer = OdometryMonitor(self.clock)
        obj.drop_odometry = threading.Event()
        obj.lidar = types.SimpleNamespace(enabled=True, error=None)
        obj.lidar.suspend = lambda: setattr(obj.lidar, "enabled", False)
        obj.log = Recorder()
        obj.injected = obj.detected = obj.confirmed_stop = None
        obj.measurements = {}
        obj.stop_verified = False
        obj.wait_ready = lambda: True
        obj.pose_fresh = lambda: True
        obj.publish_b = lambda speed, steering: self.commands.append((self.clock(), speed, steering))
        obj.stop_b = lambda: obj.publish_b(0.0, 0.0)
        obj.route_command = self.route_command
        self.refresh(0)

    def route_command(self):
        goal = self.false_goal or (self.obj.mode == "route" and self.tick >= 6)
        return (0.0 if goal else 0.85), 0.1, goal, self.clearance_s, 0.0

    def refresh(self, seconds):
        self.clock.advance(seconds)
        self.tick += 1
        speed = self.commands[-1][1] if self.commands else 0.85
        if self.ignore_stop:
            speed = 0.85
        if self.move_after_confirmation and self.obj.confirmed_stop is not None:
            speed = 0.25
        stamp = (self.tick, 0)
        if not self.dead_observer or self.tick == 1:
            self.obj.observer.update(speed, stamp)
        if not self.obj.drop_odometry.is_set():
            self.obj.link.odom.update(speed, stamp)
        elapsed = self.clock() - self.obj.injected if self.obj.injected is not None else 0
        reason = 0
        if self.initial_fault or (not self.obj.lidar.enabled and elapsed >= (1.2 if self.late_sensor else 0.55)):
            reason = 3
        if self.obj.mode == "aeb" and self.tick >= 6:
            reason = 2
        self.obj.link.sock.incoming.append(brake(self.tick % 256, int(reason != 0), reason))

    def run(self):
        clock_module = types.SimpleNamespace(monotonic=self.clock, sleep=self.refresh)
        with patch.object(B, "time", clock_module), patch.object(RUNTIME, "time", clock_module), contextlib.redirect_stdout(io.StringIO()):
            result = self.obj.execute()
        return result


class ExecutionTests(unittest.TestCase):
    def test_route_requires_and_obtains_post_command_stop_samples(self):
        rig = LoopRig("route")
        self.assertEqual(rig.run(), (True, "route_complete"))
        self.assertGreaterEqual(rig.obj.observer.stop_samples, 3)
        self.assertTrue(rig.obj.stop_verified)

    def test_route_cannot_pass_while_still_moving(self):
        self.assertEqual(LoopRig("route", ignore_stop=True).run(), (False, "stop_not_verified"))

    def test_all_three_injected_faults_stop_and_hold(self):
        for mode in ("fault-odometry", "fault-brake", "fault-lidar"):
            with self.subTest(mode=mode):
                rig = LoopRig(mode)
                self.assertEqual(rig.run(), (True, "fault_stop_and_hold_verified"))
                self.assertIsNotNone(rig.obj.detected)
                self.assertLessEqual(rig.obj.detected - rig.obj.injected, 1.0)
                self.assertLessEqual(rig.obj.confirmed_stop - rig.obj.injected, 3.0)
                self.assertTrue(rig.obj.stop_verified)
                commands = [c[1:] for c in rig.commands if c[0] >= rig.obj.detected]
                self.assertTrue(commands)
                self.assertTrue(all(c == (0, 0) for c in commands))

    def test_fault_detection_without_stop_is_fail(self):
        for mode in ("fault-odometry", "fault-brake", "fault-lidar"):
            with self.subTest(mode=mode):
                self.assertEqual(LoopRig(mode, ignore_stop=True).run(), (False, "fault_stop_not_verified"))

    def test_dead_stop_observer_is_fail(self):
        self.assertEqual(LoopRig("fault-odometry", dead_observer=True).run(),
                         (False, "verification_odometry_timeout"))

    def test_vehicle_moving_again_during_hold_is_fail(self):
        self.assertEqual(LoopRig("fault-brake", move_after_confirmation=True).run(),
                         (False, "stop_not_held"))

    def test_missing_lidar_timeout_reply_is_fail(self):
        self.assertEqual(LoopRig("fault-lidar", late_sensor=True).run(),
                         (False, "fault_not_detected_in_time"))

    def test_arrival_cannot_substitute_for_required_intervention(self):
        self.assertEqual(LoopRig("fault-brake", false_goal=True).run(),
                         (False, "goal_reached_without_required_intervention"))

    def test_normal_aeb_stop_checks_clearance(self):
        rig = LoopRig("aeb")
        self.assertEqual(rig.run(), (True, "aeb_stop_verified"))
        self.assertGreaterEqual(rig.obj.measurements["clearance_m"], 1.0)
        self.assertTrue(rig.obj.stop_verified)

    def test_aeb_stop_below_one_metre_is_fail(self):
        self.assertEqual(LoopRig("aeb", clearance_s=4.0).run(), (False, "clearance_violation"))

    def test_uninjected_fault_during_normal_route_is_fail(self):
        rig = LoopRig("route")
        rig.initial_fault = True
        self.assertEqual(rig.run(), (False, "sensor_timeout"))


class AdapterTests(unittest.TestCase):
    def make_hybrid(self):
        obj = HYBRID.HIL10Hybrid.__new__(HYBRID.HIL10Hybrid)
        obj.link = HILLink(sock=FakeSocket(), clock=Clock())
        obj.log = Recorder()
        obj.lidar = types.SimpleNamespace(error=None, close=lambda: None)
        obj.pub_a, obj.pub_b = Publisher(), Publisher()
        obj.phase6_summary = {}
        obj.end_reason = "both_goals_reached"
        return obj

    def test_hybrid_cannot_pass_without_verified_final_stop(self):
        obj = self.make_hybrid()
        obj.log.finish = lambda passed, *args, **kwargs: passed
        with patch.object(HYBRID.phase6.Phase6TwoVehicleCoordinator, "run", return_value=True), \
                patch.object(HYBRID, "verify_stop", return_value=False), \
                patch.object(HYBRID, "stop_repeatedly", side_effect=lambda stop: stop()), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(obj.run())
        self.assertEqual(obj.pub_b.commands[-1], (0, 0))
        self.assertTrue(obj.link.sock.closed)

    def test_hybrid_preserves_phase6_failure(self):
        obj = self.make_hybrid()
        obj.log.finish = lambda passed, *args, **kwargs: passed
        with patch.object(HYBRID.phase6.Phase6TwoVehicleCoordinator, "run", return_value=False), \
                patch.object(HYBRID, "verify_stop", return_value=True) as verify, \
                patch.object(HYBRID, "stop_repeatedly", side_effect=lambda stop: stop()), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(obj.run())
            verify.assert_not_called()

    def test_hybrid_pass_requires_phase6_and_fresh_stop(self):
        obj = self.make_hybrid()
        obj.link.odom.update(0)
        obj.link.accept_brake(brake())
        obj.log.finish = lambda passed, *args, **kwargs: passed
        with patch.object(HYBRID.phase6.Phase6TwoVehicleCoordinator, "run", return_value=True), \
                patch.object(HYBRID, "verify_stop", return_value=True), \
                patch.object(HYBRID, "stop_repeatedly", side_effect=lambda stop: stop()), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(obj.run())

    def test_odometry_uses_all_linear_components_and_source_stamp(self):
        header = types.SimpleNamespace(HasField=lambda name: True,
                                       stamp=types.SimpleNamespace(sec=2, nsec=3))
        msg = types.SimpleNamespace(header=header,
            twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0, y=-3, z=4)))
        self.assertEqual(RUNTIME.odometry_values(msg), (5.0, (2, 3)))

    def test_hybrid_fault_sends_zero_to_both_vehicles(self):
        obj = HYBRID.HIL10Hybrid.__new__(HYBRID.HIL10Hybrid)
        clock = Clock()
        obj.link = HILLink(sock=FakeSocket(), clock=clock)
        obj.log = Recorder()
        obj.lidar = types.SimpleNamespace(error=None)
        obj.pub_a, obj.pub_b = Publisher(), Publisher()
        obj.link.odom.update(0.85)
        obj.link.accept_brake(brake())
        obj.publish(obj.pub_b, 0.85, 0.1)
        clock.advance(0.6)
        obj.link.accept_brake(brake(2))
        with self.assertRaises(HYBRID.HILFault):
            obj.publish(obj.pub_b, 0.85, 0.1)
        self.assertEqual(obj.pub_a.commands[-1], (0.0, 0.0))
        self.assertEqual(obj.pub_b.commands[-1], (0.0, 0.0))

    def test_stop_verifier_does_not_recount_cached_zero(self):
        clock = Clock()
        monitor = OdometryMonitor(clock)
        monitor.update(0)
        fake_time = types.SimpleNamespace(monotonic=clock, sleep=clock.advance)
        with patch.object(RUNTIME, "time", fake_time):
            self.assertFalse(RUNTIME.verify_stop(monitor, lambda: None, lambda: None))

    def test_json_and_trace_preserve_stop_evidence(self):
        clock = Clock()
        link = HILLink(sock=FakeSocket(), clock=clock)
        link.odom.begin_stop()
        for _ in range(3):
            clock.advance(0.05)
            link.odom.update(0)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            log = RunLog(directory, "unit-test")
            log.record(link, 0, 0, "brake_timeout", link.odom)
            log.finish(True, "offline_sample", stop_verified=link.odom.stopped())
            data = json.loads(log.stem.with_suffix(".json").read_text())
            self.assertTrue(data["stop_verified"])
            self.assertEqual(data["stop_samples_required"], 3)
            import csv
            with log.stem.with_suffix(".csv").open() as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["observer_stop_samples"], "3")


if __name__ == "__main__":
    unittest.main()
