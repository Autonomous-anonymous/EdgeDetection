"""Pitch geometry, recorded SIL timing, and live command-gate regressions."""

import csv
import importlib
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

from test_hil10_integration import Publisher, Twist
from test_hil10_safety import Clock, FakeSocket, brake
from hil10_safety import OdometryMonitor
from run_pitch_demo import BASE_WORLD, GAZEBO, arguments, make_world


def load_controller():
    modules = {name: types.ModuleType(name) for name in ("gz", "gz.msgs10", "gz.transport13")}
    modules["gz.transport13"].Node = object
    for tag, name in (("twist", "Twist"), ("clock", "Clock"), ("pose_v", "Pose_V"), ("laserscan", "LaserScan"),
                      ("boolean", "Boolean"), ("image", "Image"), ("odometry", "Odometry"), ("stringmsg", "StringMsg")):
        module = types.ModuleType(f"gz.msgs10.{tag}_pb2")
        setattr(module, name, Twist if tag == "twist" else type(name, (), {}))
        modules[module.__name__] = module
    with patch.dict(sys.modules, modules):
        return importlib.import_module("pitch_controller")


pitch = load_controller()


class WorldTests(unittest.TestCase):
    def test_visual_variant_preserves_geometry_topics_and_placement(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.sdf"
            make_world(path)
            world = ET.parse(path).find("world")
        original = ET.parse(BASE_WORLD).find("world")
        self.assertEqual(world.get("name"), original.get("name"))
        self.assertIsNotNone(world.find("gui/plugin[@filename='MinimalScene']/camera_pose"))
        for model in original.findall("model"):
            self.assertEqual(ET.tostring(model), ET.tostring(world.find(f"model[@name='{model.get('name')}']")))
        for name, letter in ((pitch.MODEL_A, "A"), (pitch.MODEL_B, "B")):
            model = world.find(f"model[@name='{name}']")
            base = ET.parse(GAZEBO / "models" / name / "model.sdf").find("model")
            self.assertEqual(model.findtext("pose"), next(i.findtext("pose") for i in original.findall("include") if i.findtext("name") == name))
            for query in (".//collision", ".//joint", ".//sensor", ".//plugin", ".//inertial"):
                self.assertEqual([ET.tostring(x) for x in model.findall(query)], [ET.tostring(x) for x in base.findall(query)])
            chassis = model.find("link[@name='chassis']")
            self.assertTrue(chassis.findall(f"visual[@name='pitch_{letter}_0']"))

    def test_modes_are_explicit(self):
        for args in (["hil"], ["sil", "--r2100-host", "example"]):
            with self.assertRaises(SystemExit), patch("sys.stderr"):
                arguments(args)


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.demo = pitch.PitchDemo.__new__(pitch.PitchDemo)
        d = self.demo
        d.lock = threading.Lock()
        d.events = {}
        d.link = None
        d.panel = Mock()
        d.last_panel = -math.inf
        d.observed = {"final_static_obstacle": (8, 34, 0, 0, 0)}
        d.static_line_ux, d.static_line_uy, d.static_line_nx, d.static_line_ny = d.static_line_geometry()
        d.passed_barrier = d.b_waited = d.b_resumed = False

    def replay_row(self, row, allow_stop=True, allow_resume=True):
        d = self.demo
        d.a = dict(x=float(row["a_x"]), y=float(row["a_y"]), yaw=float(row["a_yaw_rad"]),
                   roll_deg=float(row["a_roll_deg"]), pitch_deg=float(row["a_pitch_deg"]))
        d.camera_used_for_control = d.static_avoidance_started = row["a_mode"] == "STATIC_DETOUR"
        d.moving_started = row["obstacle_started"] == "True"
        d.moving_completed = row["obstacle_completed"] == "True"
        d.b_dynamic_hold = row["b_hold"] == "True"
        # Replay flags stand in for the separate harness. Its pose observer
        # has dedicated tests. This archived CSV contains requests and poses.
        d.encounter_stop_verified = allow_stop and d.b_dynamic_hold and abs(float(row["b_x"]) - 13.18) < .05
        d.encounter_resume_verified = allow_resume and d.moving_completed and float(row["b_x"]) < 12
        d.b_dynamic_resume_events = int(d.encounter_resume_verified)

    def test_measured_opening_on_archived_v64_run(self):
        trace = next((GAZEBO / "results").glob("final_sil_v6_4_trace_*.csv"))
        with trace.open() as source:
            rows = csv.DictReader(source)
            start = float(next(rows)["sim_time_s"])
            for row in rows:
                elapsed = float(row["sim_time_s"]) - start
                self.replay_row(row)
                self.demo.observe(elapsed)
                if self.demo.passed_barrier and self.demo.b_resumed:
                    break
            else:
                self.fail("Recorded opening never met both pitch conditions")
        self.assertLess(self.demo.events["b_stopped_for_crossing"], self.demo.events["b_resumed_after_crossing"])
        self.assertTrue(33 < elapsed < 40, elapsed)

    def test_stop_and_positive_request_alone_do_not_prove_restart(self):
        d = self.demo
        d.a = dict(x=3, y=37, yaw=math.pi, roll_deg=0, pitch_deg=0)
        d.static_avoidance_started = True
        d.camera_used_for_control = False
        d.moving_started = d.moving_completed = True
        d.encounter_stop_verified = d.b_dynamic_hold = True
        d.b_dynamic_resume_events = 0
        d.encounter_resume_verified = False
        d.observe(10)
        d.b_dynamic_hold = False
        d.b_dynamic_resume_events = 1
        d.observe(35)
        self.assertTrue(d.b_waited)
        self.assertFalse(d.b_resumed)
        self.assertFalse(d.passed_barrier)


class CommandGateTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sock = FakeSocket()
        d = self.demo = pitch.PitchDemo.__new__(pitch.PitchDemo)
        d.link = pitch.PitchLink(sock=self.sock, clock=self.clock)
        d.b_odom = d.link.odom
        d.a_odom = OdometryMonitor(self.clock)
        for monitor in (d.a_odom, d.b_odom):
            monitor.update(.6, (1, 0))
        d.lidar = types.SimpleNamespace(error=None)
        d.motion_wall = pitch.time.monotonic()
        d.passed_barrier = d.b_resumed = False
        d.observe = Mock()
        d.pub_a, d.pub_b = Publisher(), Publisher()

    def step(self):
        return self.demo.send_commands(10, (.7, .2), (.6, -.1))

    def test_no_reply_means_no_motion_on_either_vehicle(self):
        self.assertFalse(self.step())
        self.assertEqual(self.demo.end_reason, "brake_timeout")
        self.assertEqual(self.demo.pub_a.commands, [(0, 0)])
        self.assertEqual(self.demo.pub_b.commands, [(0, 0)])

    def test_real_gate_veto_survives_drive_in_same_batch(self):
        self.sock.incoming.append(brake(254))
        self.assertTrue(self.step())
        self.assertEqual(self.demo.pub_b.commands[-1], (.6, -.1))
        self.sock.incoming.extend([brake(255, 1, 2), brake(0)])
        self.assertFalse(self.step())
        self.assertEqual(self.demo.end_reason, "ttc_threshold")
        self.assertEqual(self.demo.pub_a.commands[-1], (0, 0))
        self.assertEqual(self.demo.pub_b.commands[-1], (0, 0))
        self.assertEqual(self.demo.link.replies, 3)

    def test_replayed_frames_cannot_authorize_motion(self):
        self.sock.incoming.append(brake(10))
        self.assertTrue(self.step())
        self.clock.advance(.51)
        for monitor in (self.demo.a_odom, self.demo.b_odom):
            monitor.update(.6, (2, 0))
        self.sock.incoming.append(brake(10))
        self.assertFalse(self.step())
        self.assertEqual(self.demo.end_reason, "brake_timeout")
        self.assertEqual(self.demo.link.rejected_replies, 1)


class FinalStopTests(unittest.TestCase):
    def finish(self, b_keeps_moving):
        clock = Clock()
        d = pitch.PitchDemo.__new__(pitch.PitchDemo)
        d.link, d.mode = None, "sil"
        d.lock = threading.Lock()
        d.a_odom, d.b_odom = OdometryMonitor(clock), OdometryMonitor(clock)
        for monitor in (d.a_odom, d.b_odom):
            monitor.update(0, (1, 0))  # Cached zero before the stop request.
        d.harness = d.watchdog = Mock()
        d.harness.poll.return_value = None
        d.harness_wall = d.clock_wall = clock()
        d.a = d.b = dict(minimum_clearance_seen=2, last_pose_time=clock(), last_lidar_time=clock())
        d.passed_barrier = d.b_waited = d.b_resumed = True
        d.ground_min = 2
        d.moving_pose_failures = 0
        d.safety_ok = lambda: True
        d.end_reason, d.trace_path = "pitch_complete", "pitch_sil_test.csv"
        d.events, d.panel = {}, Mock()

        def new_odometry(dt):
            clock.advance(dt)
            d.harness_wall = d.clock_wall = clock()
            d.a["last_pose_time"] = d.a["last_lidar_time"] = clock()
            d.a_odom.update(0, (2, d.a_odom.serial))
            if b_keeps_moving:
                d.b_odom.update(.3, (2, d.b_odom.serial))
            else:
                d.b_odom.update(0, (2, d.b_odom.serial))

        d.stop_both = Mock()
        with tempfile.TemporaryDirectory() as directory, patch.object(pitch, "time", types.SimpleNamespace(monotonic=clock, sleep=new_odometry)):
            d.trace_directory = Path(directory)
            passed = d.print_result(36)
            return passed, json.loads((d.trace_directory / "pitch_sil_test.json").read_text())

    def test_both_stops_must_be_measured(self):
        passed, report = self.finish(False)
        self.assertTrue(passed)
        self.assertTrue(report["checks"]["b_stopped_at_end"])
        self.assertNotIn("r2100", report)

    def test_ignored_b_stop_fails_the_whole_scene(self):
        passed, report = self.finish(True)
        self.assertFalse(passed)
        self.assertEqual(report["reason"], "b_stopped_at_end")
