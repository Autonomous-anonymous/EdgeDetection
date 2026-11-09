"""Offline TTC budget, fail-closed and isolated-world checks for JARCRAC."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'edge-path-project/gazebo/scripts'))
from jarcrac_adas_braking import BrakeConfig, braking_decision
from jarcrac_adas_ttc_stop_hold import (
    BRAKE_CFG, WORLD_NAME, CMD_TOPIC, CONTACT_TOPIC, MAX_DRIVE_DISTANCE,
    decide_action, prepare_world,
)


class TestDecision(unittest.TestCase):
    def test_far_stationary_target_no_ttc_when_vehicle_stopped(self):
        d = braking_decision(4.0, 0.0, BrakeConfig(ttc_threshold_s=4))
        self.assertFalse(d.brake)
        self.assertEqual(d.reason, 'continue')
        self.assertTrue(math.isinf(d.ttc_s))

    def test_ttc_shorter_as_speed_increases(self):
        cfg = BrakeConfig(ttc_threshold_s=4, assumed_deceleration_mps2=4.,
                          minimum_clearance_m=.1)
        slow = braking_decision(3.0, .25, cfg)
        fast = braking_decision(3.0, 1.0, cfg)
        self.assertFalse(slow.brake)
        self.assertTrue(fast.brake)
        self.assertEqual(fast.reason, 'ttc_threshold')
        self.assertLess(fast.ttc_s, slow.ttc_s)

    def test_stopping_distance_branch(self):
        cfg = BrakeConfig(ttc_threshold_s=.25, assumed_deceleration_mps2=.25)
        d = braking_decision(2.25, 1.0, cfg)
        self.assertTrue(d.brake)
        self.assertEqual(d.reason, 'stopping_distance_budget')
        self.assertAlmostEqual(d.stopping_distance_m, 2.35, places=4)
        self.assertAlmostEqual(d.required_clearance_m, 3.35, places=4)

    def test_minimum_clearance_branch(self):
        d = braking_decision(1.1, .01)
        self.assertTrue(d.brake)
        self.assertEqual(d.reason, 'minimum_clearance')

    def test_invalid_gap_or_speed_fail_closed(self):
        for gap, v in ((math.nan, .1), (math.inf, .1), (-1, .1),
                       (4, math.nan), (4, math.inf), (4, -.1),
                       (None, .1), (4, None), ('bad', .1)):
            with self.subTest(gap=gap, speed=v):
                d = braking_decision(gap, v)
                self.assertTrue(d.brake)
                self.assertEqual(d.reason, 'invalid_brake_inputs')

    def test_invalid_configs_refused(self):
        for cfg in (BrakeConfig(ttc_threshold_s=0),
                    BrakeConfig(assumed_deceleration_mps2=0),
                    BrakeConfig(scanner_uncertainty_m=-.1),
                    BrakeConfig(minimum_clearance_m=math.nan)):
            with self.subTest(config=cfg):
                with self.assertRaises(ValueError):
                    braking_decision(3, .1, cfg)

    def test_stopping_budget_grows_with_speed(self):
        low = braking_decision(6, .1)
        high = braking_decision(6, .5)
        self.assertGreater(high.required_clearance_m, low.required_clearance_m)

    def test_early_experiment_ttc_can_trigger_in_bounded_run(self):
        d = braking_decision(3.6, .1, BRAKE_CFG)
        self.assertEqual(d.reason, 'ttc_threshold')
        self.assertTrue(d.brake)
        self.assertGreater(d.required_clearance_m, 1.0)

    def test_no_safety_gate_ignores_braking_before_progress_threshold(self):
        snap = dict(analysis={'imu': {'nearest_m': 1.0}}, speed=.1)
        reason, decision = decide_action(snap=snap, progress_m=0.0)
        self.assertEqual(reason, 'minimum_clearance')
        self.assertTrue(decision.brake)

    def test_distance_limit_fails_without_braking(self):
        snap = dict(analysis={'imu': {'nearest_m': 4.0}}, speed=.1)
        reason, decision = decide_action(snap=snap,
                                          progress_m=MAX_DRIVE_DISTANCE+.01)
        self.assertEqual(reason, 'distance_limit_without_brake')
        self.assertIsNone(decision)

    def test_no_missing_policy_inputs_can_command_motion(self):
        reason, decision = decide_action(snap={'analysis': None, 'speed': .1},
                                         progress_m=.1)
        self.assertEqual(reason, 'missing_brake_input')
        self.assertIsNone(decision)


class TestWorld(unittest.TestCase):
    def test_isolated_world_generation(self):
        model = ROOT / 'edge-path-project/gazebo/models/jarcrac/model.sdf'
        source = ROOT / 'edge-path-project/gazebo/worlds/jarcrac_terrain_obstacle_probe_world.sdf'
        if not (model.is_file() and source.is_file()):
            self.skipTest('SDF sources not available in offline stub')
        a = model.read_bytes()
        b = source.read_bytes()
        with tempfile.TemporaryDirectory() as td:
            result = prepare_world(Path(td)/'ttc.sdf')
            w = ET.parse(result).getroot().find('world')
            self.assertEqual(w.get('name'), WORLD_NAME)
            self.assertEqual(w.findtext(
                "./model[@name='jarcrac']/plugin[@name='gz::sim::systems::DiffDrive']/topic"),
                CMD_TOPIC)
            self.assertEqual(w.findtext(
                "./model[@name='jarcrac']/link[@name='chassis']/sensor[@type='contact']/topic"),
                CONTACT_TOPIC)
            self.assertEqual(w.findtext(
                "./model[@name='jarcrac']/link[@name='chassis']/sensor[@type='gpu_lidar']"
                "/ray/scan/vertical/samples"), '32')
            self.assertIsNotNone(w.find("./plugin[@name='gz::sim::systems::Imu']"))
            self.assertIsNotNone(w.find("./model[@name='jarcrac_terrain_test_obstacle']"))
        self.assertEqual(a, model.read_bytes())
        self.assertEqual(b, source.read_bytes())


if __name__ == '__main__':
    unittest.main()
