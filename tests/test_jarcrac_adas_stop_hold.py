"""Offline safety and state-transition checks for the JARCRAC stop/hold trial."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'edge-path-project/gazebo/scripts'))
from jarcrac_adas_stop_hold import (
    trigger_reason, stationary_consistency, prepare_world,
    WORLD_NAME, CMD_TOPIC, CONTACT_TOPIC, STOP_HOLD_SIM_S,
    STOP_TRIGGER_RANGE_M, MAX_DRIVE_DISTANCE,
)


def consistent(**kw):
    args=dict(current_xy=(19.8,39.9), stop_xy=(19.8,39.9),
              current_range_m=3.63, stop_range_m=3.63,
              obstacle_xy=(14.633437,37.316718),
              start_obstacle_xy=(14.633437,37.316718))
    args.update(kw)
    return stationary_consistency(**args)


class TestTrigger(unittest.TestCase):
    def test_conservative_gap_causes_stop(self):
        self.assertEqual(trigger_reason(gap_m=STOP_TRIGGER_RANGE_M, progress_m=.16),
                         'lidar_triggered_stop')
    def test_far_gap_continues(self):
        self.assertEqual(trigger_reason(gap_m=3.85, progress_m=.1), 'continue')
    def test_does_not_trigger_while_stationary(self):
        self.assertEqual(trigger_reason(gap_m=3.69, progress_m=0), 'continue')
    def test_range_below_abort_limit_never_authorizes_drive(self):
        self.assertEqual(trigger_reason(gap_m=2.9, progress_m=.15),
                         'unsafe_measurement')
    def test_distance_limit_without_trigger_is_not_pass(self):
        self.assertEqual(trigger_reason(gap_m=3.9, progress_m=MAX_DRIVE_DISTANCE),
                         'distance_limit_without_obstacle_trigger')
    def test_nan_fails(self):
        self.assertEqual(trigger_reason(gap_m=math.nan, progress_m=.1),
                         'invalid_measurement')
    def test_negative_progress_fails(self):
        self.assertEqual(trigger_reason(gap_m=3.8, progress_m=-.1),
                         'unsafe_measurement')
    def test_simulated_hold_duration(self):
        self.assertEqual(STOP_HOLD_SIM_S, 5.0)


class TestStationary(unittest.TestCase):
    def test_consistent(self):
        self.assertEqual(consistent(), 'consistent')
    def test_obstacle_range_change(self):
        self.assertEqual(consistent(current_range_m=3.9),
                         'obstacle_range_changed_during_hold')
    def test_body_drift(self):
        self.assertEqual(consistent(current_xy=(20.,39.9)),
                         'vehicle_moved_during_hold')
    def test_obstacle_world_pose_change(self):
        self.assertEqual(consistent(obstacle_xy=(15.,37.3)),
                         'known_obstacle_pose_changed')
    def test_invalid_range(self):
        self.assertEqual(consistent(current_range_m=math.inf),
                         'invalid_stationary_measurements')


class TestWorld(unittest.TestCase):
    def test_generated_world_preserves_originals(self):
        # Full repo assets are present on the user's WSL installation; a slim
        # patch-only testing environment legitimately does not have them.
        source = ROOT/'edge-path-project/gazebo/worlds/jarcrac_terrain_obstacle_probe_world.sdf'
        model = ROOT/'edge-path-project/gazebo/models/jarcrac/model.sdf'
        if not source.is_file() or not model.is_file():
            self.skipTest('Full Gazebo model files not available')
        before = (source.read_bytes(), model.read_bytes())
        with tempfile.TemporaryDirectory() as t:
            output = prepare_world(Path(t)/'isolated.sdf')
            world = ET.parse(output).getroot().find('world')
            self.assertEqual(world.get('name'), WORLD_NAME)
            vehicle=world.find("./model[@name='jarcrac']")
            self.assertEqual(vehicle.findtext("./plugin[@name='gz::sim::systems::DiffDrive']/topic"),CMD_TOPIC)
            self.assertEqual(vehicle.findtext("./link[@name='chassis']/sensor[@type='contact']/topic"),CONTACT_TOPIC)
            self.assertIsNotNone(world.find("./model[@name='jarcrac_terrain_test_obstacle']"))
        self.assertEqual((source.read_bytes(),model.read_bytes()),before)


if __name__ == '__main__':
    unittest.main()
