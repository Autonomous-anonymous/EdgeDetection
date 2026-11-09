"""Offline fail-closed checks for bounded moving JARCRAC obstacle diagnostic."""
from pathlib import Path
import math
from types import SimpleNamespace as NS
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'edge-path-project/gazebo/scripts'))
from jarcrac_imu_dynamic_obstacle import (
    prepare_world, obstacle_sample_status, health_gate,
    ObstacleMonitor, stop_confirmed,
    WORLD_NAME, CMD_TOPIC, CONTACT_TOPIC,
    STOP_SENSOR_GAP_M, MAX_DRIVE_WALL_S, MAX_SPEED_MPS,
    MAX_DRIVE_DISTANCE,
)


def row(*, valid=True, objects=1522, unknown=.08, nearest=3.84,
        imu_nearest=None, paired=True, error=0.02):
    a = dict(valid=valid, objects=objects,
             unknown_fraction=unknown, nearest_m=nearest)
    b = dict(a, nearest_m=nearest if imu_nearest is None else imu_nearest)
    return dict(paired=paired, orientation_error_deg=error,
                exact=a, imu=b)


def gate(status='obstacle_confirmed', **override):
    args=dict(status=status, age_analysis=.05,
              ages=dict(pose=.01, imu=.01, odom=.01, scan=.01),
              contact=False, speed=.1, roll_deg=2., pitch_deg=1.,
              distance=.2, yaw_change_deg=1.)
    args.update(override)
    return health_gate(**args)


class ObstacleLogicTests(unittest.TestCase):
    def test_expected_obstacle(self):
        self.assertEqual(obstacle_sample_status(row()), 'obstacle_confirmed')
        self.assertEqual(gate(), 'safe_to_probe')

    def test_missing_or_unpaired_fails_closed(self):
        self.assertNotEqual(obstacle_sample_status(None), 'obstacle_confirmed')
        self.assertNotEqual(obstacle_sample_status(row(paired=False)), 'obstacle_confirmed')

    def test_invalid_scan_or_missing_obstacle(self):
        self.assertEqual(obstacle_sample_status(row(valid=False)), 'invalid_map_classification')
        self.assertEqual(obstacle_sample_status(row(objects=0)), 'expected_obstacle_missing')
        self.assertEqual(obstacle_sample_status(row(objects=49)), 'expected_obstacle_missing')
        self.assertEqual(obstacle_sample_status(row(nearest=None)), 'invalid_obstacle_range')
        self.assertEqual(obstacle_sample_status(row(nearest=math.inf)), 'invalid_obstacle_range')
        self.assertEqual(obstacle_sample_status(row(nearest=math.nan)), 'invalid_obstacle_range')

    def test_uncertain_or_close_obstacle_stops(self):
        self.assertEqual(obstacle_sample_status(row(unknown=.30)), 'unknown_terrain_excess')
        self.assertEqual(obstacle_sample_status(row(unknown=math.nan)), 'unknown_terrain_excess')
        self.assertEqual(obstacle_sample_status(row(nearest=2.9)), 'obstacle_too_close')
        self.assertEqual(obstacle_sample_status(row(nearest=STOP_SENSOR_GAP_M)), 'obstacle_confirmed')
        self.assertEqual(obstacle_sample_status(row(nearest=3.84, imu_nearest=3.5)),
                         'imu_map_range_disagreement')

    def test_orientation_and_health_gates(self):
        self.assertEqual(obstacle_sample_status(row(error=3)), 'orientation_mismatch')
        self.assertEqual(obstacle_sample_status(row(error=math.nan)), 'invalid_orientation_error')
        self.assertEqual(gate(contact=True), 'chassis_contact')
        self.assertEqual(gate(speed=.3), 'excessive_measured_speed')
        self.assertEqual(gate(pitch_deg=20), 'excessive_tilt')
        self.assertEqual(gate(age_analysis=1.5), 'stale_analysis')
        self.assertEqual(gate(ages=dict(pose=.01, imu=1.0, odom=.01, scan=.01)), 'stale_sensor')
        self.assertEqual(gate(status='expected_obstacle_missing'), 'expected_obstacle_missing')

    def test_old_single_layer_scan_is_rejected(self):
        monitor = ObstacleMonitor.__new__(ObstacleMonitor)
        errors = []
        monitor._err = lambda name, exc: errors.append((name, str(exc)))
        monitor.on_scan(NS(frame='jarcrac::chassis::gpu_lidar_360',
                           count=721, vertical_count=1, ranges=[1.] * 721))
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0][0], 'lidar_shape')
        self.assertIn('721 x 1', errors[0][1])

    def test_stop_needs_distinct_odometry_samples(self):
        class FrozenOdom:
            def command(self, value):
                assert value == 0
            def snapshot(self):
                return dict(odom_count=1, speed=0.0)
        self.assertEqual(stop_confirmed(FrozenOdom(), deadline_wall_s=.16)[0], False)

        class NewOdom(FrozenOdom):
            def __init__(self): self.n=0
            def snapshot(self):
                self.n += 1
                return dict(odom_count=self.n, speed=0.0)
        self.assertEqual(stop_confirmed(NewOdom(), deadline_wall_s=.5), (True, 3))

    def test_motion_bounded(self):
        self.assertLessEqual(MAX_SPEED_MPS, .1)
        self.assertLessEqual(MAX_DRIVE_DISTANCE, .35)
        self.assertLess(MAX_DRIVE_DISTANCE, STOP_SENSOR_GAP_M)
        self.assertLess(MAX_DRIVE_WALL_S, 15)


class GeneratedWorldTests(unittest.TestCase):
    def test_3d_sensor_unique_commands_contact_and_static_box(self):
        source=ROOT/'edge-path-project/gazebo/worlds/jarcrac_terrain_obstacle_probe_world.sdf'
        model=ROOT/'edge-path-project/gazebo/models/jarcrac/model.sdf'
        if not source.is_file() or not model.is_file():
            self.skipTest('Full Gazebo repository fixtures unavailable')
        before=(source.read_bytes(),model.read_bytes())
        with tempfile.TemporaryDirectory() as tmp:
            output=prepare_world(Path(tmp)/'test.sdf')
            world=ET.parse(output).getroot().find('world')
            self.assertEqual(world.get('name'),WORLD_NAME)
            self.assertIsNotNone(world.find("./model[@name='jarcrac']"))
            self.assertIsNotNone(world.find("./model[@name='jarcrac_terrain_test_obstacle']"))
            model_node=world.find("./model[@name='jarcrac']")
            self.assertEqual(model_node.findtext("./plugin[@name='gz::sim::systems::DiffDrive']/topic"),CMD_TOPIC)
            lidar=model_node.find("./link[@name='chassis']/sensor[@name='gpu_lidar_front']")
            self.assertEqual(lidar.findtext('ray/scan/vertical/samples'),'32')
            self.assertEqual(model_node.findtext("./link[@name='chassis']/sensor[@type='contact']/topic"),CONTACT_TOPIC)
            self.assertIsNotNone(world.find("./plugin[@name='gz::sim::systems::Imu']"))
            self.assertIsNotNone(world.find("./plugin[@name='gz::sim::systems::Contact']"))
        self.assertEqual(before,(source.read_bytes(),model.read_bytes()))


if __name__ == '__main__':
    unittest.main()
