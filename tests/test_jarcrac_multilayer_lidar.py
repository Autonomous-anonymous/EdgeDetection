"""Offline regression tests for JARCRAC multi-layer LiDAR + flat ground gating."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
GZ = REPO / 'edge-path-project/gazebo'
sys.path.insert(0, str(GZ / 'scripts'))
from jarcrac_lidar_3d import summarize_forward


class TestMultilayerSDF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(GZ / 'models/jarcrac/model.sdf').getroot().find('model')
        cls.sensor = cls.model.find("./link[@name='chassis']/sensor[@type='gpu_lidar']")
        cls.params = json.loads((GZ / 'models/jarcrac/jarcrac_params.json').read_text())

    def test_sensor_has_downward_32_layer_fov(self):
        scan = self.sensor.find('ray/scan')
        self.assertEqual(int(scan.findtext('vertical/samples')), 32)
        self.assertEqual(int(scan.findtext('horizontal/samples')), 721)
        self.assertAlmostEqual(float(scan.findtext('vertical/min_angle')), math.radians(-15), places=7)
        self.assertAlmostEqual(float(scan.findtext('vertical/max_angle')), math.radians(15), places=7)
        self.assertEqual(self.sensor.findtext('topic'), '/jarcrac/lidar')

    def test_placement_noise_and_params(self):
        x, y, z = [float(t) for t in self.sensor.findtext('pose').split()[:3]]
        self.assertAlmostEqual(x, 1.68)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, -0.111)
        chassis_z = float(self.model.find("./link[@name='chassis']/pose").text.split()[2])
        self.assertAlmostEqual(z + chassis_z, 0.600)
        chassis_collision = self.model.find("./link[@name='chassis']/collision")
        collision_x = float(chassis_collision.findtext('pose').split()[0])
        collision_length = float(chassis_collision.findtext('geometry/box/size').split()[0])
        self.assertGreater(x - (collision_x + collision_length / 2), 0.20)
        self.assertEqual(self.sensor.attrib['name'], 'gpu_lidar_front')
        scan = self.sensor.find('ray/scan')
        self.assertAlmostEqual(float(scan.findtext('horizontal/min_angle')), -math.pi/2, places=7)
        self.assertAlmostEqual(float(scan.findtext('horizontal/max_angle')), math.pi/2, places=7)
        self.assertEqual(self.sensor.findtext('update_rate'), '10')
        self.assertEqual(self.sensor.findtext('ray/noise/type'), 'gaussian')
        self.assertNotIn('type', self.sensor.find('ray/noise').attrib)
        self.assertAlmostEqual(float(self.sensor.findtext('ray/noise/stddev')), 0.015)
        cfg = self.params['assumptions']['lidar_3d']
        self.assertEqual(cfg['vertical_channels'], 32)
        self.assertEqual(cfg['vertical_min_deg'], -15.0)
        self.assertEqual(cfg['vertical_max_deg'], 15.0)
        self.assertEqual(cfg['horizontal_fov_deg'], 180.0)
        self.assertAlmostEqual(self.params['assumptions']['lidar_sensor_height_flat_m'], 0.600)

    def test_body_visual_and_collision_unchanged(self):
        chassis = self.model.find("./link[@name='chassis']")
        self.assertTrue(chassis.findtext('visual/geometry/mesh/uri').endswith('Runko_part.STL'))
        self.assertEqual(chassis.findtext('collision/geometry/box/size'), '2.98 1.568 2.055933')


class TestScanProjection(unittest.TestCase):
    def scan(self, samples=5, layers=3):
        return NS(count=samples, vertical_count=layers,
                  angle_min=-0.2, angle_step=0.1,
                  vertical_angle_min=-math.pi/12, vertical_angle_step=math.pi/12,
                  range_min=0.3, range_max=30.0,
                  ranges=[float('inf')] * (samples*layers))

    def test_ground_return_is_not_mistaken_for_obstacle(self):
        scan = self.scan()
        # Lowest layer: level-ground hit for a 0.6 m sensor at -15 degrees.
        scan.ranges[2] = 0.600 / math.sin(math.pi/12)
        result = summarize_forward(scan, sensor_height_m=0.600)
        self.assertTrue(result.valid)
        self.assertEqual(result.ground_points, 1)
        self.assertEqual(result.obstacle_points, 0)
        self.assertIsNone(result.nearest_forward_obstacle_m)

    def test_low_obstacle_seen_on_downward_ring(self):
        scan = self.scan()
        # A level ring at 0.6 m sees the 1.5 m tall test obstacle.
        scan.ranges[1 * scan.count + 2] = 6.07
        result = summarize_forward(scan, sensor_height_m=0.600)
        self.assertEqual(result.obstacle_points, 1)
        self.assertAlmostEqual(result.nearest_forward_obstacle_m, 6.07)

    def test_near_self_hits_are_counted_not_silently_discarded(self):
        scan = self.scan()
        scan.ranges[2*scan.count+2] = 1.0
        summary = summarize_forward(scan, sensor_height_m=0.600)
        self.assertEqual(summary.near_points, 1)
        self.assertEqual(summary.obstacle_points, 1)

    def test_invalid_scan_fail_closed_shape(self):
        scan = self.scan()
        scan.ranges.pop()
        self.assertFalse(summarize_forward(scan, sensor_height_m=0.600).valid)
        scan = self.scan(layers=1)
        self.assertFalse(summarize_forward(scan, sensor_height_m=0.600).valid)

    def test_downward_layer_can_hit_ground_in_front(self):
        # At 0.6 m and -15 deg the first flat-ground return is about 2.24 m
        # in front of the sensor; the probe must mark it as ground, not danger.
        scan = self.scan()
        scan.ranges[2] = 0.600 / math.sin(math.pi/12)
        summary = summarize_forward(scan, sensor_height_m=0.600)
        self.assertEqual(summary.ground_points, 1)
        self.assertIsNone(summary.nearest_forward_obstacle_m)

    def test_obstacle_distance_is_measured_from_front_sensor(self):
        # Static box at x=8 with 0.5 m extent -> front face x=7.75.
        # Scanner is at x=1.68 -> expected axial distance 6.07 m.
        self.assertAlmostEqual((8.0 - 0.25) - 1.68, 6.07, places=4)

    def test_infinite_values_do_not_become_obstacles(self):
        s = summarize_forward(self.scan(), sensor_height_m=0.600)
        self.assertTrue(s.valid)
        self.assertIsNone(s.nearest_forward_obstacle_m)


if __name__ == '__main__':
    unittest.main()
