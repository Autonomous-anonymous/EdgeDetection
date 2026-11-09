"""Offline tests for JARCRAC's bounded turning safeguards."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'edge-path-project/gazebo/scripts'))

from jarcrac_turn_scan import (
    yaw_delta_deg, terrain_sweep, turn_gate,
    TARGET_DEG, MAX_TRANSLATION_M,
)


def gate(**kwargs):
    values = dict(
        ages=[.02]*6,
        analysis_status='obstacle_confirmed',
        analysis_age=.1,
        contact=False,
        speed=0.0,
        angular_speed=.12,
        roll=2.,
        pitch=3.,
        travel=.02,
        yaw=6.,
    )
    values.update(kwargs)
    return turn_gate(**values)


class TestTurning(unittest.TestCase):
    def test_angle_wrap(self):
        self.assertEqual(yaw_delta_deg(179,-175),6)
        self.assertEqual(yaw_delta_deg(-179,175),-6)

    def test_bound(self):
        self.assertEqual(TARGET_DEG,10.)
        self.assertLessEqual(MAX_TRANSLATION_M,.15)

    def test_nominal(self):
        self.assertIsNone(gate())

    def test_contact(self):
        self.assertEqual(gate(contact=True),'chassis_contact')

    def test_invalid_data(self):
        self.assertEqual(gate(speed=math.nan),'invalid_telemetry')
        self.assertEqual(
            gate(ages=[.1,-.1]+[.1]*4),
            'invalid_telemetry',
        )

    def test_stale_lidar_and_clock(self):
        self.assertEqual(
            gate(ages=[.01,.01,.9,.01,.01,.01]),
            'stale_sensor_or_classification',
        )
        self.assertEqual(
            gate(ages=[.01]*4+[.9,.01]),
            'stale_sensor_or_classification',
        )

    def test_stale_classification(self):
        self.assertEqual(
            gate(analysis_age=1.5),
            'stale_sensor_or_classification',
        )

    def test_missing_obstacle(self):
        self.assertEqual(
            gate(analysis_status='expected_obstacle_missing'),
            'expected_obstacle_missing',
        )

    def test_unexpected_translation(self):
        self.assertEqual(
            gate(travel=.3),
            'unexpected_translation',
        )
        self.assertEqual(
            gate(speed=.1),
            'unexpected_velocity',
        )

    def test_overspeed_tilt_yaw(self):
        self.assertEqual(
            gate(angular_speed=.4),
            'unexpected_velocity',
        )
        self.assertEqual(gate(pitch=19.),'excessive_tilt')
        self.assertEqual(gate(yaw=14.),'excessive_yaw')

    def test_sweep_planar(self):
        issue,grade = terrain_sweep(
            lambda x,y:.07*x+.02*y,0,0
        )
        self.assertIsNone(issue)
        self.assertAlmostEqual(grade,.07)

    def test_sweep_steep_and_unknown(self):
        issue,_ = terrain_sweep(lambda x,y:.8*x,0,0)
        self.assertEqual(issue,'turning_terrain_grade_exceeded')

        issue,_ = terrain_sweep(lambda x,y:None,0,0)
        self.assertEqual(issue,'unmapped_turning_footprint')


if __name__ == '__main__':
    unittest.main()
