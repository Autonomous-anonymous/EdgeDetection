"""Measured stop/observation regressions independent of Gazebo and the scene script."""

import math
from pathlib import Path
import sys
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "edge-path-project/gazebo/scripts"))
from pitch_obstacles import ObstacleObservation


def scan(stamp, x=6.0, lateral=0.0):
    ranges = []
    for i in range(721):
        angle = -math.pi + i * math.pi / 360
        distance = math.inf
        if x is not None and math.cos(angle) > 0:
            y = x * math.tan(angle)
            if abs(y-lateral) <= .8:
                distance = x / math.cos(angle)
        ranges.append(distance)
    whole = math.floor(stamp)
    return types.SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_step=math.pi/360,
                                 range_min=.1, range_max=30,
                                 header=types.SimpleNamespace(HasField=lambda key: key == "stamp",
                                                             stamp=types.SimpleNamespace(sec=whole, nsec=round((stamp-whole)*1e9))))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.observation = ObstacleObservation()
        self.observation.hold()

    def feed(self, t, *, speed=0, fresh=True, pose=(0, 0, 0), **geometry):
        self.observation.update(scan(t, **geometry), speed, pose,
                                odometry_fresh=fresh, received=t)

    def test_stopped_vehicle_needs_two_seconds_of_new_surface_observations(self):
        for i in range(20):
            self.feed(i*.1)
        self.assertFalse(self.observation.stationary)
        self.feed(2.0)
        self.assertTrue(self.observation.stationary)
        self.assertEqual(self.observation.stationary_scans, 21)

    def test_same_range_while_following_is_not_stationary(self):
        for i in range(30):
            self.feed(i*.1, speed=1, pose=(i*.1, 0, 0))
        self.assertFalse(self.observation.stationary)
        self.assertEqual(self.observation.stationary_scans, 0)

    def test_lateral_motion_with_nearly_constant_nearest_range_resets_window(self):
        for i in range(30):
            self.feed(i*.1, lateral=-.6+i*.04)
        self.assertAlmostEqual(self.observation.front, 6)
        self.assertFalse(self.observation.stationary)

    def test_one_second_pause_in_a_crossing_is_not_a_static_obstacle(self):
        for i in range(10):
            self.feed(i*.1, lateral=-1+i*.1)
        for i in range(10, 20):
            self.feed(i*.1, lateral=0)
        for i in range(20, 30):
            self.feed(i*.1, lateral=(i-20)*.1)
        self.assertFalse(self.observation.stationary)

    def test_cached_scans_do_not_confirm_stationary_or_clear(self):
        for i in range(40):
            self.feed(1.0)
        self.assertFalse(self.observation.stationary)
        self.assertEqual(self.observation.serial, 1)
        self.feed(1.1, x=None)
        for i in range(40):
            self.feed(1.1, x=None)
        self.assertEqual(self.observation.clear_scans, 1)

    def test_scan_gap_or_stale_speed_restarts_confirmation(self):
        for i in range(20):
            self.feed(i*.1)
        self.feed(2.5)
        self.assertEqual(self.observation.stationary_scans, 1)
        self.feed(2.6, fresh=False)
        self.assertEqual(self.observation.stationary_scans, 0)

    def test_pose_drift_and_missing_returns_cannot_establish_stationarity(self):
        for i in range(30):
            self.feed(i*.1, pose=(0, i*.02, 0))
        self.assertFalse(self.observation.stationary)
        for i in range(30, 60):
            self.feed(i*.1, x=None)
        self.assertFalse(self.observation.stationary)

    def test_ttc_brakes_well_before_emergency_distance(self):
        self.observation.release()
        self.feed(1, x=6, speed=3)
        self.assertGreater(self.observation.clearance, 4)
        self.assertLess(self.observation.ttc, 2)
        self.assertEqual(self.observation.command((3, .2)), (0, 0))

    def test_clearance_brakes_even_at_zero_measured_speed(self):
        self.observation.release()
        self.feed(1, x=2, speed=0)
        self.assertEqual(self.observation.command((1, .2)), (0, 0))

    def test_ttc_speed_limit_preserves_requested_turn_radius(self):
        self.observation.release()
        self.feed(1, x=8, speed=1)
        speed, turn = self.observation.command((3, .6))
        self.assertLess(speed, 3)
        self.assertAlmostEqual(turn/speed, .2)

    def test_invalid_scan_never_permits_motion_or_confirms_stationarity(self):
        msg = scan(1)
        msg.ranges[:] = [math.nan] * 721
        self.observation.update(msg, 0, (0, 0, 0), odometry_fresh=True, received=1)
        self.assertEqual(self.observation.command((1, 0)), (0, 0))
        self.assertFalse(self.observation.stationary)


if __name__ == "__main__":
    unittest.main()
