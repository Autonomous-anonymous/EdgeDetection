"""Offline regression tests for height/attitude perturbation, detection accounting."""
from __future__ import annotations

import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / 'edge-path-project/gazebo/scripts'
sys.path.insert(0, str(SCRIPTS))

from jarcrac_lidar_map_assisted import quaternion_tuple, rotate
from jarcrac_map_attitude_sensitivity import analyze, cases, multiply, perturb_attitude


def pose(yaw=0., pitch=0., roll=0., z=-9.):
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    return NS(position=NS(x=20., y=40., z=z),
              orientation=NS(w=cr*cp*cy + sr*sp*sy,
                             x=sr*cp*cy-cr*sp*sy,
                             y=cr*sp*cy+sr*cp*sy,
                             z=cr*cp*sy-sr*sp*cy))


def result(valid=True, obstacles=0, unknown=0, nearest=None, reason='ok', sensor_h=.57):
    return dict(valid=valid, obstacle_points=obstacles, unknown_points=unknown,
                nearest_obstacle_m=nearest, reason=reason,
                unknown_fraction=unknown/2500, sensor_height_m=sensor_h)


class TestAttitudePerturbations(unittest.TestCase):
    def test_seventeen_cases_have_unique_names(self):
        names = [c[0] for c in cases()]
        self.assertEqual(len(names), 17)
        self.assertEqual(len(set(names)), 17)
        self.assertEqual(names[0], 'exact')

    def test_repeated_exact_is_identical(self):
        p = pose(yaw=1.2, pitch=-.09, roll=.06)
        q = perturb_attitude(p)
        for key in ('x', 'y', 'z'):
            self.assertAlmostEqual(getattr(q.position,key), getattr(p.position,key))
        for key in ('x', 'y', 'z', 'w'):
            self.assertAlmostEqual(getattr(q.orientation,key), getattr(p.orientation,key))

    def test_world_vertical_bias(self):
        p = pose(z=-8.)
        hi = perturb_attitude(p, z_m=.25)
        lo = perturb_attitude(p, z_m=-.10)
        self.assertAlmostEqual(hi.position.z, -7.75)
        self.assertAlmostEqual(lo.position.z, -8.10)
        self.assertAlmostEqual(hi.position.x, p.position.x)

    def test_pitch_rotates_forward_axis_to_vertical(self):
        q = perturb_attitude(pose(), pitch_deg=5)
        forward = rotate(quaternion_tuple(q.orientation),(1,0,0))
        self.assertAlmostEqual(forward[2],-math.sin(math.radians(5)),places=9)
        # Right-handed rotation about +Y sends +X toward -Z.
        q = perturb_attitude(pose(), pitch_deg=-5)
        forward = rotate(quaternion_tuple(q.orientation),(1,0,0))
        self.assertAlmostEqual(forward[2],math.sin(math.radians(5)),places=9)

    def test_roll_rotates_left_axis_towards_up(self):
        q = perturb_attitude(pose(), roll_deg=2)
        left = rotate(quaternion_tuple(q.orientation),(0,1,0))
        self.assertAlmostEqual(left[2], math.sin(math.radians(2)),places=9)

    def test_local_pitch_on_existing_yaw_and_slope(self):
        p = pose(yaw=1.25, pitch=.12, roll=-.04)
        q = perturb_attitude(p, pitch_deg=-2.)
        baseline_forward = rotate(quaternion_tuple(p.orientation),(1,0,0))
        modified_forward = rotate(quaternion_tuple(q.orientation),(1,0,0))
        self.assertNotAlmostEqual(modified_forward[2],baseline_forward[2])
        self.assertAlmostEqual(math.sqrt(sum(x*x for x in quaternion_tuple(q.orientation))),1.)

    def test_mixed_offsets_do_not_mutate_original(self):
        p=pose(yaw=math.pi/2)
        q=perturb_attitude(p,z_m=.1,forward_m=.1,yaw_deg=2)
        self.assertAlmostEqual(p.position.y,40.)
        self.assertAlmostEqual(q.position.y,40.1)
        self.assertAlmostEqual(q.position.z,p.position.z+.1)
        self.assertAlmostEqual(p.orientation.z,math.sin(math.pi/4))

    def test_hamilton_product_identity(self):
        q=(.1,.2,.3,.92)
        ident=(0,0,0,1)
        self.assertEqual(multiply(q,ident),q)
        self.assertEqual(multiply(ident,q),q)

    def test_signed_roll_and_pitch_cases(self):
        names={c[0] for c in cases()}
        for key in ('z_+0.05m','z_-0.25m','pitch_+5deg','pitch_-2deg',
                    'roll_+2deg','roll_-5deg','mixed_positive','mixed_negative'):
            self.assertIn(key,names)


class TestPairedAccounting(unittest.TestCase):
    def samples(self, *, obstacle=True, incorrect_name=None, replacement=None, count=4):
        batches=[]
        for _ in range(count):
            variants={c[0]:result(obstacles=25 if obstacle else 0,
                                  nearest=3.84 if obstacle else None)
                      for c in cases()}
            if incorrect_name is not None:
                variants[incorrect_name]=replacement
            batches.append({'cases':variants})
        return batches

    def test_exact_clear_reference(self):
        stats,ok=analyze(self.samples(obstacle=False),'clear')
        self.assertTrue(ok)
        self.assertEqual(stats['exact']['obstacle_detecting_valid_samples'],0)

    def test_exact_obstacle_reference(self):
        stats,ok=analyze(self.samples(obstacle=True),'obstacle')
        self.assertTrue(ok)
        self.assertEqual(stats['exact']['obstacle_detecting_valid_samples'],4)

    def test_false_obstacle_under_incorrect_z(self):
        stats,ok=analyze(self.samples(obstacle=False, incorrect_name='z_+0.25m',
                                      replacement=result(obstacles=5,nearest=2.5)),'clear')
        self.assertTrue(ok)
        self.assertEqual(stats['z_+0.25m']['false_obstacle_vs_exact_samples'],4)

    def test_false_clear_under_pitch_error(self):
        stats,ok=analyze(self.samples(obstacle=True, incorrect_name='pitch_-5deg',
                                      replacement=result(obstacles=0)), 'obstacle')
        self.assertTrue(ok)
        self.assertEqual(stats['pitch_-5deg']['missed_obstacle_vs_exact_samples'],4)

    def test_invalid_is_not_missed_obstacle(self):
        invalid=result(valid=False,obstacles=0,reason='map_mismatch_or_unmapped_returns',unknown=2000)
        stats,ok=analyze(self.samples(obstacle=True, incorrect_name='roll_+5deg',
                                      replacement=invalid),'obstacle')
        self.assertTrue(ok)
        self.assertEqual(stats['roll_+5deg']['invalid_samples'],4)
        self.assertEqual(stats['roll_+5deg']['missed_obstacle_vs_exact_samples'],0)
        self.assertEqual(stats['roll_+5deg']['obstacle_detecting_valid_samples'],0)

    def test_nearest_distance_delta(self):
        stats,_=analyze(self.samples(obstacle=True, incorrect_name='mixed_positive',
                                     replacement=result(obstacles=24,nearest=3.9)), 'obstacle')
        self.assertAlmostEqual(stats['mixed_positive']['median_nearest_distance_delta_m_vs_exact'],.06)
        self.assertEqual(stats['mixed_positive']['median_delta_obstacle_points_vs_exact'],-1)

    def test_baseline_rejects_no_samples(self):
        stats,ok=analyze([], 'clear')
        self.assertFalse(ok)
        self.assertIsNone(stats['exact']['median_unknown_fraction'])

    def test_baseline_rejects_invalid(self):
        batches=self.samples(obstacle=True)
        batches[0]['cases']['exact']=result(valid=False,reason='sensor_height_map_mismatch')
        _,ok=analyze(batches,'obstacle')
        self.assertFalse(ok)


if __name__ == '__main__':
    unittest.main()
