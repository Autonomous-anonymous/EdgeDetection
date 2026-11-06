"""Offline tests for map pose perturbations and paired diagnostic accounting."""
from __future__ import annotations
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS

SCRIPTS = Path(__file__).resolve().parents[1] / 'edge-path-project/gazebo/scripts'
sys.path.insert(0,str(SCRIPTS))
from jarcrac_map_pose_sensitivity import (cases, frozen_pose, perturb_pose,
                                         analyze_sensitivity)
from jarcrac_lidar_map_assisted import quaternion_tuple, rotate


def pose(x=10., y=20., z=-9., yaw=0., roll=0., pitch=0.):
    # ZYX quaternion from yaw, pitch, roll
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return NS(position=NS(x=x,y=y,z=z),
              orientation=NS(w=cr*cp*cy+sr*sp*sy,
                             x=sr*cp*cy-cr*sp*sy,
                             y=cr*sp*cy+sr*cp*sy,
                             z=cr*cp*sy-sr*sp*cy))


def result(valid=True, objects=0, unknown=0, reason='ok', near=None):
    return dict(valid=valid, obstacle_points=objects, unknown_points=unknown,
                unknown_fraction=unknown/100, reason=reason, nearest_obstacle_m=near)


class TestPerturbations(unittest.TestCase):
    def test_thirteen_poses_unique(self):
        names=[r[0] for r in cases()]
        self.assertEqual(len(names),13)
        self.assertEqual(len(set(names)),13)
        self.assertEqual(names[0],'exact')

    def test_zero_offsets_preserve_pose(self):
        p=pose(yaw=-2.67,roll=.08,pitch=-.12)
        q=perturb_pose(p)
        self.assertAlmostEqual(q.position.x,p.position.x)
        self.assertAlmostEqual(q.position.y,p.position.y)
        for attr in ('x','y','z','w'):
            self.assertAlmostEqual(getattr(q.orientation,attr),getattr(p.orientation,attr))

    def test_forward_offsets_follow_body_heading(self):
        p=pose(yaw=math.pi/2)
        q=perturb_pose(p,forward_m=.25)
        self.assertAlmostEqual(q.position.x,p.position.x,places=9)
        self.assertAlmostEqual(q.position.y,p.position.y+.25,places=9)

    def test_lateral_offsets_follow_body_heading(self):
        p=pose(yaw=math.pi/2)
        q=perturb_pose(p,left_m=.1)
        self.assertAlmostEqual(q.position.x,p.position.x-.1,places=9)
        self.assertAlmostEqual(q.position.y,p.position.y,places=9)

    def test_negative_offsets(self):
        p=pose(yaw=0.)
        q=perturb_pose(p,forward_m=-.25,left_m=-.1)
        self.assertAlmostEqual(q.position.x,p.position.x-.25,places=9)
        self.assertAlmostEqual(q.position.y,p.position.y-.1,places=9)

    def test_yaw_rotation_world_vertical(self):
        p=pose(yaw=math.radians(40))
        q=perturb_pose(p,yaw_deg=5.)
        vx,vy,vz=rotate(quaternion_tuple(q.orientation),(1,0,0))
        self.assertAlmostEqual(math.degrees(math.atan2(vy,vx)),45.,places=8)
        self.assertAlmostEqual(vz,0,places=8)

    def test_pitch_roll_maintained_when_yaw_perturbed(self):
        p=pose(yaw=-1.7,roll=.1,pitch=-.2)
        q=perturb_pose(p,yaw_deg=-5)
        base_up=rotate(quaternion_tuple(p.orientation),(0,0,1))
        tilted_up=rotate(quaternion_tuple(q.orientation),(0,0,1))
        a=math.radians(-5)
        self.assertAlmostEqual(tilted_up[0],math.cos(a)*base_up[0]-math.sin(a)*base_up[1])
        self.assertAlmostEqual(tilted_up[1],math.sin(a)*base_up[0]+math.cos(a)*base_up[1])
        self.assertAlmostEqual(tilted_up[2],base_up[2])

    def test_frozen_copy_not_alias(self):
        p=pose()
        clone=frozen_pose(p)
        p.position.x=99
        self.assertAlmostEqual(clone.position.x,10)

    def test_clear_world_false_alarm_count(self):
        pair=[]
        for _ in range(3):
            all_cases={c[0]:result() for c in cases()}
            all_cases['yaw_+5deg']=result(objects=5,near=4.2)
            pair.append({'cases':all_cases})
        summary,ok=analyze_sensitivity(pair,'clear')
        self.assertTrue(ok)
        self.assertEqual(summary['yaw_+5deg']['false_obstacle_vs_exact_samples'],3)
        self.assertEqual(summary['exact']['obstacle_detecting_valid_samples'],0)

    def test_obstacle_world_missed_detection_count(self):
        pair=[]
        for _ in range(3):
            all_cases={c[0]:result(objects=9,near=3.84) for c in cases()}
            all_cases['left_-0.25m']=result()
            pair.append({'cases':all_cases})
        summary,ok=analyze_sensitivity(pair,'obstacle')
        self.assertTrue(ok)
        self.assertEqual(summary['left_-0.25m']['missed_obstacle_vs_exact_samples'],3)

    def test_invalid_not_counted_as_clear_or_missed(self):
        pair=[]
        for _ in range(3):
            all_cases={c[0]:result(objects=10,near=3.83) for c in cases()}
            all_cases['forward_+0.25m']=result(False,objects=0,unknown=100,
                                                 reason='map_mismatch_or_unmapped_returns')
            pair.append({'cases':all_cases})
        summary,ok=analyze_sensitivity(pair,'obstacle')
        self.assertTrue(ok)
        s=summary['forward_+0.25m']
        self.assertEqual(s['invalid_samples'],3)
        self.assertEqual(s['valid_scans_with_no_obstacle'],0)
        self.assertEqual(s['missed_obstacle_vs_exact_samples'],0)

    def test_baseline_requires_three_valid_samples(self):
        all_cases={c[0]:result(objects=5,near=4) for c in cases()}
        _,ok=analyze_sensitivity([{'cases':all_cases}],'obstacle')
        self.assertFalse(ok)

    def test_no_samples_diagnostic(self):
        summary,ok=analyze_sensitivity([],'clear')
        self.assertFalse(ok)
        self.assertIsNone(summary['exact']['median_unknown_fraction'])

if __name__=='__main__':
    unittest.main()
