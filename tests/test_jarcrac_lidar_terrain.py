"""Pure offline tests for experimental JARCRAC slope-aware scan projection."""
from pathlib import Path
import math
import sys
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'edge-path-project/gazebo/scripts'))
from jarcrac_lidar_terrain import summarize_terrain_forward


def make_scan(slope_x=0, slope_y=0, wall_at=None, wall_height=1.5,
              finite=True, h=121, layers=32):
    amin=-math.radians(12)
    astep=math.radians(24)/(h-1)
    vmin=-math.radians(15)
    vstep=math.radians(30)/(layers-1)
    readings=[]
    for j in range(layers):
        e=vmin+j*vstep
        for i in range(h):
            az=amin+i*astep
            dx=math.cos(e)*math.cos(az)
            dy=math.cos(e)*math.sin(az)
            dz=math.sin(e)
            den=dz-slope_x*dx-slope_y*dy
            ground_r=-0.6/den if den < 0 else math.inf
            r=ground_r if 0.3 <= ground_r < 30 else math.inf
            if wall_at is not None:
                wall_r=wall_at/dx
                y=wall_r*dy
                wz=wall_r*dz
                base=-0.6+slope_x*wall_at+slope_y*y
                if 0 <= wz-base <= wall_height and wall_r < r:
                    r=wall_r
            readings.append(r if finite else math.inf)
    return SimpleNamespace(count=h, vertical_count=layers,
                           angle_min=amin, angle_step=astep,
                           vertical_angle_min=vmin, vertical_angle_step=vstep,
                           range_min=0.3,range_max=30.,ranges=readings)

class TerrainClassificationTests(unittest.TestCase):
    def test_level_ground(self):
        result=summarize_terrain_forward(make_scan())
        self.assertTrue(result.valid,result)
        self.assertGreater(result.ground_points,150)
        self.assertEqual(result.obstacle_points,0)
        self.assertAlmostEqual(result.slope_forward,0,delta=.015)

    def test_uphill_ground_not_obstacle(self):
        result=summarize_terrain_forward(make_scan(slope_x=.13,slope_y=.035))
        self.assertTrue(result.valid,result)
        self.assertEqual(result.obstacle_points,0)
        self.assertAlmostEqual(result.slope_forward,.13,delta=.02)
        self.assertAlmostEqual(result.slope_lateral,.035,delta=.025)

    def test_downhill_ground_not_obstacle(self):
        result=summarize_terrain_forward(make_scan(slope_x=-.11,slope_y=-.025))
        self.assertTrue(result.valid,result)
        self.assertEqual(result.obstacle_points,0)
        self.assertAlmostEqual(result.slope_forward,-.11,delta=.02)

    def test_wall_on_flat_ground(self):
        r=summarize_terrain_forward(make_scan(wall_at=6.))
        self.assertTrue(r.valid,r)
        self.assertGreater(r.obstacle_points,40)
        self.assertAlmostEqual(r.nearest_forward_obstacle_m,6.,delta=.1)

    def test_wall_on_uphill(self):
        r=summarize_terrain_forward(make_scan(slope_x=.10,wall_at=5.))
        self.assertTrue(r.valid,r)
        self.assertGreater(r.obstacle_points,40)
        self.assertAlmostEqual(r.nearest_forward_obstacle_m,5.,delta=.1)

    def test_missing_data_fail_closed(self):
        r=summarize_terrain_forward(make_scan(finite=False))
        self.assertFalse(r.valid)
        self.assertIsNone(r.nearest_forward_obstacle_m)

    def test_bad_shape_fail_closed(self):
        scan=make_scan()
        scan.ranges=scan.ranges[:-1]
        r=summarize_terrain_forward(scan)
        self.assertFalse(r.valid)

    def test_wall_only_not_treated_as_ground(self):
        scan=make_scan(wall_at=3)
        # Simulate missing floor returns: keep just points at wall range.
        scan.ranges=[r if math.isfinite(r) and 2.95 < r < 3.2 else math.inf
                     for r in scan.ranges]
        r=summarize_terrain_forward(scan)
        self.assertFalse(r.valid,r)

    def test_terrain_cliff_reported_as_untrusted(self):
        scan=make_scan(slope_x=.0)
        # A discontinuous terrain return on one ring must not silently
        # get classified as confirmed flat ground.
        n=scan.count
        i=n//2
        scan.ranges[n*1+i]=1.25
        r=summarize_terrain_forward(scan)
        self.assertTrue(r.valid,r)
        self.assertTrue(r.obstacle_points or r.unknown_points)

if __name__ == '__main__':
    unittest.main()
