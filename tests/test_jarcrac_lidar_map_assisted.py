"""Offline synthetic-heightfield regression for map-assisted LiDAR probe."""
from __future__ import annotations
from pathlib import Path
from types import SimpleNamespace as NS
import math
import struct
import sys
import tempfile
import unittest

GZ_SCRIPTS=Path(__file__).resolve().parents[1]/'edge-path-project/gazebo/scripts'
sys.path.insert(0,str(GZ_SCRIPTS))
from jarcrac_lidar_map_assisted import TerrainMeshIndex, scan_to_map_summary, rotate


def write_surface(path,slope=0.0,origin_z=0.0):
    # Four triangles covering [-8, 8] x [-8, 8], for exact synthetic height.
    pts=[(-8.,-8.),(8.,-8.),(8.,8.),(-8.,8.)]
    tris=[(pts[0],pts[1],pts[2]),(pts[0],pts[2],pts[3])]
    with path.open('wb') as f:
        f.write(b'JARCRAC synthetic terrain fixture'.ljust(80,b' '))
        f.write(struct.pack('<I',len(tris)))
        for t in tris:
            xyz=[(x,y,origin_z+slope*x) for x,y in t]
            f.write(struct.pack('<12fH',0.,0.,1.,*(val for v in xyz for val in v),0))


def pose(x=0.,y=0.,z=0.,yaw=0.):
    return NS(position=NS(x=x,y=y,z=z),
              orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))


def scan(ground=True,obstacle=False,slope=0.):
    # Downward -0.18 radians ground range 2.65 yields z near slope 0.05.
    # Obstacle is another ring's 3 m level beam, substantially above ground.
    h=5;v=3
    ranges=[math.inf]*(h*v)
    if ground:
        ranges[0*h+2]=2.65
    if obstacle:
        ranges[2*h+2]=3.0
    return NS(count=h,vertical_count=v,ranges=ranges,
              angle_min=-0.1,angle_step=0.05,
              vertical_angle_min=-.18,vertical_angle_step=.09,
              range_min=.30,range_max=30.)

class TestHeightMesh(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.mesh=Path(self.tmp.name)/'terrain.stl'
        write_surface(self.mesh,slope=.05)
        self.index=TerrainMeshIndex(self.mesh,center_x=0.,center_y=0.,radius_m=8.,cell_m=1.)

    def tearDown(self):
        self.tmp.cleanup()

    def summary(self,s,model=None):
        return scan_to_map_summary(s,model or pose(),self.index,
                                   min_ground_points=1)

    def test_map_height_interpolates_sloping_terrain(self):
        self.assertAlmostEqual(self.index.height_at(3.,1.),.15,places=5)
        self.assertAlmostEqual(self.index.height_at(-2.,0.),-.10,places=5)

    def test_unmapped_returns_unknown_not_clear(self):
        self.assertIsNone(self.index.height_at(20.,0.))

    def test_sloping_ground_is_not_object(self):
        s=self.summary(scan(ground=True))
        self.assertTrue(s.valid,s.reason)
        self.assertEqual(s.ground_points,1)
        self.assertEqual(s.obstacle_points,0)
        self.assertIsNone(s.nearest_forward_obstacle_m)

    def test_obstacle_above_slope_detected(self):
        s=self.summary(scan(ground=True,obstacle=True))
        self.assertTrue(s.valid,s.reason)
        self.assertEqual(s.ground_points,1)
        self.assertEqual(s.obstacle_points,1)
        self.assertAlmostEqual(s.nearest_forward_obstacle_m,3.)

    def test_malformed_scan_invalid(self):
        s=scan(ground=True)
        s.ranges.pop()
        self.assertFalse(self.summary(s).valid)

    def test_no_ground_is_invalid_even_with_object(self):
        s=self.summary(scan(ground=False,obstacle=True))
        self.assertFalse(s.valid)
        self.assertEqual(s.reason,'insufficient_mapped_ground_support')

    def test_bad_orientation_is_invalid(self):
        p=pose()
        p.orientation.w=0
        s=self.summary(scan(ground=True),p)
        self.assertEqual(s.reason,'invalid_pose_quaternion_or_position')

    def test_far_sensor_height_detected_as_mismatch(self):
        s=self.summary(scan(ground=True),pose(z=4))
        self.assertEqual(s.reason,'sensor_height_map_mismatch')

    def test_quaternion_rotation_preserves_length(self):
        q=(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4))
        x,y,z=rotate(q,(2.,0.,1.))
        self.assertAlmostEqual(x,0.,places=5)
        self.assertAlmostEqual(y,2.,places=5)
        self.assertAlmostEqual(z,1.,places=5)

    def test_non_binary_stl_rejected(self):
        self.mesh.write_text('solid fail\nendsolid fail\n')
        with self.assertRaises(ValueError):
            TerrainMeshIndex(self.mesh,center_x=0.,center_y=0.)

    def test_unmapped_obstacle_not_classified_clear(self):
        # An unexpected return beyond indexed terrain is UNKNOWN, never free.
        s=scan(ground=True,obstacle=True)
        s.ranges[2*s.count+2]=12.0  # beyond indexed terrain and radius
        flagged=self.summary(s)
        self.assertFalse(flagged.valid)
        self.assertEqual(flagged.unknown_points,1)
        self.assertEqual(flagged.reason,'map_mismatch_or_unmapped_returns')

    def test_low_rise_not_silently_ground(self):
        s=scan(ground=True,obstacle=True)
        s.ranges[2*s.count+2]=3.0  # z=.6 at x=3 => +.45 m
        known=self.summary(s)
        self.assertEqual(known.obstacle_points,1)
        # Any residual between .15 and .30 m is explicitly UNKNOWN.
        s.ranges[2*s.count+2]=6.2 # z=.6 at x=6.2 => +.29 m
        unknown=self.summary(s)
        self.assertEqual(unknown.unknown_points,1)
        self.assertFalse(unknown.valid)  # Unknown fraction > 45% => no clear certification

if __name__=='__main__':
    unittest.main()
