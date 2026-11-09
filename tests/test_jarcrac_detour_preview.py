"""Offline geometry tests for read-only JARCRAC detour preview."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'edge-path-project/gazebo/scripts'))

from jarcrac_detour_preview import (
    candidate_waypoints, rect_distance, sample_polyline,
    screen_candidate, transform, to_local, write_svg, vehicle_radius
)


def evaluate(points, terrain=lambda x,y: 0.):
    return screen_candidate(
        points, (0,0), 0,
        (5.37,0,0,0,0,0), (.8,2,3),
        1.73, terrain
    )


class TestDetourPreview(unittest.TestCase):
    def test_coordinates(self):
        start = (20.,40.)
        yaw = -2.678
        local = (3.,-4.)
        world = transform(local,start,yaw)
        for a,b in zip(to_local(world,start,yaw),local):
            self.assertAlmostEqual(a,b)

    def test_box_distance(self):
        self.assertAlmostEqual(
            rect_distance((3,0),(0,0),0,(2,2)),2.
        )
        self.assertEqual(
            rect_distance((0,0),(0,0),0,(2,4)),0.
        )

    def test_oriented_box(self):
        self.assertAlmostEqual(
            rect_distance((0,3),(0,0),math.pi/2,(2,4)),2.
        )

    def test_left_and_right(self):
        self.assertGreater(candidate_waypoints(1)[1][1],0)
        self.assertLess(candidate_waypoints(-1)[1][1],0)
        with self.assertRaises(ValueError):
            candidate_waypoints(0)

    def test_sample_spacing(self):
        pts = sample_polyline([(0,0),(1,0),(1,1)])
        self.assertTrue(all(
            math.dist(a,b)<=.250001
            for a,b in zip(pts,pts[1:])
        ))

    def test_known_obstacle(self):
        self.assertTrue(evaluate(
            candidate_waypoints(1)
        )['geometry_screen_pass'])
        result = evaluate([(0,0),(11,0)])
        self.assertFalse(result['geometry_screen_pass'])
        self.assertIn('known_obstacle_clearance',result['issues'])

    def test_missing_terrain(self):
        result = evaluate(
            candidate_waypoints(1),
            lambda x,y: None
        )
        self.assertFalse(result['geometry_screen_pass'])
        self.assertIn('missing_terrain',result['issues'])

    def test_steep_terrain(self):
        result = evaluate(
            candidate_waypoints(1),
            lambda x,y: .8*x
        )
        self.assertFalse(result['geometry_screen_pass'])
        self.assertIn('terrain_grade_screen',result['issues'])

    def test_svg(self):
        result = evaluate(candidate_waypoints(1))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'preview.svg'
            write_svg(
                path,{'left':result,'right':result},
                (5.37,0,0,0,0,0),(.8,2,3),
                (0,0),0,1.73
            )
            ET.parse(path)
            self.assertIn(
                'NO DRIVE AUTHORIZATION',
                path.read_text()
            )

    def test_vehicle_radius_fixture(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'model.sdf'
            path.write_text(
                '<sdf><model><link name="chassis">'
                '<collision name="body_collision">'
                '<pose>-0.04 0.02 0 0 0 0</pose>'
                '<geometry><box><size>2.98 1.568 2</size>'
                '</box></geometry></collision></link></model></sdf>'
            )
            self.assertGreater(vehicle_radius(path),1.7)


if __name__=='__main__':
    unittest.main()
