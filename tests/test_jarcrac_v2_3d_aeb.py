"""Offline regression checks for isolated JARCRAC v2 3D-LiDAR AEB SIL trial."""
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
GZ = ROOT / "edge-path-project" / "gazebo"
sys.path.insert(0, str(GZ / "scripts"))

from jarcrac_lidar_3d import summarize_forward
from jarcrac_v2_aeb_policy import (
    axial_geometry, angle_difference, braking_decision, yaw_from_quaternion,
)
from jarcrac_v2_3d_aeb_sil import JarcracV2AEB, load_config, validate_files


class AEBV2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()
        cls.world = ET.parse(ROOT / cls.cfg["world"]).getroot().find("world")
        cls.model = ET.parse(GZ / "models/jarcrac/model.sdf").getroot().find("model")

    def test_sdf_config_and_topics(self):
        world, model = validate_files(self.cfg)
        self.assertTrue(world.is_file() and model.is_file())
        self.assertEqual(self.world.attrib["name"], "jarcrac_v2_front_aeb")
        sensor = self.model.find("./link[@name='chassis']/sensor[@type='gpu_lidar']")
        self.assertEqual(sensor.findtext("topic"), "/jarcrac/lidar")
        self.assertEqual(int(sensor.findtext("ray/scan/vertical/samples")), 32)
        self.assertEqual(self.cfg["topics"]["odometry"], "/model/jarcrac/odometry")

    def test_obstacle_and_contact_monitor(self):
        obstacle = self.world.find("./model[@name='jarcrac_v2_aeb_obstacle']")
        self.assertIsNotNone(obstacle)
        self.assertEqual([float(n) for n in obstacle.findtext("pose").split()][:2],
                         [8.0, 0.0])
        self.assertEqual(obstacle.findtext("link/sensor/contact/collision"), "collision")
        self.assertEqual(obstacle.findtext("link/sensor/topic"),
                         self.cfg["topics"]["obstacle_contacts"])
        self.assertTrue(self.cfg["topics"]["obstacle_contacts"].startswith(
            "/world/jarcrac_v2_front_aeb/model/"))
        self.assertIsNotNone(self.world.find("./plugin[@name='gz::sim::systems::Contact']"))

    def test_initial_clearance_model(self):
        forward_model = axial_geometry(self.cfg, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(forward_model[2], 6.25)
        initial_lidar_gap = 8 - 0.25 - 1.68
        self.assertAlmostEqual(initial_lidar_gap, 6.07)
        self.assertGreater(forward_model[2], initial_lidar_gap)

    def test_pure_policy_and_safe_brake(self):
        s = self.cfg["safety"]
        clearance = self.cfg["acceptance"]["required_clearance_m"]
        self.assertEqual(braking_decision(0.35, 4.0, s, clearance)[0], False)
        self.assertEqual(braking_decision(0.35, 1.3, s, clearance)[1],
                         "predicted_stopping_distance")
        self.assertEqual(braking_decision(0.35, 0.6, s, clearance)[1],
                         "emergency_distance")
        self.assertEqual(braking_decision(0.35, None, s, clearance)[1], "invalid_lidar")
        self.assertEqual(braking_decision(float("nan"), 4.0, s, clearance)[1],
                         "invalid_speed")

    def scan(self, range_forward=6.07, ground=False, obstacle=True):
        n = 721
        v = 32
        start = -math.pi/2
        angle_step = math.pi/(n-1)
        v_start = -math.radians(15)
        v_step = math.radians(30)/(v-1)
        arr = [math.inf] * (n*v)
        if obstacle:
            # A near-level ray near the middle of the vertical aperture
            ring = 15
            for j in range(n//2-5, n//2+6):
                arr[ring*n+j] = range_forward
        if ground:
            # Downward ray hitting z=0 at ~2.25 m from 0.6-m sensor
            arr[0*n+n//2] = 0.6 / math.sin(math.radians(15))
        return NS(count=n, vertical_count=v, ranges=arr, angle_min=start,
                  angle_step=angle_step, vertical_angle_min=v_start,
                  vertical_angle_step=v_step, range_min=.3, range_max=30.0)

    def test_3d_ground_filter_and_obstacle(self):
        scan = self.scan(ground=True)
        result = summarize_forward(scan, sensor_height_m=.6, half_angle_deg=12)
        self.assertTrue(result.valid)
        self.assertEqual(result.ground_points, 1)
        self.assertEqual(result.obstacle_points, 11)
        self.assertEqual(result.near_points, 0)
        self.assertAlmostEqual(result.nearest_forward_obstacle_m, 6.07, places=2)

    def test_ground_only_scan_does_not_drive(self):
        result = summarize_forward(self.scan(ground=True, obstacle=False),
                                   sensor_height_m=.6)
        self.assertTrue(result.valid)
        self.assertIsNone(result.nearest_forward_obstacle_m)

    def test_short_returns_are_not_masked(self):
        result = summarize_forward(self.scan(range_forward=0.7), sensor_height_m=.6)
        self.assertEqual(result.near_points, 11)
        self.assertAlmostEqual(result.nearest_forward_obstacle_m, .7, places=2)

    def test_bad_scan_shape_is_invalid(self):
        scan = self.scan()
        scan.ranges.pop()
        summary = summarize_forward(scan, sensor_height_m=.6)
        self.assertFalse(summary.valid)

    def test_flat_geometry_and_yaw(self):
        progress, lateral, gap = axial_geometry(self.cfg, (3.0, -0.1, 0.0))
        self.assertAlmostEqual(progress, 3.0)
        self.assertAlmostEqual(lateral, -0.1)
        self.assertAlmostEqual(gap, 3.25)
        self.assertAlmostEqual(yaw_from_quaternion(NS(x=0,y=0,z=0,w=1)), 0)
        self.assertAlmostEqual(angle_difference(-math.pi+.1, math.pi-.1), .2)

    def test_controller_fail_closed_and_contact_subscription(self):
        class Publisher:
            def __init__(self): self.commands=[]
            def publish(self, msg): self.commands.append(msg.linear.x)
        pub = Publisher()
        class Node:
            def advertise(self, *args): return pub
            def subscribe(self, *args): return True
        class Twist:
            def __init__(self):
                self.linear=NS(x=0.0)
                self.angular=NS(z=0.0)
        trial = JarcracV2AEB(self.cfg, Node, Twist, object, object, object, object)
        self.assertFalse(trial.snapshot()["fresh"])
        self.assertTrue(trial.contact_subscription_established)
        trial.stop()
        self.assertEqual(pub.commands[-1], 0.0)
        trial.on_scan(self.scan())
        self.assertEqual(trial.snapshot()["summary"].vertical_layers, 32)
        trial.on_scan(self.scan(ground=True, obstacle=False))
        self.assertFalse(trial.snapshot()["fresh"])
        trial.on_contacts(NS(contact=[NS()]))
        self.assertTrue(trial.snapshot()["contact"])

    def test_dry_run(self):
        run = subprocess.run([
            sys.executable,
            str(GZ / "scripts/jarcrac_v2_3d_aeb_sil.py"), "--dry-run",
        ], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr + run.stdout)
        self.assertIn("JARCRAC V2 3D AEB CONFIG: PASS", run.stdout)


if __name__ == "__main__":
    unittest.main()
