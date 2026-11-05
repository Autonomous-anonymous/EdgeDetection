"""Offline model, scenario, and pure AEB policy checks (no Gazebo needed)."""
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
GZ = ROOT / "edge-path-project" / "gazebo"
sys.path.insert(0, str(GZ / "scripts"))
from metsakone_aeb_policy import brake_decision, front_clearance, route_geometry


class TestMetsakoneSIL(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (GZ / "scenarios/metsakone_emergency_braking.json").read_text())
        cls.model = ET.parse(GZ / "models/metsakone/model.sdf")
        cls.world = ET.parse(GZ / "worlds/metsakone_aeb_world.sdf")

    def test_model_isolated_and_meshes_present(self):
        root = self.model.getroot().find("model")
        self.assertEqual(root.attrib["name"], "metsakone")
        self.assertEqual(len(root.findall("./plugin[@name='gz::sim::systems::DiffDrive']")), 1)
        plugin = root.find("./plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(len(plugin.findall("left_joint")), 2)
        self.assertEqual(len(plugin.findall("right_joint")), 2)
        self.assertEqual(plugin.findtext("wheel_radius"), "0.6")
        self.assertEqual(plugin.findtext("topic"), self.config["topics"]["cmd_vel"])
        self.assertEqual(plugin.findtext("odom_topic"), self.config["topics"]["odometry"])
        for uri in root.findall(".//mesh/uri"):
            self.assertTrue(uri.text.startswith("model://metsakone/meshes/"))
            self.assertTrue((GZ / "models/metsakone/meshes"
                             / uri.text.split("/")[-1]).is_file(), uri.text)
        self.assertEqual(root.find(".//sensor[@type='gpu_lidar']/topic").text,
                         self.config["topics"]["lidar"])

    def test_imported_model_is_preserved(self):
        original = ET.parse(
            GZ / "imported/reinforcement-learning/metsakone.sdf")
        self.assertEqual(
            len(original.getroot().find("model").findall(
                "./plugin[@name='gz::sim::systems::DiffDrive']")), 2)

    def test_world_and_scenario_match(self):
        self.assertEqual(self.world.getroot().find("world").attrib["name"],
                         "metsakone_aeb_world")
        names = [x.findtext("name") for x in self.world.findall("./world/include")]
        self.assertIn("metsakone", names)
        self.assertIn("ruskotunturi_map", names)
        self.assertEqual(self.config["obstacle"]["name"],
                         self.world.find("./world/model").attrib["name"])
        self.assertEqual(self.config["schema_version"], 1)

    def test_lidar_front_sector_and_invalid_returns(self):
        scan = SimpleNamespace(ranges=[8.0, 3.0, float("inf"), 8.0, 8.0],
                               angle_min=-math.pi / 2,
                               angle_step=math.pi / 4,
                               range_min=0.1, range_max=30.0)
        self.assertAlmostEqual(front_clearance(scan, 50.0, 0.65), 2.35)
        scan.ranges = [float("inf")] * 5
        self.assertIsNone(front_clearance(scan, 50.0, 0.65))

    def test_brake_policy(self):
        s = self.config["safety"]
        self.assertEqual(brake_decision(0.65, 3.0, s, 1.0)[0], False)
        brake, reason, _ttc = brake_decision(0.65, 1.1, s, 1.0)
        self.assertTrue(brake)
        self.assertIn(reason, ("ttc_threshold", "predicted_stopping_distance"))
        self.assertEqual(brake_decision(0.0, None, s, 1.0)[1], "invalid_lidar")

    def test_route_clearance_and_progress(self):
        x = self.config["start"]["x"]
        y = self.config["start"]["y"]
        progress, lateral, gap = route_geometry(self.config, x, y)
        self.assertAlmostEqual(progress, 0)
        self.assertAlmostEqual(lateral, 0)
        self.assertGreater(gap, 1.0)
        p2, _l2, gap2 = route_geometry(self.config, x + 0.8, y - 0.6)
        self.assertGreater(p2, 0)
        self.assertLess(gap2, gap)

    def test_controller_dry_run_without_gazebo(self):
        p = subprocess.run([
            sys.executable, str(GZ / "scripts/metsakone_emergency_braking_sil.py"),
            "--dry-run"], cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn("METSAKONE CONFIG: PASS", p.stdout)


if __name__ == "__main__":
    unittest.main()
