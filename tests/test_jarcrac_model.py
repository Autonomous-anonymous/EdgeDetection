"""Offline checks for the generated Gazebo ``jarcrac`` model.

These run without Gazebo.  They verify internal consistency of the generated
SDF (joints, masses, symmetry, ground contact, topics) and that the body mesh
is placed identically to how Mevea places it.

Set JARCRAC_MEVEA_XML to the Mevea model XML to also check that regenerating
the model reproduces the committed files byte for byte.
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
GAZEBO = REPO / "edge-path-project" / "gazebo"
MODEL_DIR = GAZEBO / "models" / "jarcrac"
GENERATOR = GAZEBO / "scripts" / "generate_jarcrac_model.py"
WORLD = GAZEBO / "worlds" / "jarcrac_flat_test_world.sdf"

sys.path.insert(0, str(GAZEBO / "scripts"))
import generate_jarcrac_model as gen  # noqa: E402


def _nums(text):
    return [float(v) for v in text.split()]


class JarcracModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = ET.parse(MODEL_DIR / "model.sdf").getroot()
        cls.model = cls.sdf.find("model")
        cls.params = json.loads((MODEL_DIR / "jarcrac_params.json").read_text())
        cls.links = {l.get("name"): l for l in cls.model.findall("link")}
        cls.joints = {j.get("name"): j for j in cls.model.findall("joint")}
        cls.plugin = next(p for p in cls.model.findall("plugin") if "DiffDrive" in p.get("name"))
        cls.margin = gen.DEFAULTS["ground_clearance_margin_m"]

    def _link_z(self, name):
        return _nums(self.links[name].find("pose").text)[2]

    def wheels(self):
        return [n for n in self.links if n.startswith("wheel")]

    # -- structure -----------------------------------------------------------

    def test_world_and_config_are_well_formed(self):
        ET.parse(WORLD)
        ET.parse(MODEL_DIR / "model.config")

    def test_every_wheel_has_a_joint_to_the_chassis(self):
        for w in self.wheels():
            j = self.joints[f"{w}_joint"]
            self.assertEqual(j.find("parent").text, "chassis")
            self.assertEqual(j.find("child").text, w)
            self.assertEqual(j.find("axis/xyz").text.split(), ["0", "1", "0"])

    def test_diff_drive_references_existing_joints_and_covers_all_driven_wheels(self):
        left = [e.text for e in self.plugin.findall("left_joint")]
        right = [e.text for e in self.plugin.findall("right_joint")]
        free = [f"{n}_joint" for n in self.params["overrides"]["free_spinning_wheels"]]
        self.assertEqual(sorted(left + right + free), sorted(self.joints))
        self.assertTrue(set(free).isdisjoint(left + right))
        self.assertEqual(free, [], "after normalising Pulley3_Left every wheel is driven")

    def test_mesh_uri_resolves_to_a_committed_file(self):
        uri = self.links["chassis"].find("visual/geometry/mesh/uri").text
        self.assertTrue(uri.startswith("model://jarcrac/"))
        self.assertTrue((MODEL_DIR / uri[len("model://jarcrac/"):]).is_file())

    # -- physics sanity ------------------------------------------------------

    def test_total_mass_matches_params_record(self):
        total = sum(float(l.find("inertial/mass").text) for l in self.links.values())
        self.assertAlmostEqual(total, self.params["total_mass_kg_in_gazebo"], places=6)

    def test_chassis_mass_is_body_plus_mevea_track_mass(self):
        ch = self.params["chassis"]
        self.assertEqual(ch["body_only_mass_kg"], 2000.0)  # Jan-2026 free-roll lineage
        tracks = ch["tracks_lumped_into_chassis"]["per_side"]
        lumped = sum(v["mass_kg_estimate"] + v["small_roller_mass_kg"] for v in tracks.values())
        chassis_mass = float(self.links["chassis"].find("inertial/mass").text)
        self.assertAlmostEqual(chassis_mass, 2000.0 + lumped, delta=0.02)
        total = 2000.0 + lumped + 12 * 10.0
        self.assertAlmostEqual(self.params["total_mass_kg_in_gazebo"], total, delta=0.05)
        self.assertTrue(2200 < total < 2450, "total should stay close to the Mevea free-roll model")

    def test_chassis_inertia_is_physical(self):
        i = self.links["chassis"].find("inertial/inertia")
        v = {k: float(i.find(k).text) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        # positive definite (Sylvester) and satisfies the triangle inequality
        self.assertGreater(v["ixx"], 0)
        self.assertGreater(v["ixx"] * v["iyy"] - v["ixy"] ** 2, 0)
        det = (v["ixx"] * (v["iyy"] * v["izz"] - v["iyz"] ** 2) - v["ixy"] * (v["ixy"] * v["izz"] - v["iyz"] * v["ixz"])
               + v["ixz"] * (v["ixy"] * v["iyz"] - v["iyy"] * v["ixz"]))
        self.assertGreater(det, 0)
        self.assertLessEqual(v["ixx"], v["iyy"] + v["izz"])
        self.assertLessEqual(v["iyy"], v["ixx"] + v["izz"])
        self.assertLessEqual(v["izz"], v["ixx"] + v["iyy"])
        # the lumped track mass can only add inertia to the Mevea body inertia (616, 1432.8, 1270.3)
        self.assertGreater(v["ixx"], 616.0)
        self.assertGreater(v["iyy"], 1270.3)  # Gazebo y (lateral) = Mevea z (1432.8)

    def test_wheels_are_left_right_symmetric(self):
        for name in self.wheels():
            if not name.endswith("_left"):
                continue
            lx, ly, lz = _nums(self.links[name].find("pose").text)[:3]
            rx, ry, rz = _nums(self.links[name.replace("_left", "_right")].find("pose").text)[:3]
            self.assertAlmostEqual(lx, rx, places=6)
            self.assertAlmostEqual(lz, rz, places=6)
            self.assertAlmostEqual(ly, -ry, places=6)
            self.assertGreater(ly, 0, "left wheels must be on +y")

    def test_lowest_wheel_rests_on_z0_and_none_are_below(self):
        bottoms = {}
        for w in self.wheels():
            col = self.links[w].find("collision")
            r = float(col.find("geometry/cylinder/radius").text) if col is not None else float(
                self.links[w].find("visual/geometry/cylinder/radius").text)
            bottoms[w] = self._link_z(w) - r
        self.assertAlmostEqual(min(bottoms.values()), self.margin, places=5)
        for w, b in bottoms.items():
            self.assertGreaterEqual(b, self.margin - 1e-6, f"{w} below ground")

    def test_all_nominal_ground_wheels_have_collision(self):
        nominal = float(self.plugin.find("wheel_radius").text)
        for side in ("left", "right"):
            with_col = [w for w in self.wheels() if w.endswith(side) and self.links[w].find("collision") is not None]
            self.assertGreaterEqual(len(with_col), 4, f"{side}: need at least 4 ground wheels")
            for w in with_col:
                r = float(self.links[w].find("collision/geometry/cylinder/radius").text)
                if w not in [f"{n}" for n in self.params["overrides"]["free_spinning_wheels"]]:
                    self.assertAlmostEqual(r, nominal, places=6)

    def test_all_wheels_share_one_radius_and_contact_wheels_sit_level(self):
        radii = set()
        for w in self.wheels():
            r = float(self.links[w].find("visual/geometry/cylinder/radius").text)
            radii.add(round(r, 6))
        self.assertEqual(radii, {0.17}, "Pulley3_Left must be normalised to the common radius")
        bottoms = [self._link_z(w) - 0.17 for w in self.wheels() if self.links[w].find("collision") is not None]
        self.assertEqual(len(bottoms), 8)
        for b in bottoms:
            self.assertAlmostEqual(b, self.margin, places=5)

    def test_body_collision_clears_ground(self):
        z0 = self._link_z("chassis")
        c = self.links["chassis"].find("collision")
        zc = _nums(c.find("pose").text)[2]
        h = _nums(c.find("geometry/box/size").text)[2]
        self.assertGreater(z0 + zc - h / 2, self.margin + 0.05)

    def test_wheel_separation_matches_track_geometry(self):
        ys = sorted({abs(_nums(self.links[w].find("pose").text)[1]) for w in self.wheels()})
        self.assertEqual(len(ys), 1)
        factor = self.params["assumptions"]["turn_factor"]
        self.assertAlmostEqual(float(self.plugin.find("wheel_separation").text), 2 * ys[0] * factor, places=4)

    def test_side_torque_is_shared_over_driven_wheels(self):
        free = set(self.params["overrides"]["free_spinning_wheels"])
        for side in ("left", "right"):
            total = sum(float(self.joints[f"{w}_joint"].find("axis/limit/effort").text)
                        for w in self.wheels() if w.endswith(side) and w not in free)
            self.assertAlmostEqual(total, 1150.0, delta=0.05)

    # -- topics -------------------------------------------------------------

    def test_topics_match_params_record(self):
        m = self.params["model"]
        self.assertEqual(self.plugin.find("topic").text, m["cmd_vel_topic"])
        self.assertEqual(self.plugin.find("odom_topic").text, m["odometry_topic"])
        sensors = {s.get("name"): s.find("topic").text for s in self.model.iter("sensor")}
        self.assertEqual(sensors["gpu_lidar_front"], m["lidar_topic"])
        self.assertEqual(sensors["front_rgb_camera"], m["camera_topic"])

    # -- mesh placement (independent derivation) ----------------------------

    def test_runko_mesh_pose_equals_derived_placement(self):
        """Apply the SDF visual pose, and separately the derived Mevea-frame placement
        plus the axis mapping, to the same raw vertices; they must agree."""
        vis = self.links["chassis"].find("visual")
        x, y, z, roll, pitch, yaw = _nums(vis.find("pose").text)
        self.assertTrue(vis.find("geometry/mesh/uri").text.endswith("Runko_part.STL"))

        def sdf_rotate(v):
            cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                      math.sin(pitch), math.cos(yaw), math.sin(yaw))
            r = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
            return [sum(r[i][j] * v[j] for j in range(3)) + t for i, t in enumerate((x, y, z))]

        mevea_pos = (-1.53, -0.588, -0.804)  # derived: Runko_part.STL aligned to Jarcrac_body.STL, no rotation

        def mevea_then_map(v):
            return list(gen.mevea_to_gz(mevea_pos[0] + v[0], mevea_pos[1] + v[1], mevea_pos[2] + v[2]))

        for v in [(0, 0, 0), (2.98, 0, 0), (0, 2.056, 0), (0, 0, 1.568), (1.1, 0.9, 0.4)]:
            for a, b in zip(sdf_rotate(v), mevea_then_map(v)):
                self.assertAlmostEqual(a, b, places=4)

    def test_body_extent_matches_the_mevea_placed_body(self):
        """Regression guard: the Runko mesh must keep spanning the same lateral and
        vertical extent as the Mevea-placed Jarcrac_body.STL (width 1.568 m,
        bottom 0.588 m below the body origin)."""
        b = self.params["overrides"]["body_mesh"]
        self.assertAlmostEqual(b["bounds_gz_min_m"][1], -0.764, places=3)
        self.assertAlmostEqual(b["bounds_gz_max_m"][1], 0.804, places=3)
        self.assertAlmostEqual(b["bounds_gz_min_m"][2], -0.588, places=3)

    # -- assumptions are recorded, not hidden -------------------------------

    def test_assumptions_and_gaps_are_documented(self):
        a = self.params["assumptions"]
        for key in ("max_speed_mps", "turn_factor", "mu", "mu2", "per_wheel_effort_nm"):
            self.assertIn(key, a)
        self.assertIn("not_modelled", self.params)
        self.assertIn("rope-particle tracks", self.params["not_modelled"])

    def test_source_radius_anomaly_is_reported_and_normalised_with_evidence(self):
        a = self.params["source_anomalies"]
        self.assertEqual([x["item"] for x in a], ["Pulley3_Left"])
        self.assertIn("normalised", a[0]["action"])
        self.assertGreaterEqual(len(a[0]["evidence"]), 3)
        self.assertEqual(self.params["overrides"]["free_spinning_wheels"], [])

    def test_no_static_tilt_expected_after_normalisation(self):
        s = self.params["static_support_check"]
        self.assertFalse(s["evaluated"])


class GeneratorFunctionTests(unittest.TestCase):
    def test_axis_mapping_is_a_proper_rotation(self):
        # det of the mapping matrix must be +1 (no mirroring).
        basis = [gen.mevea_to_gz(*e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
        det = (basis[0][0] * (basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1])
               - basis[0][1] * (basis[1][0] * basis[2][2] - basis[1][2] * basis[2][0])
               + basis[0][2] * (basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0]))
        self.assertAlmostEqual(det, 1.0)

    def test_mevea_left_maps_to_gazebo_positive_y(self):
        self.assertGreater(gen.mevea_to_gz(0, 0, -0.635)[1], 0)  # Mevea z<0 is "Left"

    def test_cylinder_inertia_formula(self):
        axis, perp = gen.cylinder_inertia(10.0, 0.17, 0.18)
        self.assertAlmostEqual(axis, 0.1445, places=6)
        self.assertAlmostEqual(perp, 10 * (3 * 0.17 ** 2 + 0.18 ** 2) / 12, places=9)

    def test_static_support_check_logic(self):
        def wh(name, side, x, y, over=False, contact=True):
            return {"name": name, "source_pulley": name, "side": side, "pos": (x, y, 0.0),
                    "radius_m": 0.2 if over else 0.17, "free_spinning": over, "ground_contact": contact}
        wheels = [wh("l", "left", 0.0, 0.6, over=True), wh("r1", "right", -0.5, -0.6), wh("r2", "right", 0.5, -0.6)]
        inside = gen.static_support_check(wheels, (0.0, 0.0), 1.2, 0.17)
        outside = gen.static_support_check(wheels, (-0.9, 0.5), 1.2, 0.17)
        self.assertTrue(inside["com_inside_triangle"])
        self.assertFalse(outside["com_inside_triangle"])
        self.assertLess(outside["com_margin_to_support_triangle_m"], 0)
        self.assertFalse(gen.static_support_check([wh("a", "left", 0, 0.6), wh("b", "right", 0, -0.6)],
                                                  (0, 0), 1.2, 0.17)["evaluated"])

    def test_composite_body_parallel_axis(self):
        r = gen.composite_body(100.0, (0, 0, 0), {"ixx": 1, "iyy": 2, "izz": 3}, [(100.0, (2.0, 0, 0))])
        self.assertAlmostEqual(r["mass"], 200.0)
        self.assertAlmostEqual(r["com"][0], 1.0)
        I = r["I"]
        self.assertAlmostEqual(I[0][0], 1.0)            # offsets are along x: no Ixx contribution
        self.assertAlmostEqual(I[1][1], 2 + 100 + 100)  # 2 + m1*d1^2 + m2*d2^2
        self.assertAlmostEqual(I[2][2], 3 + 100 + 100)
        self.assertAlmostEqual(I[0][1], 0.0)
        # body only: unchanged
        r0 = gen.composite_body(50.0, (0.1, 0, 0.2), {"ixx": 4, "iyy": 5, "izz": 6}, [])
        self.assertEqual((r0["I"][0][0], r0["I"][1][1], r0["I"][2][2]), (4, 5, 6))
        self.assertEqual(r0["com"], (0.1, 0, 0.2))

    def test_belt_path_length_matches_geometry(self):
        # two equal circles: perimeter = 2*pi*r + 2*D
        pts, length = gen.belt_samples([(0.0, 0.0, 0.17), (2.0, 0.0, 0.17)])
        self.assertAlmostEqual(length, 2 * math.pi * 0.17 + 4.0, delta=0.005)
        self.assertGreater(len(pts), 2000)

    @unittest.skipUnless(os.environ.get("JARCRAC_MEVEA_XML"), "set JARCRAC_MEVEA_XML to check regeneration")
    def test_regeneration_reproduces_committed_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run([sys.executable, str(GENERATOR), "--mevea-xml", os.environ["JARCRAC_MEVEA_XML"],
                            "--out-dir", tmp], check=True, capture_output=True)
            for name in ("model.sdf", "model.config", "jarcrac_params.json"):
                self.assertEqual((Path(tmp) / name).read_text(), (MODEL_DIR / name).read_text(), name)
            self.assertEqual((Path(tmp) / "meshes" / "Runko_part.STL").read_bytes(),
                             (MODEL_DIR / "meshes" / "Runko_part.STL").read_bytes())

    @unittest.skipUnless(os.environ.get("JARCRAC_MEVEA_XML"), "set JARCRAC_MEVEA_XML to check the opt-out flags")
    def test_keep_source_radii_flag_restores_the_mevea_wheel(self):
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run([sys.executable, str(GENERATOR), "--mevea-xml", os.environ["JARCRAC_MEVEA_XML"],
                            "--out-dir", tmp, "--keep-source-radii"], check=True, capture_output=True)
            params = json.loads((Path(tmp) / "jarcrac_params.json").read_text())
            self.assertEqual(params["overrides"]["free_spinning_wheels"], ["wheel3_left"])
            self.assertTrue(params["static_support_check"]["evaluated"])
            sdf = ET.parse(Path(tmp) / "model.sdf").getroot().find("model")
            plugin = next(p for p in sdf.findall("plugin") if "DiffDrive" in p.get("name"))
            self.assertEqual(len(plugin.findall("left_joint")), 5)
            self.assertEqual(len(plugin.findall("right_joint")), 6)


if __name__ == "__main__":
    unittest.main()
