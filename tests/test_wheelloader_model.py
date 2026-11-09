"""Offline checks for the generated Gazebo ``wheelloader`` model.

They need neither Gazebo nor the Mevea files.  Set WHEELLOADER_MEVEA_DIR to the
Mevea ``WheelLoader`` folder to also check that regeneration reproduces the
committed model byte for byte.
"""

import hashlib
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
MODEL_DIR = GAZEBO / "models" / "wheelloader"
WORLD = GAZEBO / "worlds" / "wheelloader_flat_test_world.sdf"
GENERATOR = GAZEBO / "scripts" / "generate_wheelloader_model.py"

sys.path.insert(0, str(GAZEBO / "scripts"))
import generate_wheelloader_model as gen  # noqa: E402

LINKS = {"rear_frame", "rear_axle", "front_frame", "lift_arm", "bucket_clamp", "bucket",
         "wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr"}


def nums(text):
    return [float(v) for v in text.split()]


class WheelLoaderModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(MODEL_DIR / "model.sdf").getroot().find("model")
        cls.params = json.loads((MODEL_DIR / "wheelloader_params.json").read_text())
        cls.links = {l.get("name"): l for l in cls.model.findall("link")}
        cls.joints = {j.get("name"): j for j in cls.model.findall("joint")}
        cls.z_off = cls.params["initial_pose"]["z_offset_m"]

    def pose(self, link):
        return nums(self.links[link].find("pose").text)

    def axis(self, joint):
        return nums(self.joints[joint].find("axis/xyz").text)

    # -- structure ------------------------------------------------------------

    def test_files_are_well_formed(self):
        ET.parse(WORLD)
        ET.parse(MODEL_DIR / "model.config")

    def test_link_set_and_kinematic_tree(self):
        self.assertEqual(set(self.links), LINKS)
        parent_of = {}
        for j in self.joints.values():
            child = j.find("child").text
            self.assertNotIn(child, parent_of, f"{child} has two parent joints (closed loop)")
            parent_of[child] = j.find("parent").text
        self.assertEqual(set(parent_of), LINKS - {"rear_frame"})
        for link in LINKS - {"rear_frame"}:  # every chain must reach the root without a cycle
            seen, cur = set(), link
            while cur != "rear_frame":
                self.assertNotIn(cur, seen)
                seen.add(cur)
                cur = parent_of[cur]
        self.assertEqual(next(iter(self.model.findall("link"))).get("name"), "rear_frame", "canonical link")

    def test_joint_types_and_axes(self):
        def close(a, b):
            for x, y in zip(a, b):
                self.assertAlmostEqual(x, y, places=6)
        close(self.axis("articulation_joint"), [0, 0, 1])      # vertical pivot, positive = turn left
        close(self.axis("axle_oscillation_joint"), [1, 0, 0])  # longitudinal roll axis
        close(self.axis("lift_joint"), [0, -1, 0])             # positive = raise
        close(self.axis("bucket_tilt_joint"), [0, -1, 0])      # positive = rack back
        for w in ("rl", "rr", "fl", "fr"):
            close(self.axis(f"wheel_{w}_joint"), [0, 1, 0])    # positive = roll forward
        self.assertEqual(self.joints["bucket_fixed_joint"].get("type"), "fixed")

    def test_joint_limits_follow_mevea(self):
        lim = lambda j: (float(self.joints[j].find("axis/limit/lower").text), float(self.joints[j].find("axis/limit/upper").text))
        theta0 = math.radians(41.0)  # initial clamp angle relative to the lift arm (CS_BucketClamp rz)
        lo, hi = lim("bucket_tilt_joint")
        self.assertAlmostEqual(lo, -1.4835 - theta0, delta=2e-3)
        self.assertAlmostEqual(hi, 1.3632 - theta0, delta=2e-3)
        lo, hi = lim("axle_oscillation_joint")
        self.assertAlmostEqual(lo, -0.2094, places=4)
        self.assertAlmostEqual(hi, 0.2094, places=4)
        lo, hi = lim("articulation_joint")
        self.assertAlmostEqual(lo, -hi, places=3)
        self.assertTrue(0.25 < hi < 0.5)
        lo, hi = lim("lift_joint")
        self.assertTrue(lo < 0 < hi)

    def test_cylinder_geometry_reproduces_mevea_lengths(self):
        by = {c["cylinder"]: c for c in self.params["joint_range_checks"]}
        for n in ("ForceT_SteeringL", "ForceT_SteeringR"):  # steering is straight at the start
            self.assertAlmostEqual(by[n]["length_at_initial_pose"], by[n]["L0_in_mevea"], places=3)
            self.assertAlmostEqual(by[n]["joint_angle_where_length_equals_L0"], 0.0, places=3)
        for n in ("ForceT_BucketLiftL", "ForceT_BucketLiftR"):  # lift L0 = arm lowered on the ground
            self.assertAlmostEqual(by[n]["joint_angle_where_length_equals_L0"], -0.40, delta=0.01)

    def test_cylinder_interval_on_a_known_geometry(self):
        # Cylinder from A=(0,1,0) to B0=(1,0,0) on an arm pivoting about the origin (axis z):
        # L(q)^2 = 2 - 2 sin(q), so L=1 at q=+30 deg and L=sqrt(3) at q=-30 deg.
        lo, hi, L0 = gen.cylinder_interval([0, 1, 0], [1, 0, 0], [0, 0, 0], [0, 0, 1], 1.0, math.sqrt(3))
        self.assertAlmostEqual(L0, math.sqrt(2), places=9)
        self.assertAlmostEqual(lo, -math.radians(30), delta=1e-3)
        self.assertAlmostEqual(hi, math.radians(30), delta=1e-3)

    # -- mass properties -------------------------------------------------------

    def test_masses(self):
        masses = {n: float(l.find("inertial/mass").text) for n, l in self.links.items()}
        total = sum(masses.values())
        self.assertAlmostEqual(total, self.params["mass"]["gazebo_total_kg"], delta=0.01)
        self.assertAlmostEqual(self.params["mass"]["mevea_total_kg"], 12594.177, delta=0.01)
        self.assertAlmostEqual(total - 4 * 30.0, self.params["mass"]["mevea_total_kg"], delta=0.01)
        self.assertAlmostEqual(masses["lift_arm"], 688 + 312, places=3)       # + bucket lever
        self.assertAlmostEqual(masses["bucket_clamp"], 385 + 80, places=3)    # + bucket link
        self.assertAlmostEqual(masses["bucket"], 1105, places=3)

    def test_inertia_tensors_are_physical(self):
        for name, link in self.links.items():
            i = link.find("inertial/inertia")
            v = {k: float(i.find(k).text) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
            m = [[v["ixx"], v["ixy"], v["ixz"]], [v["ixy"], v["iyy"], v["iyz"]], [v["ixz"], v["iyz"], v["izz"]]]
            self.assertTrue(gen.check_pos_def(m), name)
            self.assertLessEqual(v["ixx"], v["iyy"] + v["izz"] + 1e-9, name)
            self.assertLessEqual(v["iyy"], v["ixx"] + v["izz"] + 1e-9, name)
            self.assertLessEqual(v["izz"], v["ixx"] + v["iyy"] + 1e-9, name)

    # -- geometry ----------------------------------------------------------------

    def test_frame_chain_against_hand_calculation(self):
        """Independent of the generator: positions from numbers read off the Mevea CS tables."""
        pivot_z = 0.974  # rear axle height 0.73 + RearFrame offset 0.244
        x, y, z, *_ = self.pose("rear_frame")
        self.assertAlmostEqual(x, 1.46, places=5)
        self.assertAlmostEqual(z, pivot_z + self.z_off, places=5)
        x, y, z, roll, pitch, yaw = self.pose("lift_arm")
        self.assertAlmostEqual(x, 1.46 + 0.765, places=5)                  # FrontFrame.CS LiftArm
        self.assertAlmostEqual(z, pivot_z + 1.153 + self.z_off, places=5)
        self.assertTrue(0.3 < pitch < 0.4, "arm starts tilted down by ~20 deg (positive pitch = nose down)")
        bx, by, bz, *_ = self.pose("bucket_clamp")                          # LiftArm.CS_LiftArm_BucketClamp = (2.809, 0)
        self.assertAlmostEqual(bx, x + 2.809 * math.cos(pitch), places=5)
        self.assertAlmostEqual(bz, z - 2.809 * math.sin(pitch), places=5)
        self.assertAlmostEqual(by, 0.0, places=6)

    def test_tyres(self):
        k = self.params["kinematics"]
        bottoms = []
        for w in ("rl", "rr", "fl", "fr"):
            x, y, z, *_ = self.pose(f"wheel_{w}")
            bottoms.append(z - 0.729)
            self.assertEqual(y > 0, w.endswith("l"), f"{w} on the wrong side")
            self.assertAlmostEqual(abs(y), k["half_track"], places=4)
            r = float(self.links[f"wheel_{w}"].find("collision/geometry/cylinder/radius").text)
            self.assertAlmostEqual(r, 0.729, places=6)
        self.assertAlmostEqual(min(bottoms), 0.003, places=4)  # lowest tyre rests on z=0 (+3 mm margin)
        self.assertAlmostEqual(k["wheelbase"], 2.905, places=3)
        self.assertAlmostEqual(k["d_r"] + k["d_f"], k["wheelbase"], places=3)

    def test_speed_is_derived_from_the_mevea_powertrain(self):
        s = self.params["speed_derivation"]
        self.assertAlmostEqual(s["total_ratio_top_gear"], 5.572 * 2.086 * 6.0, delta=0.05)
        self.assertAlmostEqual(self.params["kinematics"]["max_speed_mps"], s["engine_max_radps"] / s["total_ratio_top_gear"] * 0.729, places=3)
        self.assertAlmostEqual(self.params["kinematics"]["max_speed_mps"], 2.84, delta=0.02)

    def test_cameras_look_forward_and_backward(self):
        seat = nums(self.links["rear_frame"].find("sensor[@name='camera_seat']/pose").text)
        rear = nums(self.links["rear_frame"].find("sensor[@name='camera_rear']/pose").text)
        self.assertAlmostEqual(seat[5], 0.0, places=3)           # yaw: forward
        self.assertTrue(0.0 < seat[4] < 0.3)                      # pitch: slightly down
        self.assertGreater(abs(rear[5]), 3.0)                     # yaw ~ pi: backward

    def test_initial_pose_validation_record(self):
        v = self.params["validation"]
        self.assertLess(v["joint_coincidence_max_position_error_m"], 1e-9)
        self.assertLess(v["joint_coincidence_max_rotation_error_rad"], 1e-9)
        self.assertEqual(len(v["true_joints_checked"]), 8)
        gap = v["loop_closure_joint"]["BucketLink_BucketClamp"]["position_gap_m"]
        self.assertLess(gap, 0.005)
        self.assertTrue(v["all_inertia_tensors_positive_definite"])

    # -- assets and controllers ---------------------------------------------------

    def test_every_mesh_uri_resolves(self):
        uris = [u.text for u in self.model.iter("uri")]
        self.assertGreaterEqual(len(uris), 15)
        for u in uris:
            self.assertTrue(u.startswith("model://wheelloader/"))
            f = MODEL_DIR / u[len("model://wheelloader/"):]
            self.assertTrue(f.is_file(), u)
            if f.suffix == ".obj":
                self.assertTrue(f.with_suffix(".mtl").is_file())
            if f.suffix == ".stl":
                n = int.from_bytes(f.read_bytes()[80:84], "little")
                self.assertEqual(f.stat().st_size, 84 + 50 * n)

    def test_lever_and_link_are_lumped_not_modelled(self):
        text = (MODEL_DIR / "model.sdf").read_text()
        self.assertNotIn("bucket_lever", text)
        self.assertNotIn("bucket_link", text)
        self.assertIn("BucketLever", json.dumps(self.params["mass"]["lumped"]))

    def test_plugins_reference_existing_joints_and_declared_topics(self):
        topics = []
        pos_joints = set()
        for p in self.model.findall("plugin"):
            jn = p.find("joint_name")
            if jn is None:
                continue
            self.assertIn(jn.text, self.joints)
            topics.append(p.find("topic").text)
            if "Position" in p.get("name"):
                pos_joints.add(jn.text)
        self.assertEqual(len(topics), len(set(topics)), "duplicate command topics")
        self.assertEqual(pos_joints, {"articulation_joint", "lift_joint", "bucket_tilt_joint"})
        declared = set(self.params["model"]["wheel_cmd_topics"].values()) | set(self.params["model"]["position_cmd_topics"].values())
        self.assertEqual(set(topics), declared)

    # -- generator maths ------------------------------------------------------------

    def test_mevea_euler_convention_gives_the_expected_joint_axes(self):
        z = [0, 0, 1]
        ax = gen.mv(gen.rot_mevea(math.pi / 2, math.pi / 2, 0), z)       # RearAxle_RearFrame CS
        self.assertAlmostEqual(ax[0], 1.0)
        self.assertAlmostEqual(ax[1], 0.0)
        ax = gen.mv(gen.rot_mevea(-math.pi / 2, 0, 0), z)                # RearFrame_FrontFrame CS
        self.assertAlmostEqual(ax[1], 1.0)

    def test_axis_mapping_is_a_proper_rotation_and_rpy_round_trips(self):
        M = gen.M_GZ
        det = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
               + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
        self.assertAlmostEqual(det, 1.0)
        for rpy in [(0.1, -0.2, 0.3), (1.2, 0.4, -2.0), (0.0, 0.0, math.pi)]:
            R = gen.mmul(gen.mmul(gen.rot_z(rpy[2]), gen.rot_y(rpy[1])), gen.rot_x(rpy[0]))
            back = gen.rpy_from_matrix(R)
            for a, b in zip(rpy, back):
                self.assertAlmostEqual(a, b, places=9)

    def test_composite_of_two_equal_masses(self):
        I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        c = gen.composite([(10.0, [1, 0, 0], I), (10.0, [-1, 0, 0], I)])
        self.assertAlmostEqual(c["mass"], 20.0)
        self.assertAlmostEqual(c["com"][0], 0.0)
        self.assertAlmostEqual(c["I"][0][0], 2.0)        # offsets along x add nothing to Ixx
        self.assertAlmostEqual(c["I"][1][1], 2.0 + 20.0)
        self.assertAlmostEqual(c["I"][2][2], 2.0 + 20.0)

    @unittest.skipUnless(os.environ.get("WHEELLOADER_MEVEA_DIR"), "set WHEELLOADER_MEVEA_DIR to check regeneration")
    def test_regeneration_reproduces_committed_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run([sys.executable, str(GENERATOR), "--mevea-dir", os.environ["WHEELLOADER_MEVEA_DIR"],
                            "--out-dir", tmp], check=True, capture_output=True)
            new = {p.relative_to(tmp).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(tmp).rglob("*") if p.is_file()}
            old = {p.relative_to(MODEL_DIR).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in MODEL_DIR.rglob("*") if p.is_file() and p.name not in ("README.md", "geometry_check.png")}
            self.assertEqual(new, old)


if __name__ == "__main__":
    unittest.main()
