"""Offline checks: articulated-steering kinematics, the cmd_vel bridge mapping and
the Mevea mesh converters (with hand-built input files)."""

import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
GAZEBO = REPO / "edge-path-project" / "gazebo"
sys.path.insert(0, str(GAZEBO / "scripts"))
import mevea_mesh_tools as mt  # noqa: E402
import wheelloader_cmd_bridge as bridge  # noqa: E402
import wheelloader_kinematics as kin  # noqa: E402

PARAMS = json.loads((GAZEBO / "models" / "wheelloader" / "wheelloader_params.json").read_text())
K = PARAMS["kinematics"]
DF, DR, HT, R = K["d_f"], K["d_r"], K["half_track"], K["wheel_radius"]


class KinematicsTests(unittest.TestCase):
    def test_straight_line(self):
        s = kin.wheel_speeds(2.0, 0.0, DF, DR, HT)
        for w in ("rl", "rr", "fl", "fr"):
            self.assertAlmostEqual(s[w], 2.0)
        self.assertAlmostEqual(s["yaw_rate"], 0.0)

    def test_round_trip_twist_to_articulation_and_back(self):
        for v, w in [(1.0, 0.2), (1.0, -0.2), (-1.0, 0.1), (2.0, 0.5), (0.5, 0.05)]:
            phi = kin.articulation_for(v, w, DF, DR, 1.0)
            self.assertAlmostEqual(kin.yaw_rate(v, phi, DF, DR), w, places=9)

    def test_no_lateral_slip_at_either_axle(self):
        # Rigid-body check: the velocity of each axle centre must have no component across the axle.
        v, phi = 1.5, 0.25
        w = kin.yaw_rate(v, phi, DF, DR)
        lateral_front = -v * math.sin(phi) + DR * w * math.cos(phi) + DF * w
        self.assertAlmostEqual(lateral_front, 0.0, places=9)

    def test_left_turn_makes_the_left_wheels_slower(self):
        s = kin.wheel_speeds(1.0, 0.3, DF, DR, HT)
        self.assertGreater(s["yaw_rate"], 0)
        self.assertLess(s["rl"], s["rr"])
        self.assertLess(s["fl"], s["fr"])

    def test_articulation_is_clamped_and_zero_when_stopped(self):
        self.assertAlmostEqual(kin.articulation_for(1.0, 5.0, DF, DR, 0.33), 0.33)
        self.assertAlmostEqual(kin.articulation_for(1.0, -5.0, DF, DR, 0.33), -0.33)
        self.assertEqual(kin.articulation_for(0.0, 1.0, DF, DR, 0.33), 0.0)

    def test_minimum_turn_radius_at_full_articulation(self):
        phi = K["articulation_limit_rad"]
        radius = abs(1.0 / kin.yaw_rate(1.0, phi, DF, DR))
        self.assertTrue(4.0 < radius < 12.0, radius)  # a 12 t articulated loader turns inside ~10 m


class BridgeMappingTests(unittest.TestCase):
    def test_commands_cover_every_controlled_joint_with_declared_topics(self):
        cmds = bridge.build_commands(PARAMS, 1.0, 0.1)
        declared = set(PARAMS["model"]["wheel_cmd_topics"].values()) | {PARAMS["model"]["position_cmd_topics"]["articulation"]}
        self.assertEqual(set(cmds), declared)

    def test_speed_is_limited_and_wheel_rates_are_in_rad_per_s(self):
        cmds = bridge.build_commands(PARAMS, 99.0, 0.0)
        top = K["max_speed_mps"] / R
        for w in bridge.WHEELS:
            self.assertAlmostEqual(cmds[PARAMS["model"]["wheel_cmd_topics"][w]], top, places=6)
        self.assertAlmostEqual(cmds[PARAMS["model"]["position_cmd_topics"]["articulation"]], 0.0)

    def test_stop_command_is_all_zero(self):
        self.assertTrue(all(v == 0.0 for v in bridge.build_commands(PARAMS, 0.0, 0.0).values()))


def chunk(cid, payload):
    return struct.pack("<HI", cid, 6 + len(payload)) + payload


class MeshToolTests(unittest.TestCase):
    OSGT = """osg::Geode
{
  osg::Geometry
  {
    Name "tri patch"
    StateSet TRUE
    {
      Diffuse   TRUE Front  0.100  0.200  0.300  1.000 Back  0.100  0.200  0.300  1.000
    }
    PrimitiveSetList 1
    {
      DrawElementsUInt GL_TRIANGLES 3
      {
        0 1 2
      }
    }
    VertexData
    {
      Array TRUE ArrayID 1 Vec3fArray 3
      {
        0.0 0.0 0.0
        1.0 0.0 0.0
        0.0 2.0 3.0
      }
      Indices FALSE
    }
    NormalData
    {
      Array TRUE ArrayID 2 Vec3fArray 3
      {
        0.0 1.0 0.0
        0.0 1.0 0.0
        0.0 1.0 0.0
      }
      Indices FALSE
    }
  }
}
"""

    def test_osgt_to_obj_applies_the_axis_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "a.osgt"
            src.write_text(self.OSGT)
            geoms = mt.read_osgt(src)
            self.assertEqual(len(geoms), 1)
            self.assertEqual(geoms[0]["triangles"], [(0, 1, 2)])
            self.assertEqual(geoms[0]["diffuse"], (0.1, 0.2, 0.3, 1.0))
            stats = mt.write_obj(geoms, Path(tmp) / "a.obj", to_gazebo=True)
            lines = (Path(tmp) / "a.obj").read_text().splitlines()
            v = [l for l in lines if l.startswith("v ")]
            self.assertEqual(v[2], "v 0.00000 -3.00000 2.00000")   # Mevea (0, 2, 3) -> Gazebo (0, -3, 2)
            vn = [l for l in lines if l.startswith("vn ")]
            self.assertEqual(vn[0], "vn 0.0000 0.0000 1.0000")      # Mevea up (y) -> Gazebo up (z)
            self.assertIn("o tri_patch", lines)
            self.assertEqual(stats["triangles"], 1)
            self.assertIn("Kd 0.1000 0.2000 0.3000", (Path(tmp) / "a.mtl").read_text())

    def test_3ds_to_stl(self):
        verts = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        v_payload = struct.pack("<H", 4) + b"".join(struct.pack("<3f", *v) for v in verts)
        faces = [(0, 1, 2), (0, 1, 3)]
        f_payload = struct.pack("<H", 2) + b"".join(struct.pack("<4H", a, b, c, 0) for a, b, c in faces)
        data = chunk(0x4D4D, chunk(0x3D3D, chunk(0x4000, b"box\0" + chunk(0x4100, chunk(0x4110, v_payload) + chunk(0x4120, f_payload)))))
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "t.3ds"
            src.write_bytes(data)
            v, f = mt.read_3ds(src)
            self.assertEqual(len(v), 4)
            self.assertEqual(f, [(0, 1, 2), (0, 1, 3)])
            stats = mt.write_stl(v, f, Path(tmp) / "t.stl", to_gazebo=True)
            raw = (Path(tmp) / "t.stl").read_bytes()
            self.assertEqual(len(raw), 84 + 50 * 2)
            self.assertEqual(stats["bounds"][1], [1.0, 0.0, 1.0])
            nx, ny, nz = struct.unpack_from("<3f", raw, 84)
            self.assertAlmostEqual(ny, -1.0)  # face normal +z_mevea maps to -y_gazebo
