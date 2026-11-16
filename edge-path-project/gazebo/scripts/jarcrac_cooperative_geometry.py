"""Collision envelopes of the two committed Gazebo models."""
from dataclasses import dataclass
from itertools import product
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from final_sil_v6_checks import hull

GAZEBO = Path(__file__).resolve().parents[1]
MODELS = GAZEBO / "models"
MODEL_NAMES = ("jarcrac_detailed_tracks", "aeb_vehicle_360_b")


def rotate(point, roll, pitch, yaw):
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


def apply_pose(point, pose):
    return tuple(a + b for a, b in zip(pose[:3], rotate(point, *pose[3:])))


def pose_of(element):
    pose = element.find("pose")
    if pose is None:
        return (0.0,) * 6
    if pose.attrib:
        raise ValueError("Collision envelope expects a pose in its parent frame")
    values = tuple(map(float, pose.text.split()))
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError("Invalid collision pose")
    return values


def binary_stl_vertices(path):
    data = path.read_bytes()
    if len(data) < 84 or len(data) != 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
        raise ValueError(f"Expected a binary collision STL in {path}")
    vertices = {
        point for triangle in struct.iter_unpack("<12fH", data[84:])
        for point in (triangle[3:6], triangle[6:9], triangle[9:12])
    }
    if not vertices or not all(math.isfinite(x) for p in vertices for x in p):
        raise ValueError(f"Invalid collision mesh in {path}")
    return vertices


def collision_vertices(models_dir, geometry, pose, steerable):
    box = geometry.findtext("box/size")
    if box:
        return [apply_pose(p, pose) for p in product(*[(-v / 2, v / 2) for v in map(float, box.split())])]
    mesh = geometry.find("mesh")
    if mesh is not None:
        uri = mesh.findtext("uri")
        scale = tuple(map(float, mesh.findtext("scale", "1 1 1").split()))
        if not uri.startswith("model://") or len(scale) != 3:
            raise ValueError("Unexpected collision mesh reference")
        return [apply_pose(tuple(v * s for v, s in zip(p, scale)), pose)
                for p in binary_stl_vertices(models_dir / uri.removeprefix("model://"))]
    cylinder = geometry.find("cylinder")
    if cylinder is None:
        raise ValueError(f"Unsupported collision geometry {ET.tostring(geometry, encoding='unicode')}")
    radius = float(cylinder.findtext("radius"))
    half_width = float(cylinder.findtext("length")) / 2
    if steerable:
        # Bound the complete yaw sweep of the tire.
        radial = math.hypot(radius, half_width) / math.cos(math.pi / 8)
        return [(pose[0] + radial * math.cos(i * math.pi / 4),
                 pose[1] + radial * math.sin(i * math.pi / 4), pose[2] + z)
                for i in range(8) for z in (-radius, radius)]
    # A circumscribed octagon keeps the fixed wheel cylinder conservative.
    radial = radius / math.cos(math.pi / 8)
    return [apply_pose((radial * math.cos(i * math.pi / 4),
                        radial * math.sin(i * math.pi / 4), z), pose)
            for i in range(8) for z in (-half_width, half_width)]


@dataclass(frozen=True)
class Footprint:
    vertices: tuple

    @classmethod
    def from_model(cls, models_dir, name):
        model = ET.parse(models_dir / name / "model.sdf").getroot().find("model")
        if model is None:
            raise ValueError(f"Missing model {name}")
        vertices = set()
        for link in model.findall("link"):
            link_pose = pose_of(link)
            steerable = link.get("name") in ("front_left_wheel", "front_right_wheel")
            for collision in link.findall("collision"):
                vertices.update(apply_pose(p, link_pose) for p in collision_vertices(
                    models_dir, collision.find("geometry"), pose_of(collision), steerable))
        if not vertices:
            raise ValueError(f"No collision geometry in {name}")
        return cls(tuple(sorted(vertices)))

    def at(self, state):
        projected = (rotate(p, state.roll, state.pitch, state.yaw) for p in self.vertices)
        return hull((state.x + x, state.y + y) for x, y, _ in projected)

    def bounds(self, roll=0.0, pitch=0.0):
        points = tuple(rotate(p, roll, pitch, 0.0) for p in self.vertices)
        return tuple((min(axis), max(axis)) for axis in zip(*points))

    def clearance_samples(self, roll, pitch, margin):
        points = (rotate(p, roll, pitch, 0.0) for p in self.vertices)
        boundary = hull((x+dx, y+dy) for x, y, _ in points
                        for dx, dy in product((-margin, margin), repeat=2))
        center = tuple(sum(p[i] for p in boundary)/len(boundary) for i in (0, 1))
        result = [center]
        for p, q in zip(boundary, (*boundary[1:], boundary[0])):
            steps = max(1, math.ceil(math.dist(p, q)/.40))
            for i in range(steps):
                x = p[0]+(q[0]-p[0])*i/steps
                y = p[1]+(q[1]-p[1])*i/steps
                result.extend(((x, y), ((x+center[0])/2, (y+center[1])/2)))
        return tuple(result)

    @property
    def radius(self):
        return max(math.sqrt(x*x + y*y + z*z) for x, y, z in self.vertices)


def model_footprints(models_dir=MODELS):
    return tuple(Footprint.from_model(models_dir, name) for name in MODEL_NAMES)
