#!/usr/bin/env python3
"""Generate the Gazebo ``jarcrac`` model from a Mevea JARCRAC model XML.

The Mevea model is the source of truth for geometry, mass properties and
sensor placement.  This script extracts those values and writes:

    <out>/model.sdf              Gazebo model (SDF 1.10)
    <out>/model.config
    <out>/meshes/<body>.STL      body mesh copied from the Mevea Graphics dir
    <out>/jarcrac_params.json    every extracted value + every assumption

The output is deterministic (no timestamps), so it can be snapshot-tested.

What is taken from Mevea
    body mass / inertia / centre of mass, body mesh and its placement,
    pulley positions, radii and masses, cabin camera pose and field of view.

Project decisions encoded as defaults (override on the command line)
    * Body mass comes from the Mevea XML (2000 kg in the Jan-2026 free-roll
      model, the revision whose track masses were rebalanced "closer to real").
      The Mevea track (rope) mass, which Gazebo does not simulate, is added to
      the chassis together with its inertia, so the Gazebo total matches the
      Mevea total.  --body-mass-kg overrides the body mass.
    * Pulley3_Left has radius 0.2 m in every Mevea revision while the other 11
      pulleys have 0.17 m.  It is a leftover default (rev1 had 0.2 m on pulleys
      2-6 and the small one; only this one was never corrected), so by default
      it is normalised to 0.17 m.  --keep-source-radii keeps it.
    * The detailed Runko_part.STL body mesh.
    See "overrides" in jarcrac_params.json.

What is NOT in Mevea and is therefore an explicit, tunable ASSUMPTION
    top speed, accelerations, wheel friction, turning (slip) factor,
    per-wheel drive effort, wheel width, lidar placement.
    These are listed under "assumptions" in jarcrac_params.json.

What is deliberately NOT modelled
    the 58-particle rope tracks, the diesel/hydraulic/planetary drivetrain,
    and the small upper return roller.  Tracks are approximated by driven
    road wheels (skid steer) and the drivetrain by a velocity command with a
    torque limit.  Calibrate against the Mevea free-roll / brake runs.

Axis convention
    Mevea is Y-up (x forward, y up, z right).  Gazebo is Z-up
    (x forward, y left, z up).  Mevea (x, y, z) -> Gazebo (x, -z, y).
    This is a proper rotation (+90 deg about X), so inertia axes permute
    without sign changes.

Usage
    python3 generate_jarcrac_model.py \\
        --mevea-xml path/to/Jarcrac_Rusko_..._free_roll_test2.xml \\
        --out-dir  ../models/jarcrac
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Defaults for quantities that Mevea does not define (ASSUMPTIONS)
# ---------------------------------------------------------------------------

DEFAULTS = {
    # 0 = use the body mass found in the Mevea XML (project decision).  Jarcrac_rev7
    # says 2460 kg; its inertia and CoM are identical, so inertia is not rescaled.
    "body_mass_kg": 0.0,
    # ~75 rpm hydraulic-motor output (from "JARCRAC Mevea malli.docx") times
    # the 0.17 m pulley radius gives ~1.3 m/s.  Verify against the real machine.
    "max_speed_mps": 1.3,
    "max_accel_mps2": 1.0,
    "max_yaw_rate_radps": 1.0,
    # Effective-track-width multiplier for skid steer.  1.0 = ideal kinematics.
    # Real tracks scrub, so the effective width is larger; calibrate.
    "turn_factor": 1.0,
    # Mevea pulley mass is 10 kg; Large_pulley.STL is 0.18 m wide.
    "wheel_width_m": 0.18,
    # Friction: mu along rolling direction, mu2 lateral.  Calibrate on soil.
    "mu": 1.0,
    "mu2": 0.5,
    # Mevea TorqueSource T (per side) is shared over that side's wheels.
    "side_torque_nm": 1150.0,
    # Front-centre LiDAR: x forward of the front-most body collision face (x=1.45 m).
    # Sensor pose is relative to the chassis link; chassis link starts at z=0.711 m.
    # Thus z=-0.111 m gives a nominal 0.600 m height above flat ground.
    # A physical installation would require a robust forward mounting bracket.
    "lidar_x_m": 1.68,
    "lidar_y_m": 0.0,
    "lidar_z_m": -0.111,
    # Generic 32-layer 3D lidar: chosen for forestry AEB, NOT a vendor-specific scanner.
    # Uniform beam elevations; real spinning lidars often have nonuniform patterns.
    "lidar_vertical_channels": 32,
    "lidar_vertical_min_deg": -15.0,
    "lidar_vertical_max_deg": 15.0,
    "lidar_horizontal_samples": 721,
    "lidar_scan_rate_hz": 10.0,
    "lidar_range_min_m": 0.30,
    "lidar_range_max_m": 30.0,
    "lidar_noise_stddev_m": 0.015,
    "ground_clearance_margin_m": 0.003,
}

# Evidence gathered by inspecting all Mevea revisions in the ruskotunturi repo
# (rev1-rev7 in Jarcrac-drive, 2025-11-21 .. 2026-01-05 in Jarcrac-freeroll).
ANOMALY_EVIDENCE = {
    "Pulley3_Left": [
        "radius is 0.2 m in every Mevea revision inspected (rev1 .. 2026-01-05)",
        "rev1 had 0.2 m on Pulley2-6_Left and Small_pulley_Left; later revisions corrected all of "
        "them except Pulley3_Left, which looks like a missed leftover default",
        "the mirror pulley Pulley3_Right and its visual (the 0.17 m Large_pulley.STL) use 0.17 m",
        "a 0.2 m wheel at the same height makes the left track 3 cm lower at one point, which "
        "tilts a rigid vehicle",
    ],
}

# Runko_part.STL is not referenced by any Mevea model.  Its placement was derived
# by aligning it to the Mevea-placed Jarcrac_body.STL: no rotation, median
# nearest-vertex distance 1.5 mm.  The two meshes describe the same body.
RUNKO_MESH = {
    "file": "Graphics/Runko_part.STL",
    "mevea_pos": (-1.53, -0.588, -0.804),
    "mevea_ry_rad": 0.0,
    "provenance": "derived by chamfer alignment to the Mevea-placed Jarcrac_body.STL "
                  "(median nearest-vertex distance 1.5 mm); not stated in any Mevea file",
}

GROUND_CONTACT_TOL_M = 0.01  # wheels whose bottom is this close to the lowest are collision wheels


# ---------------------------------------------------------------------------
# Frame conversion
# ---------------------------------------------------------------------------

def mevea_to_gz(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Mevea (x fwd, y up, z right) -> Gazebo (x fwd, y left, z up)."""
    return (x, -z, y)


def fmt(v: float) -> str:
    """Stable number formatting (no -0, trimmed)."""
    if abs(v) < 1e-12:
        v = 0.0
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def pose(x, y, z, r=0.0, p=0.0, yw=0.0) -> str:
    return " ".join(fmt(v) for v in (x, y, z, r, p, yw))


# ---------------------------------------------------------------------------
# Mevea parsing
# ---------------------------------------------------------------------------

def _f(elem: ET.Element, key: str) -> float:
    return float(elem.attrib[key])


def _pos(cs: ET.Element) -> tuple[float, float, float]:
    p = cs.find("Position")
    return (_f(p, "x"), _f(p, "y"), _f(p, "z"))


def stl_bounds(path: Path) -> tuple[list[float], list[float]]:
    """Axis-aligned bounds (min, max) of a binary or ASCII STL."""
    data = path.read_bytes()
    pts: list[tuple[float, float, float]] = []
    if data[:5] == b"solid" and b"facet" in data[:400]:
        import re
        for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data):
            pts.append(tuple(float(g) for g in m.groups()))  # type: ignore[arg-type]
    else:
        n = struct.unpack("<I", data[80:84])[0]
        for i in range(n):
            v = struct.unpack("<12fH", data[84 + i * 50: 84 + (i + 1) * 50])
            pts += [v[3:6], v[6:9], v[9:12]]
    mn = [min(p[i] for p in pts) for i in range(3)]
    mx = [max(p[i] for p in pts) for i in range(3)]
    return mn, mx


def parse_mevea(xml_path: Path) -> dict:
    root = ET.parse(xml_path).getroot()

    body = root.find("Bodies/Jarcarc_body")  # (sic) spelling used in the Mevea model
    if body is None:
        raise SystemExit("Bodies/Jarcarc_body not found - is this a JARCRAC Mevea XML?")
    inertia = body.find("Inertia")
    inertia_i = inertia.find("I")
    cs_by_name = {c.tag: c for c in body.find("CS")}

    gfx = root.find("Graphics/Graphics_Jarcrac_body")
    gfx_pos = gfx.find("Position")
    gfx_ori = gfx.find("Orientation")

    # Pulleys: radius and mass from the rope system, position from body CS.
    rope_cons = root.find("Particles/RopeSystems/RopeConstraints")
    pulleys = []
    for name, con in ((c.tag, c) for c in rope_cons):
        cs_name = con.attrib["AttachmentBodyCS"].split(".", 1)[1]
        side = "left" if name.endswith("_Left") else "right"
        pulleys.append({
            "name": name,
            "side": side,
            "mevea_pos": _pos(cs_by_name[cs_name]),
            "radius_m": _f(con, "Radius"),
            "mass_kg": _f(con.find("Inertia"), "mass"),
            "is_small_roller": name.startswith("Small_pulley"),
            "has_brake_input": con.attrib.get("InputPrimitiveType", "None") == "Brake",
        })

    ropes = {}
    for r in root.find("Particles/RopeSystems/Ropes"):
        side = "right" if "Right" in r.tag else "left"
        ropes[side] = {"name": r.tag, "density": _f(r, "Density"), "radius_m": _f(r, "RopeRadius"),
                       "scale_L": _f(r, "scale_L"), "nparticles": int(r.attrib["nparticles"])}
    rope = root.find("Particles/RopeSystems/Ropes/CrawlerLeft")
    torque_src = root.find("Forces/PowerTrain/Sources/TorqueSource_LeftWheel")
    diesel = root.find("Forces/Motor/MeveaDiesel")

    # Only the cabin camera (present in the drive/rev7 models) is a forward-looking
    # vehicle camera.  The free-roll models only have chase cameras, so fall back
    # to the rev7 cabin-camera values and say so.
    if "Camera_cabin" in cs_by_name:
        camera = {"name": "Camera_cabin", "mevea_pos": _pos(cs_by_name["Camera_cabin"]), "fov_deg": 45.0,
                  "is_default": False}
    else:
        camera = {"name": "Camera_cabin (default from Jarcrac_rev7.xml)", "mevea_pos": (1.3, 1.0, 0.0),
                  "fov_deg": 45.0, "is_default": True}

    return {
        "body": {
            "mass_kg": _f(inertia, "mass"),
            "Ixx": _f(inertia_i, "Ixx"), "Iyy": _f(inertia_i, "Iyy"), "Izz": _f(inertia_i, "Izz"),
            "cm_mevea": _pos(cs_by_name["CM"]),
        },
        "mesh": {
            "file": gfx.attrib["FileName"],
            "scale": float(gfx.attrib.get("Scale", 1)),
            "mevea_pos": (_f(gfx_pos, "x"), _f(gfx_pos, "y"), _f(gfx_pos, "z")),
            "mevea_ry_rad": _f(gfx_ori, "ry"),
            "mevea_rx_rad": _f(gfx_ori, "rx"),
            "mevea_rz_rad": _f(gfx_ori, "rz"),
        },
        "pulleys": pulleys,
        "ropes": ropes,
        "tracks_not_modelled": {
            "particles_per_track": int(rope.attrib["nparticles"]),
            "rope_radius_m": _f(rope, "RopeRadius"),
            "density": _f(rope, "Density"),
        },
        "drivetrain_reference": {
            "diesel_kw": _f(diesel, "Kw") if diesel is not None else None,
            "torque_source_side_nm": _f(torque_src, "T") if torque_src is not None else None,
        },
        "camera": camera,
    }


# ---------------------------------------------------------------------------
# Model derivation
# ---------------------------------------------------------------------------

def cylinder_inertia(m: float, r: float, length: float) -> tuple[float, float]:
    """(about-axis, perpendicular) inertia of a solid cylinder."""
    return 0.5 * m * r * r, m * (3 * r * r + length * length) / 12.0


def _hull(points: list) -> list:
    """Convex hull (monotone chain), counter-clockwise."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def belt_samples(circles: list, step: float = 0.002) -> tuple[list, float]:
    """Sample points (x, z) along the taut belt around the given circles.

    The Mevea track is a looped rope over the pulleys, so its path is the convex
    hull of the pulley circles.  Returns (points spaced ~step apart, length).
    """
    cloud = []
    for (cx, cz, r) in circles:
        n = max(90, int(2 * math.pi * r / 0.005))
        cloud += [(cx + r * math.cos(2 * math.pi * k / n), cz + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    hull = _hull(cloud)
    samples, length = [], 0.0
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        length += seg
        k = max(1, int(seg / step))
        samples += [(a[0] + (b[0] - a[0]) * (j + 0.5) / k, a[1] + (b[1] - a[1]) * (j + 0.5) / k) for j in range(k)]
    return samples, length


def composite_body(mass: float, cm: tuple, inertia: dict, point_masses: list) -> dict:
    """Combine a rigid body (mass, CoM, diagonal inertia) with point masses.

    Returns mass, centre of mass and the full inertia tensor about that centre.
    """
    total = mass + sum(m for m, _ in point_masses)
    com = [(mass * cm[k] + sum(m * p[k] for m, p in point_masses)) / total for k in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    I[0][0], I[1][1], I[2][2] = inertia["ixx"], inertia["iyy"], inertia["izz"]

    def add(m, p):
        d = [p[k] - com[k] for k in range(3)]
        d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        for i in range(3):
            for j in range(3):
                I[i][j] += m * ((d2 if i == j else 0.0) - d[i] * d[j])
    add(mass, cm)
    for m, p in point_masses:
        add(m, p)
    return {"mass": total, "com": tuple(com), "I": I}


def static_support_check(wheels: list, cm_xy: tuple, separation: float, common_r: float) -> dict:
    """Approximate rest check for ONE oversize wheel on rigid ground.

    The oversize wheel is the lowest point.  With the opposite side's ground
    wheels it forms a support triangle; if the centre of mass lies outside that
    triangle the vehicle will rock until another wheel touches.
    """
    over = [w for w in wheels if w.get("free_spinning")]
    if len(over) != 1:
        return {"evaluated": False, "reason": "needs exactly one oversize wheel"}
    o = over[0]
    opp = [w for w in wheels if w["side"] != o["side"] and w["ground_contact"]]
    a = (o["pos"][0], o["pos"][1])
    b = (min(w["pos"][0] for w in opp), opp[0]["pos"][1])
    c = (max(w["pos"][0] for w in opp), opp[0]["pos"][1])
    tri = [a, b, c]
    cen = tuple(sum(p[k] for p in tri) / 3 for k in (0, 1))

    def signed(p, q):  # distance of cm to edge p-q, positive on the centroid side
        ex, ey = q[0] - p[0], q[1] - p[1]
        n = math.hypot(ex, ey)
        d = lambda pt: (ex * (pt[1] - p[1]) - ey * (pt[0] - p[0])) / n
        return d(cm_xy) * (1 if d(cen) > 0 else -1)

    margin = min(signed(tri[k], tri[(k + 1) % 3]) for k in range(3))
    return {
        "evaluated": True,
        "oversize_wheel": o["source_pulley"],
        "radius_excess_m": round(o["radius_m"] - common_r, 4),
        "static_roll_tilt_deg": round(math.degrees(math.atan2(o["radius_m"] - common_r, separation)), 2),
        "com_margin_to_support_triangle_m": round(margin, 4),
        "com_inside_triangle": margin >= 0,
        "expectation": ("stable on this triangle" if margin >= 0 else
                        "centre of mass is outside the support triangle: expect the vehicle to rock "
                        "until a neighbouring wheel touches (a few degrees of settled tilt)"),
    }


def derive(mev: dict, mesh_path: Path, opts: dict) -> dict:
    # Body ------------------------------------------------------------------
    b = mev["body"]
    cm = mevea_to_gz(*b["cm_mevea"])
    # Inertia axes: Mevea (x, y, z) -> Gazebo (x, z, y): roll stays, yaw<->pitch swap.
    inertia = {"ixx": b["Ixx"], "iyy": b["Izz"], "izz": b["Iyy"]}
    body_mass = opts["body_mass_kg"] if opts["body_mass_kg"] > 0 else b["mass_kg"]

    # Mesh placement.  raw vertices -> Rz(theta) * Rx(90deg) * v + t
    m = mev["mesh"]
    if opts["body_mesh"] == "runko":
        mesh_pos, mesh_ry, mesh_scale = RUNKO_MESH["mevea_pos"], RUNKO_MESH["mevea_ry_rad"], 1.0
        mesh_note = "Runko_part.STL; placement " + RUNKO_MESH["provenance"]
    else:
        if abs(m["mevea_rx_rad"]) > 1e-9 or abs(m["mevea_rz_rad"]) > 1e-9:
            raise SystemExit("Mesh has rx/rz rotation in Mevea; generator only handles ry.")
        mesh_pos, mesh_ry, mesh_scale = m["mevea_pos"], m["mevea_ry_rad"], m["scale"]
        mesh_note = "Jarcrac_body.STL; placement read from the Mevea XML"
    mesh_t = mevea_to_gz(*mesh_pos)
    mesh_rpy = (math.pi / 2.0, 0.0, mesh_ry)

    # Mesh bounds in Gazebo body frame (for collision box and checks).
    mn, mx = stl_bounds(mesh_path)
    corners = [(x, y, z) for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
    th = mesh_ry
    gz_pts = []
    for (x, y, z) in corners:
        x, y, z = x * mesh_scale, y * mesh_scale, z * mesh_scale
        # Rx(90): (x, y, z) -> (x, -z, y)
        x1, y1, z1 = x, -z, y
        # Rz(theta)
        x2 = x1 * math.cos(th) - y1 * math.sin(th)
        y2 = x1 * math.sin(th) + y1 * math.cos(th)
        gz_pts.append((x2 + mesh_t[0], y2 + mesh_t[1], z1 + mesh_t[2]))
    bmin = [min(p[i] for p in gz_pts) for i in range(3)]
    bmax = [max(p[i] for p in gz_pts) for i in range(3)]

    # Wheels ----------------------------------------------------------------
    wheels = []
    for p in mev["pulleys"]:
        if p["is_small_roller"]:
            continue  # upper return roller: no ground interaction, omitted
        gx, gy, gz = mevea_to_gz(*p["mevea_pos"])
        wheels.append({
            "name": p["name"].lower().replace("pulley", "wheel"),  # pulley2_left -> wheel2_left
            "source_pulley": p["name"],
            "side": p["side"],
            "pos": (gx, gy, gz),
            "radius_m": p["radius_m"],
            "mass_kg": p["mass_kg"],
            "free_spinning": False,
        })
    anomalies = []
    radii = sorted(w["radius_m"] for w in wheels)
    common_r = radii[len(radii) // 2]  # median = the value most wheels share
    for w in wheels:
        if abs(w["radius_m"] - common_r) > 1e-6:
            kept = opts["keep_source_radii"]
            anomalies.append({
                "item": w["source_pulley"],
                "issue": f"radius {w['radius_m']} m differs from the common {common_r} m used by the other wheels",
                "action": ("kept as in Mevea; wheel is free-spinning (not driven) because DiffDrive "
                           "assumes one wheel radius" if kept else f"normalised to {common_r} m"),
                "evidence": ANOMALY_EVIDENCE.get(w["source_pulley"], []),
            })
            if kept:
                w["free_spinning"] = True
            else:
                w["radius_m"] = common_r
    for w in wheels:
        w["bottom_z"] = w["pos"][2] - w["radius_m"]
    # Which wheels touch the ground is decided with the NOMINAL radius, so the
    # oversize wheel does not hide the other wheels from collision.
    nominal_bottom = {w["name"]: w["pos"][2] - common_r for w in wheels}
    nominal_lowest = min(nominal_bottom.values())
    for w in wheels:
        w["ground_contact"] = nominal_bottom[w["name"]] - nominal_lowest <= GROUND_CONTACT_TOL_M
    lowest = min(w["bottom_z"] for w in wheels if w["ground_contact"])

    lefts = [w for w in wheels if w["side"] == "left"]
    rights = [w for w in wheels if w["side"] == "right"]
    if len(lefts) != len(rights) or not lefts:
        raise SystemExit("Left/right wheel counts differ or are zero.")
    separation = abs(lefts[0]["pos"][1] - rights[0]["pos"][1])
    radius = common_r

    z_off = -lowest + opts["ground_clearance_margin_m"]  # lift so the lowest wheel sits on z=0

    wheel_width = opts["wheel_width_m"]
    for w in wheels:
        w["inertia_axis"], w["inertia_perp"] = cylinder_inertia(w["mass_kg"], w["radius_m"], wheel_width)

    # Each side's torque is shared over the wheels that are actually driven.
    for side_wheels in (lefts, rights):
        driven = [w for w in side_wheels if not w["free_spinning"]]
        for w in side_wheels:
            w["effort"] = opts["side_torque_nm"] / len(driven)

    # Track (rope) mass: Mevea simulates it as rope particles; Gazebo has no tracks, so
    # the same mass is lumped into the chassis with the inertia of the belt path.
    point_masses = []
    tracks = {"included": not opts["no_track_mass"], "per_side": {}}
    small = {p["side"]: p for p in mev["pulleys"] if p["is_small_roller"]}
    if tracks["included"]:
        for side in ("left", "right"):
            rope = mev["ropes"][side]
            circles = [(w["pos"][0], w["pos"][2], w["radius_m"]) for w in wheels if w["side"] == side]
            sp = small[side]
            sx, sy, sz = mevea_to_gz(*sp["mevea_pos"])
            circles.append((sx, sz, sp["radius_m"]))
            pts, loop_len = belt_samples(circles)
            area = math.pi * rope["radius_m"] ** 2
            full = rope["density"] * area * loop_len
            m_track = full * rope["scale_L"]
            lat = lefts[0]["pos"][1] if side == "left" else rights[0]["pos"][1]
            point_masses += [(m_track / len(pts), (x, lat, z)) for (x, z) in pts]
            point_masses.append((sp["mass_kg"], (sx, sy, sz)))  # small return roller
            tracks["per_side"][side] = {
                "rope": rope["name"], "belt_path_length_m": round(loop_len, 3),
                "density_kg_m3": rope["density"], "rope_radius_m": rope["radius_m"],
                "scale_L": rope["scale_L"],
                "mass_kg_estimate": round(m_track, 2),
                "mass_kg_if_scale_L_ignored": round(full, 2),
                "small_roller_mass_kg": sp["mass_kg"],
            }
    comp = composite_body(body_mass, cm, inertia, point_masses)
    support = static_support_check(wheels, (comp["com"][0], comp["com"][1]), separation, common_r)

    cam = mev["camera"]
    camera = None
    if cam:
        cx, cy, cz = mevea_to_gz(*cam["mevea_pos"])
        camera = {"pos": (cx, cy, cz), "hfov_rad": math.radians(cam["fov_deg"]), "source": cam["name"],
                  "is_default": cam["is_default"]}

    return {
        "z_off": z_off,
        "body": {"mass": comp["mass"], "body_only_mass": body_mass, "source_mass": b["mass_kg"],
                 "cm": comp["com"], "cm_body_only": cm, "inertia": inertia, "I_tensor": comp["I"],
                 "tracks": tracks},
        "mesh": {"t": mesh_t, "rpy": mesh_rpy, "scale": mesh_scale, "bounds_min": bmin,
                 "bounds_max": bmax, "note": mesh_note, "choice": opts["body_mesh"]},
        "wheels": wheels,
        "wheel_width": wheel_width,
        "separation": separation,
        "radius": radius,
        "camera": camera,
        "anomalies": anomalies,
        "support": support,
        "opts": opts,
    }


# ---------------------------------------------------------------------------
# SDF writing
# ---------------------------------------------------------------------------

def build_sdf(d: dict, model_name: str, prefix: str, mesh_file: str) -> str:
    o = d["opts"]
    z_off = d["z_off"]
    bmin, bmax = d["mesh"]["bounds_min"], d["mesh"]["bounds_max"]
    box_size = [bmax[i] - bmin[i] for i in range(3)]
    box_center = [(bmax[i] + bmin[i]) / 2.0 for i in range(3)]
    cm = d["body"]["cm"]
    t, rpy = d["mesh"]["t"], d["mesh"]["rpy"]
    sc = d["mesh"]["scale"]

    L: list[str] = []
    a = L.append
    a('<?xml version="1.0"?>')
    a("<!--")
    a("  GENERATED by scripts/generate_jarcrac_model.py - do not edit by hand.")
    a("  Source of truth: the Mevea JARCRAC model XML (see jarcrac_params.json).")
    a("  Frame: Gazebo Z-up, x forward, y left.  Model origin = Mevea body origin,")
    a(f"  raised {fmt(z_off)} m so the lowest road wheel rests on z = 0.")
    a("-->")
    a('<sdf version="1.10">')
    a(f'  <model name="{model_name}">')
    a("    <self_collide>false</self_collide>")
    a("")
    a('    <link name="chassis">')
    a(f"      <pose>{pose(0, 0, z_off)}</pose>")
    a("      <inertial>")
    a(f"        <pose>{pose(*cm)}</pose>")
    a(f"        <mass>{fmt(d['body']['mass'])}</mass>")
    T = d["body"]["I_tensor"]
    a("        <inertia>")
    a(f"          <ixx>{fmt(T[0][0])}</ixx><iyy>{fmt(T[1][1])}</iyy><izz>{fmt(T[2][2])}</izz>")
    a(f"          <ixy>{fmt(T[0][1])}</ixy><ixz>{fmt(T[0][2])}</ixz><iyz>{fmt(T[1][2])}</iyz>")
    a("        </inertia>")
    a("      </inertial>")
    a("")
    a(f"      <!-- {d['mesh']['note']} -->")
    a('      <visual name="body_visual">')
    a(f"        <pose>{pose(*t, *rpy)}</pose>")
    a("        <geometry><mesh>")
    a(f"          <uri>model://{model_name}/meshes/{mesh_file}</uri>")
    a(f"          <scale>{fmt(sc)} {fmt(sc)} {fmt(sc)}</scale>")
    a("        </mesh></geometry>")
    a("        <material><diffuse>0.85 0.15 0.10 1</diffuse><ambient>0.85 0.15 0.10 1</ambient></material>")
    a("      </visual>")
    a("")
    a("      <!-- Simplified collision: bounding box of the body mesh (a mesh collision would be")
    a("           expensive and the Mevea body only touches the ground through its tracks). -->")
    a('      <collision name="body_collision">')
    a(f"        <pose>{pose(*box_center)}</pose>")
    a(f"        <geometry><box><size>{fmt(box_size[0])} {fmt(box_size[1])} {fmt(box_size[2])}</size></box></geometry>")
    a("      </collision>")
    a("")
    # Front-centre 180-degree multi-layer scan: Gazebo LaserScan is flattened
    # in (vertical, horizontal) order. The earlier roof position caused
    # short self-returns from downward rays striking the chassis top.
    # Changing vertical scan channels requires scan-height filtering in AEB.
    lc = int(o["lidar_vertical_channels"])
    hc = int(o["lidar_horizontal_samples"])
    if (lc < 2 or lc != o["lidar_vertical_channels"] or hc < 360
            or hc != o["lidar_horizontal_samples"]
            or not (-89.0 < o["lidar_vertical_min_deg"] < 0 < o["lidar_vertical_max_deg"] < 89.0)
            or o["lidar_vertical_max_deg"] <= o["lidar_vertical_min_deg"]
            or o["lidar_scan_rate_hz"] <= 0 or o["lidar_range_min_m"] <= 0
            or o["lidar_range_max_m"] <= o["lidar_range_min_m"]
            or o["lidar_noise_stddev_m"] < 0):
        raise ValueError("invalid multi-layer LiDAR configuration")
    # A front scanner with forward-only azimuth must start ahead of the box front.
    # Otherwise downward / edge beams can hit the vehicle, as seen with the roof mount.
    if (not all(math.isfinite(o[k]) for k in ("lidar_x_m", "lidar_y_m", "lidar_z_m"))
            or o["lidar_x_m"] <= box_center[0] + box_size[0] / 2 + 0.10):
        raise ValueError("front LiDAR must be >=10 cm ahead of chassis collision box")
    a(f'      <sensor name="gpu_lidar_front" type="gpu_lidar">')
    a(f"        <pose>{pose(o['lidar_x_m'], o['lidar_y_m'], o['lidar_z_m'])}</pose>")
    a(f"        <topic>{prefix}/lidar</topic>")
    a(f"        <update_rate>{fmt(o['lidar_scan_rate_hz'])}</update_rate>")
    a("        <ray>")
    a("          <scan>")
    a("            <horizontal>")
    a(f"              <samples>{hc}</samples><resolution>1</resolution>")
    a("              <min_angle>-1.570796327</min_angle><max_angle>1.570796327</max_angle>")
    a("            </horizontal>")
    a("            <vertical>")
    a(f"              <samples>{lc}</samples><resolution>1</resolution>")
    a(f"              <min_angle>{math.radians(o['lidar_vertical_min_deg']):.9f}</min_angle>")
    a(f"              <max_angle>{math.radians(o['lidar_vertical_max_deg']):.9f}</max_angle>")
    a("            </vertical>")
    a("          </scan>")
    a(f"          <range><min>{fmt(o['lidar_range_min_m'])}</min>"
      f"<max>{fmt(o['lidar_range_max_m'])}</max><resolution>0.01</resolution></range>")
    a("          <noise>")
    a("            <type>gaussian</type>")
    a("            <mean>0</mean>")
    a(f"            <stddev>{fmt(o['lidar_noise_stddev_m'])}</stddev>")
    a("          </noise>")
    a("        </ray>")
    a("        <always_on>true</always_on>")
    a("        <visualize>true</visualize>")
    a("      </sensor>")
    if d["camera"]:
        c = d["camera"]
        a("")
        a(f'      <sensor name="front_rgb_camera" type="camera">')
        a(f"        <!-- Mevea {c['source']} pose and field of view -->")
        a(f"        <pose>{pose(*c['pos'])}</pose>")
        a(f"        <topic>{prefix}/camera/image</topic>")
        a("        <update_rate>15</update_rate>")
        a("        <camera>")
        a(f"          <horizontal_fov>{fmt(c['hfov_rad'])}</horizontal_fov>")
        a("          <image><width>640</width><height>480</height><format>R8G8B8</format></image>")
        a("          <clip><near>0.10</near><far>100.0</far></clip>")
        a("        </camera>")
        a("        <always_on>true</always_on>")
        a("        <visualize>true</visualize>")
        a("      </sensor>")
    a("    </link>")
    a("")

    # Wheels ---------------------------------------------------------------
    for w in d["wheels"]:
        x, y, z = w["pos"]
        r, ax, perp = w["radius_m"], w["inertia_axis"], w["inertia_perp"]
        a(f'    <link name="{w["name"]}">')
        a(f"      <pose>{pose(x, y, z + z_off)}</pose>")
        a("      <inertial>")
        a(f"        <mass>{fmt(w['mass_kg'])}</mass>")
        a("        <inertia>")
        a(f"          <ixx>{fmt(perp)}</ixx><iyy>{fmt(ax)}</iyy><izz>{fmt(perp)}</izz>")
        a("          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>")
        a("        </inertia>")
        a("      </inertial>")
        a(f'      <visual name="{w["name"]}_visual">')
        a("        <pose>0 0 0 1.5707963 0 0</pose>")
        a(f"        <geometry><cylinder><radius>{fmt(r)}</radius><length>{fmt(d['wheel_width'])}</length></cylinder></geometry>")
        a("        <material><diffuse>0.08 0.08 0.08 1</diffuse><ambient>0.08 0.08 0.08 1</ambient></material>")
        a("      </visual>")
        if w["ground_contact"]:
            a(f'      <collision name="{w["name"]}_collision">')
            a("        <pose>0 0 0 1.5707963 0 0</pose>")
            a(f"        <geometry><cylinder><radius>{fmt(r)}</radius><length>{fmt(d['wheel_width'])}</length></cylinder></geometry>")
            a("        <surface><friction><ode>")
            a(f"          <mu>{fmt(o['mu'])}</mu><mu2>{fmt(o['mu2'])}</mu2><fdir1>1 0 0</fdir1>")
            a("        </ode></friction></surface>")
            a("      </collision>")
        a("    </link>")
        a("")

    # Joints ---------------------------------------------------------------
    for w in d["wheels"]:
        eff = fmt(w["effort"])
        if w["free_spinning"]:
            a("    <!-- Oversize wheel (radius differs from the others): free-spinning, not driven. -->")
        a(f'    <joint name="{w["name"]}_joint" type="revolute">')
        a("      <parent>chassis</parent>")
        a(f"      <child>{w['name']}</child>")
        a("      <axis>")
        a("        <xyz>0 1 0</xyz>")
        a(f"        <limit><lower>-1e16</lower><upper>1e16</upper><effort>{eff}</effort></limit>")
        a("      </axis>")
        a("    </joint>")
        a("")

    # Drive plugin ---------------------------------------------------------
    sep = d["separation"] * o["turn_factor"]
    a('    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">')
    for w in d["wheels"]:
        if w["side"] == "left" and not w["free_spinning"]:
            a(f"      <left_joint>{w['name']}_joint</left_joint>")
    for w in d["wheels"]:
        if w["side"] == "right" and not w["free_spinning"]:
            a(f"      <right_joint>{w['name']}_joint</right_joint>")
    a(f"      <wheel_separation>{fmt(sep)}</wheel_separation>")
    a(f"      <wheel_radius>{fmt(d['radius'])}</wheel_radius>")
    a(f"      <max_linear_velocity>{fmt(o['max_speed_mps'])}</max_linear_velocity>")
    a(f"      <min_linear_velocity>{fmt(-o['max_speed_mps'])}</min_linear_velocity>")
    a(f"      <max_linear_acceleration>{fmt(o['max_accel_mps2'])}</max_linear_acceleration>")
    a(f"      <min_linear_acceleration>{fmt(-2 * o['max_accel_mps2'])}</min_linear_acceleration>")
    a(f"      <max_angular_velocity>{fmt(o['max_yaw_rate_radps'])}</max_angular_velocity>")
    a(f"      <min_angular_velocity>{fmt(-o['max_yaw_rate_radps'])}</min_angular_velocity>")
    a(f"      <topic>/model/{model_name}/cmd_vel</topic>")
    a(f"      <odom_topic>/model/{model_name}/odometry</odom_topic>")
    a("      <odom_publish_frequency>20</odom_publish_frequency>")
    a("    </plugin>")
    a("")
    a("    <plugin filename=\"gz-sim-joint-state-publisher-system\" name=\"gz::sim::systems::JointStatePublisher\"/>")
    a("  </model>")
    a("</sdf>")
    return "\n".join(L) + "\n"


def build_config(model_name: str) -> str:
    return f"""<?xml version="1.0"?>

<model>
  <name>{model_name}</name>
  <version>1.0</version>

  <sdf version="1.10">
    model.sdf
  </sdf>

  <author>
    <name>Edge Path Project</name>
  </author>

  <description>
    JARCRAC tracked vehicle approximated as an 8-wheel skid-steer model for
    Gazebo SIL testing.  Generated from the Mevea model; see jarcrac_params.json
    for extracted values and assumptions.
  </description>
</model>
"""


def build_params(d: dict, mev: dict, xml_path: Path, model_name: str, prefix: str) -> dict:
    o = d["opts"]
    return {
        "source": {
            "mevea_xml": xml_path.name,
            "mevea_xml_sha256": hashlib.sha256(xml_path.read_bytes()).hexdigest(),
            "axis_mapping": "Mevea (x,y,z) -> Gazebo (x,-z,y)",
        },
        "model": {"name": model_name, "topic_prefix": prefix,
                  "cmd_vel_topic": f"/model/{model_name}/cmd_vel",
                  "odometry_topic": f"/model/{model_name}/odometry",
                  "lidar_topic": f"{prefix}/lidar", "camera_topic": f"{prefix}/camera/image"},
        "extracted_from_mevea": {
            "body_mass_kg_in_source_xml": d["body"]["source_mass"],
            "body_inertia_gz": d["body"]["inertia"],
            "body_cm_gz_m": [v + 0.0 for v in d["body"]["cm_body_only"]],
            "mesh_file_in_source_xml": mev["mesh"]["file"],
            "mesh_bounds_gz_min_m": [round(v, 4) for v in d["mesh"]["bounds_min"]],
            "mesh_bounds_gz_max_m": [round(v, 4) for v in d["mesh"]["bounds_max"]],
            "wheel_radius_m": d["radius"],
            "track_centreline_separation_m": round(d["separation"], 4),
            "wheels_per_side": len([w for w in d["wheels"] if w["side"] == "left"]),
            "ground_contact_wheels_per_side": len([w for w in d["wheels"] if w["side"] == "left" and w["ground_contact"]]),
            "wheel_mass_kg": d["wheels"][0]["mass_kg"],
            "camera": d["camera"] and {"source": d["camera"]["source"], "hfov_rad": round(d["camera"]["hfov_rad"], 5),
                                       "is_default": d["camera"]["is_default"]},
            "reference_only": {
                "diesel_kw": mev["drivetrain_reference"]["diesel_kw"],
                "torque_source_side_nm": mev["drivetrain_reference"]["torque_source_side_nm"],
                "tracks": mev["tracks_not_modelled"],
            },
        },
        "overrides": {
            "body_mass_kg": {"value_used": d["body"]["body_only_mass"], "source_xml_value": d["body"]["source_mass"],
                             "reason": "default: mass from the given Mevea XML (Jan-2026 free-roll lineage, whose "
                                       "track masses were rebalanced 'closer to real'); Jarcrac_rev7 says 2460 kg "
                                       "with identical inertia and CM"},
            "body_mesh": {"choice": d["mesh"]["choice"], "note": d["mesh"]["note"],
                          "bounds_gz_min_m": [round(v, 4) for v in d["mesh"]["bounds_min"]],
                          "bounds_gz_max_m": [round(v, 4) for v in d["mesh"]["bounds_max"]]},
            "free_spinning_wheels": [w["name"] for w in d["wheels"] if w["free_spinning"]],
        },
        "source_anomalies": d["anomalies"],
        "static_support_check": d["support"],
        "chassis": {
            "composite_mass_kg": round(d["body"]["mass"], 3),
            "composite_com_gz_m": [round(v + 0.0, 5) for v in d["body"]["cm"]],
            "composite_inertia_tensor": [[round(v + 0.0, 3) for v in row] for row in d["body"]["I_tensor"]],
            "body_only_mass_kg": d["body"]["body_only_mass"],
            "tracks_lumped_into_chassis": d["body"]["tracks"],
        },
        "total_mass_kg_in_gazebo": d["body"]["mass"] + sum(w["mass_kg"] for w in d["wheels"]),
        "assumptions": {
            "max_speed_mps": o["max_speed_mps"],
            "max_accel_mps2": o["max_accel_mps2"],
            "max_yaw_rate_radps": o["max_yaw_rate_radps"],
            "turn_factor": o["turn_factor"],
            "effective_wheel_separation_m": round(d["separation"] * o["turn_factor"], 4),
            "wheel_width_m": o["wheel_width_m"],
            "body_mass_kg": d["body"]["mass"],
            "mu": o["mu"], "mu2": o["mu2"],
            "per_wheel_effort_nm": {w["name"]: round(w["effort"], 3) for w in d["wheels"]},
            "lidar_x_m": o["lidar_x_m"],
            "lidar_y_m": o["lidar_y_m"],
            "lidar_z_m": o["lidar_z_m"],
            "lidar_sensor_height_flat_m": round(o["lidar_z_m"] + d["z_off"], 3),
            "lidar_3d": {
                "description": "Generic front-mounted 180 degree uniformly spaced multi-layer lidar, not a specific commercial sensor",
                "horizontal_fov_deg": 180.0,
                "mounting": "front-centre, 0.23 m forward of body collision box",
                "horizontal_samples": int(o["lidar_horizontal_samples"]),
                "vertical_channels": int(o["lidar_vertical_channels"]),
                "vertical_min_deg": o["lidar_vertical_min_deg"],
                "vertical_max_deg": o["lidar_vertical_max_deg"],
                "scan_rate_hz": o["lidar_scan_rate_hz"],
                "range_min_m": o["lidar_range_min_m"],
                "range_max_m": o["lidar_range_max_m"],
                "noise_stddev_m": o["lidar_noise_stddev_m"],
            },
            "notes": [
                "Track mass is estimated as density x rope cross-section x belt path x scale_L and lumped into the "
                "chassis; Mevea's LOG_DynamicLineSystem.txt (not in the repo zip) lists the exact masses.",
                "Top speed is derived from a note in 'JARCRAC Mevea malli.docx' (~75 rpm); verify.",
                "Friction anisotropy (fdir1) may be ignored by some physics engines; verify in your setup.",
                "Lidar has no Mevea counterpart; placement is a choice.",
            ],
        },
        "not_modelled": ["rope-particle tracks", "diesel/hydraulic/planetary drivetrain",
                         "brakes", "small upper return roller", "track width/belt visual"],
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mevea-xml", required=True, type=Path)
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--name", default="jarcrac", help="model name (and model:// directory name)")
    ap.add_argument("--topic-prefix", default=None, help="sensor topic prefix (default: /<name>)")
    for key, default in DEFAULTS.items():
        ap.add_argument("--" + key.replace("_", "-"), type=float, default=default)
    ap.add_argument("--body-mesh", choices=("runko", "jarcrac"), default="runko",
                    help="body visual: detailed Runko_part.STL (default) or Jarcrac_body.STL")
    ap.add_argument("--keep-source-radii", action="store_true",
                    help="keep outlier pulley radii exactly as in Mevea and leave the oversize wheel "
                         "free-spinning (default: normalise to the common radius)")
    ap.add_argument("--no-track-mass", action="store_true",
                    help="do not add the Mevea track (rope) mass and inertia to the chassis")
    args = ap.parse_args(argv)

    xml_path: Path = args.mevea_xml.resolve()
    mev = parse_mevea(xml_path)
    mesh_rel = RUNKO_MESH["file"] if args.body_mesh == "runko" else mev["mesh"]["file"]
    mesh_src = (xml_path.parent / mesh_rel).resolve()
    if not mesh_src.exists():
        print(f"mesh not found: {mesh_src}", file=sys.stderr)
        return 2

    opts = {k: getattr(args, k) for k in DEFAULTS}
    opts["body_mesh"] = args.body_mesh
    opts["keep_source_radii"] = args.keep_source_radii
    opts["no_track_mass"] = args.no_track_mass
    d = derive(mev, mesh_src, opts)
    prefix = args.topic_prefix or f"/{args.name}"

    out = args.out_dir
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    mesh_name = mesh_src.name
    shutil.copyfile(mesh_src, out / "meshes" / mesh_name)
    (out / "model.sdf").write_text(build_sdf(d, args.name, prefix, mesh_name), encoding="utf-8", newline="\n")
    (out / "model.config").write_text(build_config(args.name), encoding="utf-8", newline="\n")
    (out / "jarcrac_params.json").write_text(
        json.dumps(build_params(d, mev, xml_path, args.name, prefix), indent=2) + "\n",
        encoding="utf-8", newline="\n")

    wl = [w for w in d["wheels"] if w["side"] == "left"]
    print(f"wrote {out}")
    tk = d["body"]["tracks"]["per_side"]
    print(f"  chassis {d['body']['mass']:.1f} kg (body {d['body']['body_only_mass']:.0f}"
          + (f" + tracks {sum(v['mass_kg_estimate'] for v in tk.values()):.0f}" if tk else "")
          + f") + {len(d['wheels'])} wheels = "
          f"{d['body']['mass'] + sum(w['mass_kg'] for w in d['wheels']):.1f} kg total")
    print(f"  {len(wl)} wheels/side ({sum(w['ground_contact'] for w in wl)} touch ground), "
          f"radius {d['radius']} m, separation {d['separation']:.3f} m")
    sup = d["support"]
    if sup.get("evaluated"):
        print(f"  static support: roll tilt ~{sup['static_roll_tilt_deg']} deg, CoM margin "
              f"{sup['com_margin_to_support_triangle_m']*100:.1f} cm -> {sup['expectation']}")
    for an in d["anomalies"]:
        print(f"  WARNING source anomaly: {an['item']}: {an['issue']} -> {an['action']}")
    if d["camera"] and d["camera"]["is_default"]:
        print("  NOTE: no cabin camera in this Mevea XML; used default pose from Jarcrac_rev7.xml")
    print(f"  model lifted {d['z_off']:.3f} m so the lowest wheel rests on z=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
