#!/usr/bin/env python3
"""Generate the Gazebo ``wheelloader`` model from the Mevea WheelLoader model.

The Mevea model (WheelLoader/Model/WheelLoader.xml and its Graphics folder) is
the source of truth.  This script extracts geometry, mass properties, joint
axes and ranges, converts the meshes Gazebo cannot read, and writes:

    <out>/model.sdf
    <out>/model.config
    <out>/meshes/*.obj + *.mtl            visuals (from Mevea .osgt, flat colours)
    <out>/meshes/collision/*.stl          collision shapes (from Mevea .3ds)
    <out>/wheelloader_params.json         extracted values, derived values, assumptions

The output is deterministic, so it can be snapshot-tested.

Mevea structure -> Gazebo model
    9 bodies, 9 joints (one closed loop)                  8 links, tree of joints
    RearAxle   (oscillating axle, roll about x)           rear_axle   revolute, sprung/damped
    RearFrame  (cab, engine)                              rear_frame  (canonical link)
    FrontFrame (articulated about a vertical pivot)       front_frame revolute "articulation"
    LiftArm                                               lift_arm    revolute "lift"
    BucketClamp (bucket carrier)                          bucket_clamp revolute "bucket_tilt"
    Bucket                                                bucket      fixed to bucket_clamp
    BucketLever + BucketLink (Z-bar linkage)              mass lumped into lift_arm / bucket_clamp
    4 Mevea tyre force elements                           wheel_rl/rr/fl/fr revolute, velocity driven

Simplifications (all recorded in the params file)
    * Gazebo physics engines do not support closed kinematic loops, so the
      Z-bar tilt linkage is replaced by direct actuation of the bucket pivot;
      lever and link mass is lumped into their neighbours and their meshes are
      omitted.  The tilt range is Mevea's own damper range.
    * Hydraulics, the diesel / torque converter / gearbox and the LuGre tyre
      model are not simulated.  Joints are position or velocity controlled.
    * Hydraulic cylinder meshes are omitted (no cylinder force elements).
    * Visuals carry colours, not textures.

Axis convention
    Mevea: Y-up (x forward, y up, z right).  Gazebo: Z-up (x forward, y left,
    z up).  Mevea (x, y, z) -> Gazebo (x, -z, y), a proper rotation.
    Mevea CS orientation (rx, ry, rz) is R = Rx * Ry * Rz and a revolute
    joint turns about its CS z axis; both were verified by the joint-pair
    coincidence check (see "validation" in the params file).

Usage
    python3 generate_wheelloader_model.py \\
        --mevea-dir path/to/ruskotunturi/WheelLoader --out-dir ../models/wheelloader
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mevea_mesh_tools as mt  # noqa: E402

G = 9.80665

# ---------------------------------------------------------------------------
# Assumptions (Mevea does not define these for a rigid-wheel Gazebo model)
# ---------------------------------------------------------------------------

DEFAULTS = {
    # Mevea has no tyre mass (the tyre is a force element).  A wheel link needs some
    # mass; this small value is a simulation convenience, not a physical tyre mass,
    # and is added on top of the Mevea total.
    "wheel_mass_kg": 30.0,
    # Tyre_Friction_Long / _Lat static coefficients in the Mevea model.
    "mu": 0.8,
    "mu2": 0.6,
    # Top speed derived from the Mevea powertrain (see derive_speed); override here.
    "max_speed_mps": 0.0,
    # Joint controller gains: initial guesses, tune in simulation.
    "articulation_p": 5.0e6, "articulation_d": 2.0e6,
    "lift_p": 2.0e6, "lift_d": 8.0e5,
    "tilt_p": 1.0e6, "tilt_d": 1.0e5,
    "wheel_effort_nm": 30000.0,
    "ground_clearance_margin_m": 0.003,
}

CONTACT_PROFILE_HALF_WIDTH = 0.19  # Spline_TyreProfile spans -0.19 .. +0.19 m

# ---------------------------------------------------------------------------
# Linear algebra (3x3 lists, no numpy)
# ---------------------------------------------------------------------------

I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
M_GZ = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]  # Mevea -> Gazebo


def mmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mT(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def vadd(a, b):
    return [a[i] + b[i] for i in range(3)]


def vsub(a, b):
    return [a[i] - b[i] for i in range(3)]


def vnorm(a):
    return math.sqrt(sum(x * x for x in a))


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rot_mevea(rx, ry, rz):
    """Mevea CS orientation: R = Rx * Ry * Rz."""
    return mmul(mmul(rot_x(rx), rot_y(ry)), rot_z(rz))


def rot_axis(axis, ang):
    n = vnorm(axis)
    x, y, z = (axis[0] / n, axis[1] / n, axis[2] / n)
    c, s, t = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def rpy_from_matrix(R):
    """R = Rz(yaw) Ry(pitch) Rx(roll)  ->  (roll, pitch, yaw)."""
    sp = max(-1.0, min(1.0, -R[2][0]))
    pitch = math.asin(sp)
    if abs(sp) < 1 - 1e-9:
        return (math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), pitch, 0.0)


def compose(T1, T2):
    return (mmul(T1[0], T2[0]), vadd(T1[1], mv(T1[0], T2[1])))


def invert(T):
    Rt = mT(T[0])
    return (Rt, [-x for x in mv(Rt, T[1])])


def to_gz_T(T):
    """(R, p) in Mevea axes -> Gazebo axes."""
    return (mmul(mmul(M_GZ, T[0]), mT(M_GZ)), mv(M_GZ, T[1]))


def fmt(v: float) -> str:
    if abs(v) < 1e-12:
        v = 0.0
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def pose_str(T, z_off: float = 0.0) -> str:
    r, p, y = rpy_from_matrix(T[0])
    return " ".join(fmt(v) for v in (T[1][0], T[1][1], T[1][2] + z_off, r, p, y))


def snake(name: str) -> str:
    s = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name)
    return s.lower()


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def _f(e, k):
    return float(e.attrib[k])


def _T(cs_elem):
    p = cs_elem.find("Position")
    o = cs_elem.find("Orientation")
    pos = [_f(p, "x"), _f(p, "y"), _f(p, "z")]
    R = rot_mevea(_f(o, "rx"), _f(o, "ry"), _f(o, "rz")) if o is not None else I3
    return (R, pos)


def parse_mevea(xml_path: Path) -> dict:
    root = ET.parse(xml_path).getroot()
    bodies = {}
    for b in root.find("Bodies"):
        inertia = b.find("Inertia")
        i = inertia.find("I")
        Ixx, Iyy, Izz = _f(i, "Ixx"), _f(i, "Iyy"), _f(i, "Izz")
        Ixy, Izx, Iyz = _f(i, "Ixy"), _f(i, "Izx"), _f(i, "Iyz")
        bodies[b.tag] = {
            "mass": _f(inertia, "mass"),
            "I": [[Ixx, Ixy, Izx], [Ixy, Iyy, Iyz], [Izx, Iyz, Izz]],
            "rel_to": b.get("RelTo"), "rel_cs": (b.get("RelToCS") or "").split(".")[-1],
            "cm_cs": (b.get("CmCS") or "").split(".")[-1], "cmi_cs": (b.get("CmiCS") or "").split(".")[-1],
            "vis": [x for x in (b.get("VisualizationGraphics") or "").split(";") if x],
            "coll": [x for x in (b.get("CollisionGraphics") or "").split(";") if x],
            "cs": {c.tag: _T(c) for c in b.find("CS")},
        }
    constraints = []
    for c in root.find("Constraints"):
        constraints.append({
            "name": c.tag, "type": c.attrib["Type"], "cut": c.attrib.get("CutJoint") == "Yes",
            "b1": c.attrib["Body1"], "cs1": c.attrib["Body1CS"].split(".")[-1],
            "b2": c.attrib["Body2"], "cs2": c.attrib["Body2CS"].split(".")[-1],
        })
    graphics = {}
    for g in root.find("Graphics"):
        pos, ori = g.find("Position"), g.find("Orientation")
        graphics[g.tag] = {
            "file": g.attrib["FileName"], "scale": float(g.attrib.get("Scale", 1)),
            "T": (rot_mevea(_f(ori, "rx"), _f(ori, "ry"), _f(ori, "rz")),
                  [_f(pos, "x"), _f(pos, "y"), _f(pos, "z")]),
        }
    tyres = {}
    for t in root.find("Forces/Tyre"):
        tyres[t.tag] = {"body": t.attrib["Body"], "cs": t.attrib["BodyCS"].split(".")[-1],
                        "r": _f(t, "r"), "width_attr": _f(t, "width"), "K": _f(t, "K"), "C": _f(t, "C")}
    dummies = {d.tag: {"rel_to": d.get("RelTo"), "vis": [x for x in (d.get("VisualizationGraphics") or "").split(";") if x]}
               for d in root.find("Dummies")}
    dampers = {d.tag: {k: v for k, v in d.attrib.items() if v} for d in root.find("Forces/B2BM")}
    cylinders = {}
    for f in root.find("Forces/B2BF"):
        cylinders[f.tag] = {"b1": f.attrib["Body1"], "cs1": f.attrib["Body1CS"].split(".")[-1],
                            "b2": f.attrib["Body2"], "cs2": f.attrib["Body2CS"].split(".")[-1],
                            "L0": _f(f, "L0"), "Lmin": _f(f, "Lmin"), "Lmax": _f(f, "Lmax")}
    cameras = {}
    for c in root.find("Cameras"):
        cameras[c.tag] = {"ref_body": c.get("RefBody"), "ref_cs": (c.get("RefCS") or "").split(".")[-1],
                          "look_body": c.get("RefBodyLookAt"), "look_cs": (c.get("RefCSLookAt") or "").split(".")[-1],
                          "fov_deg": float(c.get("fov", 60))}
    pt = root.find("Forces/PowerTrain")
    gear = pt.find("Gears/Gearbox")
    powertrain = {
        "top_gear_ratio": _f(gear, "ifw3"),
        "front_diff_ratio": _f(pt.find("Differentials/FrontDifferential"), "i"),
        "planet_ratio": _f(pt.find("Planets/PlanetGearFL"), "i"),
    }
    motor = root.find("Splines/Spline_Motor")
    xs = [float(v) for v in motor.find("x").attrib.values()]
    ys = [float(v) for v in motor.find("y").attrib.values()]
    return {"bodies": bodies, "constraints": constraints, "graphics": graphics, "tyres": tyres,
            "dummies": dummies, "dampers": dampers, "cylinders": cylinders, "cameras": cameras,
            "powertrain": powertrain, "engine_curve": list(zip(xs, ys))}


# ---------------------------------------------------------------------------
# Kinematic chain at the initial configuration
# ---------------------------------------------------------------------------

def world_transforms(mev: dict) -> dict:
    bodies = mev["bodies"]
    W = {"Ground": (I3, [0.0, 0.0, 0.0])}
    # The floating constraint places the rear axle on the ground body.
    fl = next(c for c in mev["constraints"] if c["type"] == "Floating")
    T1 = compose(W[fl["b1"]], bodies[fl["b1"]]["cs"][fl["cs1"]])
    W[fl["b2"]] = compose(T1, invert(bodies[fl["b2"]]["cs"][fl["cs2"]]))
    pending = [b for b in bodies if b not in W]
    while pending:
        progressed = False
        for name in list(pending):
            parent = bodies[name]["rel_to"]
            if parent in W:
                W[name] = compose(W[parent], bodies[parent]["cs"][bodies[name]["rel_cs"]])
                pending.remove(name)
                progressed = True
        if not progressed:
            raise SystemExit(f"cannot resolve body placement for {pending}")
    return W


def joint_frames(mev: dict, W: dict) -> dict:
    """World pose of each constraint's two CS, plus coincidence errors."""
    out = {}
    for c in mev["constraints"]:
        P1 = compose(W[c["b1"]], mev["bodies"][c["b1"]]["cs"][c["cs1"]])
        P2 = compose(W[c["b2"]], mev["bodies"][c["b2"]]["cs"][c["cs2"]])
        d = vsub(P1[1], P2[1])
        Rrel = mmul(mT(P1[0]), P2[0])
        cosang = max(-1.0, min(1.0, (Rrel[0][0] + Rrel[1][1] + Rrel[2][2] - 1) / 2))
        out[c["name"]] = {"P1": P1, "P2": P2, "pos_err": vnorm(d), "rot_err": math.acos(cosang),
                          "axis_world": [P1[0][0][2], P1[0][1][2], P1[0][2][2]]}
    return out


def rel_angle(W, a, b, axis):
    """Angle of body b's frame relative to body a's frame about `axis` (a-frame)."""
    R = mmul(mT(W[a][0]), W[b][0])
    # rotation about z only in this model, so read it from the matrix
    if axis == "z":
        return math.atan2(R[1][0], R[0][0])
    raise ValueError(axis)


def angle_at_length(A, B0, J, axis, target, step=0.0005, span=3.0):
    """Joint angle (nearest to 0) at which the cylinder length equals `target`."""
    best, best_q = None, 0.0
    n = int(2 * span / step)
    for i in range(n + 1):
        q = -span + i * step
        B = vadd(J, mv(rot_axis(axis, q), vsub(B0, J)))
        e = abs(vnorm(vsub(A, B)) - target)
        if best is None or e < best - 1e-12 or (abs(e - best) <= 1e-12 and abs(q) < abs(best_q)):
            best, best_q = e, q
    return best_q, best


def cylinder_interval(A, B0, J, axis, Lmin, Lmax, step=0.0005, span=3.0):
    """Joint-angle interval [q_lo, q_hi] around q=0 where Lmin <= |A - B(q)| <= Lmax."""
    def length(q):
        B = vadd(J, mv(rot_axis(axis, q), vsub(B0, J)))
        return vnorm(vsub(A, B))
    L0 = length(0.0)
    n = int(span / step)
    lo = hi = 0.0
    q = 0.0
    for _ in range(n):
        q += step
        if not (Lmin - 1e-9 <= length(q) <= Lmax + 1e-9):
            break
        hi = q
    q = 0.0
    for _ in range(n):
        q -= step
        if not (Lmin - 1e-9 <= length(q) <= Lmax + 1e-9):
            break
        lo = q
    return lo, hi, L0


# ---------------------------------------------------------------------------
# Mass properties
# ---------------------------------------------------------------------------

def composite(items: list) -> dict:
    """items: (mass, com[3], inertia 3x3 about its own com) all in one frame."""
    M = sum(m for m, _, _ in items)
    com = [sum(m * c[k] for m, c, _ in items) / M for k in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    for m, c, Ii in items:
        d = vsub(c, com)
        d2 = sum(x * x for x in d)
        for a in range(3):
            for b in range(3):
                I[a][b] += Ii[a][b] + m * ((d2 if a == b else 0.0) - d[a] * d[b])
    return {"mass": M, "com": com, "I": I}


def body_item_in(W, mev, src: str, target: str):
    """Mass item of body `src` expressed in the frame of body `target`."""
    b = mev["bodies"][src]
    cm_local = b["cs"][b["cm_cs"]][1]
    Tt = invert(W[target])
    Ts = compose(Tt, W[src])  # src frame in target frame
    com = vadd(Ts[1], mv(Ts[0], cm_local))
    I = mmul(mmul(Ts[0], b["I"]), mT(Ts[0]))
    return (b["mass"], com, I)


def check_pos_def(I) -> bool:
    d1 = I[0][0]
    d2 = I[0][0] * I[1][1] - I[0][1] ** 2
    d3 = (I[0][0] * (I[1][1] * I[2][2] - I[1][2] ** 2) - I[0][1] * (I[0][1] * I[2][2] - I[1][2] * I[0][2])
          + I[0][2] * (I[0][1] * I[1][2] - I[1][1] * I[0][2]))
    return d1 > 0 and d2 > 0 and d3 > 0


# ---------------------------------------------------------------------------
# Derivation
# ---------------------------------------------------------------------------

LINKS = {  # Gazebo link -> Mevea body
    "rear_axle": "RearAxle", "rear_frame": "RearFrame", "front_frame": "FrontFrame",
    "lift_arm": "LiftArm", "bucket_clamp": "BucketClamp", "bucket": "Bucket",
}
LUMPS = {"lift_arm": ["LiftArm", "BucketLever"], "bucket_clamp": ["BucketClamp", "BucketLink"]}
WHEELS = {"wheel_rl": "Tyre_RL", "wheel_rr": "Tyre_RR", "wheel_fl": "Tyre_FL", "wheel_fr": "Tyre_FR"}


def derive_speed(mev: dict, r: float) -> dict:
    pt, curve = mev["powertrain"], mev["engine_curve"]
    ratio = pt["top_gear_ratio"] * pt["front_diff_ratio"] * pt["planet_ratio"]
    w_max = max(x for x, _ in curve)
    return {"total_ratio_top_gear": ratio, "engine_max_radps": w_max,
            "speed_mps": w_max / ratio * r,
            "note": "engine max speed / (gearbox x differential x planet) x tyre radius; "
                    "torque converter slip and efficiencies ignored"}


def derive(mev: dict, mesh_info: dict, opts: dict) -> dict:
    bodies = mev["bodies"]
    W = world_transforms(mev)
    jf = joint_frames(mev, W)

    # wheels ---------------------------------------------------------------
    wheels = {}
    for lname, tname in WHEELS.items():
        t = mev["tyres"][tname]
        T = compose(W[t["body"]], bodies[t["body"]]["cs"][t["cs"]])
        wheels[lname] = {"tyre": tname, "parent": t["body"], "T": T, "r": t["r"], "K": t["K"], "C": t["C"]}
    bottoms = [w["T"][1][1] - w["r"] for w in wheels.values()]  # Mevea y is up
    z_off = -min(bottoms) + opts["ground_clearance_margin_m"]

    # geometry used by the steering kinematics ---------------------------------
    pivot = jf["RearFrame_FrontFrame"]["P1"][1]
    rear_x = (wheels["wheel_rl"]["T"][1][0] + wheels["wheel_rr"]["T"][1][0]) / 2
    front_x = (wheels["wheel_fl"]["T"][1][0] + wheels["wheel_fr"]["T"][1][0]) / 2
    kin = {"d_r": pivot[0] - rear_x, "d_f": front_x - pivot[0],
           "half_track": abs(wheels["wheel_rr"]["T"][1][2]), "wheel_radius": wheels["wheel_rl"]["r"],
           "wheelbase": front_x - rear_x}

    # joint ranges -----------------------------------------------------------
    ranges, notes = {}, []
    cyl = mev["cylinders"]

    def attach(c, side):  # world position of a cylinder end
        b, cs = (c["b1"], c["cs1"]) if side == 1 else (c["b2"], c["cs2"])
        return compose(W[b], bodies[b]["cs"][cs])[1]

    # lift
    J = jf["FrontFrame_LiftArm"]
    lift_axis = J["axis_world"]
    los, his = [], []
    for n in ("ForceT_BucketLiftL", "ForceT_BucketLiftR"):
        lo, hi, L0 = cylinder_interval(attach(cyl[n], 1), attach(cyl[n], 2), J["P1"][1], lift_axis,
                                       cyl[n]["Lmin"], cyl[n]["Lmax"])
        los.append(lo)
        his.append(hi)
        qL0, _ = angle_at_length(attach(cyl[n], 1), attach(cyl[n], 2), J["P1"][1], lift_axis, cyl[n]["L0"])
        notes.append({"cylinder": n, "length_at_initial_pose": round(L0, 4), "L0_in_mevea": round(cyl[n]["L0"], 4),
                      "joint_angle_where_length_equals_L0": round(qL0, 4)})
    ranges["lift"] = {"lower": max(los), "upper": min(his), "axis_world": lift_axis}

    # steering
    J = jf["RearFrame_FrontFrame"]
    st_axis = J["axis_world"]
    los, his = [], []
    for n in ("ForceT_SteeringL", "ForceT_SteeringR"):
        lo, hi, L0 = cylinder_interval(attach(cyl[n], 1), attach(cyl[n], 2), J["P1"][1], st_axis,
                                       cyl[n]["Lmin"], cyl[n]["Lmax"])
        los.append(lo)
        his.append(hi)
        qL0, _ = angle_at_length(attach(cyl[n], 1), attach(cyl[n], 2), J["P1"][1], st_axis, cyl[n]["L0"])
        notes.append({"cylinder": n, "length_at_initial_pose": round(L0, 4), "L0_in_mevea": round(cyl[n]["L0"], 4),
                      "joint_angle_where_length_equals_L0": round(qL0, 4)})
    ranges["articulation"] = {"lower": max(los), "upper": min(his), "axis_world": st_axis}

    # bucket tilt: Mevea's own damper limits, relative angle clamp-vs-lift-arm about z
    dmp = mev["dampers"]["BucketClampDamper"]
    theta0 = rel_angle(W, "LiftArm", "BucketClamp", "z")
    ranges["bucket_tilt"] = {"lower": float(dmp["Thetamin"]) - theta0, "upper": float(dmp["Thetamax"]) - theta0,
                             "axis_world": jf["LiftArm_BucketClamp"]["axis_world"],
                             "initial_relative_angle_mevea": theta0}
    # rear axle oscillation
    ax = mev["dampers"]["RearAxleDamper"]
    ranges["axle_oscillation"] = {"lower": float(ax["Thetamin"]), "upper": float(ax["Thetamax"]),
                                  "axis_world": jf["RearAxle_RearFrame"]["axis_world"],
                                  "K": float(ax["K"]), "C": float(ax["C"])}

    # mass properties --------------------------------------------------------
    inertials = {}
    for link, body in LINKS.items():
        srcs = LUMPS.get(link, [body])
        comp = composite([body_item_in(W, mev, s, body) for s in srcs])
        inertials[link] = {
            "mass": comp["mass"],
            "com_gz": mv(M_GZ, comp["com"]),
            "I_gz": mmul(mmul(M_GZ, comp["I"]), mT(M_GZ)),
            "sources": srcs,
            "pos_def": check_pos_def(comp["I"]),
        }
    wm = opts["wheel_mass_kg"]
    r = wheels["wheel_rl"]["r"]
    wI_axis, wI_perp = 0.5 * wm * r * r, wm * (3 * r * r + 0.38 ** 2) / 12.0

    speed = derive_speed(mev, kin["wheel_radius"])
    max_speed = opts["max_speed_mps"] if opts["max_speed_mps"] > 0 else speed["speed_mps"]

    return {"W": W, "jf": jf, "wheels": wheels, "z_off": z_off, "kin": kin, "ranges": ranges,
            "cyl_notes": notes, "inertials": inertials, "wheel_inertia": (wI_axis, wI_perp),
            "speed": speed, "max_speed": max_speed, "opts": opts, "mesh_info": mesh_info}


# ---------------------------------------------------------------------------
# Meshes
# ---------------------------------------------------------------------------

def convert_meshes(mev: dict, mevea_dir: Path, out_dir: Path) -> dict:
    """Convert every visual (.osgt) and collision (.3ds) mesh the bodies use."""
    model_dir = mevea_dir / "Model"
    vis_dir = out_dir / "meshes"
    col_dir = vis_dir / "collision"
    vis_dir.mkdir(parents=True, exist_ok=True)
    col_dir.mkdir(parents=True, exist_ok=True)
    info = {"visual": {}, "collision": {}}
    graphics = mev["graphics"]
    needed_vis, needed_col = set(), set()
    used = set(LINKS.values())  # lever and link are lumped away: no meshes for them
    for bn, b in mev["bodies"].items():
        if bn in used:
            needed_vis.update(b["vis"])
            needed_col.update(b["coll"])
    for d in mev["dummies"].values():
        if d["rel_to"] in mev["tyres"]:
            needed_vis.update(d["vis"])
    for name in sorted(needed_vis):
        src = (model_dir / graphics[name]["file"]).resolve()
        stem = snake(src.stem)
        if stem in info["visual"]:
            continue
        stats = mt.write_obj(mt.read_osgt(src), vis_dir / f"{stem}.obj", to_gazebo=True)
        info["visual"][stem] = {"source": str(Path(graphics[name]["file"]).name), "triangles": stats["triangles"],
                                "bounds_gz": [[round(v, 3) for v in stats["bounds"][0]], [round(v, 3) for v in stats["bounds"][1]]]}
    for name in sorted(needed_col):
        src = (model_dir / graphics[name]["file"]).resolve()
        stem = snake(src.stem)
        v, f = mt.read_3ds(src)
        stats = mt.write_stl(v, f, col_dir / f"{stem}.stl", to_gazebo=True)
        info["collision"][stem] = {"source": src.name, "triangles": stats["triangles"],
                                   "bounds_gz": [[round(x, 3) for x in stats["bounds"][0]], [round(x, 3) for x in stats["bounds"][1]]]}
    return info


# ---------------------------------------------------------------------------
# SDF
# ---------------------------------------------------------------------------

def build_sdf(d: dict, mev: dict, name: str) -> str:
    z = d["z_off"]
    o = d["opts"]
    W = d["W"]
    bodies = mev["bodies"]
    graphics = mev["graphics"]
    L: list[str] = []
    a = L.append

    def vis_block(gname, indent="      ", link_T=None):
        g = graphics[gname]
        stem = snake(Path(g["file"]).stem)
        Tg = to_gz_T(g["T"])
        a(f'{indent}<visual name="{snake(gname)}">')
        a(f"{indent}  <pose>{pose_str(Tg)}</pose>")
        a(f"{indent}  <geometry><mesh><uri>model://{name}/meshes/{stem}.obj</uri></mesh></geometry>")
        a(f"{indent}</visual>")

    def col_block(gname, indent="      "):
        g = graphics[gname]
        stem = snake(Path(g["file"]).stem)
        a(f'{indent}<collision name="{snake(gname)}">')
        a(f"{indent}  <geometry><mesh><uri>model://{name}/meshes/collision/{stem}.stl</uri></mesh></geometry>")
        a(f"{indent}</collision>")

    a('<?xml version="1.0"?>')
    a("<!--")
    a("  GENERATED by scripts/generate_wheelloader_model.py - do not edit by hand.")
    a("  Source of truth: the Mevea WheelLoader model (see wheelloader_params.json).")
    a("  Frame: Gazebo Z-up, x forward, y left.  Model origin = Mevea world origin (rear axle")
    a(f"  centre line, ground level), raised {fmt(z)} m so the lowest tyre rests on z = 0.")
    a("-->")
    a('<sdf version="1.10">')
    a(f'  <model name="{name}">')
    a("    <self_collide>false</self_collide>")
    a("")

    # links ---------------------------------------------------------------
    order = ["rear_frame", "rear_axle", "front_frame", "lift_arm", "bucket_clamp", "bucket"]
    for link in order:
        body = LINKS[link]
        Tl = to_gz_T(W[body])
        inr = d["inertials"][link]
        I = inr["I_gz"]
        a(f'    <link name="{link}">')
        a(f"      <pose>{pose_str(Tl, z)}</pose>")
        a("      <inertial>")
        a(f"        <pose>{' '.join(fmt(v) for v in inr['com_gz'])} 0 0 0</pose>")
        a(f"        <mass>{fmt(inr['mass'])}</mass>")
        a("        <inertia>")
        a(f"          <ixx>{fmt(I[0][0])}</ixx><iyy>{fmt(I[1][1])}</iyy><izz>{fmt(I[2][2])}</izz>")
        a(f"          <ixy>{fmt(I[0][1])}</ixy><ixz>{fmt(I[0][2])}</ixz><iyz>{fmt(I[1][2])}</iyz>")
        a("        </inertia>")
        a("      </inertial>")
        for g in bodies[body]["vis"]:
            vis_block(g)
        for g in bodies[body]["coll"]:
            col_block(g)
        if link == "rear_frame":
            for cname, cam in d["cameras"].items():
                a(f'      <sensor name="{snake(cname)}" type="camera">')
                a(f"        <pose>{cam['pose']}</pose>")
                a(f"        <topic>/{name}/{snake(cname)}/image</topic>")
                a("        <update_rate>15</update_rate>")
                a("        <camera>")
                a(f"          <horizontal_fov>{fmt(math.radians(cam['fov_deg']))}</horizontal_fov>")
                a("          <image><width>640</width><height>480</height><format>R8G8B8</format></image>")
                a("          <clip><near>0.1</near><far>150.0</far></clip>")
                a("        </camera>")
                a("        <always_on>true</always_on>")
                a("        <visualize>true</visualize>")
                a("      </sensor>")
        a("    </link>")
        a("")

    wa, wp = d["wheel_inertia"]
    for wname, w in d["wheels"].items():
        Tw = to_gz_T(w["T"])
        left = wname.endswith(("_rl", "_fl"))
        vis_g = next(v for dn, dd in mev["dummies"].items() if dd["rel_to"] == w["tyre"] for v in dd["vis"])
        a(f'    <link name="{wname}">')
        a(f"      <pose>{pose_str(Tw, z)}</pose>")
        a("      <inertial>")
        a(f"        <mass>{fmt(o['wheel_mass_kg'])}</mass>")
        a("        <inertia>")
        a(f"          <ixx>{fmt(wp)}</ixx><iyy>{fmt(wa)}</iyy><izz>{fmt(wp)}</izz>")
        a("          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>")
        a("        </inertia>")
        a("      </inertial>")
        vis_block(vis_g)
        a(f'      <collision name="{wname}_collision">')
        a("        <pose>0 0 0 1.5707963 0 0</pose>")
        a(f"        <geometry><cylinder><radius>{fmt(w['r'])}</radius><length>{fmt(2 * CONTACT_PROFILE_HALF_WIDTH)}</length></cylinder></geometry>")
        a("        <surface><friction><ode>")
        a(f"          <mu>{fmt(o['mu'])}</mu><mu2>{fmt(o['mu2'])}</mu2><fdir1>1 0 0</fdir1>")
        a("        </ode></friction></surface>")
        a("      </collision>")
        a("    </link>")
        a("")

    # joints --------------------------------------------------------------
    R = d["ranges"]

    def axis_gz(axis_world):
        return mv(M_GZ, axis_world)

    def joint(jname, jtype, parent, child, mname, axis_w, lo=None, hi=None, effort=1.0e7,
              damping=0.0, spring=None, comment=None):
        # joint frame origin = joint CS position, expressed in the child link frame
        c = d["jf"][mname]
        child_body = LINKS.get(child, None)
        if comment:
            a(f"    <!-- {comment} -->")
        a(f'    <joint name="{jname}" type="{jtype}">')
        a(f"      <parent>{parent}</parent>")
        a(f"      <child>{child}</child>")
        if jtype != "fixed":
            Tc = to_gz_T(c["P1"])
            Tcl = to_gz_T(W[child_body])
            local = compose(invert(Tcl), (I3, Tc[1]))  # joint position in child frame
            a(f"      <pose>{' '.join(fmt(v) for v in local[1])} 0 0 0</pose>")
            ax = axis_gz(axis_w)
            a("      <axis>")
            a(f'        <xyz expressed_in="__model__">{" ".join(fmt(v) for v in ax)}</xyz>')
            if lo is not None:
                a(f"        <limit><lower>{fmt(lo)}</lower><upper>{fmt(hi)}</upper><effort>{fmt(effort)}</effort></limit>")
            else:
                a(f"        <limit><effort>{fmt(effort)}</effort></limit>")
            dyn = f"<damping>{fmt(damping)}</damping>" if damping else ""
            if spring is not None:
                dyn += f"<spring_stiffness>{fmt(spring)}</spring_stiffness><spring_reference>0</spring_reference>"
            if dyn:
                a(f"        <dynamics>{dyn}</dynamics>")
            a("      </axis>")
        a("    </joint>")
        a("")

    ax = R["axle_oscillation"]
    joint("axle_oscillation_joint", "revolute", "rear_frame", "rear_axle", "RearAxle_RearFrame",
          ax["axis_world"], ax["lower"], ax["upper"], damping=ax["C"], spring=ax["K"],
          comment="Oscillating rear axle: Mevea spring K and damper C, end stops at +-12 deg")
    ar = R["articulation"]
    joint("articulation_joint", "revolute", "rear_frame", "front_frame", "RearFrame_FrontFrame",
          ar["axis_world"], ar["lower"], ar["upper"], effort=1.0e7,
          comment="Articulation pivot (vertical); range from the steering cylinder stroke; positive = front frame turns left")
    lf = R["lift"]
    joint("lift_joint", "revolute", "front_frame", "lift_arm", "FrontFrame_LiftArm",
          lf["axis_world"], lf["lower"], lf["upper"], effort=1.0e7,
          comment="Lift arm; range from the lift cylinder stroke; angle 0 = Mevea initial pose, positive = raise")
    tl = R["bucket_tilt"]
    joint("bucket_tilt_joint", "revolute", "lift_arm", "bucket_clamp", "LiftArm_BucketClamp",
          tl["axis_world"], tl["lower"], tl["upper"], effort=1.0e7,
          comment="Bucket pivot, driven directly (Z-bar loop not simulated); Mevea damper range; positive = rack back")
    a('    <joint name="bucket_fixed_joint" type="fixed">')
    a("      <parent>bucket_clamp</parent>")
    a("      <child>bucket</child>")
    a("    </joint>")
    a("")
    for wname, w in d["wheels"].items():
        a(f'    <joint name="{wname}_joint" type="revolute">')
        a(f"      <parent>{w['parent'] == 'RearAxle' and 'rear_axle' or 'front_frame'}</parent>")
        a(f"      <child>{wname}</child>")
        a("      <axis>")
        a('        <xyz expressed_in="__model__">0 1 0</xyz>')
        a(f"        <limit><effort>{fmt(o['wheel_effort_nm'])}</effort></limit>")
        a("      </axis>")
        a("    </joint>")
        a("")

    # plugins -------------------------------------------------------------
    for wname in d["wheels"]:
        a('    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">')
        a(f"      <joint_name>{wname}_joint</joint_name>")
        a(f"      <topic>/{name}/{wname}/cmd_vel</topic>")
        a("    </plugin>")
    pos_ctl = [("articulation_joint", "articulation", o["articulation_p"], o["articulation_d"]),
               ("lift_joint", "lift", o["lift_p"], o["lift_d"]),
               ("bucket_tilt_joint", "bucket_tilt", o["tilt_p"], o["tilt_d"])]
    for jn, short, p, dd in pos_ctl:
        a('    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">')
        a(f"      <joint_name>{jn}</joint_name>")
        a(f"      <topic>/{name}/{short}/cmd_pos</topic>")
        a(f"      <p_gain>{fmt(p)}</p_gain><i_gain>0</i_gain><d_gain>{fmt(dd)}</d_gain>")
        a("      <cmd_max>1.0e7</cmd_max><cmd_min>-1.0e7</cmd_min>")
        a("    </plugin>")
    a('    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>')
    a('    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">')
    a("      <odom_publish_frequency>20</odom_publish_frequency>")
    a("      <dimensions>3</dimensions>")
    a("    </plugin>")
    a("  </model>")
    a("</sdf>")
    return "\n".join(L) + "\n"


def build_config(name: str) -> str:
    return f"""<?xml version="1.0"?>

<model>
  <name>{name}</name>
  <version>1.0</version>

  <sdf version="1.10">
    model.sdf
  </sdf>

  <author>
    <name>Edge Path Project</name>
  </author>

  <description>
    Articulated wheel loader (about 12.6 t) for Gazebo, generated from the Mevea
    WheelLoader model.  See wheelloader_params.json for extracted values,
    simplifications and assumptions.
  </description>
</model>
"""


def camera_poses(d: dict, mev: dict) -> dict:
    """Camera sensor poses in the rear_frame link frame, from Mevea Ref / LookAt CS."""
    W, bodies = d["W"], mev["bodies"]
    out = {}
    Tl = invert(to_gz_T(W["RearFrame"]))
    for cname in ("Camera_Seat", "Camera_Rear"):
        c = mev["cameras"][cname]
        ref = compose(W[c["ref_body"]], bodies[c["ref_body"]]["cs"][c["ref_cs"]])[1]
        look = compose(W[c["look_body"]], bodies[c["look_body"]]["cs"][c["look_cs"]])[1]
        dv = mv(M_GZ, vsub(look, ref))
        dv = mv(Tl[0], dv)
        yaw = math.atan2(dv[1], dv[0])
        pitch = -math.atan2(dv[2], math.hypot(dv[0], dv[1]))
        p = mv(M_GZ, ref)
        p_link = compose(Tl, (I3, p))[1]
        out[cname] = {"pose": " ".join(fmt(v) for v in (*p_link, 0.0, pitch, yaw)), "fov_deg": c["fov_deg"]}
    return out


def build_params(d: dict, mev: dict, mevea_dir: Path, name: str, mesh_info: dict) -> dict:
    o = d["opts"]
    inr = d["inertials"]
    mevea_total = sum(b["mass"] for n, b in mev["bodies"].items() if n != "Ground")
    gz_total = sum(v["mass"] for v in inr.values()) + 4 * o["wheel_mass_kg"]
    errs = d["jf"]
    xml_path = mevea_dir / "Model" / "WheelLoader.xml"
    kin = d["kin"]
    R = d["ranges"]
    return {
        "source": {"mevea_xml": "WheelLoader/Model/WheelLoader.xml",
                   "mevea_xml_sha256": hashlib.sha256(xml_path.read_bytes()).hexdigest(),
                   "axis_mapping": "Mevea (x,y,z) -> Gazebo (x,-z,y)",
                   "cs_euler_convention": "R = Rx*Ry*Rz; revolute joint axis = CS z"},
        "model": {"name": name,
                  "wheel_cmd_topics": {w: f"/{name}/{w}/cmd_vel" for w in d["wheels"]},
                  "position_cmd_topics": {k: f"/{name}/{k}/cmd_pos" for k in ("articulation", "lift", "bucket_tilt")},
                  "odometry_topic": f"/model/{name}/odometry",
                  "camera_topics": {c: f"/{name}/{snake(c)}/image" for c in d["cameras"]}},
        "kinematics": {**{k: round(v, 5) for k, v in kin.items()},
                       "max_speed_mps": round(d["max_speed"], 4),
                       "articulation_limit_rad": round(min(-R["articulation"]["lower"], R["articulation"]["upper"]), 5),
                       "reference_point": "rear axle centre; yaw positive = left; articulation positive = front frame left"},
        "joint_ranges_rad": {k: {"lower": round(v["lower"], 5), "upper": round(v["upper"], 5)} for k, v in R.items()},
        "joint_range_checks": d["cyl_notes"],
        "speed_derivation": d["speed"],
        "initial_pose": {"z_offset_m": round(d["z_off"], 4),
                         "note": "Mevea starts with the tyres at static-deflection heights (front 4 mm into "
                                 "the ground, rear 9 mm above); rigid wheels settle ~13 mm on spawn"},
        "mass": {"links_kg": {k: round(v["mass"], 3) for k, v in inr.items()},
                 "wheel_each_kg_assumed": o["wheel_mass_kg"],
                 "mevea_total_kg": round(mevea_total, 3),
                 "gazebo_total_kg": round(gz_total, 3),
                 "lumped": {k: list(v) for k, v in LUMPS.items()}},
        "validation": {
            "true_joints_checked": [c["name"] for c in mev["constraints"] if not c["cut"]],
            "joint_coincidence_max_position_error_m": max(errs[c["name"]]["pos_err"] for c in mev["constraints"] if not c["cut"]),
            "joint_coincidence_max_rotation_error_rad": max(errs[c["name"]]["rot_err"] for c in mev["constraints"] if not c["cut"]),
            "loop_closure_joint": {c["name"]: {"position_gap_m": round(errs[c["name"]]["pos_err"], 5),
                                               "note": "spherical cut joint closing the Z-bar loop; rotation is unconstrained"}
                                   for c in mev["constraints"] if c["cut"]},
            "all_inertia_tensors_positive_definite": all(v["pos_def"] for v in inr.values()),
            "lift_cylinder_L0": "Mevea L0 equals the cylinder length with the arm lowered and the bucket on the ground "
                                "(see joint_range_checks), not the length at the initial pose where the bucket is ~1.09 m up",
        },
        "assumptions": {
            "mu": o["mu"], "mu2": o["mu2"],
            "wheel_effort_nm": o["wheel_effort_nm"],
            "tyre_collision_width_m": 2 * CONTACT_PROFILE_HALF_WIDTH,
            "controller_gains": {k: o[k] for k in ("articulation_p", "articulation_d", "lift_p", "lift_d", "tilt_p", "tilt_d")},
            "inertia_notes": ["Mevea inertia is treated as defined about the CM with axes parallel to the body frame; "
                              "LiftArm and BucketClamp also have a separate CmiCS (CS_CMI) whose meaning is not documented "
                              "(effect: at most ~36 kg m^2 on the lift arm, ~8 kg m^2 on the clamp).",
                              "Off-diagonal terms are taken as tensor components (the sign convention is not documented)."],
        },
        "not_modelled": ["Z-bar tilt linkage (BucketLever/BucketLink kinematics) - mass lumped, meshes omitted",
                         "hydraulic cylinders and valves", "diesel engine, torque converter, gearbox, differentials",
                         "LuGre tyre model and tyre compliance (K=2.636e6 N/m, C=3.25e4 N s/m)",
                         "brakes", "sand/particle bucket interaction", "textures"],
        "meshes": mesh_info,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mevea-dir", required=True, type=Path, help="the WheelLoader folder of the Mevea repo")
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--name", default="wheelloader")
    for key, default in DEFAULTS.items():
        ap.add_argument("--" + key.replace("_", "-"), type=float, default=default)
    args = ap.parse_args(argv)

    mevea_dir = args.mevea_dir.resolve()
    xml_path = mevea_dir / "Model" / "WheelLoader.xml"
    if not xml_path.exists():
        print(f"not found: {xml_path}", file=sys.stderr)
        return 2
    opts = {k: getattr(args, k) for k in DEFAULTS}
    mev = parse_mevea(xml_path)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    mesh_info = convert_meshes(mev, mevea_dir, args.out_dir)
    d = derive(mev, mesh_info, opts)
    d["cameras"] = camera_poses(d, mev)

    out = args.out_dir
    (out / "model.sdf").write_text(build_sdf(d, mev, args.name), encoding="utf-8", newline="\n")
    (out / "model.config").write_text(build_config(args.name), encoding="utf-8", newline="\n")
    params = build_params(d, mev, mevea_dir, args.name, mesh_info)
    (out / "wheelloader_params.json").write_text(json.dumps(params, indent=2) + "\n", encoding="utf-8", newline="\n")

    print(f"wrote {out}")
    print(f"  mass: Mevea {params['mass']['mevea_total_kg']:.0f} kg, Gazebo {params['mass']['gazebo_total_kg']:.0f} kg "
          f"(incl. 4 x {opts['wheel_mass_kg']:.0f} kg assumed tyres)")
    print(f"  max speed {d['max_speed']:.2f} m/s; wheelbase {d['kin']['wheelbase']:.3f} m")
    for k, v in params["joint_ranges_rad"].items():
        print(f"  {k:17s} [{v['lower']:+.3f}, {v['upper']:+.3f}] rad")
    v = params["validation"]
    print(f"  joint coincidence ({len(v['true_joints_checked'])} true joints): max pos err "
          f"{v['joint_coincidence_max_position_error_m']:.2e} m, max rot err {v['joint_coincidence_max_rotation_error_rad']:.2e} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
