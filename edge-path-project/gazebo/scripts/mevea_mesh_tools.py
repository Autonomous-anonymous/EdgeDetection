#!/usr/bin/env python3
"""Convert Mevea mesh formats that Gazebo cannot read.

    OpenSceneGraph ASCII (.osgt)  ->  Wavefront OBJ + MTL   (visuals, flat colours)
    Autodesk 3DS (.3ds)           ->  binary STL            (collision shapes)

Both converters are standard-library only.  Textures are not carried over:
each OSG geometry keeps its material diffuse colour.

Axis handling
    Mevea is Y-up (x forward, y up, z right); Gazebo is Z-up (x forward, y left,
    z up).  Pass ``to_gazebo=True`` to bake the mapping (x, y, z) -> (x, -z, y)
    into the vertices and normals.  It is a proper rotation, so triangle winding
    is preserved.
"""

from __future__ import annotations

import re
import struct
from pathlib import Path


def mevea_to_gz_point(p):
    return (p[0], -p[2], p[1])


# ---------------------------------------------------------------------------
# OSG ASCII (.osgt)
# ---------------------------------------------------------------------------

_ARRAY = r"Array (?:TRUE|FALSE) ArrayID (\d+)(?: Vec3fArray (\d+)\s*\{(.*?)\})?"


def read_osgt(path: Path) -> list[dict]:
    """Return a list of geometries: name, diffuse (r, g, b, a), vertices,
    normals, triangles (index triples)."""
    text = Path(path).read_text(encoding="latin-1").replace("\r", "")
    blocks = re.split(r"\n\s*osg::Geometry\s*\n", text)[1:]
    arrays: dict[int, list] = {}
    geoms = []

    def floats3(body: str) -> list:
        v = [float(x) for x in body.split()]
        return [tuple(v[i:i + 3]) for i in range(0, len(v) - 2, 3)]

    def grab(block: str, key: str):
        m = re.search(key + r"\s*\{\s*" + _ARRAY, block, re.S)
        if not m:
            return None
        aid = int(m.group(1))
        if m.group(3) is not None:
            arrays[aid] = floats3(m.group(3))
        return arrays.get(aid)

    for block in blocks:
        name = re.search(r'Name "([^"]*)"', block)
        diff = re.search(r"Diffuse\s+TRUE Front\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", block)
        verts = grab(block, "VertexData")
        norms = grab(block, "NormalData")
        prim = re.search(r"DrawElementsUInt GL_TRIANGLES (\d+)\s*\{(.*?)\}", block, re.S)
        if verts is None or prim is None:
            continue
        idx = [int(x) for x in prim.group(2).split()]
        tris = [tuple(idx[i:i + 3]) for i in range(0, len(idx) - 2, 3)]
        geoms.append({
            "name": name.group(1) if name else f"geom{len(geoms)}",
            "diffuse": tuple(float(diff.group(k)) for k in range(1, 5)) if diff else (0.5, 0.5, 0.5, 1.0),
            "vertices": verts,
            "normals": norms if norms and len(norms) == len(verts) else None,
            "triangles": tris,
        })
    return geoms


def transform_geoms(geoms: list[dict], to_gazebo: bool) -> list[dict]:
    if not to_gazebo:
        return geoms
    out = []
    for g in geoms:
        h = dict(g)
        h["vertices"] = [mevea_to_gz_point(v) for v in g["vertices"]]
        h["normals"] = [mevea_to_gz_point(n) for n in g["normals"]] if g["normals"] else None
        out.append(h)
    return out


def bounds(points) -> tuple[list, list]:
    pts = list(points)
    return ([min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)])


def write_obj(geoms: list[dict], obj_path: Path, to_gazebo: bool = True, mtl_name: str | None = None) -> dict:
    """Write geometries to OBJ + MTL.  Returns {'triangles': n, 'bounds': (min, max)}."""
    obj_path = Path(obj_path)
    geoms = transform_geoms(geoms, to_gazebo)
    mtl_name = mtl_name or obj_path.with_suffix(".mtl").name
    lines = [f"# converted from Mevea OSG geometry; axes {'Gazebo (x fwd, y left, z up)' if to_gazebo else 'Mevea'}",
             f"mtllib {mtl_name}"]
    mats: dict[tuple, str] = {}
    vbase = 1
    ntri = 0
    allv = []
    for g in geoms:
        key = tuple(round(c, 4) for c in g["diffuse"])
        mats.setdefault(key, f"mat{len(mats)}")
        lines.append("o " + re.sub(r"\s+", "_", g["name"]))
        lines.append(f"usemtl {mats[key]}")
        lines += [f"v {round(v[0], 5) + 0.0:.5f} {round(v[1], 5) + 0.0:.5f} {round(v[2], 5) + 0.0:.5f}" for v in g["vertices"]]
        allv += g["vertices"]
        if g["normals"]:
            lines += [f"vn {round(n[0], 4) + 0.0:.4f} {round(n[1], 4) + 0.0:.4f} {round(n[2], 4) + 0.0:.4f}" for n in g["normals"]]
            lines += [f"f {a + vbase}//{a + vbase} {b + vbase}//{b + vbase} {c + vbase}//{c + vbase}"
                      for a, b, c in g["triangles"]]
        else:
            lines += [f"f {a + vbase} {b + vbase} {c + vbase}" for a, b, c in g["triangles"]]
        vbase += len(g["vertices"])
        ntri += len(g["triangles"])
    obj_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")

    mtl = []
    for key, name in mats.items():
        r, g_, b, a = key
        mtl += [f"newmtl {name}", f"Ka {r:.4f} {g_:.4f} {b:.4f}", f"Kd {r:.4f} {g_:.4f} {b:.4f}",
                "Ks 0.1 0.1 0.1", "Ns 20", f"d {a:.4f}", ""]
    obj_path.with_name(mtl_name).write_text("\n".join(mtl), encoding="utf-8", newline="\n")
    return {"triangles": ntri, "bounds": bounds(allv)}


# ---------------------------------------------------------------------------
# 3DS
# ---------------------------------------------------------------------------

def read_3ds(path: Path) -> tuple[list, list]:
    """Return (vertices, triangles) merged over all objects in the file."""
    data = Path(path).read_bytes()

    def chunks(start, end):
        pos = start
        while pos + 6 <= end:
            cid, length = struct.unpack_from("<HI", data, pos)
            if length < 6:
                return
            yield cid, pos + 6, min(pos + length, end)
            pos += length

    verts: list = []
    tris: list = []
    for cid, s, e in chunks(0, len(data)):
        if cid != 0x4D4D:
            continue
        for cid2, s2, e2 in chunks(s, e):
            if cid2 != 0x3D3D:
                continue
            for cid3, s3, e3 in chunks(s2, e2):
                if cid3 != 0x4000:
                    continue
                name_end = data.index(b"\0", s3) + 1
                for cid4, s4, e4 in chunks(name_end, e3):
                    if cid4 != 0x4100:
                        continue
                    base = len(verts)
                    for cid5, s5, e5 in chunks(s4, e4):
                        if cid5 == 0x4110:
                            n = struct.unpack_from("<H", data, s5)[0]
                            verts += [struct.unpack_from("<3f", data, s5 + 2 + 12 * i) for i in range(n)]
                        elif cid5 == 0x4120:
                            n = struct.unpack_from("<H", data, s5)[0]
                            for i in range(n):
                                a, b, c, _ = struct.unpack_from("<4H", data, s5 + 2 + 8 * i)
                                tris.append((a + base, b + base, c + base))
    return verts, tris


def write_stl(vertices: list, triangles: list, path: Path, to_gazebo: bool = True) -> dict:
    """Write a binary STL.  Returns {'triangles': n, 'bounds': (min, max)}."""
    vs = [mevea_to_gz_point(v) for v in vertices] if to_gazebo else list(vertices)
    out = bytearray(b"converted from Mevea 3DS".ljust(80, b"\0"))
    out += struct.pack("<I", len(triangles))
    for a, b, c in triangles:
        p, q, r = vs[a], vs[b], vs[c]
        u = (q[0] - p[0], q[1] - p[1], q[2] - p[2])
        w = (r[0] - p[0], r[1] - p[1], r[2] - p[2])
        n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
        out += struct.pack("<12fH", n[0] / ln, n[1] / ln, n[2] / ln, *p, *q, *r, 0)
    Path(path).write_bytes(bytes(out))
    return {"triangles": len(triangles), "bounds": bounds(vs)}
