"""Read-only map-assisted JARCRAC LiDAR diagnostic (NOT an AEB controller).

Compare live front LiDAR returns against the Ruskotunturi *collision* STL,
with Gazebo's exact model pose for localization. A collision mesh is not a
safety-rated map; map mismatch, missing heights, pose errors and small rises
are UNKNOWN, never silently FREE. This module does not publish commands.

No external Python dependencies: binary STL and triangle spatial lookup use
only the standard library. Assumes one Gazebo terrain collision mesh at world
pose (0,0,0) without a collision-local transform, matching Ruskotunturi.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import struct


@dataclass(frozen=True)
class MapScanSummary:
    valid: bool
    reason: str
    scan_returns: int
    mapped_returns: int
    ground_points: int
    obstacle_points: int
    unknown_points: int
    nearest_forward_obstacle_m: float | None
    unknown_fraction: float
    lidar_height_above_mapped_ground_m: float | None
    terrain_mesh_triangles: int


class TerrainMeshIndex:
    """XY spatial buckets of non-vertical triangle facets; exact z lookup.

    The maximum z at a given XY is used for overlapping triangles. This is
    ONLY an experimental terrain heightfield assumption; overhangs, caves or
    stacked geometry require a proper 3D raycasting/mapping approach.
    """
    def __init__(self, stl_path: str | Path, *, center_x: float, center_y: float,
                 radius_m: float = 36.0, cell_m: float = 0.60):
        if not (0 < cell_m <= 2.0 and 5 <= radius_m <= 150):
            raise ValueError('invalid mesh index resolution/radius')
        self.path = Path(stl_path)
        self.cx = float(center_x)
        self.cy = float(center_y)
        self.radius = float(radius_m)
        self.cell = float(cell_m)
        self.triangles: list[tuple] = []
        self.buckets: dict[tuple[int,int], list[int]] = {}
        self.xy_min_x = self.cx - self.radius
        self.xy_max_x = self.cx + self.radius
        self.xy_min_y = self.cy - self.radius
        self.xy_max_y = self.cy + self.radius
        lo_x, hi_x = math.floor(self.xy_min_x/self.cell), math.floor(self.xy_max_x/self.cell)
        lo_y, hi_y = math.floor(self.xy_min_y/self.cell), math.floor(self.xy_max_y/self.cell)
        self.triangle_count_in_roi = 0
        for (a,b,c) in _read_binary_stl_triangles(self.path):
            (x1,y1,z1),(x2,y2,z2),(x3,y3,z3) = a,b,c
            if not all(math.isfinite(t) for t in (*a,*b,*c)):
                continue
            det = (y2-y3)*(x1-x3) + (x3-x2)*(y1-y3)
            if abs(det) < 1e-10:
                continue  # near-vertical triangle, not a well-defined height surface
            minx = max(lo_x, math.floor(min(x1,x2,x3)/self.cell))
            maxx = min(hi_x, math.floor(max(x1,x2,x3)/self.cell))
            miny = max(lo_y, math.floor(min(y1,y2,y3)/self.cell))
            maxy = min(hi_y, math.floor(max(y1,y2,y3)/self.cell))
            if minx > maxx or miny > maxy:
                continue
            k = len(self.triangles)
            # precompute barycentric denominator and normalized coordinates
            self.triangles.append((x1,y1,z1,x2,y2,z2,x3,y3,z3,1.0/det))
            self.triangle_count_in_roi += 1
            for gx in range(minx, maxx+1):
                for gy in range(miny, maxy+1):
                    self.buckets.setdefault((gx,gy), []).append(k)
        if not self.triangles:
            raise ValueError('no terrain triangles near vehicle; check model pose/map frame')

    def height_at(self, x: float, y: float) -> float | None:
        if not (math.isfinite(x) and math.isfinite(y)
                and self.xy_min_x <= x <= self.xy_max_x
                and self.xy_min_y <= y <= self.xy_max_y):
            return None
        bucket = self.buckets.get((math.floor(x/self.cell),math.floor(y/self.cell)), ())
        best = -math.inf
        for x1,y1,z1,x2,y2,z2,x3,y3,z3,invdet in (self.triangles[t] for t in bucket):
            a = ((y2-y3)*(x-x3)+(x3-x2)*(y-y3))*invdet
            b = ((y3-y1)*(x-x3)+(x1-x3)*(y-y3))*invdet
            c = 1.0-a-b
            if min(a,b,c) >= -1e-5:
                z = a*z1+b*z2+c*z3
                if z > best:
                    best = z
        return best if math.isfinite(best) else None


def _read_binary_stl_triangles(path: Path):
    with path.open('rb') as file:
        header = file.read(84)
        if len(header) != 84:
            raise ValueError('invalid STL header')
        count = struct.unpack_from('<I',header,80)[0]
        if path.stat().st_size != 84 + 50*count:
            raise ValueError('expected Gazebo binary collision STL; size mismatch')
        for _ in range(count):
            face = file.read(50)
            if len(face) != 50:
                raise ValueError('truncated STL')
            v = struct.unpack_from('<12f',face)
            yield ((v[3],v[4],v[5]),(v[6],v[7],v[8]),(v[9],v[10],v[11]))


def quaternion_tuple(q):
    vals = tuple(float(getattr(q,k)) for k in ('x','y','z','w'))
    mag = math.sqrt(sum(x*x for x in vals))
    if not math.isfinite(mag) or mag < 1e-6:
        raise ValueError('invalid model orientation quaternion')
    return tuple(x/mag for x in vals)


def rotate(q, v):
    """Rotate 3-vector by normalized (qx,qy,qz,qw) quaternion."""
    x,y,z,w = q
    vx,vy,vz = v
    tx,ty,tz = 2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return (vx+w*tx+(y*tz-z*ty),
            vy+w*ty+(z*tx-x*tz),
            vz+w*tz+(x*ty-y*tx))


def scan_to_map_summary(scan, model_pose, index: TerrainMeshIndex, *,
                        sensor_offset=(1.68,0.,0.600), sector_half_deg=12.0,
                        ground_tolerance_m=0.15, obstacle_min_height_m=0.30,
                        obstacle_max_height_m=3.0, min_ground_points=30,
                        max_unknown_fraction=0.45) -> MapScanSummary:
    def invalid(why, n=0, mapped=0, ground=0, objs=0, unknown=0, nearest=None,
                h=None):
        return MapScanSummary(False,why,n,mapped,ground,objs,unknown,nearest,
                              (unknown/n if n else 1.),h,index.triangle_count_in_roi)
    h = int(getattr(scan,'count',0))
    v = int(getattr(scan,'vertical_count',0))
    if not (h>=3 and v>=2 and len(getattr(scan,'ranges',[])) == h*v):
        return invalid('invalid_scan_shape')
    angles=('angle_min','angle_step','vertical_angle_min','vertical_angle_step',
            'range_min','range_max')
    if not all(math.isfinite(getattr(scan,x,math.nan)) for x in angles):
        return invalid('invalid_scan_angles')
    if not (scan.angle_step>0 and scan.vertical_angle_step>0
            and 0<scan.range_min<scan.range_max and 0<sector_half_deg<45
            and 0<ground_tolerance_m<obstacle_min_height_m<obstacle_max_height_m):
        return invalid('invalid_thresholds_or_angles')
    try:
        q=quaternion_tuple(model_pose.orientation)
        model_t=(float(model_pose.position.x),float(model_pose.position.y),
                 float(model_pose.position.z))
        if not all(map(math.isfinite,model_t)):
            return invalid('invalid_pose_position')
    except (ValueError,AttributeError,TypeError):
        return invalid('invalid_pose_quaternion_or_position')
    off=rotate(q,sensor_offset)
    origin=tuple(model_t[i]+off[i] for i in range(3))
    terrain_origin=index.height_at(origin[0],origin[1])
    sensor_height=(origin[2]-terrain_origin) if terrain_origin is not None else None
    if sensor_height is None or not (0.15<=sensor_height<=1.80):
        return invalid('sensor_height_map_mismatch',h=sensor_height)
    indices=[(i,scan.angle_min+i*scan.angle_step) for i in range(h)
             if abs(scan.angle_min+i*scan.angle_step)<=math.radians(sector_half_deg)]
    if len(indices)<3:
        return invalid('no_forward_bins',h=sensor_height)
    n=mapped=ground=obstacles=unknown=0
    nearest=math.inf
    for ring in range(v):
        elev=scan.vertical_angle_min+ring*scan.vertical_angle_step
        ce,se=math.cos(elev),math.sin(elev)
        for col,az in indices:
            r=float(scan.ranges[ring*h+col])
            if not(math.isfinite(r) and scan.range_min<=r<scan.range_max):
                continue
            xf=r*ce*math.cos(az)
            if xf<=0:
                continue
            n+=1
            v_world=rotate(q,(xf,r*ce*math.sin(az),r*se))
            wx,wy,wz=(origin[j]+v_world[j] for j in range(3))
            expected_z=index.height_at(wx,wy)
            if expected_z is None:
                unknown+=1
                continue
            mapped+=1
            residual=wz-expected_z
            if abs(residual)<=ground_tolerance_m:
                ground+=1
            elif obstacle_min_height_m<=residual<=obstacle_max_height_m:
                obstacles+=1
                nearest=min(nearest,xf)
            else:
                unknown+=1
    if n==0:
        return invalid('no_finite_forward_returns',h=sensor_height)
    nearest=(nearest if math.isfinite(nearest) else None)
    if ground<min_ground_points:
        return invalid('insufficient_mapped_ground_support',n,mapped,ground,obstacles,unknown,nearest,sensor_height)
    if unknown/n>max_unknown_fraction:
        return invalid('map_mismatch_or_unmapped_returns',n,mapped,ground,obstacles,unknown,nearest,sensor_height)
    return MapScanSummary(True,'ok_with_unknowns' if unknown else 'ok',n,mapped,
                          ground,obstacles,unknown,nearest,unknown/n,sensor_height,
                          index.triangle_count_in_roi)
