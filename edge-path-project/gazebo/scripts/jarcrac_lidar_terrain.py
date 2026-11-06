"""Conservative experimental JARCRAC local-ground 3-D LiDAR classifier.

Fits an anchored planar terrain patch z=-sensor_height+a*x+b*y in the LiDAR
frame. Sloped/uneven forest ground is not generally planar; fail closed when
planarity/support is insufficient. This is a *diagnostic*, not approved AEB.
The flat-ground SIL controller remains unchanged.
"""

from __future__ import annotations
from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class TerrainSummary:
    valid: bool
    reason: str
    vertical_layers: int
    horizontal_samples: int
    obstacle_points: int
    ground_points: int
    unknown_points: int
    near_points: int
    nearest_forward_obstacle_m: float | None
    slope_forward: float | None
    slope_lateral: float | None
    ground_fit_rms_m: float | None
    ground_support_span_m: float


def summarize_terrain_forward(scan, *, sensor_height_m=0.600,
                              half_angle_deg=12.0, min_obstacle_height_m=0.30,
                              max_obstacle_height_m=3.0,
                              max_slope=0.35, ground_tolerance_m=0.12,
                              max_fit_rms_m=0.09, minimum_ground_points=25,
                              minimum_support_span_m=1.4,
                              near_radius_m=1.6):
    h = int(getattr(scan, 'count', 0))
    v = int(getattr(scan, 'vertical_count', 0))
    bad = lambda reason: TerrainSummary(False, reason, v, h, 0, 0, 0, 0,
                                       None, None, None, None, 0.0)
    if not (h >= 3 and v >= 3 and len(scan.ranges) == h*v
            and 0 < sensor_height_m < 3 and 0 < half_angle_deg <= 30
            and 0 < min_obstacle_height_m < max_obstacle_height_m
            and 0 < max_slope < 1 and ground_tolerance_m > 0):
        return bad('invalid_scan_dimensions_or_config')
    attrs = ('angle_min', 'angle_step', 'vertical_angle_min',
             'vertical_angle_step', 'range_min', 'range_max')
    if not all(math.isfinite(getattr(scan, x, math.nan)) for x in attrs):
        return bad('invalid_scan_angles_or_limits')
    if not (scan.angle_step > 0 and scan.vertical_angle_step > 0
            and 0 < scan.range_min < scan.range_max):
        return bad('invalid_scan_angles_or_limits')
    indices = [(i, scan.angle_min + i*scan.angle_step) for i in range(h)]
    indices = [(i,a) for i,a in indices if abs(a) <= math.radians(half_angle_deg)]
    if len(indices) < 5:
        return bad('insufficient_azimuth_bins')

    points = []
    for ring in range(v):
        elev = scan.vertical_angle_min + ring*scan.vertical_angle_step
        ce, se = math.cos(elev), math.sin(elev)
        for i, az in indices:
            r = float(scan.ranges[ring*h+i])
            if not (math.isfinite(r) and scan.range_min <= r < scan.range_max):
                continue
            x, y, z = r*ce*math.cos(az), r*ce*math.sin(az), r*se
            if x > 0:
                points.append((x,y,z,elev))
    if len(points) < minimum_ground_points:
        return bad('insufficient_finite_returns')

    # Fit on downward/near-horizontal returns only; ensure candidates are
    # physically compatible with a locally sloped ground anchored beneath the
    # scanner. This limits high obstacles being treated as supporting ground.
    candidates = [p for p in points
                  if p[0] >= 1.1 and p[0] <= 18
                  and p[3] <= math.radians(2)
                  and abs((p[2]+sensor_height_m)/p[0]) <= 1.45*max_slope]
    if len(candidates) < minimum_ground_points:
        return bad('insufficient_ground_candidates')
    # Deterministic downsampling makes performance approximately invariant to
    # horizontal sensor resolution.
    stride = max(1, len(candidates)//550)
    fit_points = candidates[::stride]
    rng = random.Random(42)
    best = []
    best_a = best_b = 0.0
    for _ in range(100):
        p, q = rng.sample(fit_points, 2)
        x1,y1,z1,_ = p
        x2,y2,z2,_ = q
        det = x1*y2-x2*y1
        if abs(det) < 0.05:
            continue
        z1 += sensor_height_m
        z2 += sensor_height_m
        a = (z1*y2-z2*y1)/det
        b = (x1*z2-x2*z1)/det
        if math.hypot(a,b) > max_slope:
            continue
        inliers = [t for t in fit_points if abs(t[2]+sensor_height_m-a*t[0]-b*t[1])
                   <= ground_tolerance_m]
        if len(inliers) > len(best):
            best, best_a, best_b = inliers, a, b
    if len(best) < minimum_ground_points:
        return bad('no_reliable_ground_plane')

    # Refine the anchored fit with 2x2 least squares. The physical 0.6 m
    # height assumption is intentional; a floating plane could latch onto a
    # large flat obstacle. Monitor RMS to detect local nonplanarity.
    sxx=sum(p[0]**2 for p in best)
    sxy=sum(p[0]*p[1] for p in best)
    syy=sum(p[1]**2 for p in best)
    sxz=sum(p[0]*(p[2]+sensor_height_m) for p in best)
    syz=sum(p[1]*(p[2]+sensor_height_m) for p in best)
    det=sxx*syy-sxy*sxy
    if abs(det) < 1e-6:
        return bad('ground_fit_degenerate')
    a=(sxz*syy-syz*sxy)/det
    b=(sxx*syz-sxy*sxz)/det
    if math.hypot(a,b) > max_slope:
        return bad('ground_slope_exceeded')
    ground_fit = [p for p in candidates if abs(p[2]+sensor_height_m-a*p[0]-b*p[1])
                  <= ground_tolerance_m]
    span=(max(p[0] for p in ground_fit)-min(p[0] for p in ground_fit)) if ground_fit else 0
    lateral_span=(max(p[1] for p in ground_fit)-min(p[1] for p in ground_fit)) if ground_fit else 0
    if (len(ground_fit) < minimum_ground_points or span < minimum_support_span_m
            or lateral_span < 0.25):
        return bad('insufficient_spatial_ground_support')
    rms=math.sqrt(sum((p[2]+sensor_height_m-a*p[0]-b*p[1])**2
                      for p in ground_fit)/len(ground_fit))
    if rms > max_fit_rms_m:
        return bad('nonplanar_ground_patch')

    obstacles = ground = unknown = near = 0
    nearest = math.inf
    for x,y,z,_elev in points:
        height=z+sensor_height_m-a*x-b*y
        if abs(height) <= ground_tolerance_m:
            ground += 1
        elif min_obstacle_height_m <= height <= max_obstacle_height_m:
            obstacles += 1
            near += int(x < near_radius_m)
            nearest=min(nearest,x)
        elif height < -ground_tolerance_m or (ground_tolerance_m < height < min_obstacle_height_m):
            # Do not silently declare low rises or sharp drops as clear terrain.
            unknown += 1

    # This is a diagnostic; ambiguous points are exposed in the result and
    # must NOT be interpreted as a clear-path certificate by a controller.
    return TerrainSummary(True,'ok_with_unknowns' if unknown else 'ok',v,h,
                          obstacles,ground,unknown,near,
                          nearest if math.isfinite(nearest) else None,
                          a,b,rms,span)
