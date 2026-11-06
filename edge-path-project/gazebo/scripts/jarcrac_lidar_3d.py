"""Pure calculations for inspecting JARCRAC's Gazebo 3-D LaserScan.

Gazebo publishes ranges in vertical-major order, each vertical layer having
`count` horizontal samples. The scanner's vertical field of view is intentionally
pointed partly downward. On a *flat world* its ground returns must not be
interpreted as obstacle returns.

IMPORTANT: This flat-plane filter is a development diagnostic, NOT a validated
terrain-segmentation or certified autonomous emergency-braking algorithm. It
cannot differentiate deep ruts, slopes, close self-occlusions or low obstacles
below the chosen vertical-height gate. AEB must fail closed on invalid scans.
"""

from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanSummary:
    valid: bool
    reason: str
    vertical_layers: int
    horizontal_samples: int
    obstacle_points: int
    ground_points: int
    near_points: int
    nearest_forward_obstacle_m: float | None
    min_obstacle_height_m: float


def summarize_forward(scan, *, sensor_height_m: float, half_angle_deg: float = 12.0,
                      ground_z_m: float = 0.0, min_obstacle_height_m: float = 0.30,
                      max_obstacle_height_m: float = 3.0,
                      body_self_radius_m: float = 1.6) -> ScanSummary:
    """Project a flat-ground, level-mounted 3-D scan into forward obstacle points.

    On a real forest floor, replace this simple height gate with an estimated
    local terrain surface / ground segmentation before using it for AEB.

    `body_self_radius_m` is only for counting possible self-returns. It does
    NOT discard them, because silently masking near points is unsafe.
    """
    h = int(getattr(scan, "count", 0))
    v = int(getattr(scan, "vertical_count", 0))
    readings = scan.ranges
    if not (h > 0 and v >= 2 and len(readings) == h * v and
            math.isfinite(sensor_height_m) and sensor_height_m > ground_z_m and
            0 < half_angle_deg < 90 and 0 < min_obstacle_height_m < max_obstacle_height_m and
            math.isfinite(scan.angle_min) and math.isfinite(scan.angle_step) and
            math.isfinite(scan.vertical_angle_min) and math.isfinite(scan.vertical_angle_step) and
            scan.vertical_angle_step > 0 and scan.angle_step > 0):
        return ScanSummary(False, "invalid_shape_or_angles", v, h, 0, 0, 0, None,
                           min_obstacle_height_m)
    if not (math.isfinite(scan.range_min) and math.isfinite(scan.range_max)
            and 0 < scan.range_min < scan.range_max):
        return ScanSummary(False, "invalid_range_limits", v, h, 0, 0, 0, None,
                           min_obstacle_height_m)

    center_indices = [i for i in range(h)
                      if abs(scan.angle_min + i * scan.angle_step)
                      <= math.radians(half_angle_deg)]
    if not center_indices:
        return ScanSummary(False, "no_forward_angular_bins", v, h, 0, 0, 0, None,
                           min_obstacle_height_m)

    ground = obstacles = near = 0
    nearest = math.inf
    for ring in range(v):
        elev = scan.vertical_angle_min + ring * scan.vertical_angle_step
        ce, se = math.cos(elev), math.sin(elev)
        for i in center_indices:
            r = float(readings[ring * h + i])
            if not (math.isfinite(r) and scan.range_min <= r < scan.range_max):
                continue
            az = scan.angle_min + i * scan.angle_step
            forward = r * ce * math.cos(az)
            hit_height = sensor_height_m + r * se - ground_z_m
            if forward <= 0:
                continue
            if hit_height < min_obstacle_height_m:
                ground += 1
                continue
            if hit_height > max_obstacle_height_m:
                continue
            obstacles += 1
            if forward < body_self_radius_m:
                near += 1
            nearest = min(nearest, forward)

    return ScanSummary(True, "ok", v, h, obstacles, ground, near,
                       (nearest if math.isfinite(nearest) else None),
                       min_obstacle_height_m)
