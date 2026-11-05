"""Pure, offline-testable LiDAR AEB policy for the experimental Metsakone SIL trial.

Distances in this module are provisional, model-envelope estimates. Runtime
collision checks and physical model calibration are still required.
"""
import math


def front_clearance(scan, half_angle_deg, lidar_to_front_envelope_m):
    """Return nearest valid forward LiDAR range, corrected for front overhang."""
    half_angle = math.radians(half_angle_deg)
    candidates = []
    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_step
        if (abs(angle) <= half_angle
                and math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max):
            candidates.append(distance)
    if not candidates:
        return None
    return max(0.0, min(candidates) - lidar_to_front_envelope_m)


def brake_decision(speed_mps, clearance_m, safety, required_clearance_m):
    """Return (brake, reason, TTC) without depending on Gazebo bindings."""
    if clearance_m is None or not math.isfinite(clearance_m):
        return True, "invalid_lidar", math.inf
    speed = max(0.0, float(speed_mps))
    ttc = clearance_m / speed if speed >= 0.10 else math.inf
    if clearance_m <= safety["emergency_distance_m"]:
        return True, "emergency_distance", ttc
    if ttc <= safety["ttc_threshold_s"]:
        return True, "ttc_threshold", ttc
    assumed_deceleration = safety["assumed_braking_deceleration_mps2"]
    stopping_budget = (required_clearance_m
                       + speed * safety["reaction_allowance_s"]
                       + speed * speed / (2.0 * assumed_deceleration))
    if clearance_m <= stopping_budget:
        return True, "predicted_stopping_distance", ttc
    return False, "clear", ttc


def route_geometry(scenario, x, y):
    """Return (progress, lateral offset, approximate physical clearance).

    The estimated front reach is a provisional model footprint bound. It is
    intentionally NOT a measured mesh-to-obstacle separation.
    """
    start, obstacle = scenario["start"], scenario["obstacle"]
    dx = obstacle["x"] - start["x"]
    dy = obstacle["y"] - start["y"]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    rx, ry = x - start["x"], y - start["y"]
    progress = rx * ux + ry * uy
    lateral = -rx * uy + ry * ux
    half_obstacle = abs(ux) * obstacle["half_x"] + abs(uy) * obstacle["half_y"]
    clearance = (length - progress
                 - scenario["safety"]["estimated_model_front_reach_m"]
                 - half_obstacle)
    return progress, lateral, clearance
