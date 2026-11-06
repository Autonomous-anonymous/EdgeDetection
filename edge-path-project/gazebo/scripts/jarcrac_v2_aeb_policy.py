"""Pure calculations for JARCRAC v2 front-mounted LiDAR AEB on a flat plane.

This is an experimental velocity-command controller, NOT hydraulic braking,
verified obstacle clearance, terrain segmentation, or functional safety logic.
"""
import math


def braking_decision(speed_mps, forward_gap_m, safety, required_clearance_m):
    """Return (brake, reason, TTC). Invalid observations demand a stop."""
    if (forward_gap_m is None or not math.isfinite(forward_gap_m)
            or forward_gap_m < 0):
        return True, "invalid_lidar", math.inf
    if not math.isfinite(speed_mps) or speed_mps < 0:
        return True, "invalid_speed", math.inf
    decel = safety["assumed_deceleration_mps2"]
    if not math.isfinite(decel) or decel <= 0:
        raise ValueError("assumed_deceleration_mps2 must be positive")
    if required_clearance_m <= 0:
        raise ValueError("required_clearance_m must be positive")
    v = max(0.0, float(speed_mps))
    ttc = forward_gap_m / v if v >= 0.10 else math.inf
    if forward_gap_m <= safety["emergency_distance_m"]:
        return True, "emergency_distance", ttc
    if ttc <= safety["ttc_threshold_s"]:
        return True, "ttc_threshold", ttc
    budget = (required_clearance_m + v * safety["reaction_allowance_s"]
              + (v*v) / (2 * decel))
    if forward_gap_m <= budget:
        return True, "predicted_stopping_distance", ttc
    return False, "clear", ttc


def axial_geometry(config, pose):
    """Return (progress, lateral deviation, estimated axial chassis gap).

    Only valid in this axis-aligned flat-world test, not mesh-to-mesh clearance.
    """
    x, y, _yaw = pose
    obs = config["obstacle"]
    return (x - config["start"]["x"], y - config["start"]["y"],
            obs["x"] - obs["half_x"] - x - config["safety"]["estimated_front_reach_m"])


def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))
