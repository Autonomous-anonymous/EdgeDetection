"""Kinematics of the articulated wheel loader (pure Python, no Gazebo needed).

Geometry (all lengths in metres, from wheelloader_params.json):
    d_r     rear axle centre -> articulation pivot
    d_f     articulation pivot -> front axle centre
    half_track   lateral distance from the centreline to a wheel
    wheel_radius

Reference point of the commanded twist: the REAR AXLE CENTRE.
    v      speed of the rear axle centre along the rear frame (m/s)
    omega  yaw rate of the rear frame (rad/s, positive = left / counter-clockwise)
    phi    articulation angle, front frame relative to rear frame
           (rad, positive = front frame turned left)

Steady-state no-slip constraint at both axles (phi_dot = 0):

    omega = v * sin(phi) / (d_f + d_r * cos(phi))

so, given v and omega,

    k = omega / v
    sin(phi - alpha) = k * d_f / sqrt(1 + (k * d_r)^2),   alpha = atan(k * d_r)
"""

from __future__ import annotations

import math


def yaw_rate(v: float, phi: float, d_f: float, d_r: float) -> float:
    """Rear-frame yaw rate for a steady articulation angle phi."""
    return v * math.sin(phi) / (d_f + d_r * math.cos(phi))


def articulation_for(v: float, omega: float, d_f: float, d_r: float, phi_limit: float) -> float:
    """Articulation angle that realises (v, omega), clamped to +-phi_limit."""
    if abs(v) < 1e-6:
        return 0.0
    k = omega / v
    a = k * d_r
    r = math.hypot(1.0, a)
    arg = max(-1.0, min(1.0, k * d_f / r))
    phi = math.atan(a) + math.asin(arg)
    return max(-phi_limit, min(phi_limit, phi))


def wheel_speeds(v: float, phi: float, d_f: float, d_r: float, half_track: float) -> dict:
    """Linear speed (m/s, positive = forward) of each wheel's contact patch."""
    w = yaw_rate(v, phi, d_f, d_r)
    v_front = v * math.cos(phi) + d_r * w * math.sin(phi)  # front axle centre, along the front frame
    return {
        "rl": v - w * half_track, "rr": v + w * half_track,
        "fl": v_front - w * half_track, "fr": v_front + w * half_track,
        "yaw_rate": w,
    }


def twist_to_commands(v: float, omega: float, geo: dict, v_max: float, phi_limit: float) -> dict:
    """Wheel joint velocities (rad/s) and articulation angle (rad) for a Twist."""
    v = max(-v_max, min(v_max, v))
    phi = articulation_for(v, omega, geo["d_f"], geo["d_r"], phi_limit)
    s = wheel_speeds(v, phi, geo["d_f"], geo["d_r"], geo["half_track"])
    r = geo["wheel_radius"]
    return {
        "articulation": phi,
        "wheel_rl": s["rl"] / r, "wheel_rr": s["rr"] / r,
        "wheel_fl": s["fl"] / r, "wheel_fr": s["fr"] / r,
        "yaw_rate": s["yaw_rate"],
    }
