"""Pure, fail-closed experimental TTC + stopping-distance decision for JARCRAC.

The input ``sensor_gap_m`` is scanner-forward obstacle range, NOT verified
3-D body clearance. The configurable subtraction is a CONSERVATIVE uncertainty
allowance, not a calibrated sensor/body transformation. Closing speed may be
approximated by vehicle odometry ONLY in a separately verified static-obstacle
scene; general moving-object TTC requires tracked relative velocity.

This is an experimental Gazebo decision function, not real-vehicle safety code.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BrakeConfig:
    ttc_threshold_s: float = 2.5
    reaction_time_s: float = 0.35
    assumed_deceleration_mps2: float = 0.25
    minimum_clearance_m: float = 1.0
    scanner_uncertainty_m: float = 0.35
    minimum_closing_speed_mps: float = 0.02

    def validate(self) -> None:
        vals = (self.ttc_threshold_s, self.reaction_time_s,
                self.assumed_deceleration_mps2, self.minimum_clearance_m,
                self.scanner_uncertainty_m, self.minimum_closing_speed_mps)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('Non-finite brake configuration')
        if not (self.ttc_threshold_s > 0 and self.reaction_time_s >= 0
                and self.assumed_deceleration_mps2 > 0
                and self.minimum_clearance_m > 0
                and self.scanner_uncertainty_m >= 0
                and self.minimum_closing_speed_mps > 0):
            raise ValueError('Invalid brake configuration')


@dataclass(frozen=True)
class BrakeDecision:
    brake: bool
    reason: str
    scanner_gap_m: float | None
    conservative_gap_m: float | None
    closing_speed_mps: float | None
    ttc_s: float | None
    stopping_distance_m: float | None
    required_clearance_m: float | None


def braking_decision(sensor_gap_m: float, closing_speed_mps: float,
                     config: BrakeConfig = BrakeConfig()) -> BrakeDecision:
    """Brake on invalid inputs, hard clearance, stopping budget or TTC.

    The stop budget is v*t_reaction + v^2/(2*a) + minimum clearance.
    TTC uses the conservative sensor-range proxy divided by closing speed.
    Invalid inputs always produce a brake decision, never ``continue``.
    """
    config.validate()
    if (not isinstance(sensor_gap_m, (int, float))
            or isinstance(sensor_gap_m, bool)
            or not isinstance(closing_speed_mps, (int, float))
            or isinstance(closing_speed_mps, bool)
            or not math.isfinite(sensor_gap_m)
            or not math.isfinite(closing_speed_mps)
            or sensor_gap_m < 0 or closing_speed_mps < 0):
        return BrakeDecision(True, 'invalid_brake_inputs', None, None, None,
                             None, None, None)
    gap = max(0.0, sensor_gap_m - config.scanner_uncertainty_m)
    stopping = (closing_speed_mps * config.reaction_time_s
                + closing_speed_mps**2 / (2 * config.assumed_deceleration_mps2))
    required = stopping + config.minimum_clearance_m
    ttc = (gap / closing_speed_mps
           if closing_speed_mps >= config.minimum_closing_speed_mps
           else math.inf)
    if gap <= config.minimum_clearance_m:
        reason = 'minimum_clearance'
    elif gap <= required:
        reason = 'stopping_distance_budget'
    elif ttc <= config.ttc_threshold_s:
        reason = 'ttc_threshold'
    else:
        reason = 'continue'
    return BrakeDecision(reason != 'continue', reason, sensor_gap_m, gap,
                         closing_speed_mps, ttc, stopping, required)
