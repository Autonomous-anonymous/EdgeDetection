# JARCRAC SIL results

This directory stores versioned raw Gazebo SIL and diagnostic evidence. The
flat-world AEB runner writes a JSON summary and matching CSV trace when
invoked with `--report`.

The operator's 2026-11-06 flat-world AEB runs are committed as
`v2_front_3d_aeb_first_trial.{json,csv}`,
`v2_front_3d_aeb_0p7mps_trial.{json,csv}`, and
`v2_front_3d_aeb_1p0mps_trial.{json,csv}`. All three reported PASS, using
estimated clearance and simulated DiffDrive deceleration rather than
calibrated hydraulic braking.

`v2_contact_positive_trial.json` records a separate PASS for the deliberate
chassis–obstacle contact-sensor control. It is **not** AEB acceptance evidence.

The committed `terrain_*.json`, `map_*.json` and `imu_*.json` files include
terrain, map-assisted perception, localization sensitivity and simulated IMU
diagnostics. The [dynamic clear-terrain result](imu_dynamic_clear_diagnostic.json)
records limited motion and sensor comparisons. In the [moving-vehicle obstacle diagnostic](jarcrac_dynamic_obstacle_recheck.json), JARCRAC advanced 0.3551 m toward a known static obstacle and recorded eight paired samples during motion.

See the [JARCRAC v2 SIL guide](../../../../JARCRAC_V2_SIL.md) for the
wheel-based model's reproduction steps and limits.

## Detailed tracked model

- [Terrain forward and stop](jarcrac_detailed_terrain_forward_stop.json)
  records short, low-speed motion and stopping.
- [Half-step TTC stop-and-hold](jarcrac_detailed_ttc_stop_hold_halfstep.json)
  records a stop and a five-second hold in a known static-obstacle scene using
  an instrumentation threshold.
- [Track-ground contact](jarcrac_detailed_track_contact.json) and
  [chassis contact](jarcrac_detailed_contact_positive.json) record stationary
  simulated contact checks.
- [Detour preview](jarcrac_detailed_detour_preview.json) records a read-only
  geometry check.
- [Solo detour and rejoin](jarcrac_compact_rejoin_laptop_pass.json) records a live Gazebo PASS. After driving around the obstacle, JARCRAC travelled 1.502 m aligned with the route over 7.622 simulated seconds and stopped at its goal.
- [Two-vehicle compact trial](jarcrac_compact_two_vehicle_laptop_pass.json) records a live Gazebo PASS. JARCRAC rejoined with 1.504 m of aligned travel over 6.145 simulated seconds. Vehicle B stopped for the crossing, resumed and reached its goal. Both vehicles finished stopped.

Both compact trials used Gazebo odometry. Independent localization, calibrated terrain AEB and JARCRAC-specific R2100 HIL remain open. Live reports are also unavailable for the two watchdog and bounded-motion experiments described in the 12 November handoff. [Compact trial guide](../../docs/JARCRAC_COMPACT_SIL.md) · [Historical handoff](../../docs/CAMINO_JARCRAC_HANDOFF.md)
