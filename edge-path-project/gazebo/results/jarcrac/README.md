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
records limited motion and sensor comparisons.

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

The [tracked-model handoff](../../docs/CAMINO_JARCRAC_HANDOFF.md) documents
the watchdog and bounded-motion scripts added on 2026-11-12. Their live-run
reports are absent from this branch. Calibrated terrain AEB, autonomous detour
and route rejoining, and JARCRAC-specific HIL remain open.
