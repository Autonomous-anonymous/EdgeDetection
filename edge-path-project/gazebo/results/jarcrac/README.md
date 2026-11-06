# JARCRAC v2 experiment output

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

Newer read-only terrain, map-assisted, localization-sensitivity and simulated
IMU diagnostics are named `terrain_*.json`, `map_*.json` and
`imu_*.json`. They originate in the operator's WSL checkout and are to be
committed separately from the Markdown documentation update.

See the repository-root `JARCRAC_V2_SIL.md` for interpretation, source files,
limitations and reproduction commands. No physical JARCRAC HIL or dynamic
terrain AEB success is claimed.
