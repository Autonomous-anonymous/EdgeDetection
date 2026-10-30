# Gazebo result files

This directory contains validation output from Gazebo and Ruskotunturi SIL tests.

## Current final SIL baseline

The current committed reporting baseline is **Ruskotunturi final SIL v6.4**.

Primary evidence:

- `ruskotunturi_final_sil_demo_v6_4_results.csv` — PASS, `final_demo_complete`, no failed checks.
- `final_sil_v6_4_trace_20261026T133025_782767.csv` — matching per-step trace referenced by the v6.4 result row.

The v6.4 result records both vehicles completing their required mission state, static avoidance completion, camera use for control, moving-obstacle completion, verified stop/resume behaviour, no loading-zone overlap violation and no terrain or LiDAR violation.

## Earlier final-demo results

`ruskotunturi_final_sil_demo_results.csv` and the earlier v5/v5.1-era files are retained as development evidence. They are not the current reporting baseline.

`ruskotunturi_final_sil_demo_results_legacy.csv` preserves still older development runs from earlier controller/result schemas. Rows from incompatible schemas should not be combined directly without explicit conversion.

Other CSV files in this directory correspond to staged Gazebo validation tests such as AEB, TTC, LiDAR avoidance, Ruskotunturi baseline/static/moving obstacles, sensor fusion and multi-vehicle coordination.

See the repository-level `VALIDATION.md` for the current SIL/HIL evidence catalogue and documentation index.
