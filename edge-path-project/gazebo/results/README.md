# Gazebo result files

This directory contains validation output from **CAMINO Gazebo SIL and R2100 HIL** scenarios. The branch preserves Gazebo evidence, including earlier development runs, but excludes unrelated CARLA and video evidence.

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

## Native R2100 AEB hardware acceptance (2026-11-05)

The physical R2100 running the Debian 12 CI-built ARM64 executable passed the common HIL AEB scenario with `aeb_stop_verified`, `ttc_threshold`, approximately zero final speed, four stop-confirmation samples, **1.236 m clearance** (minimum 1.0 m) and no runtime fault. The original committed evidence is `hil10/hil10_aeb_20261105T070006_b18e73a4.json` and the matching `.csv`. See `../../hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md` for verified artifact hashes and target runtime identity.
