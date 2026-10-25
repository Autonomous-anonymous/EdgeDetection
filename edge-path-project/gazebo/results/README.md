# Gazebo result files

This directory contains validation output from Gazebo and Ruskotunturi SIL tests.

## Final SIL demo

`ruskotunturi_final_sil_demo_results.csv` uses the current final-demo result schema written by `scripts/ruskotunturi_final_sil_demo.py`. It contains the validated v5/v5.1 PASS runs and is the file to use for current reporting or analysis.

`ruskotunturi_final_sil_demo_results_legacy.csv` preserves earlier development runs from older controller/result schemas. These rows are kept as development evidence, but their columns are not compatible with the current final-demo writer and should not be combined directly with the current CSV.

Other CSV files in this directory correspond to earlier staged Gazebo validation tests (AEB, TTC, LiDAR avoidance, Ruskotunturi baseline/static/moving obstacles, sensor fusion and multi-vehicle coordination).
