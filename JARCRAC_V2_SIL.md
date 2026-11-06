# CAMINO — JARCRAC v2 front 3D LiDAR AEB (SIL)

This experiment is a new, independent Gazebo Harmonic **software-in-the-loop** demonstration on the `camino` branch. It does **not** execute the current AEB algorithm on the R2100. For physical CAN HIL use the existing shared scenario and `R2100_INSTRUCTOR_DEMO.md`.

## Implementation

- Mevea-derived visual: `edge-path-project/gazebo/models/jarcrac/meshes/Runko_part.STL`.
- Collision: the original simplified body bounding box, intentionally retained.
- Running gear: 12 normalized 0.17 m wheels, 6 drive joints per side, driven by Gazebo DiffDrive (not rope tracks).
- Mass: 2312.24 kg total (includes **estimated** track mass), **not** a calibrated vehicle dynamics model.
- LiDAR: front centre at nominal 0.60 m height, 180° horizontally, 721 horizontal samples, 32 vertical layers from −15° to +15°, 10 Hz, 0.3–30 m, modelled Gaussian sigma 0.015 m.
- AEB: flat-ground LiDAR obstacle-vs-ground filtering, TTC / predicted stopping budget, odometry verified stop-and-hold, fail-closed handling of stale/missing sensors; JSON and CSV report.

## Quick start

From a clean checkout of the `camino` branch with Gazebo Harmonic and Python `gz.transport13` / `gz.msgs10`:

**Terminal 1** (close other JARCRAC worlds first):

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/jarcrac_v2_front_aeb_world.sdf
```

**Terminal 2:**

```bash
cd ~/edge-path-project-git
python3 -u edge-path-project/gazebo/scripts/jarcrac_v2_3d_aeb_sil.py \
  --report edge-path-project/gazebo/results/jarcrac/v2_front_3d_aeb_first_trial.json
```

**Offline tests:**

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac*.py' -v
python3 edge-path-project/gazebo/scripts/jarcrac_v2_3d_aeb_sil.py --dry-run
```

The model generator can regenerate SDF from the Mevea XML if the user has access to the model sources. Regenerating tests require the external XML; otherwise two checks skip.

**Only LiDAR probe:** start `jarcrac_flat_test_world.sdf` instead, then:

```bash
python3 -u edge-path-project/gazebo/scripts/jarcrac_lidar_probe.py \
  --topic /jarcrac/lidar --sensor-height 0.600 --duration 15
```

## Operator-reported live validation (2026-11-06)

The local WSL/Gazebo test output supplied by the operator reported:

| Field | Observed |
| --- | --- |
| Overall result | `PASS` / `aeb_stop_and_estimated_clearance_verified` |
| Nominal drive speed | 0.35 m/s |
| AEB trigger | `predicted_stopping_distance` |
| Progress at trigger | 4.6121 m |
| Final route progress | 4.6433 m |
| Distance after trigger | 0.0312 m |
| First low-speed observation | 0.2 simulated seconds after trigger |
| Final LiDAR gap | 1.3788 m |
| Final *estimated* body gap | 1.6067 m |
| Final speed | 0.0 m/s |
| Contact transport subscribed | true |
| Positive collision-monitor test | **false / not validated** |
| Obstacle contact observed | false |

The prior no-motion LiDAR probe received 100 scans, `layers=32`, `horizontal=721`, `possible_self_pts=0`, with `nearest_forward` approximately 6.02 m. These are operator-provided console observations, not a simulation rerun in the environment used to prepare this integration.

**Preserve the original generated evidence from WSL**, `edge-path-project/gazebo/results/jarcrac/v2_front_3d_aeb_first_trial.json` and matching `.csv`, when committing to Git. Do not substitute this summary for raw measurements.

## Validation boundaries

The result demonstrates automatic braking in **flat-world SIL**, not R2100 HIL, VIL, or real-world vehicle safety. The fixed-height ground filter is not usable without modification on uneven forestry ground. Estimated clearance is not true mesh-to-mesh separation. An advertised/subscribed contact topic is not a positive collision-detection test. DiffDrive's configured deceleration approximately explains the short stopping distance; the true 2-ton-plus vehicle's hydraulic braking remains unvalidated. Confirm Mevea-derived mesh rights before pushing to a public GitHub repository.

## Reference docs

- `edge-path-project/gazebo/models/jarcrac/README.md` — SDF generator parameters and design decisions
- `edge-path-project/gazebo/models/jarcrac/NOTICE.md` — source and redistribution warning
- `R2100_INSTRUCTOR_DEMO.md` — existing native R2100 physical-CAN HIL workflow
