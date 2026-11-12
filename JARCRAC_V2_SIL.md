# CAMINO — JARCRAC v2 front 3D LiDAR AEB (SIL)

This guide covers the wheel-based JARCRAC v2 **software-in-the-loop** experiment in Gazebo Harmonic. See the [tracked-model handoff](edge-path-project/gazebo/docs/CAMINO_JARCRAC_HANDOFF.md) for the separate native-track model. The [physical CAN HIL reference](edge-path-project/hil/README.md) uses Ruskotunturi Vehicle B. JARCRAC-specific R2100 HIL remains open.

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
| Positive collision-monitor test | Not performed in this original AEB trial; later positive-control test **PASS** |
| Obstacle contact observed | false |

The prior no-motion LiDAR probe received 100 scans, `layers=32`, `horizontal=721`, `possible_self_pts=0`, with `nearest_forward` approximately 6.02 m. These are operator-provided console observations, not a simulation rerun in the environment used to prepare this integration.

**Preserve the original generated evidence from WSL**, `edge-path-project/gazebo/results/jarcrac/v2_front_3d_aeb_first_trial.json` and matching `.csv`, when committing to Git. Do not substitute this summary for raw measurements.

## Additional operator-reported validation (2026-11-06)

The following results were obtained in the operator's WSL / Gazebo Harmonic environment. The raw measurements originate from the recorded trials, not a fresh simulation performed by this documentation update. The experimental Python sources, test worlds, regression tests and recorded diagnostics are now committed on `camino`.

### Flat-world AEB at three speeds

| Commanded speed | AEB trigger progress | Travel after trigger | Final estimated body gap | Outcome |
| --- | ---: | ---: | ---: | --- |
| 0.35 m/s | 4.6121 m | 0.0312 m | 1.6067 m | PASS |
| 0.70 m/s | 3.7124 m | 0.1243 m | 2.4133 m | PASS |
| 1.00 m/s | 2.5835 m | 0.2555 m | 3.4110 m | PASS |

All three trials ended at zero measured speed, with `predicted_stopping_distance` as the AEB reason and no reported obstacle-contact event. Body clearance is an *estimate*, not measured mesh separation. DiffDrive's assumed deceleration is not a calibrated JARCRAC hydraulic brake response. Versioned evidence: `v2_front_3d_aeb_first_trial.{json,csv}`, `v2_front_3d_aeb_0p7mps_trial.{json,csv}`, and `v2_front_3d_aeb_1p0mps_trial.{json,csv}` under `edge-path-project/gazebo/results/jarcrac/`.

A separate **deliberate-contact positive control** reported PASS at 0.15 m/s: 474 contact updates explicitly identified `jarcrac::chassis::body_collision` and `jarcrac_v2_aeb_obstacle::body::collision`; final measured speed was zero. Repeated updates are not 474 independent collisions. Evidence: `v2_contact_positive_trial.json`. This verifies the obstacle contact topic can observe this collision pair, not that all possible collisions are covered.

### Ruskotunturi perception and localization diagnostics (stationary, read-only)

| Experiment | Operator-observed result |
| --- | --- |
| LiDAR-only, no added obstacle | 13/13 sampled ground fits valid; no detected objects; 231–286 uncertain returns per sample |
| LiDAR-only, known tall box | 13/13 sampled fits valid; approximately 1,469–1,488 obstacle returns, nearest ≈3.84–3.85 m |
| Map-assisted, clear scene | 12/12 valid samples; 0 obstacle returns and 0 unknown returns |
| Map-assisted, known tall box | 12/12 valid and obstacle-detecting samples; ≈1,521–1,523 obstacle returns, nearest ≈3.84 m; ≈203–206 uncertain returns |
| Map pose errors (±0.10/0.25 m horizontal, ±2/5° yaw) | 8 clear + 7 obstacle scans × 13 pose cases = 195 valid evaluations; no false obstacle-presence results or missed obstacle-presence results in tested conditions |
| Map height/pitch/roll sensitivity | Height ±0.25 m and pitch errors frequently invalidated classification. Roll ±5° created false obstacle flags in all 10 clear-scene samples for both signs; invalid samples are **not** counted as safe/clear |
| IMU + map-LiDAR on clear / obstacle scenes | 12/12 and 14/14 samples agreed with simulator reference classification; no invalid samples |
| IMU mounting-offset / body-tilt controls | 13/13 mounting-offset samples consistent; 40/40 body-tilt orientation samples consistent (simulated ideal sensors, reported 0.000° maximum error) |

Map-assisted LiDAR uses the existing `ruskotunturi-map-collision.stl` as a height-map prior and **Gazebo's exact XYZ localization**. The IMU stage substitutes IMU-derived orientation while retaining exact simulated position. The body-tilt orientation control uses zero gravity, not slope driving. The pose-perturbation tests process largely stationary scenes; they do not establish real-world localization tolerance. All new classification and IMU experiments are separate from the original flat-ground AEB controller.

### Experimental files and reproduction

Reusable source files committed for this stage:

- `gazebo/scripts/jarcrac_lidar_terrain.py`, `jarcrac_terrain_probe.py` — LiDAR-only sloping-ground classifier and read-only probe.
- `gazebo/scripts/jarcrac_lidar_map_assisted.py`, `jarcrac_map_assisted_probe.py` — mesh-assisted ground classifier and read-only comparison.
- `gazebo/scripts/jarcrac_map_pose_sensitivity.py`, `jarcrac_map_attitude_sensitivity.py` — position, yaw, height, pitch and roll perturbation studies.
- `gazebo/scripts/jarcrac_imu_stage.py` — experimental IMU-equipped worlds (generated outside Git) and IMU/LiDAR diagnostic.
- `gazebo/scripts/jarcrac_contact_positive.py` — independent positive-contact control; its generated world is kept outside Git.
- `gazebo/worlds/jarcrac_terrain_probe_world.sdf` and `jarcrac_terrain_obstacle_probe_world.sdf` — terrain reference and known-obstacle worlds.
- `tests/test_jarcrac_lidar_terrain.py`, `test_jarcrac_lidar_map_assisted.py`, `test_jarcrac_map_pose_sensitivity.py`, `test_jarcrac_map_attitude_sensitivity.py`, `test_jarcrac_imu_stage.py` — offline regressions.

Paths above are relative to `edge-path-project/` for Gazebo files and to the repository root for `tests/`. Operator-generated reports include `terrain_*.json`, `map_*.json`, `imu_*.json` in `edge-path-project/gazebo/results/jarcrac/`; these are **diagnostic**, not AEB PASS evidence.

From the project root:

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac*.py' -v
python3 edge-path-project/gazebo/scripts/jarcrac_imu_stage.py prepare
```

The second command generates IMU experiment worlds under `~/jarcrac_imu_experiment/`. Do not run multiple JARCRAC worlds or velocity command publishers at once.

### Dynamic terrain LiDAR + IMU SIL (2026-11-09)

A separate experimental world, generated by
`edge-path-project/gazebo/scripts/jarcrac_imu_dynamic.py prepare`, adds an
IMU and uses an isolated velocity-command topic, bounded travel, stale-input
checks and a chassis-contact monitor. The original JARCRAC AEB world and
controller are unchanged.

Operator-provided Gazebo results for the short clear-terrain test:

| Dynamic measurement | Recorded value |
| --- | ---: |
| Command speed | 0.10 m/s |
| Command distance limit | 0.35 m |
| Actual displacement | 0.3605 m |
| Final speed | 0.0 m/s |
| Fresh paired scan samples / moving comparisons | 15 / 8 |
| Maximum IMU-to-Gazebo orientation error | 0.0059° |
| Moving pitch / roll span | 1.2361° / 0.3960° |
| Maximum lateral deviation | 0.0004 m |
| Contact messages / observed contact | 0 / false |
| Callback / processing errors | None |
| Test result | **PASS**, distance limit reached |

The 0.0105 m target-distance overshoot remained below the experimental hard
distance limit of 0.65 m. The clear-scene samples produced no unexpected
obstacle returns, and the vehicle stopped as recorded by Gazebo odometry.
Evidence is in
[`imu_dynamic_clear_diagnostic.json`](edge-path-project/gazebo/results/jarcrac/imu_dynamic_clear_diagnostic.json);
offline tests in `tests/test_jarcrac_imu_dynamic.py`.

Reproduce the test only in a clear, visually inspected Gazebo scene, with
no competing velocity publishers:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/jarcrac_imu_dynamic.py prepare
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# In terminal 1:
gz sim -r ~/jarcrac_imu_experiment/jarcrac_imu_dynamic_clear.sdf
# In terminal 2, from the project root:
python3 -u edge-path-project/gazebo/scripts/jarcrac_imu_dynamic.py drive \
  --report edge-path-project/gazebo/results/jarcrac/imu_dynamic_clear_diagnostic.json
```

This result verifies **one short low-speed simulated movement on clear terrain**.
Position still comes from exact Gazebo ground truth, pairing uses wall-time
reception proximity rather than aligned simulation timestamps, and IMU noise
and bias are not representative of physical hardware. This is **not**
obstacle-detection-under-motion acceptance, terrain AEB, a real brake
calibration or JARCRAC-specific physical HIL.

**Next milestone:** a separate controlled, low-speed dynamic obstacle
perception test, without enabling terrain AEB; then simulation-timestamp
alignment and uncertainty injection.

## Validation boundaries

The result demonstrates automatic braking in **flat-world SIL**, not R2100 HIL, VIL, or real-world vehicle safety. The fixed-height ground filter is not usable without modification on uneven forestry ground. Estimated clearance is not true mesh-to-mesh separation. The original flat-world trials subscribed to the contact topic but did not intentionally induce collision; a later separate deliberate-contact control verified this exact collision pair and topic. DiffDrive's configured deceleration approximately explains the short stopping distance; the true 2-ton-plus vehicle's hydraulic braking remains unvalidated. The model includes Mevea-derived mesh data; see `edge-path-project/gazebo/models/jarcrac/NOTICE.md` for attribution and third-party redistribution scope.

## Reference docs

- `edge-path-project/gazebo/models/jarcrac/README.md` — SDF generator parameters and design decisions
- `edge-path-project/gazebo/models/jarcrac/NOTICE.md` — source and redistribution warning
- `edge-path-project/hil/README.md` — the separately validated native R2100 physical-CAN HIL workflow
