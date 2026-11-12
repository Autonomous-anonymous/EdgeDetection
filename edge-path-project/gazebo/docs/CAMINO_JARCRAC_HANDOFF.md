# CAMINO — JARCRAC Development Handoff

Updated 2026-11-12.

## Repository and environment

- Repository: Autonomous-anonymous/EdgeDetection
- Development branch: camino
- Local path: ~/edge-path-project-git
- Platform: WSL Ubuntu
- Gazebo: Harmonic, Gazebo Sim 8.15.0
- Python transport: gz.transport13
- Main vehicle: JARCRAC tracked forestry vehicle
- Goal: obstacle detection, TTC braking, 5-second hold,
  safe detour around obstacle, then route rejoining
- Scope: experimental Software-in-the-Loop (SIL)

## Important rules

- Do not modify the original JARCRAC model.
- Do not push changes without explicit user approval.
- Test changes locally before committing.
- Prefer complete WSL shell blocks and Python heredocs.
- Use one Gazebo simulation and one velocity publisher at a time.
- Do not authorize obstacle detours from geometry screening alone.

## Detailed tracked JARCRAC

The [detailed model and track meshes](../models/jarcrac_detailed_tracks/) are
committed. The original [wheel-based JARCRAC v2](../../../JARCRAC_V2_SIL.md)
remains a separate model.

Total simulated mass: 2312.236 kg.
Uses native TrackedVehicle and two TrackController plugins.

The model publishes 180-degree, 721 × 32 LiDAR scans on `/jarcrac/lidar`
and odometry on `/model/jarcrac/detailed_odometry`. The generated worlds add
the IMU on `/jarcrac/imu` and a chassis contact sensor.

The [mesh provenance record](../models/jarcrac_detailed_tracks/PROVENANCE.txt)
identifies the source ZIP and its stated Mevea/NUVE-LAB GPL-3.0-or-later
provenance. Independent verification of redistribution rights remains open.

## Validated simulation experiments

Detailed tracks:
- Flat +9-degree simulated turn: PASS.
- Terrain +9-degree simulated turn: PASS.
- Terrain -9-degree simulated turn: PASS.
- Six staged clockwise turns: PASS.
- Actual cumulative clockwise rotation: -56.137 degrees.
- Physical steering rates vary with terrain/orientation.
- Track odometry reports about 0.24 rad/s for a 0.12 rad/s
  command and does NOT reliably represent physical yaw.
- Use actual Gazebo pose and IMU for heading verification.

Detailed-track forward/stop:
- Flat +0.10 m/s: PASS.
  Final progress 0.1240 m.
  Travel after zero command 0.00265 m.
- Ruskotunturi +0.10 m/s: PASS.
  Final progress 0.12662 m.
  Travel after zero command 0.00344 m.

3D LiDAR:
- 721 horizontal x 32 vertical = 23072 ranges: PASS.
- Obstacle detection with map-assisted LiDAR: PASS for
  the known static obstacle scene.

TTC braking and hold:
- Detailed-track controller: PASS.
- Early experimental TTC threshold: 32.7 simulated seconds.
- Trigger at 0.3133 m forward progress.
- Final progress: 0.3161 m.
- Additional travel after trigger: about 2.84 mm.
- Hold duration: 5.058 simulated seconds.
- The TTC setting is for this instrumentation trial. Calibrated braking remains open.

Detour preview:
- Read-only static geometry screen: both sides PASS.
- Right corridor preferred provisionally.
- Right obstacle-clearance proxy: 1.806 m.
- Left obstacle-clearance proxy: 1.732 m.
- Right maximum local terrain grade: 0.089.
- Left maximum local terrain grade: 0.127.
- Neither route is authorized for driving.
- Side coverage, swept path, dynamic feasibility and
  route rejoining are NOT validated.

## Near returns with the original LiDAR sampling grid

Detailed model with visuals:
- Exactly 32 returns below 1 m at azimuth +45 degrees.
- All 32 vertical layers are affected.
- Adjacent azimuth columns see ranges above about 2.1 m.
- Seen on flat ground and Ruskotunturi.

A/B experiments:
- Isolated LiDAR, vehicle removed: 0 close returns.
- Chassis visual removed: 30 close returns.
- Track visuals removed: 28 close returns.
- All vehicle visuals removed: 0 close returns.
- LiDAR physically rotated +3 degrees:
  32 close returns still at sensor-relative +45 degrees.

Current hypothesis:
- Vehicle-visual-dependent GPU LiDAR rendering artifact,
  potentially involving an OGRE2 cubemap face boundary.
- Not yet confirmed as a Gazebo rendering defect.
- Do NOT filter or ignore the +45-degree column yet.

The isolated half-step test below reduced the near returns in the tested
scenes. The original model still uses the original sampling grid.

## Known limitations

- Stationary fixtures verified contact detection. The world-qualified contact
  subscription during driven TTC runs still needs verification.
- Map-assisted classification uses Gazebo ground-truth XYZ.
- Sensor pairing is based on receive time, not exact
  simulation-timestamp synchronization.
- No verified full-corridor occupancy or tracked-object TTC.
- No autonomous detour or route rejoining demonstrated.
- All driving limits are experimental SIL limits.

## Recreate the experimental worlds

From the repository root, generate the flat and terrain worlds from the
committed model and meshes:

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_make_detailed_tracks.py --case both
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

The script writes `jarcrac_detailed_flat.sdf` and `jarcrac_detailed_terrain.sdf`
under `~/jarcrac_imu_experiment/`. The source ZIP is needed only to reimport
the track assets.

To generate the isolated half-step variant after creating the terrain world:

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_make_terrain_halfstep.py
```

## Where to look

Scripts:
  edge-path-project/gazebo/scripts/jarcrac_detailed_*.py

Reports are indexed in the [JARCRAC SIL results](../results/jarcrac/README.md).

Original TTC logic:
  edge-path-project/gazebo/scripts/jarcrac_adas_ttc_stop_hold.py

Original map classification:
  edge-path-project/gazebo/scripts/jarcrac_lidar_map_assisted.py

## 2026-11-11 half-step LiDAR validation

Experimental change:
- Horizontal LiDAR angular sampling shifted by +0.125 degrees.
- Physical sensor pose unchanged.
- 721 horizontal samples, 32 vertical layers, 180-degree span.
- Original detailed model remains unchanged.

Results:
- Flat world: 0 short-range (<1 m) returns in 3/3 scans.
- Ruskotunturi: 0 short-range (<1 m) returns in 3/3 scans.
- Known obstacle detected in 3/3 map-assisted scans.
- Obstacle bearing: +0.7 degrees.
- Nearest elevated obstacle ranges: 3.909, 3.881, 3.889 m.
- Elevated returns within obstacle bearing +/-8 degrees:
  1184, 1184, 1185.
- GPU cubemap seam remains a hypothesis, not proven root cause.

Detailed-track half-step TTC stop/hold:
- Result: PASS.
- Initial obstacle gap: 3.8925 m.
- TTC brake reason: ttc_threshold.
- Trigger TTC: 32.401 s (instrumentation-only threshold).
- Progress at brake trigger: 0.314595 m.
- Final progress: 0.317339 m.
- Post-trigger travel: 0.002744 m.
- Final speed: 0 m/s.
- Stationary hold: 5.035 simulated seconds.
- Fresh hold scans: 14.
- Processing/callback errors: 0.
- Dropped analysis samples: 0.
- Contact messages: 0 in this run.

See the [committed half-step TTC result](../results/jarcrac/jarcrac_detailed_ttc_stop_hold_halfstep.json).

## 2026-11-11 simulated collision-contact checks

Both STL track collisions positively verified in Gazebo DART.

Stationary flat-world test:
- Left track-ground contacts: 46
- Right track-ground contacts: 46
- Collision pair names matched both STL track collision shapes
- Vehicle Z displacement: -0.003142 m
- Test result: PASS
- No drive plugins or velocity commands in this fixture
- Detailed track meshes and original model unchanged

Earlier chassis contact-positive test:
- Falling sphere contacted chassis body_collision
- Correct chassis-sphere collision pair observed
- Test result: PASS

DART's mesh initialization warnings did not prevent actual
track-ground contact generation in this test.

- [Track-ground contact](../results/jarcrac/jarcrac_detailed_track_contact.json)
- [Chassis contact positive control](../results/jarcrac/jarcrac_detailed_contact_positive.json)

## 2026-11-12 watchdog and bounded-motion scripts

The [watchdog monitor](../scripts/jarcrac_detailed_watchdog_monitor.py) observes
sensor freshness and contact status. The
[bounded-motion experiment](../scripts/jarcrac_arbiter_bounded_motion.py)
uses a synthetic command request, an in-process watchdog and Gazebo
ground-truth pose in a separate test world. Its live-run report and the
monitor's report are absent from the branch. Independent process-level
stop protection, swept-path clearance, autonomous detour and route rejoining
remain open.
