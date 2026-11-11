# CAMINO — JARCRAC Development Handoff

Updated: 2026-11-11

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
- Scope: simulation-in-the-loop only, not safety certified

## Important rules

- Do not modify the original JARCRAC model.
- Do not push changes without explicit user approval.
- Test changes locally before committing.
- Prefer complete WSL shell blocks and Python heredocs.
- Use one Gazebo simulation and one velocity publisher at a time.
- Do not authorize obstacle detours from geometry screening alone.

## Detailed tracked JARCRAC

Imported from the user's jarcrac_gazebo(2).zip using:

  edge-path-project/gazebo/scripts/jarcrac_make_detailed_tracks.py

Experimental model:

  edge-path-project/gazebo/models/jarcrac_detailed_tracks/

The original models/jarcrac/model.sdf remains unchanged.

Total simulated mass: 2312.236 kg.
Uses native TrackedVehicle and two TrackController plugins.

LiDAR restored to original 180-degree, 721 x 32 configuration.
Topic: /jarcrac/lidar
IMU: /jarcrac/imu
Detailed odometry: /model/jarcrac/detailed_odometry

Important: imported model and meshes may not yet be on GitHub.
The source ZIP is stored on the user's PC.
The ZIP README identifies Mevea/NUVE-LAB GPL-3.0-or-later
provenance; review redistribution compliance before publishing.

## Validated simulation experiments

Detailed tracks:
- Flat +9-degree physical turn: PASS.
- Terrain +9-degree physical turn: PASS.
- Terrain -9-degree physical turn: PASS.
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
- NOT calibrated automatic emergency braking.

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

## Current unresolved LiDAR anomaly

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

## Immediate next experiment

Create a separate detailed flat-world A/B variant that shifts
the LiDAR horizontal angular sampling grid by +0.125 degrees,
without moving the sensor or changing its 721 x 32 resolution.

Run three stationary LiDAR scans and check whether the
single-column +45-degree near-return anomaly disappears.

If successful:
1. Repeat on Ruskotunturi.
2. Verify detection of real obstacles in affected sectors.
3. Revalidate TTC stopping and 5-second hold.
4. Investigate contact-sensor positive detection.
5. Evaluate swept trajectory and side corridor clearance.
6. Only then consider bounded detour driving.

## Known limitations

- No independent safety watchdog.
- Chassis contact sensor has reported zero contact messages;
  positive detection capability remains unvalidated.
- Map-assisted classification uses Gazebo ground-truth XYZ.
- Sensor pairing is based on receive time, not exact
  simulation-timestamp synchronization.
- No verified full-corridor occupancy or tracked-object TTC.
- No autonomous detour or route rejoining demonstrated.
- All driving limits are experimental SIL limits.

## Recreating test worlds on this PC

From repository root:

  python3 edge-path-project/gazebo/scripts/jarcrac_make_detailed_tracks.py --case both

Requires the locally imported detailed model assets.

Generated worlds are in:

  ~/jarcrac_imu_experiment/

Set the Gazebo resource path:

  export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models"

## Where to look

Scripts:
  edge-path-project/gazebo/scripts/jarcrac_detailed_*.py

Reports:
  edge-path-project/gazebo/results/jarcrac/

Original TTC logic:
  edge-path-project/gazebo/scripts/jarcrac_adas_ttc_stop_hold.py

Original map classification:
  edge-path-project/gazebo/scripts/jarcrac_lidar_map_assisted.py

Continue with the +0.125-degree LiDAR sampling-grid experiment.
Do not jump straight to autonomous detour driving.

## Update — 2026-11-11: Half-step LiDAR validation PASSED

This section supersedes the earlier pending half-step experiment.

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
- Contact messages: 0 (positive contact sensing unverified).

Report:
  edge-path-project/gazebo/results/jarcrac/
  jarcrac_detailed_ttc_stop_hold_halfstep.json

Next priorities:
1. Positively validate chassis contact sensing in an isolated test.
2. Review half-step sensor behavior under additional orientations.
3. Validate swept-path clearance and side perception.
4. Implement an independent command/sensor watchdog.
5. Only then consider a bounded right-hand detour trial.

No autonomous detour or route rejoining has been validated.
No half-step modification has been integrated into the original model.

## Update — 2026-11-11: Physical collision contacts verified

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

Remaining:
- Verify world-qualified contact subscription in TTC adapters
- Implement and validate independent watchdog
- Verify swept-path and side-corridor clearance
- Validate bounded detour and route rejoining

Reports:
- gazebo/results/jarcrac/jarcrac_detailed_track_contact.json
- gazebo/results/jarcrac/jarcrac_detailed_contact_positive.json
