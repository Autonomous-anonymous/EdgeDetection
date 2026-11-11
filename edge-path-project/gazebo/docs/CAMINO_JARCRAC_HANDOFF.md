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
