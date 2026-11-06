# jarcrac - Gazebo model generated from the Mevea JARCRAC model

First-pass digital-twin approximation of the JARCRAC tracked vehicle for Gazebo
SIL testing. **Generated** - edit `scripts/generate_jarcrac_model.py`, not
`model.sdf`.

## Regenerate

```bash
cd edge-path-project/gazebo
python3 scripts/generate_jarcrac_model.py \
  --mevea-xml /path/to/ruskotunturi/Jarcrac-freeroll/Jarcrac_Rusko_rev_2026_01_05_11-free_roll_test2.xml \
  --out-dir models/jarcrac
# a second instance with its own topics:
python3 scripts/generate_jarcrac_model.py --mevea-xml ... --out-dir models/jarcrac_b \
  --name jarcrac_b --topic-prefix /jarcrac_b
```

Defaults follow the Mevea model (see "Decisions" below). Overrides:
`--keep-source-radii` (keep the 0.2 m Pulley3_Left), `--body-mass-kg N`
(e.g. 2460 for the rev7 value), `--no-track-mass`, `--body-mesh jarcrac`.

Tunable assumptions (all recorded in `jarcrac_params.json`): `--max-speed-mps`,
`--max-accel-mps2`, `--max-yaw-rate-radps`, `--turn-factor`, `--mu`, `--mu2`,
`--side-torque-nm`, `--lidar-x-m`, `--lidar-y-m`, `--lidar-z-m`.

## Run

```bash
cd edge-path-project/gazebo
export GZ_SIM_RESOURCE_PATH=$PWD/models
gz sim -r worlds/jarcrac_flat_test_world.sdf
gz topic -t /model/jarcrac/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}, angular: {z: 0.0}'
```

Topics: `/model/jarcrac/cmd_vel`, `/model/jarcrac/odometry`, `/jarcrac/lidar`,
`/jarcrac/camera/image`.

## What it is

* **Chassis, 2192 kg:** the Mevea body (2000 kg, Mevea inertia and centre of
  mass) plus the Mevea track (rope) mass and the two small return rollers,
  lumped in with the inertia of the belt path (see `geometry_check.png`).
* **12 road wheels, 10 kg each:** the Mevea pulleys, all radius 0.17 m, track
  centrelines 1.27 m apart; four per side touch the ground. A `DiffDrive`
  plugin gives skid steer; each side's 1150 N m is shared over its 6 wheels.
* **Total 2312 kg** = the Mevea Jan-2026 free-roll model's total, with the track
  part estimated (below).
* Open tube-frame body mesh `Runko_part.STL` (visual); collision is its
  bounding box.

## What it is NOT

Not modelled: the 58-particle rope tracks themselves, the diesel / hydraulic /
planetary drivetrain, brakes. Speed, acceleration, friction and turning are
assumptions until calibrated against the Mevea free-roll and brake runs.
Stopping is governed by DiffDrive's deceleration limit (-2 m/s^2, an
assumption), not by Mevea's brake model.

## Decisions

1. **Pulley3_Left normalised from 0.2 m to 0.17 m.** It is 0.2 m in every Mevea
   revision, but rev1 had 0.2 m on Pulley2-6_Left and the small pulley too;
   later revisions corrected all of them except this one. Its mirror pulley and
   its 3D visual use 0.17 m. It reads as a leftover default, and keeping it makes
   a rigid vehicle rest rolled by 1.35 deg. `--keep-source-radii` restores it
   (the wheel is then free-spinning and the generator reports the predicted
   rocking).
2. **Mass: 2000 kg body + tracks.** The Jan-2026 free-roll line is where the
   track masses were rebalanced "closer to real" (docx, Rev13); rev7 (2460 kg)
   appears to predate that work. Tracks are lumped in, because Gazebo does not
   simulate the rope particles Mevea carries.
   **The track mass is an estimate:** rope density x cross-section x belt path
   length x `scale_L` = about 86 kg per track (96 kg if `scale_L` is ignored).
   Mevea writes the exact masses to `LOG_DynamicLineSystem.txt`, which is not in
   the repo; replace the estimate if you can supply it.
3. **Camera**: the free-roll models have no forward camera, so the
   `Camera_cabin` pose from `Jarcrac_rev7.xml` is used (marked `is_default`).
4. **Body = `Runko_part.STL`**, placed by aligning it to the Mevea-placed
   `Jarcrac_body.STL` (no rotation, 1.5 mm median error), so derived, not stated.

## Status

Verified offline (`tests/test_jarcrac_model.py`, 28 tests): well-formed SDF,
joint and plugin consistency, symmetry, level ground contact, mass and inertia
(composite, parallel-axis), belt geometry, mesh placement, regeneration
reproducibility including the `--keep-source-radii` mode.
**Subsequent Gazebo SIL evidence (2026-11-06):** The revised front-mounted
32-layer LiDAR detected the flat-world obstacle and the experimental AEB
reported PASS at 0.35, 0.70 and 1.00 m/s. A separate deliberate collision
verified the contact sensor. Map-assisted perception and simulated IMU controls
were also exercised on Ruskotunturi; these are stationary diagnostics, not
terrain AEB or real-vehicle calibration. See the repository-root
`JARCRAC_V2_SIL.md` for measured results, scripts and limits.

Still to verify against real dynamics: wheel/track response, turning behaviour,
hydraulic braking, uneven-terrain driving, and JARCRAC-specific R2100 HIL.

Current LiDAR: front-centre at chassis (1.68, 0, -0.111) m, nominally 0.60 m
above flat ground, 0.23 m ahead of the front collision-box face; 180-degree
horizontal, 32 layers from -15 to +15 degrees. This is a generic sensor.
The existing single-plane AEB controller must not be used unchanged.

## Licence

Geometry, mass data and the mesh derive from the NUVE-LAB / Mevea model, which
is GPL-3.0-or-later (Copyright Mevea Ltd 2018-2021, NUVE-LAB 2026). Check the
implications before publishing this repository.

## Front-centre 3D LiDAR mount (2026-11-06)

The previous roof-mount revision yielded repeated short possible self-returns
near 0.63 m. The revised **front-centre** sensor at chassis pose
`1.68 0 -0.111 0 0 0` now scans 180 degrees (±90°) horizontally and has
32 vertical layers from -15° to +15° at 10 Hz. On the unperturbed model it
is 0.60 m above level ground and 0.23 m ahead of the front collision-box face.

The box collision and `Runko_part.STL` visual remain unchanged. This sensor
position requires a real mounting bracket and is an uncalibrated simulation
choice. The flat-world obstacle is expected near **6.07 m** in the forward
scan (7.75 m obstacle face minus 1.68 m scanner X). See the repository-root
`JARCRAC_V2_SIL.md` for commands, checks, and limitations.

**Do not connect the older single-plane AEB policy to these multi-layer rays.**
A separate front-3D-LiDAR AEB controller has completed the documented flat-world SIL trials.
