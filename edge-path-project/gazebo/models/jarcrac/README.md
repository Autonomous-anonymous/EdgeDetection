# jarcrac - Gazebo model generated from the Mevea JARCRAC model

First-pass digital-twin approximation of the JARCRAC tracked vehicle for Gazebo
SIL testing. **Generated** - edit `scripts/generate_jarcrac_model.py`, not
`model.sdf`.

## Run the existing model (no Mevea source files required)

The generated `models/jarcrac/model.sdf`, meshes and Gazebo worlds are already
in this repository. **Do not run the model generator just to launch Gazebo.**
Use the commands in a **WSL bash terminal**, not Windows Command Prompt. They
start from the clone location used by the project (`~/edge-path-project-git`);
change that prefix if you cloned the repository elsewhere. This absolute-from-
home path also works when WSL initially opens in `/mnt/c/Users/...`.

### Flat test world — visual inspection and optional manual motion

Terminal 1 (keep it open while Gazebo runs):

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r worlds/jarcrac_flat_test_world.sdf
```

Terminal 2 (a separate WSL terminal, after Gazebo has loaded):

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo
gz topic -l | grep jarcrac
```

You should see JARCRAC transport topics. The flat test world includes a static
box obstacle about 8 m ahead. For an **optional flat-world-only** brief manual
movement check, use Terminal 2 with no other velocity publisher active:

```bash
gz topic -t /model/jarcrac/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.1}, angular: {z: 0.0}'
sleep 1
gz topic -t /model/jarcrac/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}'
```

These are individual topic publications, **not a safety-rated bounded
controller**; do not assume that one velocity command will time out on its own.
Keep Gazebo attended and send zero velocity if movement persists. For the
controlled flat-world 3D-LiDAR AEB experiment, follow the repository-root
[`JARCRAC_V2_SIL.md`](../../../../JARCRAC_V2_SIL.md) instead.

### Ruskotunturi — stationary terrain perception

Close any other running Gazebo world first. In Terminal 1:

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r worlds/jarcrac_terrain_probe_world.sdf
```

To inspect the **known-obstacle** terrain scene instead, close that world and
launch `gz sim -r worlds/jarcrac_terrain_obstacle_probe_world.sdf` from the
same directory. These worlds are for stationary perception diagnostics; **do
not run the flat-world AEB controller or manually drive on this terrain.**

### Ruskotunturi — previously validated short, clear-terrain motion

This is a distinct low-speed LiDAR/IMU experiment, **not terrain AEB**. Its
world is generated outside Git under `~/jarcrac_imu_experiment/` and is not
the same as the stationary probe worlds. First, from a WSL terminal:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/jarcrac_imu_dynamic.py prepare
```

Terminal 1 (close other JARCRAC worlds first):

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r ~/jarcrac_imu_experiment/jarcrac_imu_dynamic_clear.sdf
```

Terminal 2 (after the correct world has loaded):

```bash
cd ~/edge-path-project-git
python3 -u edge-path-project/gazebo/scripts/jarcrac_imu_dynamic.py drive \
  --report /tmp/jarcrac_imu_dynamic_recheck.json
```

Use the WSL **system `python3` with Gazebo `gz` bindings**, not the project's
`.venv-requirements` packaging environment. Do not run another JARCRAC world
or `cmd_vel` publisher concurrently. The dynamic controller sends zero on
exit and restricts motion, but it does not establish collision avoidance or
safe terrain braking. See [`JARCRAC_V2_SIL.md`](../../../../JARCRAC_V2_SIL.md)
for limitations and recorded evidence.

Topics on the standard model: `/model/jarcrac/cmd_vel`,
`/model/jarcrac/odometry`, `/jarcrac/lidar`, `/jarcrac/camera/image`.
The dynamic world deliberately uses `/model/jarcrac/dynamic_cmd_vel` instead.

## Regenerate the model (optional, external Mevea files required)

This is a **developer operation**, not part of running the existing Gazebo
worlds. The `--mevea-xml` argument must refer to a *real* Mevea XML file on
your filesystem; `/path/to/...` and `...` in older examples were placeholders.
The corresponding mesh files must also be available relative to that XML.
The following example writes to `/tmp`, preserving the committed model:

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo
MEVEA_XML="/replace/with/real/path/to/Jarcrac_Rusko_rev_2026_01_05_11-free_roll_test2.xml"
if [ ! -f "$MEVEA_XML" ]; then
  echo "Mevea XML not found: $MEVEA_XML"
else
  python3 scripts/generate_jarcrac_model.py \
    --mevea-xml "$MEVEA_XML" \
    --out-dir /tmp/jarcrac_regenerated
fi
```

An additional model can be generated using `--name jarcrac_b` and
`--topic-prefix /jarcrac_b`, but **generating a model does not spawn a second
vehicle**; you must separately configure a world with the second model and
its resource path.

Defaults follow the Mevea model (see "Decisions" below). Overrides:
`--keep-source-radii` (keep the 0.2 m Pulley3_Left), `--body-mass-kg N`
(e.g. 2460 for the rev7 value), `--no-track-mass`, `--body-mesh jarcrac`.

Tunable assumptions (all recorded in `jarcrac_params.json`): `--max-speed-mps`,
`--max-accel-mps2`, `--max-yaw-rate-radps`, `--turn-factor`, `--mu`, `--mu2`,
`--side-torque-nm`, `--lidar-x-m`, `--lidar-y-m`, `--lidar-z-m`.

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
terrain AEB or real-vehicle calibration. A separate 2026-11-09 low-speed
**clear-terrain dynamic LiDAR/IMU perception** trial passed (0.3605 m travel,
eight moving paired samples, final speed zero). It did not include an obstacle
while moving and does not validate terrain AEB. See the repository-root
`JARCRAC_V2_SIL.md` for measured results, scripts and limits.

Still to verify against real dynamics: wheel/track response, turning behaviour,
hydraulic braking, uneven-terrain driving, and JARCRAC-specific R2100 HIL.

Current LiDAR: front-centre at chassis (1.68, 0, -0.111) m, nominally 0.60 m
above flat ground, 0.23 m ahead of the front collision-box face; 180-degree
horizontal, 32 layers from -15 to +15 degrees. This is a generic sensor.
The existing single-plane AEB controller must not be used unchanged.

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
