# wheelloader - Gazebo model generated from the Mevea WheelLoader model

An articulated wheel loader (about 12.6 t) for Gazebo SIL testing. **Generated**:
edit `scripts/generate_wheelloader_model.py`, not `model.sdf`.

## Regenerate

```bash
cd edge-path-project/gazebo
python3 scripts/generate_wheelloader_model.py \
  --mevea-dir /path/to/ruskotunturi/WheelLoader \
  --out-dir models/wheelloader
```

Needs only the Python standard library. It reads `Model/WheelLoader.xml`, converts
the Mevea meshes Gazebo cannot load (`.osgt` visuals -> OBJ, `.3ds` collision -> STL)
and writes the model, meshes and `wheelloader_params.json` (every extracted value,
derived value and assumption). Options: `--wheel-mass-kg`, `--mu`, `--mu2`,
`--max-speed-mps`, the controller gains (`--lift-p` ...), `--wheel-effort-nm`.

## Run

```bash
cd edge-path-project/gazebo
export GZ_SIM_RESOURCE_PATH=$PWD/models
gz sim -r worlds/wheelloader_flat_test_world.sdf

# in a second terminal: the cmd_vel bridge (articulated steering + wheel speeds)
python3 scripts/wheelloader_cmd_bridge.py --params models/wheelloader/wheelloader_params.json

# drive: v = speed of the rear axle centre (m/s), w = yaw rate (rad/s, + = left)
gz topic -t /model/wheelloader/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 1.0}, angular: {z: 0.1}'

# work equipment: angles in rad relative to the Mevea start pose
gz topic -t /wheelloader/lift/cmd_pos        -m gz.msgs.Double -p 'data: 0.5'   # + = raise
gz topic -t /wheelloader/bucket_tilt/cmd_pos -m gz.msgs.Double -p 'data: -0.8'  # + = rack back
```

Topics: `/model/wheelloader/cmd_vel` (to the bridge), `/model/wheelloader/odometry`,
`/wheelloader/{articulation,lift,bucket_tilt}/cmd_pos`,
`/wheelloader/wheel_{rl,rr,fl,fr}/cmd_vel`, `/wheelloader/camera_seat/image`,
`/wheelloader/camera_rear/image`.

## What it is

* 10 links: `rear_frame` (cab, engine; canonical link), `rear_axle` (oscillating),
  `front_frame`, `lift_arm`, `bucket_clamp`, `bucket`, four wheels.
* Joints and ranges, all taken from Mevea:
  * `articulation_joint` +-0.33 rad (19 deg), from the steering cylinder stroke
  * `lift_joint` -0.43 .. +1.10 rad, from the lift cylinder stroke
  * `bucket_tilt_joint` -2.20 .. +0.65 rad, Mevea's own clamp damper limits
  * `axle_oscillation_joint` +-12 deg with Mevea's spring (K = 1e4) and damper (C = 8e4)
* Mass and inertia per body from Mevea, 12 594 kg; the bucket lever (312 kg) and link
  (80 kg) are lumped into the lift arm and the bucket clamp. Gazebo total 12 714 kg
  (4 x 30 kg assumed wheels on top).
* Collision shapes are Mevea's own simplified collision meshes; wheels are cylinders
  (radius 0.729 m, Mevea contact width 0.38 m, friction 0.8 / 0.6 from Mevea).
* Top speed 2.84 m/s, derived from the Mevea gearbox, differential and planet ratios
  and the engine's maximum speed (torque-converter slip ignored).

## How the geometry was verified

All 8 true joints of the Mevea model coincide in the world frame with exactly zero
error when the body frames are chained with the convention in the generator, which
confirms the frame chain, the Euler order (R = Rx Ry Rz) and every joint axis. The
steering cylinders reproduce Mevea's L0 exactly; Mevea's lift-cylinder L0 equals the
length with the arm lowered and the bucket on the ground (the model starts with the
bucket raised about 1.1 m). See `geometry_check.png`.

## What it is NOT

* No Z-bar tilt linkage: Gazebo's physics engines do not support closed kinematic
  loops, so the bucket pivot is driven directly. The tilt cylinder's coupling to the
  lift arm is lost, so "bucket angle" here is relative to the lift arm, not to the ground.
* No hydraulics, diesel engine, torque converter, gearbox or differentials: joints are
  position or velocity controlled.
* Rigid wheels: Mevea's tyres are compliant (K = 2.6e6 N/m, 12 mm static deflection).
  The Mevea start pose has the front tyres 4 mm in the ground and the rear 9 mm above
  it, so expect the model to settle by about 13 mm at spawn.
* No textures (colours only); hydraulic cylinder meshes omitted.
* No lidar. Mevea has none, and on a loader the bucket blocks a bumper mount while a roof
  mount sits above low obstacles. Choose the placement before AEB-style testing.

## Assumptions to calibrate

Controller gains (initial guesses), wheel effort limit (30 kN m, above the traction
limit so tyres slip before the actuator saturates), the 30 kg wheel mass, and
inertia conventions (Mevea's `CmiCS` on the lift arm and bucket clamp is undocumented;
effect at most ~36 kg m^2). All recorded in `wheelloader_params.json`.

## Status

Verified offline (`tests/test_wheelloader_model.py` and
`tests/test_wheelloader_kinematics_and_meshes.py`): well-formed files, kinematic tree,
joint axes and limits, masses and inertia, hand-calculated pose chain, tyre geometry,
cameras, mesh URIs, converters, steering kinematics, bridge mapping, regeneration.

**Gazebo Harmonic flat-world motion smoke test: PASS (2026-11-09).**
With `scripts/wheelloader_cmd_bridge.py` running, the operator's
`scripts/wheelloader_runtime_smoke.py` issued 0.20 m/s for 3 seconds
and reported **0.5375 m odometry displacement**, 83 odometry messages,
and **0.0 m/s final forward speed**. See
[`results/wheelloader_flat_smoke.json`](../../results/wheelloader_flat_smoke.json).

Reproduce from `edge-path-project/gazebo/`, using a fresh world:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# Terminal 1
gz sim -r worlds/wheelloader_flat_test_world.sdf
# Terminal 2: integrated controller and movement test
python3 -u scripts/wheelloader_runtime_smoke.py
```

This records *Gazebo odometry*, not independent ground-truth displacement.
Still unverified: stability across repeated runs, PID lift/bucket behaviour,
articulated turn radius, axle oscillation, realistic tyre response, terrain
driving, and calibrated hydraulic braking. The model has no LiDAR and is
not part of the JARCRAC AEB/HIL validation pathway.
