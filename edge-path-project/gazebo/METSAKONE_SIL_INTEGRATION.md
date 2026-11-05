# Experimental Metsakone SIL AEB integration

This feature-branch experiment packages the imported articulated forestry
vehicle as a Gazebo Harmonic model and connects its virtual LiDAR, odometry,
and velocity control to a new **single-vehicle SIL emergency-braking trial**.
It is not yet a tested replacement for the passing CAMINO SIL v6.4 or R2100
HIL tests. Original imported files are preserved in
`gazebo/imported/reinforcement-learning/`.

## Run from the repository root (WSL Ubuntu with Gazebo Harmonic)

Check the offline configuration and tests first:

```bash
python3 -m unittest tests.test_metsakone_sil -v
python3 edge-path-project/gazebo/scripts/metsakone_emergency_braking_sil.py --dry-run
```

In terminal 1:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/metsakone_aeb_world.sdf
```

In terminal 2, after Gazebo has started:

```bash
cd ~/edge-path-project-git
python3 -u edge-path-project/gazebo/scripts/metsakone_emergency_braking_sil.py \
  --report edge-path-project/gazebo/results/metsakone/first_trial.json
```

Use a fresh Gazebo world per trial; do not run the older SIL/HIL controller
concurrently. The script publishes only to `/model/metsakone/cmd_vel`.
A successful run prints `METSAKONE AEB SIL RESULT: PASS`; otherwise the
exit status is non-zero and a failure reason is printed.

## Adapted model interfaces

| Function | Gazebo topic | Transport type |
| --- | --- | --- |
| Propulsion / stop | `/model/metsakone/cmd_vel` | `gz.msgs.Twist` |
| Odometry | `/model/metsakone/odometry` | `gz.msgs.Odometry` |
| Forward LiDAR | `/model/metsakone/lidar` | `gz.msgs.LaserScan` |
| Front contact | `/model/metsakone/contacts/front` | `gz.msgs.Contacts` |
| Articulated steering | `/model/metsakone/steering_cmd` | `gz.msgs.Double` |

One `DiffDrive` plugin commands all four wheels, using a 0.6 m nominal
wheel radius (matching the source wheel geometry) and 1.450058 m separation.
The source's competing front-drive and rear-wheel controllers have been
removed **only from the adapted model**. The articulation joint and its
position controller remain. This first AEB test uses zero yaw and straight
motion; articulated turns and obstacle detours remain future work.

The forward LiDAR was increased from 36 to 361 beams and from 10 m to 30 m
range, retaining 10 Hz and an approximately 180-degree field of view.
The 0.65 m LiDAR-to-front-envelope correction and 2.5 m estimated front
reach are **provisional assumptions**, not measured collision-mesh bounds.
Likewise, the vehicle's `z=-9.16` spawn height, wheel physics, braking
deceleration, and clearances must be checked in Gazebo.

## Pass and failure criteria

The script requires fresh LiDAR, odometry and model pose; correct start pose;
nonzero vehicle motion before braking; repeated zero-speed odometry samples;
a stop hold; no observed obstacle contact; and at least 1.0 m **estimated**
clearance from both corrected LiDAR and nominal route geometry. It exits
nonzero for stale sensors, failure to move, insufficient estimated clearance,
collision, route deviation, or timeout. The policy includes the project's
1.7 s TTC and 0.8 m emergency threshold plus a conservative model-specific
predicted-stopping-distance guard.

**PASS is provisional** until mesh envelope and dynamics are calibrated and
a live Gazebo run validates the result. This trial does not use the native
R2100, CAN messages, or `run_staged_scenario.py`. Those existing interfaces
remain unchanged. Re-run separate SIL/HIL acceptance after connecting the new
model, and do not carry over prior vehicles' validated evidence as proof.
