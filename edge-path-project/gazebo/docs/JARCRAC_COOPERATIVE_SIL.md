# JARCRAC cooperative SIL on Ruskotunturi

This Gazebo Harmonic experiment uses the committed Ruskotunturi terrain, the
tracked JARCRAC model as Vehicle A and the Ackermann model as Vehicle B.
The routes approach a merge on a low-relief part of the map, then share a
lane. The experiment has its own start, merge and goals. The dated v6.4 run
remains the reference for the full loading-zone scenario.

The [compact v6.4-derived SIL demo](JARCRAC_COMPACT_SIL.md) uses the original
merge and moving crossing with shorter routes and a separate rejoin trial.

## Front LiDAR and obstacle response

The generated world retains JARCRAC's chassis-mounted 180° front LiDAR,
including the 721 horizontal samples, 32 vertical layers, update rate and
noise settings. Its sampling grid uses the 0.125° shift from the earlier
[half-step terrain trial](../results/jarcrac/jarcrac_detailed_ttc_stop_hold_halfstep.json)
to avoid sampling exactly on the reported rendering seam. The field of
view and sensor mount stay the same.

The controller compares 3-D LiDAR returns with the ground-only collision mesh
using a timestamped vehicle pose. Ground returns support clearance along
the scanned bearing. Elevated or inconsistent returns bound that clearance.
Unobserved space has no clearance evidence unless it was seen recently from
another position. The generated test area has a declared clear launch
rectangle to cover the front scan's minimum range and rear blind area at
startup.

JARCRAC brakes when the intended path or stopping envelope lacks verified
clearance. Once stationary, it waits five continuous seconds. It proceeds
on-route if the path clears. With a persistent blockage it can follow
observed portions of a forward detour on its private approach and then
rejoin when it can verify the body sweep. Loss of clearance keeps it stopped.
The planner receives ranges and vehicle state. The box positions are known
to the world generator and can be checked by a separate Gazebo evaluator.

## Merge access

- A has normal priority. B gains permission after A's full footprint clears
  the merge and the space B needs to exit. A's position, heading and motion
  along the shared route determine when this happens.
- B's pending grant can be revoked. If B has entered or its stopping distance
  puts it too close to brake short, it retains the reservation and A yields.
  Both vehicles' commands pass through the coordinator. A reservation lasts
  through the exit of the rear of the vehicle.
- The coordinator compares body polygons extracted from the two models and
  accounts for translational and angular stopping distance. It checks
  separation on the shared route. Missing, incompatible or stale vehicle
  state commands both vehicles to stop and denies new entry. A committed
  reservation remains held during telemetry loss.

## Localisation and operating limits

Select a source explicitly. `gazebo-odometry` uses simulator
OdometryPublisher messages. `external` subscribes to
`/camino/cooperative/a/state` and `/camino/cooperative/b/state`. Publish
`gz.msgs.Odometry` in Gazebo's world axes with header `frame_id=camino_map`,
`child_frame_id=camino_jarcrac` or `camino_vehicle_b`, and increasing Gazebo-time
stamps. Supply world X, Y, absolute Z and orientation. Linear velocity and
yaw rate are derived from consecutive poses, and the scan must pair within
0.10 s. The controller does not use the LiDAR message's simulator world pose.
The separate evaluator subscribes to exact Gazebo model poses.

This is a low-speed SIL fixture for static obstacles. JARCRAC's command speed
is capped at 0.10 m/s. The stopping budgets use assumed decelerations of
0.18 m/s² for JARCRAC and 0.35 m/s² for B, plus yaw deceleration and timing
allowances. Those values and the track response need live testing on this
route. Recent scan history can support an already observed area behind the
scanner in this static scene. It cannot establish clearance from a new
object entering a blind area. An obstruction across the shared lane can
hold the vehicles. Vehicle B's actuator has no independent command-expiry
protection in this experiment.

## Reproduce

From the repository root, run the offline tests:

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac_cooperative_sil.py' -v
```

The suite checks terrain and obstacle returns with synthetic rays against
the committed mesh and exercises the merge cases without Gazebo. The
closed-loop tests use kinematics and projected front-range fixtures. They
test clear-route travel, the five-second wait and detours around boxes
placed at undisclosed locations.

For Gazebo, start all terminals from the repository root with the same
`GZ_PARTITION`. Generate a fresh world in terminal 1:

```bash
export GZ_PARTITION=camino_ruskotunturi_trial
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
python3 edge-path-project/gazebo/scripts/jarcrac_cooperative_sil.py prepare /tmp/camino-ruskotunturi.sdf --obstacle -16.4 0
gz sim -r /tmp/camino-ruskotunturi.sdf
```

The generator's obstacle coordinates are local route metres. The origin is
Gazebo XY `(23, 48)` and local +X points 30° clockwise from world +X. Omit
`--obstacle` for the clear-route trial, or use `--obstacle -15.8 -0.55` as the
second placement. Use a fresh world and report filenames for each run.

In terminal 2, start the evaluator before the controller:

```bash
export GZ_PARTITION=camino_ruskotunturi_trial
python3 edge-path-project/gazebo/scripts/jarcrac_cooperative_evaluate.py --obstacles 1 --report /tmp/camino-ruskotunturi-eval.json
```

Use `--obstacles 0` for the clear-route trial. In terminal 3, start the controller:

```bash
export GZ_PARTITION=camino_ruskotunturi_trial
python3 edge-path-project/gazebo/scripts/jarcrac_cooperative_sil.py run --state-source gazebo-odometry --trace /tmp/camino-ruskotunturi-control.jsonl
```

The evaluator checks sequential A/B entry, both goals and minimum planar
footprint gaps of 1.0 m between vehicles and 0.55 m to fixture boxes. The
controller trace records scan evidence, stationary speed, hold, detour and
rejoin. Use that trace to verify the five-second wait during a live run.
