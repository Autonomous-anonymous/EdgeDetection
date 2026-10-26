# Gazebo SIL baseline: v6.4

This guide describes the v6.4 SIL baseline merged into `main` from `sil-development`. Earlier scripts,
worlds and CSV results remain available as development evidence. Mevea is outside
this project's scope. Raspberry Pi/CAN integration is the later HIL stage.

## Running the integrated demo

Environment: Ubuntu 24.04 (including WSL), Gazebo Harmonic, Python bindings
`gz.transport13` and `gz.msgs10`. Use the repository's existing models and terrain.
From the repository root, in terminal 1:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_final_sil_demo_v6_world.sdf
```

Wait for the running world to load, then in terminal 2:

```bash
python3 edge-path-project/gazebo/scripts/ruskotunturi_final_sil_demo_v6.py
```

Expect a v6.4 heading and `Fresh start OK`. Close and reopen Gazebo for every
trial. The controller checks initial positions, headings, sensors and clock;
it deliberately does not reset or unpause the world, following a reset-related
native Gazebo crash encountered during testing. Do not use Reset or run multiple
demos concurrently: vehicle sensor/actuator topics are shared.

## Files and responsibilities

| File under scripts/ | Responsibility |
| --- | --- |
| ruskotunturi_final_sil_demo_v6.py | Navigation, camera/LiDAR avoidance, merge/loading coordination and result evaluation |
| final_sil_v6_harness.py | Separate process driving the moving obstacle; observed-pose verification and bounded retries |
| final_sil_v6_watchdog.py | Separate command arbiter stopping stale or unsafe commands |
| final_sil_v6_checks.py | Sensor validation, footprint geometry and initial-scene checks |

The controller starts and cleans up its harness/watchdog processes. Normally
it publishes requested commands; the watchdog forwards valid commands to the
vehicles. The controller also has an emergency zero-command cleanup path.

A avoids the static orange obstacle using camera confirmation and LiDAR,
then returns to its planned route. B responds to LiDAR obstacles without using
the harness's encounter-start/completion flags as control gates. Clear-to-resume
confirmation uses distinct scans. A has merge priority; B slows and holds 7 m
before the merge until A creates headway on the shared route.

After visiting (-40,-36), A exits toward (-48,-36). B's final approach is gated
on A completing its exit and leaving sufficient space around the reserved
loading approach. Both vehicles use the existing 1.8 m goal tolerance. The
visible loading marker is larger and does not define the acceptance condition.

## Safety and scenario validity

- Required minimum measured planar envelope clearance: 1.00 m.
- Full vehicle bounding box: x=+-1.35 m, y=+-1.05 m, z=0..1.50 m relative to
  the model origin; update these bounds if vehicle geometry changes.
- LiDAR point-to-footprint clearance and a separate simulator-pose geometry
  oracle are checked. The oracle includes roll/pitch projection and identifies
  the closest vehicle/obstacle pair.
- Roll and pitch limits are 25 and 30 degrees. Sensor health, freshness, command
  freshness and an advancing simulation clock are checked independently.
- B's loading reservation uses a projected bounding radius plus 0.30 m lateral
  tracking allowance, with 1.25 m planning margin to A. Actual clearance checks
  remain in force regardless of that reservation.
- Obstacle targets use absolute simulation time, avoiding accumulated nominal-step
  drift. A 100 ms request wait, up to three attempts and a 2-second observation
  window allow recovery from missing acknowledgements only with observed poses.
  Excessive time lag or a required pose step above 0.35 m invalidates the run.
- PASS requires mission completion, obstacle-response evidence, coordination and
  safety criteria. The summary lists unmet PASS conditions when a run fails.

These are simulation validation mechanisms, not certified safety systems.
The ground-truth oracle uses conservative planar projections, not a full 3-D
terrain/contact analysis. Camera confirmation is a controlled orange-target
detector, and simulator poses supply ideal localization. Controller/watchdog
simultaneous failure still needs an actuator-side timeout for a stronger design.

## Evidence and regression tests

The committed v6.4 result CSV and matching trace document the operator's successful Gazebo run:

| Metric | Reported value |
| --- | --- |
| Result / end reason | PASS / final_demo_complete |
| Duration | 184.11 s |
| A exit / A loading visit / B loading arrival | All YES |
| Static avoidance / camera used | YES / YES, no camera fallback |
| Moving obstacle completed / unrecovered pose failures | YES / 0 |
| B hold / resume events | 1 / 1 |
| Merge profile / stop / release events | 1 / 1 / 1 |
| Loading approach holds | 0; the gate did not need to intervene |
| Minimum ground-truth clearance | 1.903 m, A / static obstacle |
| Minimum LiDAR clearance A / B | 1.921 m / 2.169 m |
| Maximum roll/pitch A | 19.147 / 21.397 degrees |
| Maximum roll/pitch B | 9.972 / 20.273 degrees |
| Terrain / LiDAR / loading-overlap violation | All NO |

The original run evidence is now committed:

- [Result CSV](results/ruskotunturi_final_sil_demo_v6_4_results.csv): PASS, no unmet checks, duration 184.108 s, minimum ground-truth clearance 1.902952 m.
- [Matching per-step trace](results/final_sil_v6_4_trace_20261026T133025_782767.csv), named by the result CSV's `trace_file` field.

The table rounds values for readability. These files document the operator's run; updating this guide did not execute a new simulation.

New runs write into `results/`:

- `ruskotunturi_final_sil_demo_v6_4_results.csv`
- `final_sil_v6_4_trace_*.csv`

Existing CSVs cover separate AEB speed sweeps, relative-TTC scenarios, obstacle
variants including blocked paths, Ruskotunturi static/moving obstacles, sensor
fusion and two-vehicle coordination. These support staged SIL validation; their
historical failures are retained and should not be removed as if they passed.

Run the standard-library regression suite from the repository root:

```bash
python3 -m unittest discover -s tests -p 'test_final_sil_v6.py' -v
```

61 checks passed when this baseline was prepared. Tests exercise controller,
harness and watchdog logic with transport doubles, plus numerical geometry.
They are not substitutes for Gazebo runs. The successful nominal run does not
establish all-weather perception, repeatability on every host, HIL operation or
full CAMINO digital-twin/WSMS functionality.

## Documentation and local cleanup

This guide and the root README supersede temporary `FINAL_SIL*.md` patch notes.
Those notes and their `:Zone.Identifier` download metadata may be removed from
the local checkout. They are excluded from Git, along with Python bytecode.
Keep the main READMEs, scripts, worlds, regression tests and result CSVs/traces.
