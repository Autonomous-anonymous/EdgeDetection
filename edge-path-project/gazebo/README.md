# Gazebo SIL baseline: v6.4

This guide describes the v6.4 SIL baseline merged into `main` from `sil-development`. Earlier scripts,
worlds and CSV results remain available as development evidence. Mevea is outside
this project's scope. The later HIL stage uses a real reComputer Industrial R2100
with physical CAN while Gazebo remains the virtual world and vehicle-dynamics source.

For the short two-vehicle presentation with a fixed camera, Finnish status panel and optional physical CAN, see the [pitch instructions](PITCH_DEMO.md).

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

## Common SIL/HIL emergency-braking scenario

The repository also contains one shared staged scenario for direct SIL-to-HIL
comparison of the longitudinal AEB safety function:

```text
edge-path-project/gazebo/scenarios/emergency_braking.json
```

The scenario uses:

```text
world: edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
vehicle: aeb_vehicle_360_b
required clearance: 1.00 m
stop speed: 0.05 m/s
stop confirmation: 3 samples
valid brake reasons: emergency_distance, ttc_threshold
```

Start the common world in terminal 1:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

Close and reopen Gazebo before each new SIL or HIL trial so Vehicle B starts from
the defined initial state. Do not run another Vehicle B command publisher at the
same time.

Run the SIL target in terminal 2:

```bash
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target sil
```

The SIL adapter uses Vehicle B pose, odometry and `/aeb360b/lidar`, follows the
same route used by the HIL test and applies the same AEB thresholds as the current
embedded controller. LiDAR range is converted to physical body clearance before
the AEB decision so the SIL and HIL distance semantics match.

The duplicated SIL/HIL AEB constants are protected against drift by
`tests/test_aeb_threshold_consistency.py`, which compares TTC threshold,
emergency distance, sensor timeout, control period and minimum TTC speed.

Verified SIL result on 2026-11-02:

```text
STAGED SCENARIO RESULT: PASS
End reason: aeb_stop_verified
AEB reason: ttc_threshold
Trigger clearance: 1.374 m
Trigger speed: 0.850 m/s
Trigger TTC: 1.617 s
Ground-truth clearance: 1.253 m
```

For the HIL target, first ensure the R2100 runtime has passed its target-side
preflight and all managed HIL processes are running. With the same Gazebo world
running, execute:

```bash
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target hil \
  --r2100-host <R2100_IP>
```

The HIL backend is the validated `ruskotunturi_hil10_b_test.py --mode aeb` path.
It sends raw Vehicle B LiDAR to the R2100 perception process, carries the resulting
signals across physical CAN to the embedded AEB controller and applies the returned
brake state to Gazebo.

Verified HIL result on 2026-11-02:

```text
HIL-10 RESULT: PASS
End reason: aeb_stop_verified
AEB reason: ttc_threshold
```

The successful HIL run is retained as committed raw evidence:

```text
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.json
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.csv
```

The JSON records `passed=true`, `stop_verified=true`, `aeb_reason=ttc_threshold`
and 1.189 m recorded clearance against the 1.000 m requirement. The common runner
currently selects and validates the execution target but intentionally leaves
Gazebo and R2100 runtime lifecycle management explicit.

Offline coverage for the common scenario is:

```bash
python3 -m unittest tests.test_staged_scenario -v
```

Verified result:

```text
Ran 6 tests
OK
```

AEB threshold-consistency coverage is:

```bash
python3 -m unittest tests.test_aeb_threshold_consistency -v
```

Verified result:

```text
Ran 5 tests
OK
```

With all current offline tests included, the complete repository suite passes
**168 / 168**.

## Independent HIL process-termination fail-safe

The current Gazebo/HIL tooling also includes:

```text
edge-path-project/gazebo/scripts/hil_command_watchdog.py
edge-path-project/gazebo/scripts/hil_process_termination_test.py
```

The automated harness waits until Vehicle B is moving, kills the normal HIL route
runner with `SIGKILL`, requires a new zero command from the independent watchdog,
and verifies the subsequent stop from Gazebo odometry.

The final committed evidence is:

```text
edge-path-project/gazebo/results/hil10/hil_process_termination_20261102T115256Z.json
```

It records `passed=true`, `source_dirty=false`, source commit
`269a1c4feb5ce91a5174f25243f21be71cce959f`, watchdog STOP latency 0.261 s,
17 stop-confirmation samples and stop-and-hold verification 1.616 s after
`SIGKILL`.

## Files and responsibilities

| File under scripts/ | Responsibility |
| --- | --- |
| ruskotunturi_final_sil_demo_v6.py | Navigation, camera/LiDAR avoidance, merge/loading coordination and result evaluation |
| final_sil_v6_harness.py | Separate process driving the moving obstacle; observed-pose verification and bounded retries |
| final_sil_v6_watchdog.py | Separate command arbiter stopping stale or unsafe commands |
| final_sil_v6_checks.py | Sensor validation, footprint geometry and initial-scene checks |
| run_staged_scenario.py | Shared scenario loader and SIL/HIL target selector |
| ruskotunturi_emergency_braking_sil.py | SIL backend for the shared emergency-braking scenario |
| ruskotunturi_hil10_b_test.py | Existing HIL backend used by the shared scenario with `--mode aeb` |
| hil_command_watchdog.py | Independent Vehicle B stale-command STOP latch for HIL process-loss protection |
| hil_process_termination_test.py | Automated `SIGKILL` fail-safe validation and JSON evidence writer |

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
The WSL-side HIL watchdog is independent of the HIL runner process but not of the
PC/WSL host; PC power loss, physical CAN disconnection and CAN bus-off remain
outside the demonstrated coverage.

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

The original run evidence is committed:

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

## Scope boundary

The current project deliverable validates the staged **SIL -> HIL** path. A
Vehicle-in-the-Loop (VIL) stage is not implemented and is explicitly outside the
current deliverable scope.

## Documentation and local cleanup

This guide and the root README supersede temporary `FINAL_SIL*.md` patch notes.
Those notes and their `:Zone.Identifier` download metadata may be removed from
the local checkout. They are excluded from Git, along with Python bytecode.
Keep the main READMEs, scripts, worlds, regression tests and result CSVs/traces.
