# HIL-10: measurement freshness and communication-loss tests

Date: 29 October 2026. Baseline: `hil-development`, commit
`52331fde756991988cbb94f01e6d33d615c8d7df`.

> This is an English translation of the original HIL-10 package guide. The
> baseline checks, manual startup procedure and validation status below describe
> that package when it was prepared. For the current deployed runtime, signal
> mapping and completed hardware validation, see the [current HIL guide](README.md).
> The original filename is retained to preserve existing references.

This package adds HIL-10 tests alongside the existing HIL-9 implementation.
It does not replace the HIL-9 scripts, worlds, R2100 programs or earlier results.
The corrected behaviour is used when you run the **HIL-10 scripts** listed in
this guide. Running the old HIL-9 scripts still uses their original behaviour.

## 1. What has been fixed

- The maximum permitted local age of Vehicle B's odometry and brake message is
  0.50 s. Missing or stale data blocks Vehicle B's motion command.
- Each speed sample is sent to the R2100 at most once. An old speed value is not
  replayed as a new CAN speed update. This allows the R2100's existing sensor
  timeout monitoring to detect loss of speed measurements as well.
- If a measurement message contains a Gazebo timestamp, that timestamp must
  advance. Freshness is measured using the PC's monotonic clock. Clocks on
  different machines are not compared.
- Invalid, duplicate or out-of-order brake replies do not refresh the brake
  state. Counter wrap from 255 to 0 is allowed.
- A communication fault detected after motion begins latches the stop for the
  remainder of that test run. A temporary recovery does not automatically
  release the brake.
- Stop verification requires at least three new consecutive odometry messages
  with a speed at or below 0.05 m/s, received after the stop command. Reading
  the same cached value repeatedly does not satisfy this condition. Speed is
  the magnitude of the linear velocity vector, including lateral and vertical
  components.
- Vehicle B's route test verifies the stop before reporting PASS. The hybrid
  test retains the Phase 6 acceptance criteria and adds verification that B
  has stopped. If an HIL fault interrupts the hybrid run, zero commands are
  sent to both vehicles.
- Each run saves its own CSV trace and JSON summary under
  `edge-path-project/gazebo/results/hil10/`. The old SIL results CSV is not modified.

The 1.00 m AEB clearance requirement and the R2100 braking thresholds are unchanged.

## 2. Installation and software checks in WSL

Stop any running test controllers and close Gazebo. Extract the ZIP into the
project root, `~/edge-path-project-git`, merging the package's `edge-path-project`
and `tests` directories with the existing directories of the same names. The
files are new; the package does not contain replacements for HIL-9 source files.

Run these commands from the project root:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
python3 -m unittest discover -s tests -p 'test_hil10*.py' -v
```

Expected result: baseline dependencies report `OK`, and **42 software tests / OK**.
The check reads files without modifying them. `DIFFERS` or `MISSING` means that
the local baseline needs to be checked before a hardware run. Do not delete
your own changes or results to satisfy the check.

The software tests require neither Gazebo nor a CAN device. They use the actual
freshness checks and test execution loop, while elapsed time, Gazebo message
imports and vehicle response are replaced with controlled test doubles. The
R2100's existing AEB decision logic is tested directly without a CAN interface.
These tests are not new Gazebo–R2100 hardware results.

## 3. Test order and worlds

Run each row in the table as a separate test. **Restart Gazebo and all three
R2100 processes between every run.** The R2100 AEB latch and the test's latched
fault state are reset only for a new run. Do not use Gazebo's reset button
during a test: a timestamp moving backwards is treated as an old measurement.

| Order | Test | Gazebo world | Script / option |
|---|---|---|---|
| 1 | Vehicle B's normal route and final stop | Phase 6 | B test, `--mode route` |
| 2 | Two-vehicle hybrid run (HIL-9D regression) | Phase 6 | Hybrid test |
| 3 | Obstacle-triggered AEB (HIL-9E regression) | HIL-9E obstacle world | B test, `--mode aeb` |
| 4 | Loss of odometry supplied to Vehicle B's controller | Phase 6 | B test, `--mode fault-odometry` |
| 5 | Loss of returned brake messages | Phase 6 | B test, `--mode fault-brake` |
| 6 | Loss of LiDAR data sent to the R2100 | Phase 6 | B test, `--mode fault-lidar` |

HIL-10 sends Vehicle B's raw LiDAR data itself. **Do not start a separate
`gazebo_lidar_sender.py` process.** An old HIL-9 controller or any other process
sending commands to Vehicle B must not run at the same time either.

### Terminal 1: WSL / Gazebo

Use the Phase 6 world for the route, hybrid and communication-loss tests:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf
```

For the AEB test only (`--mode aeb`), start this world instead:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

### Terminal 2: R2100

Use the programs that worked in HIL-9 and the physical CAN0–CAN1 connection.
Both interfaces use 500 kbit/s. Check their status:

```bash
ip -details link show can0
ip -details link show can1
```

If the interfaces are not ready, apply the CAN configuration described in
`edge-path-project/hil/README.md`. This package does not require changes to the
R2100 source code.

Stop the three R2100 processes from the previous test before starting them again.
If they were started as background jobs in the same terminal, the PIDs saved
below let you stop those specific processes. On the first run, stop the old
HIL-9 processes in the terminals or sessions where you started them.

```bash
cd ~/EdgeDetection/edge-path-project/hil
HIL10_LOG_DIR="$HOME/hil_test_logs/hil10_$(date -u +%Y%m%dT%H%M%S)"
mkdir -p "$HIL10_LOG_DIR"

python3 -u r2100_lidar_perception.py > "$HIL10_LOG_DIR/perception.log" 2>&1 &
HIL10_PERCEPTION_PID=$!
python3 -u r2100_can_gateway.py --external-perception > "$HIL10_LOG_DIR/gateway.log" 2>&1 &
HIL10_GATEWAY_PID=$!
python3 -u r2100_aeb_controller.py > "$HIL10_LOG_DIR/aeb.log" 2>&1 &
HIL10_AEB_PID=$!
```

`--external-perception` is required: the LiDAR perception process owns CAN
signal `0x101`. The gateway forwards Vehicle B's speed as signal `0x100` and
returns the `0x200` brake state. Missing sensor data and `sensor_timeout` are
expected before the PC test starts.

### Terminal 3: WSL / HIL-10 test

Enter the current IP address once in this terminal:

```bash
cd ~/edge-path-project-git
read -r -p "R2100 IP address: " R2100_IP
```

First test: Vehicle B's normal route:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode route --r2100-host "$R2100_IP"
```

Second test: the two-vehicle hybrid run:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_hybrid_test.py --r2100-host "$R2100_IP"
```

Third test: AEB in the obstacle world:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode aeb --r2100-host "$R2100_IP"
```

The three separate commands for the communication-loss tests are below. Run
them **one at a time**, restarting Gazebo and the R2100 processes between runs.

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-odometry --r2100-host "$R2100_IP"
```

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-brake --r2100-host "$R2100_IP"
```

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-lidar --r2100-host "$R2100_IP"
```

At startup, the test waits up to 12 seconds for fresh pose and odometry data
and the R2100's DRIVE state. The vehicle is not allowed to move before these
conditions are met. An incorrect starting position produces `startup_not_ready`
and a request to restart the world.

### After a run and before the next test

The PC test stops its own LiDAR sender and sends stop commands. Close Gazebo
in terminal 1. Stop the R2100 processes you started in terminal 2 and wait for
them to exit:

```bash
kill -INT "$HIL10_PERCEPTION_PID" "$HIL10_GATEWAY_PID" "$HIL10_AEB_PID"
wait "$HIL10_PERCEPTION_PID" "$HIL10_GATEWAY_PID" "$HIL10_AEB_PID"
```

Keep each run's R2100 log directory. For the next test, start the correct world,
restart the three R2100 processes, and then start the PC test.

## 4. PASS criteria for communication-loss tests

By default, the selected fault is injected 2 seconds after motion is first
observed, while Vehicle B's measured speed is still at least 0.20 m/s and the
motion command is positive. You can change the delay using `--fault-after`,
but use the default value for the initial tests to keep them comparable.

| Fault | What is interrupted | Expected detection |
|---|---|---|
| `fault-odometry` | Odometry updates to the HIL command gate and speed transmission | Local `odometry_timeout`; the R2100's `sensor_timeout` may be detected first |
| `fault-brake` | Acceptance of brake replies received by the PC | Local `brake_timeout` |
| `fault-lidar` | Transmission of Vehicle B's raw LiDAR data to the R2100 | A fresh brake reply from the R2100 with reason `sensor_timeout` |

All three tests require:

1. Vehicle B moves before the selected fault is injected.
2. The correct fault is detected within 1.0 s of injection.
3. Both the speed and steering commands remain zero after detection.
4. The stop is verified from three new measurements within 3.0 s of injection.
5. The verified stop is maintained for at least 1.0 s, with measurements
   remaining fresh.

The expected successful end reason is `fault_stop_and_hold_verified`. The
normal route passes with `route_complete`, the hybrid test with
`hybrid_complete`, and the AEB test with `aeb_stop_verified`. A fault message
alone or reaching the goal is not enough for a communication-loss test to
PASS. A fault occurring before injection is a FAIL.

During the odometry-loss test, a separate verification observer continues
reading odometry from the same Gazebo vehicle. It does not supply speed data
to the controller. This allows the controller's measurement loss and the
vehicle's actual simulated motion to be checked separately. If Gazebo's
odometry source itself stops completely, the stop cannot be verified and
the test is a FAIL.

These are controlled software fault injections. By themselves, they do not
test physical CAN cable disconnection, CAN bus-off, all packet delays, forced
process termination or PC power loss. A command watchdog running outside the
test process is a separate future development task. This package does not
implement it.

## 5. Result logs and troubleshooting

Each run produces a pair of files:

- `hil10_<mode>_<timestamp>_<identifier>.csv`: measurement age, speed, requested
  and applied commands, brake reason/counter, latched fault state and stop samples.
- A `.json` file with the same stem: PASS/FAIL, end reason, stop verification
  and test-specific results. Communication-loss tests include detection and
  stopping latency; AEB includes clearance; the hybrid test includes the
  Phase 6 acceptance criteria.

The monotonic seconds in the CSV are local to the PC and are not calendar time.
The JSON field `baseline_commit` identifies the baseline version; the HIL-10
changes are local files from this separately distributed package. The package
does not create a Git commit or push changes.

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo/results/hil10
ls -lt
explorer.exe .
```

If a test fails, keep its JSON file and matching CSV, along with the three
R2100 logs. For `startup_not_ready`, first check that the world is running,
the IP address is correct, the R2100 processes are running, and no second
LiDAR sender or controller is active. For example, inspect the R2100 logs with:

```bash
tail -n 20 "$HIL10_LOG_DIR/perception.log" "$HIL10_LOG_DIR/gateway.log" "$HIL10_LOG_DIR/aeb.log"
```

The LiDAR log should show complete 721-beam scans before fault injection.
`stop_not_verified` and `verification_odometry_timeout` do not count as
verified stops, even if the vehicle appears stationary in the visualisation.

## 6. Completion criteria

Status when this package was prepared:

- Software checks: **42/42 PASS**.
- Gazebo + R2100 + physical CAN: **not yet run with this HIL-10 version**.

The next milestone at that point was to complete all six tests in the table
successfully on the local hardware, retaining a CSV/JSON pair and R2100 logs
for each run. Those results would provide the basis for recording HIL-10 as
a hardware-validated stage and committing the accepted files and results to
`hil-development`. See the [current HIL guide](README.md#hil-10-hardware-validation-summary)
for the subsequent hardware-validation status.
