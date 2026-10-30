# Hardware-in-the-Loop (HIL) validation

This directory documents the Hardware-in-the-Loop implementation used to extend the Gazebo SIL baseline onto a real embedded controller and physical CAN link.

The embedded platform is a **Seeed Studio reComputer Industrial R2100**. Gazebo Harmonic runs in WSL and provides the virtual vehicles, terrain, LiDAR and odometry. The R2100 receives simulation data over UDP, maps the relevant signals onto physical CAN, executes the AEB decision, and returns the brake state to the simulator.

The current validated Ruskotunturi milestone is **HIL-10**. The HIL path also includes hardware-validated **Configurable Signal Mapping**, **Configuration-driven Signal Codec**, **Embedded Deployment** and **Deployment Preflight** work packages.

This is a research and development setup, not a certified functional-safety implementation.

## Architecture

The Ruskotunturi HIL architecture keeps route steering in WSL and moves the longitudinal AEB safety path to the R2100.

```text
                     WSL / Gazebo Harmonic
              +--------------------------------+
              |                                |
              | Vehicle B route steering       |
              | Vehicle B odometry             |
              | Vehicle B 360 deg LiDAR        |
              |                                |
              +---------+--------------+-------+
                        |              ^
             LiDAR UDP  |              | brake UDP
              port 15010|              | port 15000
                        v              |
              +--------------------------------+
              |      reComputer R2100          |
              |                                |
              | r2100_lidar_perception.py      |
              | r2100_can_gateway.py           |
              | r2100_aeb_controller.py        |
              +----------+----------+----------+
                         |          ^
                    CAN0 |          | CAN1
                         v          |
                      physical CAN
                    CAN0_H <-> CAN1_H
                    CAN0_L <-> CAN1_L
```

For the current bench setup:

- `can0` is the simulation/gateway/perception side.
- `can1` is the AEB controller side.
- CAN0 and CAN1 are physically connected H-to-H and L-to-L.
- Classic 11-bit CAN is used at **500 kbit/s**.
- Vehicle B steering stays in the Phase 6 SIL route controller.
- Vehicle B longitudinal AEB is evaluated on the R2100.
- Vehicle A remains SIL-controlled in the full hybrid mission.

## Network transport

HIL-10 was validated with a direct wired Ethernet connection between the WSL/Windows host and the R2100. A private static subnet is recommended because it avoids dependence on WLAN, DHCP or an overlay network during timing-sensitive HIL tests.

Example lab configuration:

```text
PC Ethernet       : 192.168.50.1/24
R2100 Ethernet    : 192.168.50.2/24
Default gateway   : not required
DNS               : not required
```

These addresses are only an example private RFC1918 lab subnet. They are not a project requirement. Use the actual reachable R2100 address in commands via `<R2100_IP>` or a shell variable. Later deployment and codec-regression runs also confirmed that the same HIL scripts work over another reachable management/overlay path when the selected R2100 address is used consistently.

## Main files

R2100 and transport components:

```text
edge-path-project/hil/
├── config/
│   ├── ruskotunturi_hil.json
│   ├── ruskotunturi_hil_alt_ids.json
│   └── ruskotunturi_hil_alt_scaling.json
├── deployment/
│   ├── build_r2100_hil.py
│   ├── deploy_r2100_hil.sh
│   ├── r2100_hil_main.py
│   ├── start_hil.sh
│   ├── status_hil.sh
│   └── stop_hil.sh
├── can_protocol.py
├── signal_codec.py
├── signal_mapping.py
├── r2100_preflight.py
├── gazebo_hil_bridge.py
├── gazebo_lidar_sender.py
├── r2100_aeb_controller.py
├── r2100_can_gateway.py
├── r2100_lidar_perception.py
├── HIL10_OFFLINE_VALIDATION.txt
├── HIL10_TESTING_FI.md
├── hil10_package_manifest.json
└── README.md
```

HIL-10 runtime and validation scripts:

```text
edge-path-project/gazebo/scripts/
├── hil10_preflight.py
├── hil10_runtime.py
├── hil10_safety.py
├── ruskotunturi_hil10_b_test.py
└── ruskotunturi_hil10_hybrid_test.py
```

The signal-mapping, codec, deployment and deployment-preflight automated software tests are:

```text
tests/
├── test_signal_mapping.py
├── test_signal_mapping_gateway.py
├── test_signal_mapping_aeb.py
├── test_signal_mapping_perception.py
├── test_signal_codec.py
├── test_signal_codec_end_to_end.py
├── test_embedded_deployment.py
└── test_r2100_preflight.py
```

Earlier HIL-9 scripts and the dedicated AEB obstacle world remain available as development history.

## CAN protocol

The default CAN identifiers are provided by the active signal-mapping profile. Scalar payload encoding is now performed through `signal_codec.py` using the active signal definition. The structured brake payload remains the specialized `brake_command_v1` codec in `can_protocol.py`.

| Default CAN ID | Signal | Default encoding |
| --- | --- | --- |
| `0x100` | `VEHICLE_SPEED` | `uint16_le`, scale `0.01 m/s/bit` |
| `0x101` | `OBSTACLE_DISTANCE` | `uint16_le`, scale `0.001 m/bit`, `0xFFFF` = no obstacle |
| `0x200` | `BRAKE_COMMAND` | `brake_command_v1`: byte 0 active, byte 1 reason, byte 2 rolling counter |

Brake reason values:

```text
0 = none
1 = emergency_distance
2 = ttc_threshold
3 = sensor_timeout
```

The R2100 AEB controller uses:

```text
TTC threshold        : 1.7 s
Emergency distance   : 0.80 m
Sensor timeout       : 0.50 s
Control period       : 0.05 s
```

## Configurable Signal Mapping

The active HIL CAN identifiers, interfaces and signal definitions are loaded from a shared JSON profile through `signal_mapping.py`. The same profile is consumed by:

```text
r2100_can_gateway.py
r2100_lidar_perception.py
r2100_aeb_controller.py
```

The normal Ruskotunturi profile is:

```text
edge-path-project/hil/config/ruskotunturi_hil.json
```

It defines the current bench mapping:

```text
gateway / perception interface : can0
controller interface           : can1
expected CAN bitrate           : 500000
vehicle_speed CAN ID           : 0x100
obstacle_distance CAN ID       : 0x101
brake_command CAN ID           : 0x200
```

The loader validates required signals, classic 11-bit CAN-ID range, duplicate CAN identifiers, supported directions, interface names, schema version and encoding-specific fields. `uint16_le` requires a positive `scale`. `brake_command_v1` rejects scalar-only fields such as `scale` and `no_value_raw`.

Validate the default profile from the repository root with:

```bash
python3 edge-path-project/hil/signal_mapping.py
```

A second profile proves that runtime CAN identifiers are not tied to the defaults:

```text
edge-path-project/hil/config/ruskotunturi_hil_alt_ids.json
```

It uses:

```text
vehicle_speed      : 0x110
obstacle_distance  : 0x111
brake_command      : 0x210
```

The alternate-ID profile was tested on the same R2100 and physical CAN bench. All three R2100 processes used the alternate profile, `candump` confirmed physical traffic on `0x110`, `0x111` and `0x210`, and the unchanged HIL-10 route completed with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

The default profile was also regression-tested after all three runtime components were converted to shared configuration and completed the normal HIL-10 route with `route_complete` PASS.

Use the portable discovery form for the mapping tests:

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_signal_mapping*.py' \
  -v
```

The original Configurable Signal Mapping work package validated 12 mapping tests before the codec work was added.

## Configuration-driven Signal Codec

The signal profiles already contained `encoding`, `scale` and `no_value_raw` metadata, but the original runtime still used hardcoded scalar conversions in `can_protocol.py`. The Configuration-driven Signal Codec work package makes those fields active runtime configuration for scalar CAN signals.

`signal_codec.py` currently supports `uint16_le` scalar signals and provides shared encode/decode functions using the `SignalDefinition` loaded from the active JSON profile.

The runtime integration is:

```text
Gazebo speed
  -> r2100_can_gateway.py
  -> encode_signal_value(vehicle_speed definition)
  -> physical CAN
  -> r2100_aeb_controller.py
  -> decode_signal_value(vehicle_speed definition)

LiDAR clearance
  -> r2100_lidar_perception.py
  -> encode_signal_value(obstacle_distance definition)
  -> physical CAN
  -> r2100_aeb_controller.py
  -> decode_signal_value(obstacle_distance definition)
```

The gateway's fallback obstacle-distance path uses the same codec when external perception is not selected.

The codec validates the configured scale, rejects negative scalar values, enforces the uint16 range and prevents a real measurement from colliding with a configured reserved `no_value_raw`. Decoding maps `no_value_raw` back to `None`.

The structured `BRAKE_COMMAND` payload is deliberately not treated as a scalar signal. It continues to use the specialized `brake_command_v1` codec.

A dedicated alternate-scaling profile proves that numeric scaling can change without runtime-code edits:

```text
edge-path-project/hil/config/ruskotunturi_hil_alt_scaling.json
```

The CAN identifiers remain unchanged while the scalar resolution changes:

```text
vehicle_speed      : 0.01  -> 0.02 m/s/bit
obstacle_distance  : 0.001 -> 0.002 m/bit
```

The end-to-end software test verifies that the changed raw CAN values are decoded back to the same physical values by the AEB controller. For example, with `vehicle_speed` scale `0.02`, `1.2 m/s` is transmitted as raw value `60` and decoded back to `1.2 m/s`.

Run the current signal-mapping and codec tests with:

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_signal*.py' \
  -v
```

Run the deployment tests with:

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_embedded_deployment.py' \
  -v
```

Validated combined result for the codec work package at completion:

```text
37 / 37 PASS
```

Hardware validation was performed from a newly deployed R2100 package on the physical CAN loop. The alternate-scaling profile completed the unchanged HIL-10 route with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

The default profile was then run again using the **same new deployment release** and also completed with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

This demonstrates that `vehicle_speed` and `obstacle_distance` scaling can be changed from JSON configuration without editing the gateway, perception or AEB runtime code, while preserving the physical-value semantics across the real R2100 and CAN chain.

## Embedded Deployment

The embedded runtime can be built and deployed as a versioned package instead of copying individual Python source files to the R2100 manually.

The build entry point is:

```text
edge-path-project/hil/deployment/build_r2100_hil.py
```

It creates a generated deployment directory under `edge-path-project/hil/dist/r2100_hil/`. The generated `dist/` directory is ignored by Git and is rebuilt from source when needed.

The package contains:

```text
r2100_hil.pyz
config/
├── ruskotunturi_hil.json
├── ruskotunturi_hil_alt_ids.json
└── ruskotunturi_hil_alt_scaling.json
manifest.json
start_hil.sh
status_hil.sh
stop_hil.sh
```

The zipapp includes `signal_codec.py` and `r2100_preflight.py` together with the gateway, perception, AEB, mapping and protocol runtime modules.

`r2100_hil.pyz` provides one executable entry point for validation, preflight and the three R2100 runtime components:

```bash
r2100_hil.pyz check
r2100_hil.pyz preflight
r2100_hil.pyz perception
r2100_hil.pyz gateway
r2100_hil.pyz aeb
```

`check` validates the deployment configuration without opening CAN interfaces. `preflight` performs R2100 runtime-readiness checks before the managed processes are started.

The current deployment format intentionally depends on `python3` being installed on the R2100. It is not a standalone native binary and does not claim to remove the target Python runtime dependency.

Build locally from the repository root with:

```bash
python3 edge-path-project/hil/deployment/build_r2100_hil.py
```

Deploy the generated package over SSH/SCP with:

```bash
edge-path-project/hil/deployment/deploy_r2100_hil.sh <R2100_IP>
```

The deploy script performs the build, creates an archive, transfers it to the R2100, extracts it into a timestamped release directory, runs the deployed `r2100_hil.pyz check`, and updates the `current` symlink only after successful validation.

The resulting target layout is:

```text
~/r2100_hil/
├── current -> releases/r2100_hil_<timestamp>/
└── releases/
    ├── r2100_hil_<timestamp>/
    └── ...
```

The deployed runtime is controlled with:

```bash
~/r2100_hil/current/start_hil.sh
~/r2100_hil/current/status_hil.sh
~/r2100_hil/current/stop_hil.sh
```

A non-default profile can be selected for the managed runtime with `R2100_HIL_CONFIG`, for example:

```bash
R2100_HIL_CONFIG=~/r2100_hil/current/config/ruskotunturi_hil_alt_scaling.json \
  ~/r2100_hil/current/start_hil.sh
```

The start script records process IDs and creates a dedicated log directory. The stop script terminates the AEB, gateway and perception processes in a controlled order. The zipapp converts `SIGTERM` into the existing controller shutdown path so the AEB process can execute its final fail-safe brake transmission logic before exiting normally.

Versioned release directories also provide a simple rollback mechanism. Rollback was validated by creating two releases, changing the `current` symlink back to the older release and successfully running its deployment configuration check. The symlink was then returned to the newer validated release.

Hardware validation was performed with the R2100 processes running only from the deployed package under `~/r2100_hil/current/`, rather than from the Git working tree. The HIL-10 route completed with `route_complete` PASS.

Independent developer validation was also completed from a second developer machine. The second developer built the deployment package locally, deployed it to the R2100 over SSH/SCP, started the deployed runtime, and completed the unchanged HIL-10 route with `route_complete` PASS. This provides an additional reproducibility check that the deployment workflow is not tied to the original development machine.

This work package therefore demonstrates a repeatable build -> validate -> deploy -> run -> stop -> rollback workflow for the current R2100 HIL runtime.

## Deployment Preflight

The deployed zipapp includes an R2100-specific readiness check:

```bash
~/r2100_hil/current/r2100_hil.pyz preflight
```

The preflight loads the selected mapping profile and verifies the runtime prerequisites that can be checked locally before the HIL processes start:

- the active JSON configuration is valid,
- the configured gateway and controller CAN interfaces exist,
- both CAN interfaces are `UP`,
- both CAN interfaces report `ERROR-ACTIVE`,
- the configured CAN bitrate matches the live interfaces,
- UDP port `15000` is available for the CAN gateway,
- UDP port `15010` is available for LiDAR perception.

`start_hil.sh` runs this preflight automatically before starting perception, gateway or AEB. Because the shell script uses fail-fast execution, a failed preflight prevents a partial or conflicting runtime startup.

A normal hardware preflight on the validated R2100 bench produced:

```text
[PASS] config: ruskotunturi_hil
[PASS] CAN can0: UP, ERROR-ACTIVE, bitrate=500000
[PASS] CAN can1: UP, ERROR-ACTIVE, bitrate=500000
[PASS] UDP 15000: available for gateway
[PASS] UDP 15010: available for LiDAR perception

R2100 HIL preflight: PASS
```

Fail-closed behavior was validated by running `start_hil.sh` a second time while the original deployment processes were already running. Preflight detected both occupied UDP ports and returned `FAIL`; no second runtime was started, and the original perception, gateway and AEB processes remained `RUNNING`.

After integrating automatic preflight into `start_hil.sh`, the normal HIL-10 route was regression-tested again over the deployed R2100 and physical CAN loop:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

Dedicated preflight and deployment tests can be run with:

```bash
python3 -m unittest discover -s tests -p 'test_r2100_preflight.py' -v
python3 -m unittest discover -s tests -p 'test_embedded_deployment.py' -v
```

Validated results are:

```text
preflight unit tests : 4 / 4 PASS
deployment tests     : 8 / 8 PASS
signal regression    : 30 / 30 PASS
```

The deployment preflight is intentionally local to the R2100. It verifies target readiness; it does not prove that Gazebo is running or that the WSL test selected a reachable R2100 address. Those remain end-to-end HIL prerequisites.

## Important distance semantics

`OBSTACLE_DISTANCE` is treated as **free clearance from the physical vehicle body to the obstacle**, not raw LiDAR-origin range.

Vehicle B uses a roof-centred 360-degree LiDAR. The body model used by HIL perception is:

```text
vehicle half length : 1.30 m
vehicle half width  : 0.60 m
front LiDAR sector  : +/-10 deg
```

For each valid ray in the front sector, the R2100 perception calculates the vehicle-body radius in that direction and subtracts it from the LiDAR range before transmitting the configured obstacle-distance CAN signal.

This correction is safety-critical for the current geometry. Using raw LiDAR-origin range caused TTC braking to trigger too late because the LiDAR is approximately 1.3 m behind the front edge of the vehicle body.

## LiDAR UDP transport

Vehicle B produces a 360-degree scan with **721 beams**. A legacy full scan is about 1468 bytes and was not reliable over the original tested WSL/network path.

The LiDAR transport therefore supports two packet formats:

```text
HILR = legacy complete scan
HILC = chunked scan
```

`gazebo_lidar_sender.py` keeps the legacy format for scans that fit within the configured safe UDP payload and automatically chunks larger scans. The current maximum UDP payload is 900 bytes.

For the 721-beam Ruskotunturi scan this normally produces two UDP packets per scan. `r2100_lidar_perception.py` reassembles the chunks before evaluating the front sector.

If LiDAR updates become stale, the perception process deliberately stops transmitting the obstacle-distance signal. The AEB controller then detects stale sensor data and enters the `sensor_timeout` fail-safe brake state.

HIL-10 integrates the raw LiDAR sender into `ruskotunturi_hil10_b_test.py`; do **not** run a separate `gazebo_lidar_sender.py` during HIL-10 B tests.

## HIL-10 freshness and fail-safe rules

HIL-10 was introduced after identifying a stale-measurement weakness in the earlier HIL-9D adapter: the odometry timestamp was stored, but the cached speed value could still be reused without checking its age.

The HIL-10 runtime closes that gap with explicit freshness rules:

- Vehicle B odometry and returned brake state have a maximum local age of **0.50 s**.
- Missing, stale or invalid odometry blocks motion.
- One odometry speed sample is transmitted to the R2100 at most once; cached speed is not replayed as a heartbeat.
- If a Gazebo source timestamp is present, it must advance for the sample to count as new.
- Duplicate, reordered or inconsistent brake replies do not refresh the brake lease.
- Brake counter wrap from `255` to `0` is accepted.
- A communication/freshness fault detected after motion begins latches the stop for the remainder of that test run.
- A stop is verified only from at least **three distinct post-command odometry samples** at or below **0.05 m/s**.
- Re-reading a cached zero-speed value cannot produce a PASS.
- The stop observer uses the magnitude of the full linear velocity vector.

## CAN interface setup

On the R2100, configure both CAN interfaces at 500 kbit/s before starting HIL processes:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can1 down 2>/dev/null || true

sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can1 type can bitrate 500000 restart-ms 100

sudo ip link set can0 txqueuelen 100
sudo ip link set can1 txqueuelen 100

sudo ip link set can0 up
sudo ip link set can1 up
```

Useful checks:

```bash
ip -details link show can0
ip -details link show can1
```

The validated bench loop uses:

```text
CAN0_H <-> CAN1_H
CAN0_L <-> CAN1_L
```

## HIL-10 offline validation

Before hardware execution, run the preflight and HIL-10 software tests from the repository root:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
python3 -m unittest discover -s tests -p 'test_hil10*.py' -v
```

Validated HIL-10 result:

```text
Ran 42 tests
OK
```

The tests cover freshness, stale cached speed, duplicate/reordered replies, source timestamps, stop verification, fail-closed behavior and HIL-10 execution/adaptor logic.

## Standard HIL-10 three-terminal workflow

The reliable HIL-10 startup order is:

```text
1. WSL: Gazebo world
2. R2100: perception + CAN gateway + AEB
3. WSL: HIL-10 test script
```

The HIL-10 B test starts its own raw LiDAR sender, so a fourth LiDAR process is not required.

### Terminal 1 - WSL / Gazebo

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r \
  edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf
```

### Terminal 2 - R2100 / deployed runtime

Restart all three R2100 processes between test runs because the AEB latch and test state must start cleanly.

```bash
~/r2100_hil/current/start_hil.sh
~/r2100_hil/current/status_hil.sh
```

`start_hil.sh` runs the R2100 deployment preflight automatically and starts the three runtime components only after it passes.

For a non-default profile, set `R2100_HIL_CONFIG` before invoking the same start script.

### Terminal 3 - WSL / HIL-10

```bash
cd ~/edge-path-project-git
read -r -p "R2100 IP address: " R2100_IP

python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode route \
  --r2100-host "$R2100_IP"
```

Other supported HIL-10 modes are `aeb`, `fault-odometry`, `fault-brake` and `fault-lidar`; the hybrid mission uses `ruskotunturi_hil10_hybrid_test.py`.

## HIL-10 hardware validation summary

All six planned HIL-10 hardware scenarios passed on the Gazebo -> network -> R2100 -> physical CAN -> R2100 -> network -> Gazebo loop.

| Test | Purpose | Result |
| --- | --- | --- |
| `route` | Vehicle B normal route and verified final stop | PASS |
| `hybrid` | Two-vehicle Phase 6 hybrid mission | PASS |
| `aeb` | Intentional obstacle-triggered R2100 AEB stop | PASS |
| `fault-odometry` | Remove odometry from HIL command/speed path | PASS |
| `fault-brake` | Stop accepting returned brake replies | PASS |
| `fault-lidar` | Stop raw LiDAR transmission to R2100 | PASS |

Validated fault-detection latencies with `--fault-after 8.0` were:

```text
fault-odometry : 0.506 s -> odometry_timeout
fault-brake    : 0.509 s -> brake_timeout
fault-lidar    : 0.559 s -> sensor_timeout
```

Each fault run required the vehicle to move before injection, detect the expected fault within the allowed window, command zero speed and steering, verify stopping from new odometry samples, and hold the verified stop.

## Manual physical CAN interruption and recovery validation

A manual robustness test was also performed by physically interrupting the CAN0-CAN1 bench connection while Vehicle B was driving.

The first experiment exposed `ENOBUFS` handling in the AEB transmit path. The controller was hardened so transient CAN transmit errors (`ENOBUFS`, `EAGAIN` and `EWOULDBLOCK`) are treated as dropped frames instead of fatal process errors. The brake rolling counter advances only after successful CAN transmission.

The repeated test verified:

```text
normal driving
 -> physical CAN interruption
 -> fail-safe communication/sensor timeout
 -> vehicle stop
 -> CAN bus-off/error state observed
 -> AEB, gateway and perception remained alive
 -> physical CAN connection restored
 -> CAN0 and CAN1 returned to ERROR-ACTIVE
 -> new route run completed without restarting the R2100 processes
 -> PASS
```

## HIL-10 result evidence

Each HIL-10 hardware run writes a CSV trace and matching JSON summary under:

```text
edge-path-project/gazebo/results/hil10/
```

Generated HIL-10 CSV/JSON outputs are ignored by default for new runs so repeated regression tests do not clutter version control. Significant hardware-validation conclusions are recorded in this README.

## HIL-9 validation summary

HIL-9 remains useful as the development history that established the Ruskotunturi hybrid architecture.

| Stage | Purpose | Result |
| --- | --- | --- |
| HIL-9A | Ruskotunturi Phase 6 SIL baseline | PASS |
| HIL-9B | Vehicle B raw 721-beam LiDAR to R2100 perception | PASS |
| HIL-9C | Vehicle B route steering with R2100 longitudinal AEB | PASS |
| HIL-9D | Full hybrid two-vehicle Phase 6 mission | PASS |
| HIL-9E | Intentional automated R2100 AEB intervention | PASS |

The HIL-9E automated stop used TTC intervention and verified **1.160 m** conservative body-to-obstacle clearance against a **1.000 m** requirement.

HIL-10 supersedes the HIL-9 runtime for freshness-sensitive validation because HIL-10 prevents cached speed replay and requires fresh post-stop measurements.

## Key integration findings

### 1. UDP MTU / scan size

A 721-beam raw LiDAR scan exceeded the reliable UDP payload size on the original tested WSL-to-R2100 path. Chunking the scan into payloads no larger than 900 bytes resolved the problem while retaining legacy single-packet support for smaller scans.

### 2. Sensor-origin distance is not body clearance

The first intentional Ruskotunturi AEB test used raw LiDAR-origin range as the obstacle-distance signal. Because the LiDAR is roof-centred, this did not represent physical bumper clearance. The R2100 perception was changed to transmit body-to-obstacle clearance instead.

### 3. Stale data must not be refreshed by cached values

HIL-10 identified and fixed a separate integration issue: locally cached odometry could otherwise be re-sent as if it were a new speed measurement. Freshness must therefore be tied to actual new source samples, not simply to repeated packet transmission.

### 4. Fail-safe behavior is independently testable

HIL-10 verified three independent communication-loss paths:

```text
odometry loss -> local odometry_timeout -> stop
brake-return loss -> local brake_timeout -> stop
LiDAR loss -> R2100 sensor_timeout -> stop
```

### 5. CAN transmit saturation must not terminate the safety controller

A physical CAN interruption can prevent the SocketCAN transmit queue from draining and produce `ENOBUFS`. After hardening the send path, the controller survived the interruption and resumed normal operation when the physical CAN connection recovered.

### 6. Shared signal mapping must be validated end to end

Moving CAN IDs into a configuration file is only useful if every active producer and consumer uses the same profile. The alternate-ID hardware run verified the configured IDs on the physical CAN bus and completed the route without Python runtime edits.

### 7. Deployment should not depend on the target Git working tree

The validated embedded deployment packages the runtime into a generated Python zipapp plus external configuration and control scripts. Hardware routes were executed from the deployed `~/r2100_hil/current/` release rather than from `~/EdgeDetection`. A second developer independently built and deployed the same workflow from another development machine and also completed the HIL-10 route successfully.

### 8. Numeric signal scaling must be shared by producer and consumer

A configurable `scale` value is only safe if every producer and consumer interprets the same physical signal using the same profile. The gateway, LiDAR perception and AEB controller now use the shared scalar codec. The alternate-scaling profile changed the raw CAN representation while preserving the decoded physical values, and both alternate-scaling and default-profile routes passed on the same newly deployed R2100 release.

### 9. Runtime readiness should fail before process startup

Configuration validity alone does not prove that the R2100 is ready to run HIL. The deployment preflight therefore checks the configured CAN interfaces, their live state and bitrate, plus the UDP ports required by gateway and LiDAR perception. A conflicting second startup was rejected before any new runtime processes were launched, while the original deployment remained healthy.

## Scope and limitations

The current validation still does **not** cover every possible hardware or system failure. Follow-on targets include:

- a standalone native executable that does not require Python on the target,
- arbitrary packet delay/jitter profiles,
- forced process termination,
- PC power loss,
- an external command watchdog outside the current test runtime,
- broader multi-node CAN fault cases beyond the current two-interface bench loop.

The scalar codec currently supports `uint16_le`; structured `brake_command_v1` remains a specialized codec rather than a fully generic schema-driven payload description.

The deployment preflight verifies local R2100 readiness but does not verify that Gazebo is running, that WSL can reach the selected R2100 address, or that end-to-end UDP traffic is flowing before the HIL test starts.

## Troubleshooting

If a test remains in a fail-safe state at startup, check that:

- `~/r2100_hil/current/r2100_hil.pyz preflight` passes,
- the correct Gazebo world is running,
- `can0` and `can1` are UP at 500 kbit/s,
- all three R2100 processes were started with the same mapping profile,
- the physical CAN H/L loop is connected,
- the **same reachable R2100 IP address** is supplied to the HIL test for the network path currently in use,
- no old HIL controller is still publishing Vehicle B commands,
- no separate LiDAR sender is running during a HIL-10 B test.

A `startup_not_ready` result can occur when Gazebo is not running or when the test sends UDP traffic to an R2100 address that is not reachable on the currently active network path. The R2100 deployment preflight cannot detect either WSL-side condition.

Only one process should publish Vehicle B commands during a test.

## Current status

The repository now contains a verified progression from SIL to embedded HIL:

```text
CARLA experiments
 -> Gazebo ADAS SIL
 -> Ruskotunturi multi-vehicle SIL
 -> physical CAN HIL
 -> Ruskotunturi hybrid HIL-9
 -> HIL-10 freshness + communication fault validation
 -> configurable signal mapping on the R2100 CAN chain
 -> reproducible embedded deployment package
 -> configuration-driven scalar signal codec
 -> deployment preflight and fail-closed startup
```

Current evidence:

```text
HIL-10 offline software validation       : 42 / 42 PASS
HIL-10 hardware scenarios                : 6 / 6 PASS
original signal-mapping offline tests    : 12 / 12 PASS
mapping + deployment offline tests       : 19 / 19 PASS
codec work-package regression            : 37 / 37 PASS
current signal regression                : 30 / 30 PASS
deployment tests                         : 8 / 8 PASS
deployment preflight unit tests          : 4 / 4 PASS
R2100 deployment preflight               : PASS
preflight duplicate-start fail-closed    : PASS
preflight-integrated HIL route           : PASS / route_complete
default mapping route regression         : PASS
alternate CAN-ID route                   : PASS
alternate IDs on physical CAN            : 0x110 / 0x111 / 0x210 observed
alternate scaling route                  : PASS
default scaling on same new deployment   : PASS
embedded package build/check             : PASS
versioned SSH/SCP deployment             : PASS
deployed-package HIL route               : PASS
second-developer deployment route        : PASS
start / stop / status control            : PASS
release rollback                         : PASS
wired Ethernet transport                 : validated
alternate reachable network path         : validated
physical CAN loop                        : validated
manual CAN interruption + recovery       : PASS
```
