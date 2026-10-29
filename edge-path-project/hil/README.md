# Hardware-in-the-Loop (HIL) validation

This directory documents the Hardware-in-the-Loop implementation used to extend the Gazebo SIL baseline onto a real embedded controller and physical CAN link.

The embedded platform is a **Seeed Studio reComputer Industrial R2100**. Gazebo Harmonic runs in WSL and provides the virtual vehicles, terrain, LiDAR and odometry. The R2100 receives simulation data over UDP, maps the relevant signals onto physical CAN, executes the AEB decision, and returns the brake state to the simulator.

The current validated Ruskotunturi milestone is **HIL-10**. The HIL path also includes a hardware-validated **Configurable Signal Mapping** work package for selecting the CAN identifiers and CAN interfaces from a shared configuration profile.

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

These addresses are only an example private RFC1918 lab subnet. They are not a project requirement. Use the actual R2100 address in commands via `<R2100_IP>` or a shell variable.

WLAN or another management connection may remain enabled in parallel, but HIL UDP traffic should use the selected wired R2100 address when a direct Ethernet link is available.

## Main files

R2100 and transport components:

```text
edge-path-project/hil/
├── config/
│   ├── ruskotunturi_hil.json
│   └── ruskotunturi_hil_alt_ids.json
├── can_protocol.py
├── signal_mapping.py
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

Earlier HIL-9 validation scripts remain available:

```text
edge-path-project/gazebo/scripts/
├── ruskotunturi_hil_b_route_test.py
├── ruskotunturi_phase6_hybrid_hil_test.py
└── ruskotunturi_hil9e_aeb_test.py
```

The HIL-10 and signal-mapping automated software tests are:

```text
tests/
├── test_hil10_integration.py
├── test_hil10_safety.py
├── test_signal_mapping.py
├── test_signal_mapping_gateway.py
├── test_signal_mapping_aeb.py
└── test_signal_mapping_perception.py
```

The dedicated AEB obstacle world is:

```text
edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

## CAN protocol

The protocol payload encoding is defined in `can_protocol.py`. The default CAN identifiers are provided by the active signal-mapping profile.

| Default CAN ID | Signal | Encoding |
| --- | --- | --- |
| `0x100` | `VEHICLE_SPEED` | uint16 little-endian, 0.01 m/s per bit |
| `0x101` | `OBSTACLE_DISTANCE` | uint16 little-endian millimetres, `0xFFFF` = no obstacle |
| `0x200` | `BRAKE_COMMAND` | byte 0 active, byte 1 reason, byte 2 rolling counter |

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

The active HIL CAN identifiers and interface selection are loaded from a shared JSON profile through `signal_mapping.py`. The same profile is consumed by:

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

The profile also records signal direction, source, encoding and scale metadata. At the current implementation stage, CAN-ID and interface selection are active runtime configuration. Payload encoding and numeric scaling are still implemented by `can_protocol.py`; the corresponding profile metadata is validated but is not yet used to replace the codec implementation.

The loader validates the profile before use, including required signals, classic 11-bit CAN-ID range, duplicate CAN identifiers, positive scale values, supported directions, interface names and schema version.

Validate the default profile from the repository root with:

```bash
python3 edge-path-project/hil/signal_mapping.py
```

A second profile is committed specifically as proof that the runtime is not tied to the default identifiers:

```text
edge-path-project/hil/config/ruskotunturi_hil_alt_ids.json
```

It uses:

```text
vehicle_speed      : 0x110
obstacle_distance  : 0x111
brake_command      : 0x210
```

The alternate profile was tested on the same R2100 and physical CAN bench. All three R2100 processes were started with the alternate profile, `candump` confirmed physical traffic on `0x110`, `0x111` and `0x210`, and the unchanged HIL-10 route completed with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

The default profile was also regression-tested after all three runtime components were converted to shared configuration and completed the normal HIL-10 route with `route_complete` PASS.

Dedicated offline tests cover the loader plus gateway, AEB and perception integration:

```bash
python3 -m unittest \
  tests.test_signal_mapping \
  tests.test_signal_mapping_gateway \
  tests.test_signal_mapping_aeb \
  tests.test_signal_mapping_perception \
  -v
```

Validated result:

```text
Ran 12 tests
OK
```

This work package therefore demonstrates that the active R2100 processes can share one mapping profile and that the physical CAN identifiers can be changed consistently without editing the Python runtime code.

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

These rules are implemented in the HIL-10 runtime rather than by modifying the legacy HIL-9 scripts.

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

Before hardware execution, run the preflight and software tests from the repository root:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
python3 -m unittest discover -s tests -p 'test_hil10*.py' -v
```

Validated result:

```text
Ran 42 tests
OK
```

The tests cover freshness, stale cached speed, duplicate/reordered replies, source timestamps, stop verification, fail-closed behavior and HIL-10 execution/adaptor logic.

A local `DIFFERS` result from `hil10_preflight.py` must be inspected before a hardware run. During HIL-10 development, the known local change to `ruskotunturi_hil_b_route_test.py` added immediate brake-state transition logging for response-time measurement.

## Standard HIL-10 three-terminal workflow

The reliable HIL-10 startup order is:

```text
1. WSL: Gazebo world
2. R2100: perception + CAN gateway + AEB
3. WSL: HIL-10 test script
```

The HIL-10 B test starts its own raw LiDAR sender, so a fourth LiDAR process is not required.

### Terminal 1 - WSL / Gazebo

Phase 6 world for route, hybrid and fault tests:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r \
  edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf
```

AEB world:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r \
  edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

### Terminal 2 - R2100

Restart all three R2100 processes between test runs because the AEB latch and test state must start cleanly.

By default all three processes load `config/ruskotunturi_hil.json`. A different mapping profile can be selected with `--config` on each process.

```bash
cd ~/EdgeDetection/edge-path-project/hil

HIL10_LOG_DIR="$HOME/hil_test_logs/hil10_$(date -u +%Y%m%dT%H%M%S)"
mkdir -p "$HIL10_LOG_DIR"

python3 -u r2100_lidar_perception.py \
  > "$HIL10_LOG_DIR/perception.log" 2>&1 &
HIL10_PERCEPTION_PID=$!

python3 -u r2100_can_gateway.py --external-perception \
  > "$HIL10_LOG_DIR/gateway.log" 2>&1 &
HIL10_GATEWAY_PID=$!

python3 -u r2100_aeb_controller.py \
  > "$HIL10_LOG_DIR/aeb.log" 2>&1 &
HIL10_AEB_PID=$!
```

For the alternate-ID validation profile, pass the same config to all three processes:

```bash
python3 -u r2100_lidar_perception.py \
  --config config/ruskotunturi_hil_alt_ids.json

python3 -u r2100_can_gateway.py \
  --config config/ruskotunturi_hil_alt_ids.json \
  --external-perception

python3 -u r2100_aeb_controller.py \
  --config config/ruskotunturi_hil_alt_ids.json
```

### Terminal 3 - WSL / HIL-10

Set the current R2100 address once:

```bash
cd ~/edge-path-project-git
read -r -p "R2100 IP address: " R2100_IP
```

Normal route:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode route \
  --r2100-host "$R2100_IP"
```

Hybrid mission:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_hybrid_test.py \
  --r2100-host "$R2100_IP"
```

Intentional AEB:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode aeb \
  --r2100-host "$R2100_IP"
```

Fault injection tests:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode fault-odometry \
  --r2100-host "$R2100_IP"

python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode fault-brake \
  --r2100-host "$R2100_IP"

python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py \
  --mode fault-lidar \
  --r2100-host "$R2100_IP"
```

The fault tests default to a shorter automatic injection delay. The validated hardware runs used `--fault-after 8.0` to allow a longer period of visually observable normal driving before fault injection.

## HIL-10 hardware validation summary

All six planned HIL-10 hardware scenarios passed on the Gazebo -> Ethernet -> R2100 -> physical CAN -> R2100 -> Ethernet -> Gazebo loop.

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

The validated fault runs ended with:

```text
HIL-10 RESULT: PASS
End reason: fault_stop_and_hold_verified
```

The `route`, `hybrid` and `aeb` runs also passed using the same HIL-10 runtime.

## Manual physical CAN interruption and recovery validation

A manual robustness test was also performed by physically interrupting the CAN0-CAN1 bench connection while Vehicle B was driving.

The interruption caused the CAN interface to enter an error state and produced a fail-safe communication timeout in the Gazebo-side controller. During the first experiment, `r2100_aeb_controller.py` terminated because SocketCAN returned `ENOBUFS` (`No buffer space available`) when the disconnected CAN transmit queue could not drain.

The AEB controller was therefore hardened so that transient CAN transmit errors (`ENOBUFS`, `EAGAIN` and `EWOULDBLOCK`) are treated as dropped CAN frames instead of fatal process errors. The brake rolling counter advances only after a successful CAN transmission.

The test was then repeated successfully:

```text
normal driving
 -> physical CAN interruption
 -> fail-safe communication/sensor timeout
 -> vehicle stop
 -> CAN bus-off/error state observed
 -> AEB, gateway and perception processes remained alive
 -> physical CAN connection restored
 -> CAN0 and CAN1 returned to ERROR-ACTIVE
 -> new route run completed without restarting the R2100 processes
 -> PASS
```

The recovery route ended with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

This manual test is additional robustness validation and is not one of the six automated HIL-10 hardware scenarios listed above.

## HIL-10 result evidence

Each HIL-10 hardware run writes a CSV trace and a matching JSON summary under:

```text
edge-path-project/gazebo/results/hil10/
```

The JSON records PASS/FAIL, end reason and scenario-specific verification data. The CSV records the time history used to evaluate freshness, commands, brake state and stop verification.

Generated HIL-10 CSV/JSON outputs are ignored by default for new runs so repeated regression tests do not clutter version control. Existing committed evidence may remain in repository history, while significant hardware-validation conclusions are recorded in this README. Keep local CSV traces when time-series analysis is needed and JSON summaries when compact run metadata is useful.

R2100 process logs should also be retained for failed or diagnostic runs.

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

The vehicle remained stopped after each injected fault.

### 5. CAN transmit saturation must not terminate the safety controller

A physical CAN interruption can prevent the SocketCAN transmit queue from draining and produce `ENOBUFS`. The AEB controller must treat this as a transient transport failure rather than terminate. After hardening the send path, the controller survived the interruption and resumed normal operation when the physical CAN connection recovered.

### 6. Shared signal mapping must be validated end to end

Moving CAN IDs into a configuration file is only useful if every active producer and consumer uses the same profile. The gateway, LiDAR perception and AEB controller therefore load the same mapping. The alternate-ID hardware run verified the configured IDs on the physical CAN bus and completed the route without Python runtime edits.

## Scope and limitations

The six automated HIL-10 fault/functional scenarios use controlled software-level injections around the real R2100 and physical CAN path. A separate manual test additionally validated physical CAN disconnection, bus-off/error-state behavior and recovery without restarting the R2100 processes.

The current validation still does **not** cover every possible hardware or system failure. Examples of remaining follow-on targets include:

- configuration-driven replacement of the current hardcoded payload scaling/codec implementation,
- arbitrary packet delay/jitter profiles,
- forced process termination,
- PC power loss,
- an external command watchdog outside the current test runtime,
- broader multi-node CAN fault cases beyond the current two-interface bench loop.

## Troubleshooting

If a test remains in a fail-safe state at startup, inspect the R2100 chain:

```bash
tail -20 "$HIL10_LOG_DIR/perception.log"
tail -20 "$HIL10_LOG_DIR/gateway.log"
tail -20 "$HIL10_LOG_DIR/aeb.log"
```

Check that:

- the correct Gazebo world is running,
- `can0` and `can1` are UP at 500 kbit/s,
- all three R2100 processes were started with the same mapping profile,
- the physical CAN H/L loop is connected,
- the R2100 IP address is reachable,
- no old HIL controller is still publishing Vehicle B commands,
- no separate LiDAR sender is running during a HIL-10 B test.

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
```

Current evidence:

```text
HIL-10 offline software validation : 42 / 42 PASS
HIL-10 hardware scenarios          : 6 / 6 PASS
signal-mapping offline tests       : 12 / 12 PASS
default mapping route regression   : PASS
alternate CAN-ID route             : PASS
alternate IDs on physical CAN      : 0x110 / 0x111 / 0x210 observed
wired Ethernet transport           : validated
physical CAN loop                  : validated
manual CAN interruption + recovery : PASS
```
