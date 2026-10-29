# Hardware-in-the-Loop (HIL) validation

This directory contains the Hardware-in-the-Loop implementation used to extend the Gazebo SIL baseline onto a real embedded controller and physical CAN link.

The current embedded platform is a **Seeed Studio reComputer Industrial R2100**. Gazebo Harmonic runs in WSL and provides the virtual vehicles, terrain, LiDAR and odometry. The R2100 receives simulation data over UDP, maps the relevant signals onto physical CAN, executes the AEB decision, and returns the brake state to the simulator.

The current validated Ruskotunturi milestone is **HIL-9**.

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
- Vehicle A remains SIL-controlled in the full hybrid HIL-9D mission.

## Main files

```text
edge-path-project/hil/
├── can_protocol.py
├── gazebo_hil_bridge.py
├── gazebo_lidar_sender.py
├── r2100_aeb_controller.py
├── r2100_can_gateway.py
├── r2100_lidar_perception.py
└── README.md
```

Ruskotunturi HIL validation scripts are under:

```text
edge-path-project/gazebo/scripts/
├── ruskotunturi_hil_b_route_test.py
├── ruskotunturi_phase6_hybrid_hil_test.py
└── ruskotunturi_hil9e_aeb_test.py
```

The dedicated HIL-9E obstacle world is:

```text
edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

## CAN protocol

The current protocol is defined in `can_protocol.py`.

| CAN ID | Signal | Encoding |
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

## Important distance semantics

`0x101 OBSTACLE_DISTANCE` is treated as **free clearance from the physical vehicle body to the obstacle**, not raw LiDAR-origin range.

Vehicle B uses a roof-centred 360-degree LiDAR. The body model used by the HIL perception is:

```text
vehicle half length : 1.30 m
vehicle half width  : 0.60 m
front LiDAR sector  : +/-10 deg
```

For each valid ray in the front sector, the R2100 perception calculates the vehicle-body radius in that direction and subtracts it from the LiDAR range before transmitting `0x101`.

This correction is safety-critical for the current geometry. Using raw LiDAR-origin range caused the TTC brake to trigger too late because the LiDAR is approximately 1.3 m behind the front edge of the vehicle body.

## LiDAR UDP transport

Vehicle B produces a 360-degree scan with **721 beams**. A legacy full scan was about 1468 bytes and was not reliable over the tested WSL/network path.

The LiDAR transport therefore supports two packet formats:

```text
HILR = legacy complete scan
HILC = chunked scan
```

`gazebo_lidar_sender.py` keeps the legacy format for scans that fit within the configured safe UDP payload and automatically chunks larger scans. The current maximum UDP payload is 900 bytes.

For the 721-beam Ruskotunturi scan this normally produces two UDP packets per scan. `r2100_lidar_perception.py` reassembles the chunks before evaluating the front sector.

If LiDAR updates become stale, the perception process deliberately stops transmitting `0x101`. The AEB controller then detects stale sensor data and enters the `sensor_timeout` fail-safe brake state.

## CAN interface setup

On the R2100, configure both CAN interfaces at 500 kbit/s before starting the HIL processes:

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

## Standard three-terminal startup order

The most reliable workflow uses only three terminals and always starts the components in this order:

```text
1. WSL: Gazebo world
2. R2100: perception + CAN gateway + AEB
3. WSL: LiDAR sender
4. WSL: mission/test script last
```

Starting the mission last prevents the vehicle from moving before the HIL safety path is ready.

Use the current R2100 address in place of `<R2100_IP>`.

### Terminal 1 - WSL / Gazebo

For the HIL-9D Phase 6 hybrid mission:

```bash
cd ~/edge-path-project-git

export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r \
  edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf
```

For the HIL-9E AEB stop test use the dedicated world instead:

```bash
gz sim -r \
  edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

### Terminal 2 - R2100

```bash
cd ~/EdgeDetection/edge-path-project/hil
mkdir -p ~/hil_test_logs

pkill -f r2100_lidar_perception.py 2>/dev/null
pkill -f r2100_can_gateway.py 2>/dev/null
pkill -f r2100_aeb_controller.py 2>/dev/null

python3 -u r2100_lidar_perception.py \
  > ~/hil_test_logs/perception.log 2>&1 &

python3 -u r2100_can_gateway.py --external-perception \
  > ~/hil_test_logs/gateway.log 2>&1 &

python3 -u r2100_aeb_controller.py \
  > ~/hil_test_logs/aeb.log 2>&1 &
```

### Terminal 3 - WSL / LiDAR sender

```bash
cd ~/edge-path-project-git/edge-path-project/hil

pkill -f gazebo_lidar_sender.py 2>/dev/null

python3 -u gazebo_lidar_sender.py \
  --r2100-host <R2100_IP> \
  --lidar-topic /aeb360b/lidar \
  > ~/hil_test_logs/lidar_sender.log 2>&1 &
```

Before starting the mission, verify on the R2100 that complete 721-beam scans are arriving:

```bash
tail -5 ~/hil_test_logs/perception.log
```

Expected output contains values similar to:

```text
RX lidar | ... | beams=721 | ... | CAN=0x101 | ... | can_tx_drops=0
```

## HIL-9C - Vehicle B standalone route test

This isolates Vehicle B. Steering is retained from the Phase 6 SIL route controller while longitudinal AEB is handled through the R2100 and physical CAN.

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo/scripts

python3 -u ruskotunturi_hil_b_route_test.py \
  --r2100-host <R2100_IP>
```

Vehicle A is not commanded by this script.

The validated normal-route run reached the B goal with no R2100 AEB events.

## HIL-9D - Full hybrid two-vehicle mission

HIL-9D subclasses the verified Phase 6 two-vehicle coordinator rather than reimplementing the SIL scenario.

Vehicle A remains under the Phase 6 SIL controller. Vehicle B keeps Phase 6 route steering, but its longitudinal safety path is routed through the R2100:

```text
B odometry speed -> UDP -> gateway -> CAN 0x100
B LiDAR -> chunked UDP -> R2100 perception -> CAN 0x101
CAN 0x100 + 0x101 -> R2100 AEB -> CAN 0x200
CAN 0x200 -> gateway -> UDP -> WSL -> B command override
```

Run:

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo/scripts

python3 -u ruskotunturi_phase6_hybrid_hil_test.py \
  --r2100-host <R2100_IP>
```

Validated HIL-9D result:

```text
Vehicle A goal reached   : YES
Vehicle B goal reached   : YES
Coordinator wait events  : 1
Coordinator releases     : 1
Conflict overlap         : NO
Vehicle A min clearance  : 4.980 m
Vehicle B min clearance  : 6.022 m
Required clearance       : 1.000 m
Terrain safety violation : NO
LiDAR safety violation   : NO
End reason               : both_goals_reached
Duration                 : 39.26 s
RESULT                    : PASS

Vehicle B final speed    : 0.046 m/s
Vehicle B stop verified  : YES
R2100 AEB overrides      : 0
HIL-9D RESULT            : PASS
```

The zero AEB override count is expected for the normal mission: HIL-9D verifies that the hybrid architecture does not interfere with a safe Phase 6 mission. Intentional AEB intervention is verified separately by HIL-9E.

## HIL-9E - Automated intentional AEB stop test

HIL-9E places a static obstacle directly on Vehicle B's route and validates an end-to-end R2100 AEB intervention.

The world is:

```text
edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

The obstacle is centred at:

```text
(-3.0, 32.25)
```

Run the automated wrapper after the standard HIL processes and LiDAR sender are active:

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo/scripts

python3 -u ruskotunturi_hil9e_aeb_test.py \
  --r2100-host <R2100_IP>
```

PASS requires all of the following:

- Vehicle B first enters a normal DRIVE state.
- A valid R2100 AEB reason is observed (`ttc_threshold` or `emergency_distance`).
- Vehicle speed falls to at most 0.05 m/s for three consecutive samples.
- Conservative body-to-obstacle clearance at the confirmed stop is at least 1.00 m.

Validated HIL-9E result:

```text
Vehicle moved             : YES
Valid R2100 AEB trigger   : YES
AEB reason                : ttc_threshold
AEB trigger s             : 2.740 m
Confirmed stop s          : 2.740 m
Final measured speed      : 0.000 m/s
Body-to-obstacle clearance: 1.160 m
Required clearance        : 1.000 m

HIL-9E RESULT             : PASS
```

This test verifies the complete intervention path:

```text
Gazebo 360 LiDAR
 -> chunked UDP
 -> R2100 perception
 -> physical CAN 0x101
 -> R2100 AEB
 -> physical CAN 0x200
 -> R2100 gateway
 -> UDP
 -> Gazebo Vehicle B command override
 -> vehicle stops before obstacle
```

## HIL-9 validation summary

| Stage | Purpose | Result |
| --- | --- | --- |
| HIL-9A | Ruskotunturi Phase 6 SIL baseline | PASS |
| HIL-9B | Vehicle B raw 721-beam LiDAR to R2100 perception | PASS |
| HIL-9C | Vehicle B route steering with R2100 longitudinal AEB | PASS |
| HIL-9D | Full hybrid two-vehicle Phase 6 mission | PASS |
| HIL-9E | Intentional automated R2100 AEB intervention | PASS |

HIL-9A baseline evidence included both vehicles reaching their goals, one coordinator wait/release event, no conflict overlap, minimum clearances above 4.9 m, and no terrain or LiDAR safety violation.

## Key integration findings

### 1. UDP MTU / scan size

A 721-beam raw LiDAR scan exceeded the reliable UDP payload size on the tested WSL-to-R2100 path. Small packets arrived while the approximately 1468-byte legacy scan did not. Chunking the scan into payloads no larger than 900 bytes resolved the issue while retaining legacy single-packet support for smaller scans.

### 2. Sensor-origin distance is not body clearance

The first intentional Ruskotunturi AEB test used raw LiDAR-origin range as `0x101`. The AEB decision itself triggered correctly, but because the LiDAR is roof-centred, the front of the 2.6 m vehicle body was already approximately 1.3 m closer to the obstacle than the reported sensor range implied.

The R2100 perception was therefore changed to transmit body-to-obstacle clearance. After this correction, the automated HIL-9E test stopped Vehicle B with **1.160 m** conservative clearance against a **1.000 m** requirement.

### 3. Fail-safe sensor timeout

When LiDAR becomes stale, `r2100_lidar_perception.py` does not replace the missing scan with a synthetic "no obstacle" value. It stops updating `0x101`, allowing `r2100_aeb_controller.py` to detect stale sensor data and command the `sensor_timeout` brake state.

## Troubleshooting

If the route script remains in `BRAKE | reason=sensor_timeout`, check the chain in this order:

```bash
# R2100
tail -10 ~/hil_test_logs/perception.log
tail -10 ~/hil_test_logs/gateway.log
tail -15 ~/hil_test_logs/aeb.log
```

The perception log should show 721-beam scans and `can_tx_drops=0`.

If no LiDAR scans arrive:

- verify the Gazebo world is running,
- verify `/aeb360b/lidar` is the selected topic,
- verify `gazebo_lidar_sender.py` is running,
- verify the R2100 address,
- verify UDP port 15010 is reachable.

If CAN transmission fails:

- verify `can0` and `can1` are UP,
- verify both are configured for 500 kbit/s,
- verify the physical H/L loop wiring,
- inspect CAN error counters with `ip -details link show can0` and `can1`.

Only one process should publish Vehicle B commands during a test. Stop old scenario/controller processes before starting a new run.

## Current status

The repository now contains a verified progression from SIL to embedded HIL:

```text
CARLA experiments
 -> Gazebo ADAS SIL
 -> Ruskotunturi multi-vehicle SIL
 -> physical CAN HIL
 -> Ruskotunturi hybrid HIL-9
```

HIL-9 demonstrates both a normal two-vehicle hybrid mission and a deliberately triggered end-to-end embedded AEB intervention with an automated physical-clearance PASS/FAIL criterion.
