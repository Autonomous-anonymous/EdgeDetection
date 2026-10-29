# Autonomous Vehicle SIL-to-HIL Testing

This repository documents a staged autonomous-vehicle development and validation workflow using **CARLA**, **Gazebo Harmonic**, and a real embedded **Seeed Studio reComputer Industrial R2100** connected through physical CAN.

The project progresses from early perception and Automatic Emergency Braking (AEB) experiments to off-road multi-vehicle Software-in-the-Loop (SIL) validation and finally to Hardware-in-the-Loop (HIL) execution in the Ruskotunturi environment.

Current reference points:

- **Gazebo SIL baseline:** v6.4 multi-vehicle Ruskotunturi demo
- **HIL branch:** `hil-development`
- **Current HIL milestone:** HIL-9, including a full hybrid two-vehicle mission and an automated end-to-end R2100 AEB stop test

Detailed guides:

- [Gazebo SIL guide](edge-path-project/gazebo/README.md)
- [R2100 HIL guide](edge-path-project/hil/README.md)

## Project goals

The project is intended to provide a reproducible staged workflow for autonomous-vehicle function development:

1. develop ADAS and navigation logic in simulation,
2. validate perception and safety functions independently,
3. integrate navigation, obstacle handling and multi-vehicle coordination,
4. record repeatable PASS / FAIL evidence,
5. move selected safety functions onto real embedded hardware,
6. preserve the same scenario semantics across SIL and HIL where practical.

Earlier CARLA and Gazebo stages are retained intentionally as development evidence rather than treated as obsolete code.

## Development stages

### 1. CARLA perception and AEB

The first stage used CARLA for basic virtual-vehicle and ADAS development. It includes:

- RGB camera processing,
- edge detection and path prediction,
- static-obstacle AEB,
- target-speed and TTC tests,
- moving-obstacle / relative-TTC tests,
- route navigation,
- early multi-vehicle coordination experiments.

The main CARLA files are under:

```text
edge-path-project/
edge-path-project/aeb_sil/
```

### 2. Gazebo ADAS port

The controller workflow was then ported to Gazebo Harmonic. This phase validates the same general ADAS concepts with Gazebo vehicle models and virtual sensors.

Implemented Gazebo functions include:

- Automatic Emergency Braking,
- TTC-based collision avoidance,
- LiDAR obstacle avoidance,
- 360-degree LiDAR safety monitoring,
- RGB camera input,
- automated PASS / FAIL tests,
- CSV result logging.

Gazebo files are organised as:

```text
edge-path-project/gazebo/
├── models/
├── scripts/
├── worlds/
└── results/
```

### 3. Ruskotunturi off-road SIL

The Ruskotunturi terrain is the main off-road validation environment.

The staged SIL work includes:

- terrain navigation,
- static-obstacle avoidance,
- moving-obstacle interaction,
- oncoming TTC testing,
- camera + LiDAR sensor fusion,
- adaptive avoidance behaviour,
- two-vehicle coordination,
- final integrated multi-vehicle SIL validation.

The committed v6.4 baseline completed the integrated Ruskotunturi SIL mission with both vehicles completing their tasks and the required safety criteria satisfied. Detailed architecture, evidence, limitations and startup instructions are maintained in the [Gazebo SIL guide](edge-path-project/gazebo/README.md).

### 4. R2100 Hardware-in-the-Loop

The HIL phase moves the longitudinal AEB safety path onto a real reComputer Industrial R2100 while Gazebo continues to provide the virtual world and vehicle dynamics.

The current bench architecture is:

```text
Gazebo Vehicle B LiDAR --------------------+
                                           |
                                           v
                                    UDP LiDAR transport
                                           |
                                           v
                                  R2100 LiDAR perception
                                           |
                                      CAN 0x101
                                           |
                                           v
Gazebo B speed -> UDP -> R2100 gateway -> CAN0
                                           |
                                      physical CAN
                                           |
                                          CAN1
                                           |
                                           v
                                  R2100 AEB controller
                                           |
                                      CAN 0x200
                                           |
                                           v
                                  R2100 CAN gateway
                                           |
                                           v
                                         UDP
                                           |
                                           v
                                  Gazebo B brake override
```

The current HIL setup uses classic 11-bit CAN at **500 kbit/s** with CAN0 and CAN1 physically connected on the bench.

Vehicle B retains SIL route steering while its longitudinal AEB decision is evaluated on the R2100. This separation keeps the HIL boundary explicit and makes the embedded safety path independently testable.

See the [R2100 HIL guide](edge-path-project/hil/README.md) for the CAN protocol, three-terminal startup sequence, troubleshooting and validation evidence.

## HIL-9 validation

HIL-9 extends the verified Ruskotunturi Phase 6 scenario into the embedded HIL setup.

| Stage | Purpose | Result |
| --- | --- | --- |
| HIL-9A | Ruskotunturi Phase 6 SIL baseline | PASS |
| HIL-9B | Vehicle B 721-beam LiDAR to R2100 perception | PASS |
| HIL-9C | Vehicle B route with R2100 longitudinal AEB | PASS |
| HIL-9D | Full hybrid two-vehicle mission | PASS |
| HIL-9E | Automated intentional end-to-end AEB intervention | PASS |

The HIL-9D hybrid mission retained the normal Phase 6 behaviour: both vehicles reached their goals, the conflict-zone coordinator performed one wait/release cycle, no conflict overlap occurred, and Vehicle B was verified stopped at mission completion.

HIL-9E deliberately placed a static obstacle on Vehicle B's route. The embedded R2100 AEB triggered on TTC, Vehicle B stopped at 0.000 m/s, and the automated test measured **1.160 m body-to-obstacle clearance** against a **1.000 m requirement**.

Two integration issues found during HIL-9 are now part of the implementation:

- the 721-beam LiDAR scan is chunked into MTU-safe UDP packets before R2100 reassembly,
- obstacle distance sent on CAN is corrected from raw LiDAR-origin range to physical vehicle-body clearance.

The HIL guide documents both findings and their rationale.

## CAN protocol

The current HIL protocol is intentionally small:

```text
0x100 VEHICLE_SPEED
0x101 OBSTACLE_DISTANCE
0x200 BRAKE_COMMAND
```

`OBSTACLE_DISTANCE` represents body-to-obstacle clearance for the current Ruskotunturi HIL perception path. A stale LiDAR does not produce a synthetic "no obstacle" value; missing distance updates cause the embedded AEB controller to enter its sensor-timeout fail-safe brake state.

The exact encoding is defined in:

```text
edge-path-project/hil/can_protocol.py
```

## Repository structure

```text
EdgeDetection/
├── README.md
├── tests/
└── edge-path-project/
    ├── aeb_sil/                 # CARLA SIL controllers and scenarios
    ├── results/                 # CARLA / early SIL result evidence
    ├── gazebo/
    │   ├── README.md            # Current Gazebo SIL guide
    │   ├── models/              # Gazebo vehicles and Ruskotunturi model
    │   ├── scripts/             # SIL and HIL scenario controllers/tests
    │   ├── worlds/              # Gazebo test environments
    │   └── results/             # Gazebo / Ruskotunturi validation results
    ├── hil/
    │   ├── README.md            # R2100 HIL architecture and operating guide
    │   ├── can_protocol.py
    │   ├── gazebo_lidar_sender.py
    │   ├── r2100_can_gateway.py
    │   ├── r2100_lidar_perception.py
    │   └── r2100_aeb_controller.py
    ├── carla_*.py               # Earlier CARLA experiments
    ├── main.py
    └── environment.yml
```

## Running the current demonstrations

For the final Gazebo-only SIL baseline, follow:

```text
edge-path-project/gazebo/README.md
```

For the current R2100 HIL setup and HIL-9 tests, follow:

```text
edge-path-project/hil/README.md
```

The HIL guide uses a repeatable three-terminal startup order:

```text
1. WSL: Gazebo world
2. R2100: perception + gateway + AEB
3. WSL: LiDAR sender, then mission/test script
```

## Testing philosophy

The project uses staged validation rather than moving directly to a fully integrated scenario.

Functions are tested independently before integration, including:

- AEB and TTC logic,
- LiDAR perception,
- obstacle avoidance,
- camera + LiDAR fusion,
- multi-vehicle coordination,
- physical CAN transport,
- embedded fail-safe behaviour,
- normal hybrid mission execution,
- deliberate end-to-end AEB intervention.

This keeps failures diagnosable and provides evidence for each transition from SIL toward HIL.

## Results and evidence

CSV result files are committed intentionally when they represent useful validation evidence. They should not be globally ignored because they document controller behaviour at different development stages.

Primary result locations are:

```text
edge-path-project/results/          # CARLA / early SIL
edge-path-project/gazebo/results/   # Gazebo / Ruskotunturi SIL
```

HIL-9 console PASS criteria, architecture and validated measurements are documented in `edge-path-project/hil/README.md`.

## Current status

**CARLA development:** retained and documented  
**Gazebo ADAS SIL:** implemented  
**Ruskotunturi multi-vehicle SIL:** PASS baseline available  
**R2100 + physical CAN HIL:** implemented  
**Ruskotunturi HIL-9A ... HIL-9E:** PASS  

The project has therefore progressed beyond HIL preparation: a real embedded controller and physical CAN link are now part of the validated Ruskotunturi test loop.
