# Autonomous Vehicle SIL-to-HIL Testing

This repository documents a staged autonomous-vehicle development and validation workflow using **CARLA**, **Gazebo Harmonic**, and a real embedded **Seeed Studio reComputer Industrial R2100** connected through physical CAN.

The project progresses from perception and ADAS experiments to off-road multi-vehicle Software-in-the-Loop (SIL) validation and Hardware-in-the-Loop (HIL) execution in the Ruskotunturi environment.

## Current reference points

- **Gazebo SIL baseline:** v6.4 multi-vehicle Ruskotunturi demo
- **HIL platform:** Ruskotunturi R2100 HIL
- **Historical functional validation suite:** HIL-10 Freshness and Fault Validation
- **Development branch:** `hil-development`
- **Current offline repository suite:** 152 / 152 PASS (2026-10-30)

Detailed documentation:

- [Gazebo SIL guide](edge-path-project/gazebo/README.md)
- [R2100 HIL guide](edge-path-project/hil/README.md)
- [Validation and evidence index](VALIDATION.md)

## Project goals

The project provides a reproducible staged workflow for autonomous-vehicle function development:

1. develop ADAS and navigation logic in simulation,
2. validate perception and safety functions independently,
3. integrate navigation, obstacle handling and multi-vehicle coordination,
4. record repeatable PASS / FAIL evidence,
5. move selected safety functions onto real embedded hardware,
6. preserve scenario and signal semantics across SIL and HIL where practical.

Earlier CARLA, Gazebo and HIL stages are retained intentionally as development evidence rather than treated as disposable code.

## Development stages

### 1. CARLA perception and AEB

The first stage used CARLA for basic virtual-vehicle and ADAS development, including RGB camera processing, edge detection/path prediction, static-obstacle AEB, TTC tests, moving-obstacle experiments, route navigation and early multi-vehicle coordination.

The main CARLA files are under:

```text
edge-path-project/
edge-path-project/aeb_sil/
```

### 2. Gazebo ADAS development

The workflow was ported to Gazebo Harmonic and expanded with:

- Automatic Emergency Braking,
- TTC-based collision handling,
- LiDAR obstacle avoidance,
- 360-degree LiDAR safety monitoring,
- RGB camera input and camera + LiDAR fusion work,
- automated PASS / FAIL checks,
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

The current committed SIL baseline is **v6.4**. It demonstrates a two-vehicle Ruskotunturi mission with static-obstacle avoidance, moving-obstacle interaction, camera-assisted control, coordination and a common loading-zone objective.

Primary committed evidence:

```text
edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv
edge-path-project/gazebo/results/final_sil_v6_4_trace_20261026T133025_782767.csv
```

The v6.4 result records `PASS` with `end_reason=final_demo_complete` and no failed checks.

See the [Gazebo SIL guide](edge-path-project/gazebo/README.md) for architecture, validation details and startup instructions.

### 4. Ruskotunturi R2100 HIL

The HIL phase moves the longitudinal AEB safety path onto a real reComputer Industrial R2100 while Gazebo continues to provide the virtual world and vehicle dynamics.

The current bench architecture is:

```text
Gazebo Vehicle B LiDAR
        |
        v
UDP LiDAR transport
        |
        v
R2100 LiDAR perception
        |
physical CAN
        |
R2100 AEB controller
        |
physical CAN
        |
R2100 CAN gateway
        |
        v
UDP brake state
        |
        v
Gazebo Vehicle B brake override
```

Vehicle speed also travels from Gazebo to the R2100 gateway and onto physical CAN. The bench uses classic 11-bit CAN at **500 kbit/s** with CAN0 and CAN1 physically connected.

Vehicle B retains SIL route steering while its longitudinal AEB decision is evaluated on the R2100. The current embedded AEB safety path is LiDAR-based; camera + LiDAR fusion is demonstrated on the SIL side rather than claimed as part of the current embedded HIL decision path.

## HIL validation progression

### HIL-9 development history

HIL-9 established the Ruskotunturi hybrid architecture:

| Stage | Purpose | Result |
| --- | --- | --- |
| HIL-9A | Ruskotunturi Phase 6 SIL baseline | PASS |
| HIL-9B | Vehicle B raw LiDAR to R2100 perception | PASS |
| HIL-9C | Vehicle B route with R2100 longitudinal AEB | PASS |
| HIL-9D | Full hybrid two-vehicle mission | PASS |
| HIL-9E | Intentional end-to-end R2100 AEB intervention | PASS |

HIL-9 remains useful development history, but it is not the current HIL reference point.

### HIL-10 Freshness and Fault Validation

HIL-10 added explicit freshness and fail-safe validation for odometry, returned brake state and LiDAR-related sensor timeout behaviour. The focused HIL-10 offline suite passed **42 / 42**, and all six planned hardware scenarios passed.

Later R2100 HIL work packages added:

- configurable signal mapping,
- configuration-driven scalar signal codecs,
- repeatable embedded zipapp deployment,
- R2100 deployment preflight and fail-closed startup,
- WSL-side HIL startup-readiness diagnostics.

The current full offline repository suite passes:

```text
Ran 152 tests
OK
```

See [VALIDATION.md](VALIDATION.md) for the current evidence catalogue and the distinction between historical and current preflight checks.

## CAN and signal configuration

The default HIL signal set is:

```text
0x100 VEHICLE_SPEED
0x101 OBSTACLE_DISTANCE
0x200 BRAKE_COMMAND
```

CAN IDs, interfaces and scalar scaling are no longer treated as fixed runtime constants. The R2100 runtime loads shared JSON signal-mapping profiles, and alternate CAN-ID and alternate-scaling profiles have been validated on the physical CAN bench.

`OBSTACLE_DISTANCE` represents physical vehicle-body clearance in the current Ruskotunturi HIL perception path.

## Deployment

The current R2100 runtime is built as a Python zipapp plus external configuration and control scripts. It supports versioned SSH/SCP deployment, configuration validation, target-side preflight, managed start/status/stop operation and rollback.

The deployed runtime intentionally requires `python3` on the R2100; the project does not currently claim a Python-free native executable.

Typical target control is:

```bash
~/r2100_hil/current/r2100_hil.pyz preflight
~/r2100_hil/current/start_hil.sh
~/r2100_hil/current/status_hil.sh
~/r2100_hil/current/stop_hil.sh
```

## Repository structure

```text
EdgeDetection/
├── README.md
├── VALIDATION.md
├── tests/
└── edge-path-project/
    ├── aeb_sil/                 # CARLA SIL controllers and scenarios
    ├── results/                 # CARLA / early SIL evidence
    ├── gazebo/
    │   ├── README.md            # Current Gazebo SIL guide
    │   ├── models/
    │   ├── scripts/
    │   ├── worlds/
    │   └── results/
    └── hil/
        ├── README.md            # Detailed R2100 HIL guide
        ├── config/
        ├── deployment/
        ├── signal_mapping.py
        ├── signal_codec.py
        ├── r2100_preflight.py
        ├── r2100_can_gateway.py
        ├── r2100_lidar_perception.py
        └── r2100_aeb_controller.py
```

## Running current validation

Run the complete offline repository suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

For the current Gazebo SIL baseline, follow:

```text
edge-path-project/gazebo/README.md
```

For the current R2100 HIL setup, deployment and HIL-10-based validation workflow, follow:

```text
edge-path-project/hil/README.md
```

The filename `edge-path-project/gazebo/scripts/hil10_preflight.py` is historical. It checks integrity against the original HIL-9-derived dependency snapshot and is **not** the current R2100 readiness check. Current target readiness is checked with the deployed `r2100_hil.pyz preflight`; WSL-side startup conditions are reported by the HIL startup diagnostics.

## Evidence policy

Validation results are retained when they provide useful development or acceptance evidence. Failed or interrupted runs may be kept deliberately as development history and must not be presented as current PASS evidence.

The cross-project evidence catalogue in [VALIDATION.md](VALIDATION.md) identifies the current baseline artifacts and any known evidence gaps.

## Current status

**CARLA development:** retained as development history  
**Gazebo ADAS SIL:** implemented  
**Ruskotunturi multi-vehicle SIL v6.4:** PASS  
**R2100 + physical CAN HIL:** implemented and hardware-validated  
**HIL-10 freshness/fault hardware scenarios:** 6 / 6 PASS  
**Configurable mapping / codec / deployment / preflight:** implemented and validated  
**HIL startup readiness diagnostics:** implemented and validated  
**Full offline repository suite:** 152 / 152 PASS  

The next project work should build on this verified baseline rather than reorganising the existing `gazebo/`, `hil/` and `tests/` structure.
