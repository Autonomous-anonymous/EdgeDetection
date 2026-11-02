# Autonomous Vehicle SIL-to-HIL Testing

This repository documents a staged autonomous-vehicle development and validation workflow using **CARLA**, **Gazebo Harmonic**, and a real embedded **Seeed Studio reComputer Industrial R2100** connected through physical CAN.

The project progresses from perception and ADAS experiments to off-road multi-vehicle Software-in-the-Loop (SIL) validation and Hardware-in-the-Loop (HIL) execution in the Ruskotunturi environment.

## Current reference points

- **Gazebo SIL baseline:** v6.4 multi-vehicle Ruskotunturi demo
- **HIL platform:** Ruskotunturi R2100 HIL
- **Historical functional validation suite:** HIL-10 Freshness and Fault Validation
- **Common SIL/HIL scenario:** `emergency_braking`
- **Independent HIL command watchdog:** process-termination fail-safe validated
- **SIL/HIL AEB threshold consistency:** 5 / 5 PASS
- **Verified baseline tag:** `r2100-hil-verified-2026-11-02`
- **Development branch:** `hil-development`
- **Current offline repository suite:** 168 / 168 PASS (2026-11-02)

Detailed documentation:

- [Gazebo SIL guide](edge-path-project/gazebo/README.md)
- [Short two-vehicle Gazebo pitch with optional live R2100/CAN](edge-path-project/gazebo/PITCH_DEMO.md)
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
├── scenarios/
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

### 5. Common SIL/HIL emergency-braking scenario

The repository includes one shared staged scenario definition:

```text
edge-path-project/gazebo/scenarios/emergency_braking.json
```

It uses the same Gazebo world, Vehicle B route, static obstacle and acceptance criteria for both targets. The target-selecting runner is:

```text
edge-path-project/gazebo/scripts/run_staged_scenario.py
```

The shared acceptance criteria require the vehicle to move before braking, use a valid AEB reason (`emergency_distance` or `ttc_threshold`), stop below 0.05 m/s for three confirmation samples and retain at least 1.00 m ground-truth clearance.

With the world running, the SIL target is selected with:

```bash
python3 edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target sil
```

With the same world and the R2100 HIL runtime ready, the HIL target is selected with:

```bash
python3 edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target hil \
  --r2100-host <R2100_IP>
```

Both targets passed on 2026-11-02 with `end_reason=aeb_stop_verified` and AEB reason `ttc_threshold`. The SIL run stopped with 1.253 m ground-truth clearance. The HIL run exercised raw Gazebo LiDAR transport, R2100 perception, physical CAN, the embedded AEB controller and the returned brake state before stopping successfully.

The successful HIL run now has committed raw evidence:

```text
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.json
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.csv
```

The common runner selects and validates the target-specific backend but intentionally does not yet launch Gazebo or start the R2100 runtime itself; those lifecycle steps remain explicit.

### 6. Independent HIL process-termination fail-safe

An independent WSL-side command watchdog protects Vehicle B against loss of the HIL command-producing process:

```text
edge-path-project/gazebo/scripts/hil_command_watchdog.py
edge-path-project/gazebo/scripts/hil_process_termination_test.py
```

The watchdog arms only after it has observed the normal Vehicle B command stream. If command updates then stop for 0.30 s, it latches a fail-safe STOP and continuously publishes zero speed and steering commands. The latch is not automatically released during the same watchdog run.

The automated process-termination harness starts the watchdog and HIL route runner, waits until Vehicle B is moving, kills the HIL runner with `SIGKILL`, verifies that the watchdog publishes a post-kill zero command, and verifies the physical simulated vehicle stop from new odometry measurements.

The final committed 2026-11-02 evidence run produced:

```text
Vehicle B before fault        0.850 m/s
HIL runner termination        SIGKILL
watchdog STOP latency         0.261 s after SIGKILL
final observed speed          ~0.000 m/s
stop confirmation samples     17
stop-and-hold verification    1.616 s after SIGKILL
source commit                 269a1c4feb5ce91a5174f25243f21be71cce959f
source dirty                  false
result                        PASS
```

The committed machine-readable result is:

```text
edge-path-project/gazebo/results/hil10/hil_process_termination_20261102T115256Z.json
```

This is stronger than a normal controller cleanup test because `SIGKILL` prevents the killed HIL runner from executing its own final STOP cleanup. The independent watchdog therefore provides the post-failure Vehicle B command that is explicitly verified by the test harness.

With the Phase 6 Gazebo world running and the R2100 runtime ready, the automated test is:

```bash
python3 -u edge-path-project/gazebo/scripts/hil_process_termination_test.py \
  --r2100-host <R2100_IP>
```

### 7. SIL/HIL AEB threshold consistency

The SIL emergency-braking backend and the embedded R2100 AEB controller keep separate runtime constants so the validated embedded runtime does not depend on Gazebo code. A regression test now prevents those copies from drifting silently:

```text
tests/test_aeb_threshold_consistency.py
```

It verifies equality of the TTC threshold, emergency distance, sensor timeout, control period and minimum TTC speed. The dedicated result is **5 / 5 PASS**.

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
- WSL-side HIL startup-readiness diagnostics,
- a common SIL/HIL emergency-braking scenario and target-selecting runner,
- an independent HIL command watchdog with automated `SIGKILL` process-termination validation,
- SIL/HIL AEB threshold-consistency regression coverage.

The current full offline repository suite passes:

```text
Ran 168 tests
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
    │   ├── scenarios/           # Shared staged SIL/HIL scenario definitions
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

For the current Gazebo SIL baseline and the shared emergency-braking scenario, follow:

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

## Scope boundary

The current deliverable covers the staged **SIL -> HIL** workflow and embedded AEB validation. A Vehicle-in-the-Loop (VIL) stage is **not implemented and is explicitly out of scope for the current project deliverable**. The repository does not claim a completed VIL implementation.

## Current status

**CARLA development:** retained as development history  
**Gazebo ADAS SIL:** implemented  
**Ruskotunturi multi-vehicle SIL v6.4:** PASS  
**R2100 + physical CAN HIL:** implemented and hardware-validated  
**HIL-10 freshness/fault hardware scenarios:** 6 / 6 PASS  
**Configurable mapping / codec / deployment / preflight:** implemented and validated  
**HIL startup readiness diagnostics:** implemented and validated  
**Verified R2100 baseline tag:** `r2100-hil-verified-2026-11-02`  
**Common SIL/HIL emergency-braking scenario:** SIL PASS / HIL PASS  
**Independent HIL process-termination fail-safe:** PASS  
**SIL/HIL AEB threshold consistency:** 5 / 5 PASS  
**Full offline repository suite:** 168 / 168 PASS  
**VIL stage:** out of scope / not implemented  

The current embedded AEB scope is complete. Remaining work is optional refinement such as unified SIL/HIL result formatting, additional lifecycle automation or a second shared SIL/HIL scenario; full local path planning is not required for the validated scope.
