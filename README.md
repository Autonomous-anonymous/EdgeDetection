# CAMINO — Gazebo SIL and R2100 Hardware-in-the-Loop

This `camino` branch is a focused, runnable delivery of the **CAMINO Software-in-the-Loop (SIL) → Hardware-in-the-Loop (HIL)** workflow. It contains **Gazebo Harmonic** vehicle simulations, Ruskotunturi worlds and models, scenario runners, recorded SIL/HIL evidence, and the **Seeed Studio reComputer Industrial R2100** controller with physical CAN.

This branch intentionally excludes the earlier CARLA prototype, its standalone scripts and CSV results, and its input/output videos. Those assets remain on the original repository branches. Gazebo result CSV/JSON/trace files are retained as SIL/HIL evidence, including historical failed/diagnostic runs identified as such.

## Project scope and current baselines

| Area | Current implementation and evidence |
| --- | --- |
| Gazebo SIL | Ruskotunturi final two-vehicle **v6.4** demo: **PASS**, `final_demo_complete` |
| Obstacle handling in SIL | Camera/LiDAR static avoidance, moving-obstacle hold-and-resume, merge coordination and loading-zone arrival |
| Shared SIL/HIL AEB | One `emergency_braking` scenario and target-selecting runner; SIL and HIL **PASS** |
| R2100 physical HIL | Perception → physical CAN → embedded AEB → returned brake state |
| Native embedded release | Debian 12 ARM64 CI-built ELF, verified hashes and physical R2100 `check` / `preflight` / runtime startup |
| Native functional acceptance | **2026-11-05 PASS:** `aeb_stop_verified`, `ttc_threshold`, 1.236 m clearance, four stop-confirmation samples, no fault |
| Regression coverage | 190/190 offline Python tests previously PASS; CI also checks ARM64 packaging and deployment design |
| Boundary | **SIL → HIL only**; VIL is not implemented or claimed |

The final SIL demo's moving obstacle is managed by **path-clearance-based LiDAR hold/resume**; it does not use the shared scenario's TTC-based AEB policy for that moving obstacle. The TTC-based emergency brake is demonstrated by the separate shared `emergency_braking` scenario.

## JARCRAC v2 — experimental terrain perception (2026-11-06)

The independent Mevea-derived JARCRAC Gazebo model has operator-reported **flat-ground AEB PASS** results at **0.35, 0.70 and 1.00 m/s**, plus a **positive chassis–obstacle contact-sensor control**. Subsequent *read-only* Ruskotunturi diagnostics demonstrated LiDAR-only and terrain-map-assisted classification, sensitivity to localization and attitude errors, and four simulated **32-layer LiDAR + IMU** calibration controls. The map-assisted method uses Gazebo ground-truth position; the IMU tests used ideal simulated sensing.

The experiments are **not JARCRAC-specific R2100 HIL**, validated localization, dynamic forest-terrain braking or a safety case. See the [JARCRAC v2 SIL guide](JARCRAC_V2_SIL.md) for measurements, source-file locations and limitations. Experimental source files and additional JSON evidence produced in the local WSL checkout must be committed separately from this documentation update.

## Repository layout

```text
.
├── README.md                         # CAMINO scope and quick start
├── REQUIREMENTS.md                   # CAMINO/course requirements traceability
├── VALIDATION.md                     # Evidence catalogue and test history
├── .github/workflows/                # Offline and Debian 12 ARM64 CI
├── tests/                            # SIL, HIL, mapping and deployment tests
└── edge-path-project/
    ├── gazebo/
    │   ├── README.md                 # Final SIL architecture and instructions
    │   ├── models/                   # Vehicles and Ruskotunturi terrain
    │   ├── worlds/                   # Gazebo SIL/HIL scenarios
    │   ├── scenarios/                # Shared emergency-braking definition
    │   ├── scripts/                  # Gazebo simulations and HIL runners
    │   └── results/                  # Versioned Gazebo SIL/HIL evidence
    └── hil/
        ├── README.md                 # R2100 physical-CAN HIL and deployment
        ├── NATIVE_DEPLOYMENT_ACCEPTANCE.md
        ├── config/                    # CAN/sensor mapping profiles
        ├── deployment/                # Native build and managed runtime scripts
        ├── signal_mapping.py
        ├── signal_codec.py
        ├── r2100_lidar_perception.py
        ├── r2100_can_gateway.py
        └── r2100_aeb_controller.py
```

## Prerequisites

- Linux/WSL Ubuntu with **Gazebo Harmonic** and its Python `gz` transport/message bindings available to the Python interpreter used for simulation.
- Python for controller scripts and offline tests. The complete offline suite also requires the pinned **PyInstaller** build dependency.
- **For physical HIL only:** network access to the R2100, a Debian 12 ARM64 standalone release, CAN0/CAN1 physical wiring and the expected 500 kbit/s CAN configuration.
- Keep the Gazebo and R2100 runtime sessions separate; do not run multiple controllers publishing to the same Gazebo topics.

The standalone R2100 executable does **not** require the development Python virtual environment on the embedded device. Build it in Debian 12 **ARM64** (as in GitHub Actions), not in x86-64 WSL. The CI release contains a provenance file recording its source commit and artifact SHA-256 digests.

## Run the final Gazebo SIL v6.4 demo

From **WSL terminal 1** (with the system Python/Gazebo environment):

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_final_sil_demo_v6_world.sdf
```

From **WSL terminal 2**:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/ruskotunturi_final_sil_demo_v6.py
```

Start with a fresh Gazebo world each run; do not use the world reset feature while a prior controller or watchdog is still active. Consult the [Gazebo SIL guide](edge-path-project/gazebo/README.md) for state-machine behaviour, metrics and reset limitations.

**Committed SIL v6.4 PASS evidence:**

- [Final result CSV](edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv)
- [Matching trace](edge-path-project/gazebo/results/final_sil_v6_4_trace_20261026T133025_782767.csv)

## Run the shared emergency-braking SIL/HIL scenario

Use the **dedicated** AEB world for both targets (not the two-vehicle final SIL world).

**WSL terminal 1:**

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

**WSL terminal 2, SIL target:**

```bash
cd ~/edge-path-project-git
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking --target sil
```

**WSL terminal 2, physical HIL target (instead of SIL):** First start the verified R2100 managed runtime from its deployed **ARM64 release**. Then run:

```bash
cd ~/edge-path-project-git
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking --target hil --r2100-host <R2100_IP>
```

The R2100 host is whichever **reachable IP or hostname** your laptop uses for UDP transport. It is not hard-coded in the staged runner. The HIL data channels use UDP **15010** (raw LiDAR) and **15000** (brake/gateway); SSH credentials and SSH port are separate. Do not commit passwords or site-specific access credentials.

**The common AEB acceptance criteria:** the vehicle must move before braking; intervention reason is `ttc_threshold` or `emergency_distance`; final speed ≤0.05 m/s with at least three independent stop samples; ground-truth clearance ≥1.0 m.

The native full-HIL acceptance on **2026-11-05** reported `HIL-10 RESULT: PASS`, `aeb_stop_verified`, `ttc_threshold`, final speed ≈0, **1.2356799457994787 m** clearance, four stop samples and `fault=null`.

**Committed native AEB evidence:**

- [2026-11-05 result JSON](edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.json)
- [2026-11-05 trace CSV](edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.csv)
- [Native release hash/target acceptance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md)

## R2100 hardware and native deployment

```text
Gazebo Vehicle B LiDAR + speed
          │ UDP
          v
R2100 LiDAR perception / CAN gateway
          │ physical CAN0 ↔ CAN1 (500 kbit/s)
          v
R2100 TTC / emergency-distance AEB controller
          │ CAN brake + UDP return
          v
Gazebo Vehicle B braking response
```

Default signal IDs: `0x100` vehicle speed, `0x101` obstacle distance, `0x200` brake command. Stage-specific SIL/HIL signal configuration is generated from the shared semantic interface; the R2100 supports mapping profiles and codecs.

On the device, use the control scripts **in the specific deployed native release directory** after checking that it contains `r2100_hil`:

```bash
cd ~/r2100_hil/releases/<NATIVE_RELEASE>
./r2100_hil check
./r2100_hil preflight
./start_hil.sh
./status_hil.sh
# After testing:
./stop_hil.sh
```

The earlier `~/r2100_hil/current` symlink may point to an older deployment: do **not** assume it points to the CI-built native executable. See the [R2100 HIL guide](edge-path-project/hil/README.md) and the [native hardware acceptance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md). The 2026-11-05 accepted release was separately deployed into `~/r2100_hil/releases/r2100_hil_ci_46b51c5_20261105`.

## Offline regression tests and CI

On WSL:

```bash
cd ~/edge-path-project-git
python3 -m venv .venv-requirements
source .venv-requirements/bin/activate
python -m pip install -r edge-path-project/hil/deployment/requirements-build.txt
python -m unittest discover -s tests -v
```

A previously observed full-suite baseline is **190 tests / OK**; the GitHub workflow also runs offline and deployment-design tests and builds/smoke-tests the native executable **inside Debian 12 on ARM64**. The virtual environment used for packaging may not contain Gazebo Python bindings: use the appropriate system Python for Gazebo scenarios.

Results generated by tests can be ignored by Git unless deliberately force-added as acceptance evidence. Prior unsuccessful/diagnostic results are not presented as successful current validation.

## Documentation and scope boundaries

- [JARCRAC v2 3D LiDAR AEB (experimental SIL)](JARCRAC_V2_SIL.md)
- [Gazebo SIL guide](edge-path-project/gazebo/README.md)
- [R2100 HIL guide](edge-path-project/hil/README.md)
- [Native ARM64 acceptance and provenance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md)
- [SIL/HIL results catalogue](VALIDATION.md)
- [Requirements and acceptance mapping](REQUIREMENTS.md)

This is a **Gazebo SIL → physical R2100 HIL** scope, not CARLA or Vehicle-in-the-Loop (VIL). The final SIL demo contains obstacle avoidance and moving-obstacle hold/resume; the standalone shared scenario validates TTC-based emergency braking. The evidence is simulation and HIL test evidence, not a certified safety case. PC power loss, physical CAN disconnection, bus-off and full WSMS/digital-twin capabilities remain outside the demonstrated acceptance.
