# CAMINO — Gazebo SIL and R2100 Hardware-in-the-Loop

This `camino` branch documents the **CAMINO Software-in-the-Loop (SIL) → Hardware-in-the-Loop (HIL)** workflow. It contains **Gazebo Harmonic** vehicle simulations, Ruskotunturi worlds and models, scenario runners, recorded results, and the software for the **Seeed Studio reComputer Industrial R2100** controller with physical CAN.

This branch intentionally excludes the earlier CARLA prototype, its standalone scripts and CSV results, and its input/output videos. Those assets remain on the original repository branches. Gazebo result CSV/JSON/trace files are retained as SIL/HIL evidence, including historical failed/diagnostic runs identified as such.

## Project scope and current baselines

| Area | Current implementation and evidence |
| --- | --- |
| Gazebo SIL | Ruskotunturi final two-vehicle **v6.4** demo: **PASS**, `final_demo_complete` |
| Obstacle handling in SIL | Camera/LiDAR static avoidance, moving-obstacle hold-and-resume, merge coordination and loading-zone arrival |
| Shared SIL/HIL AEB | One `emergency_braking` scenario and target-selecting runner. SIL **PASS** in the operator record and HIL **PASS** with committed JSON and CSV. |
| R2100 physical HIL | Perception → physical CAN → embedded AEB → returned brake state |
| Native embedded release | Debian 12 ARM64 CI-built ELF, verified hashes and physical R2100 `check` / `preflight` / runtime startup |
| Native functional acceptance | **2026-11-05 PASS** on release `46b51c5`. `aeb_stop_verified`, `ttc_threshold`, 1.236 m clearance, four stop-confirmation samples, no fault. |
| Regression coverage | [2026-11-12 CI](VALIDATION.md#offline-validation-and-packaging) on `51f6f139` recorded 422 passed and 3 skipped out of 425. ARM64 packaging and deployment checks passed. |
| Boundary | **SIL → HIL**. Vehicle-in-the-Loop (VIL) is out of scope. |

The final SIL demo uses **path-clearance-based LiDAR hold/resume** for the moving obstacle. The separate `emergency_braking` scenario demonstrates TTC-based emergency braking.

## Experimental JARCRAC SIL

The Mevea-derived JARCRAC is the primary experimental forestry vehicle. The [v2 wheel-based model](JARCRAC_V2_SIL.md) has recorded flat-world AEB passes at **0.35, 0.70 and 1.00 m/s**, with a separate positive chassis-contact control. Ruskotunturi tests cover stationary perception, sensitivity to pose errors and simulated IMU calibration. The [2026-11-09 terrain diagnostic](edge-path-project/gazebo/results/jarcrac/imu_dynamic_clear_diagnostic.json) recorded 0.3605 m of travel at a 0.10 m/s command, eight moving LiDAR/IMU comparisons, 0.0059° maximum orientation error and a final speed of zero.

The separate [detailed tracked model](edge-path-project/gazebo/docs/CAMINO_JARCRAC_HANDOFF.md) and its meshes are committed. Its [results](edge-path-project/gazebo/results/jarcrac/README.md) cover short motion, contact checks and an experimental TTC stop-and-hold with a known static obstacle. The watchdog and bounded-motion scripts are available, with their live-run reports still absent from the branch. These experiments use Gazebo ground-truth position and receive-time sensor pairing. Independent localization, calibrated terrain AEB, autonomous detour and route rejoining, and JARCRAC-specific R2100 HIL remain unverified.

## Optional wheel loader for teammate use

A separate Mevea-derived articulated [wheel-loader Gazebo model](edge-path-project/gazebo/models/wheelloader/README.md) is included for reuse, not as a replacement for JARCRAC. Its **flat-world odometry-based motion smoke test passed**: 0.5375 m displacement and 0.0 m/s final forward speed ([committed result](edge-path-project/gazebo/results/wheelloader_flat_smoke.json)). This is not calibrated vehicle dynamics, articulation or bucket validation.

From `edge-path-project/gazebo/`, run the world with `GZ_SIM_RESOURCE_PATH="$PWD/models"` and `gz sim -r worlds/wheelloader_flat_test_world.sdf`; run `python3 -u scripts/wheelloader_runtime_smoke.py` in another terminal; the script automatically manages its controller bridge. See the model README for joint topics, reproduction steps, caveats and its [attribution notice](edge-path-project/gazebo/models/wheelloader/NOTICE.md). **Third-party mesh redistribution permissions have not been independently verified; see the attribution notice for asset provenance.**

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

Start each WSL terminal in the root of your `camino` checkout.

## Run the final Gazebo SIL v6.4 demo

From **WSL terminal 1** (with the system Python/Gazebo environment):

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_final_sil_demo_v6_world.sdf
```

From **WSL terminal 2**:

```bash
python3 edge-path-project/gazebo/scripts/ruskotunturi_final_sil_demo_v6.py
```

Start with a fresh Gazebo world each run; do not use the world reset feature while a prior controller or watchdog is still active. Consult the [Gazebo SIL guide](edge-path-project/gazebo/README.md) for state-machine behaviour, metrics and reset limitations.

**Committed SIL v6.4 PASS evidence:**

- [Final result CSV](edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv)
- [Matching trace](edge-path-project/gazebo/results/final_sil_v6_4_trace_20261026T133025_782767.csv)

## Run the shared emergency-braking SIL/HIL scenario

Use the dedicated AEB world for both targets.

**WSL terminal 1:**

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

**WSL terminal 2, SIL target:**

```bash
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking --target sil
```

For the physical HIL target, first start the verified R2100 runtime from its deployed **ARM64 release**. Then run in WSL terminal 2:

```bash
read -r -p "Reachable R2100 IP address or hostname: " R2100_IP
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking --target hil --r2100-host "$R2100_IP"
```

The R2100 host is whichever **reachable IP or hostname** your laptop uses for UDP transport. It is not hard-coded in the staged runner. The HIL data channels use UDP **15010** (raw LiDAR) and **15000** (brake/gateway); SSH credentials and SSH port are separate. Do not commit passwords or site-specific access credentials.

The common AEB criteria require vehicle motion before braking, an intervention reason of `ttc_threshold` or `emergency_distance`, final speed ≤0.05 m/s with at least three independent stop samples, and ground-truth clearance ≥1.0 m.

The [2026-11-02 SIL operator record](edge-path-project/gazebo/README.md#common-silhil-emergency-braking-scenario) reports a pass with 1.253 m ground-truth clearance. The **2026-11-05** native HIL JSON and trace record `aeb_stop_verified`, `ttc_threshold`, final speed ≈0, **1.236 m** clearance, four stop samples and `fault=null`.

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

The default CAN IDs are `0x100` for vehicle speed, `0x101` for obstacle distance and `0x200` for the brake command. The staged runner generates a mapping from the shared interface for the selected target. The SIL backend reads that mapping at launch. The R2100 loads the profile packaged with its deployed release, or the profile selected by `R2100_HIL_CONFIG`. The native build generates the default HIL profile from the same interface.

On the device, substitute the name of the verified native release installed on your R2100:

```bash
cd "$HOME/r2100_hil/releases/<NATIVE_RELEASE>"
./r2100_hil check
./r2100_hil preflight
./start_hil.sh
./status_hil.sh
```

After testing, run `./stop_hil.sh` from that release directory. The 2026-11-05 acceptance used `~/r2100_hil/releases/r2100_hil_ci_46b51c5_20261105`. The `~/r2100_hil/current` symlink may still select an older deployment. See the [R2100 HIL guide](edge-path-project/hil/README.md#embedded-deployment) and the [native release provenance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md).

## Offline regression tests and CI

From the repository root on WSL:

```bash
python3 -m venv .venv-requirements
source .venv-requirements/bin/activate
python -m pip install -r edge-path-project/hil/deployment/requirements-build.txt
python -m unittest discover -s tests -v
```

The [validation index](VALIDATION.md#offline-validation-and-packaging) records the dated offline CI and ARM64 build results. Use a Python interpreter with the Gazebo bindings for simulation runs.

Results generated by tests can be ignored by Git unless deliberately force-added as acceptance evidence. Prior unsuccessful/diagnostic results are not presented as successful current validation.

## Documentation and scope boundaries

- [JARCRAC v2 3D LiDAR AEB (experimental SIL)](JARCRAC_V2_SIL.md)
- [Gazebo SIL guide](edge-path-project/gazebo/README.md)
- [R2100 HIL guide](edge-path-project/hil/README.md)
- [Native ARM64 acceptance and provenance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md)
- [SIL/HIL results catalogue](VALIDATION.md)
- [Requirements and acceptance mapping](REQUIREMENTS.md)

The broader CAMINO worksite management system, machine-specific digital twins and formal functional-safety certification are outside this branch's demonstrated scope. The [HIL guide](edge-path-project/hil/README.md#manual-physical-can-interruption-and-recovery-validation) records a separate operator-reported CAN interruption and recovery on the two-interface bench loop. PC power loss and broader CAN fault coverage remain open.
