# Sanitised course and project requirements map

This file summarises the project objectives derived from the course material and CAMINO introduction without reproducing environment-specific connection details or credentials.

The purpose of this map is traceability: each objective is linked to the current implementation, a reproducible verification command where available, recorded evidence and any remaining gap.

## Requirements traceability

| Objective | Current implementation | Verification / evidence | Assessment |
| --- | --- | --- | --- |
| Existing vehicle and environment models | Gazebo vehicle models and the Ruskotunturi environment are used as the current simulation basis. | `edge-path-project/gazebo/README.md`; `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv` | **Implemented.** The course explicitly permits existing Gazebo models. |
| Virtual sensors and an ADAS controller | Gazebo LiDAR and RGB camera work, AEB, obstacle avoidance, camera + LiDAR fusion work and multi-vehicle coordination are implemented in SIL. | `python3 -m unittest discover -s tests -p 'test_final_sil_v6.py' -v`; committed v6.4 SIL result and trace | **Implemented in SIL.** Current embedded HIL AEB uses the LiDAR-based safety path rather than camera + LiDAR fusion. |
| Simple collision-avoidance / emergency-braking function | Static avoidance, moving-obstacle interaction, TTC logic and emergency braking are implemented across the staged SIL/HIL work. | SIL v6.4 PASS evidence; HIL-10 hardware validation documented in `edge-path-project/hil/README.md`; committed standalone route PASS result; 2026-11-02 fresh-checkout HIL acceptance in `VALIDATION.md` | **Implemented.** The required embedded scope is the safety/AEB path; a general-purpose WSMS or dynamic local path planner is outside the current core scope. |
| Embedded controller with physical CAN | The R2100 receives simulated vehicle/sensor data, maps it to physical CAN, evaluates the AEB path and returns brake state to Gazebo. | HIL-10 hardware scenarios; R2100 deployment/preflight evidence; 2026-11-02 fresh-checkout physical-CAN route acceptance | **Implemented and hardware-validated.** |
| Automatic signal mapping and configuration between virtual signals and CAN | Shared JSON profiles configure CAN interfaces, CAN IDs, directions, encoding and scalar scaling. Gateway, perception and AEB consume the same mapping. | `python3 -m unittest discover -s tests -p 'test_signal*.py' -v`; alternate-ID and alternate-scaling hardware runs documented in the HIL guide | **Substantially implemented.** Automatic discovery of arbitrary simulator signals and fully automatic phase-to-phase transformation are not yet demonstrated. |
| Executable deployment before transfer to embedded hardware | The R2100 runtime is built as a Python zipapp with external configuration and managed deployment scripts. | `python3 edge-path-project/hil/deployment/build_r2100_hil.py`; `python3 edge-path-project/hil/dist/r2100_hil/r2100_hil.pyz check`; `python3 -m unittest discover -s tests -p 'test_embedded_deployment.py' -v`; clean build/deploy acceptance in `VALIDATION.md` | **Implemented.** The target still requires Python 3; the course wording does not explicitly require a Python-free native binary. |
| Documentation and version-control workflow | Root, Gazebo, HIL, validation and requirements documentation identify the staged workflow and current baselines. Development uses dedicated branches. | `README.md`; `VALIDATION.md`; `REQUIREMENTS.md`; `edge-path-project/gazebo/README.md`; `edge-path-project/hil/README.md` | **Established.** Fresh-checkout acceptance and raw-evidence retention are complete; only the verified-baseline tag remains for baseline closure. |
| Multiple vehicle instances | The v6.4 Ruskotunturi SIL baseline uses two vehicles, independent routes, coordination and a common loading-zone objective. A hybrid two-vehicle HIL mission has also passed. | v6.4 result/trace; HIL-9D / later hybrid evidence in the HIL guide | **Implemented and demonstrated.** |
| Reproducibility and independent work packages | Offline tests, GitHub Actions, versioned deployment, configuration checks, target-side preflight, startup diagnostics and retained evidence reduce hidden setup dependencies. A formal clean checkout was built, deployed and hardware-accepted on 2026-11-02. | `python3 -m unittest discover -s tests -v`; `.github/workflows/offline-validation.yml`; deployment workflow; `VALIDATION.md` | **Verified baseline established.** The clean run exposed the R2100 CAN-interface initialization as an explicit host prerequisite; the tested build and deployed artifact hashes matched exactly. |
| Staged SIL -> HIL -> VIL methodology with less manual intervention | SIL and HIL use related Ruskotunturi scenarios and shared signal semantics; HIL deployment/readiness is automated. | Current SIL/HIL guides and validation evidence | **Partially complete relative to the broader CAMINO goal.** A common scenario definition and runner that selects SIL or HIL is the next focused automation milestone. |

## Current verification baseline

Run the complete offline suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Verified result from the formal clean checkout on 2026-11-02:

```text
source revision: ed1bb2e8496c36965f51a2fcb70af938777f872f
Ran 152 tests
OK
```

The clean checkout used Python 3.12.3 / x86_64 WSL. The GitHub Actions workflow `.github/workflows/offline-validation.yml` runs the same offline suite on Python 3.13 and independently builds and validates the R2100 deployment package.

The same clean checkout also completed:

```text
R2100 deployment build           PASS
generated zipapp configuration   PASS
deployment regression tests      8 / 8 PASS
R2100 deployment                 PASS
R2100 target preflight           PASS
managed runtime startup          PASS
physical-CAN HIL route           PASS
end reason                       route_complete
```

The tested deployment release was:

```text
/home/recomputer/r2100_hil/releases/r2100_hil_20261102T085551Z
```

The local build and deployed target hashes matched exactly:

```text
manifest.json
23a00a6eddfee877b9beda997ad3b6b0672d22db7c4a4a4cc676bcb4e13f35a4

r2100_hil.pyz
e92b8f7a6a3367a4e0b01ee52e5ba8eb23046aa76fb9cf7f6af1264ad948ec37
```

## R2100 host prerequisite identified by clean acceptance

The deployment package does not configure the host's physical CAN interfaces. The target preflight intentionally checks them and fails closed if they are not ready.

The clean acceptance established the following required bench state before managed runtime startup:

```text
can0 / can1       UP
CAN state         ERROR-ACTIVE
bitrate           500000 bit/s
restart-ms        100
txqueuelen        100
physical loop     CAN0_H <-> CAN1_H, CAN0_L <-> CAN1_L
```

This setup is a host prerequisite, not part of application deployment. Once applied, `r2100_hil.pyz preflight` passed and `start_hil.sh` started all three managed components successfully.

## Committed standalone HIL route evidence

The successful diagnostics-integrated HIL route is committed at:

```text
edge-path-project/gazebo/results/hil10/hil10_route_20261030T101421_6ac599e1.json
```

It records:

```text
passed: true
end_reason: route_complete
stop_verified: true
fault: null
```

The raw result was added in commit `3f5642f` (`Add successful HIL route evidence`). Its `baseline_commit=52331fde756991988cbb94f01e6d33d615c8d7df` field identifies the historical HIL-10 baseline used by the test format and must not be treated as the source revision of this later run.

The 2026-11-02 clean acceptance produced an additional committed route evidence pair:

```text
edge-path-project/gazebo/results/hil10/hil10_route_20261102T090358_69a081f9.json
edge-path-project/gazebo/results/hil10/hil10_route_20261102T090358_69a081f9.csv
passed: true
end_reason: route_complete
stop_verified: true
fault: null
```

Unlike the older evidence record, this acceptance has an explicit external traceability record for its source revision, deployed release and exact manifest/zipapp hashes in `VALIDATION.md`. Commit `e78868d` added the evidence pair; commit `cee3445` normalized the CSV line endings without changing the measurement content.

## Remaining recommended milestones

1. Tag the verified baseline.
2. Add one common scenario definition/runner that selects SIL or HIL, applies the matching configuration, checks readiness and collects comparable results.
3. If time remains, add an independent command watchdog and process-termination tests.
