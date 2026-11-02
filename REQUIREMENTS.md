# Sanitised course and project requirements map

This file summarises the project objectives derived from the course material and CAMINO introduction without reproducing environment-specific connection details or credentials.

The purpose of this map is traceability: each objective is linked to the current implementation, a reproducible verification command where available, recorded evidence and any remaining gap.

## Requirements traceability

| Objective | Current implementation | Verification / evidence | Assessment |
| --- | --- | --- | --- |
| Existing vehicle and environment models | Gazebo vehicle models and the Ruskotunturi environment are used as the current simulation basis. | `edge-path-project/gazebo/README.md`; `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv` | **Implemented.** The course explicitly permits existing Gazebo models. |
| Virtual sensors and an ADAS controller | Gazebo LiDAR and RGB camera work, AEB, obstacle avoidance, camera + LiDAR fusion work and multi-vehicle coordination are implemented in SIL. | `python3 -m unittest discover -s tests -p 'test_final_sil_v6.py' -v`; committed v6.4 SIL result and trace | **Implemented in SIL.** Current embedded HIL AEB uses the LiDAR-based safety path rather than camera + LiDAR fusion. |
| Simple collision-avoidance / emergency-braking function | Static avoidance, moving-obstacle interaction, TTC logic and emergency braking are implemented across the staged SIL/HIL work. The shared `emergency_braking` scenario now exercises equivalent SIL and HIL AEB acceptance conditions. | SIL v6.4 PASS evidence; HIL-10 hardware validation; 2026-11-02 fresh-checkout HIL acceptance; common SIL/HIL scenario PASS recorded in `VALIDATION.md` | **Implemented.** The required embedded scope is the safety/AEB path; a general-purpose WSMS or dynamic local path planner is outside the current core scope. |
| Embedded controller with physical CAN | The R2100 receives simulated vehicle/sensor data, maps it to physical CAN, evaluates the AEB path and returns brake state to Gazebo. An independent WSL-side command watchdog also protects Vehicle B if the HIL runner process disappears. | HIL-10 hardware scenarios; R2100 deployment/preflight evidence; 2026-11-02 fresh-checkout physical-CAN route acceptance; common HIL emergency-braking PASS; `SIGKILL` process-termination fail-safe PASS in `VALIDATION.md` | **Implemented and hardware-validated.** The forced HIL runner termination case is also fail-safe validated. |
| Automatic signal mapping and configuration between virtual signals and CAN | Shared JSON profiles configure CAN interfaces, CAN IDs, directions, encoding and scalar scaling. Gateway, perception and AEB consume the same mapping. | `python3 -m unittest discover -s tests -p 'test_signal*.py' -v`; alternate-ID and alternate-scaling hardware runs documented in the HIL guide | **Substantially implemented.** Automatic discovery of arbitrary simulator signals and fully automatic phase-to-phase transformation are not demonstrated, but the current staged workflow does not require them for the validated AEB path. |
| Executable deployment before transfer to embedded hardware | The R2100 runtime is built as a Python zipapp with external configuration and managed deployment scripts. | `python3 edge-path-project/hil/deployment/build_r2100_hil.py`; `python3 edge-path-project/hil/dist/r2100_hil/r2100_hil.pyz check`; `python3 -m unittest discover -s tests -p 'test_embedded_deployment.py' -v`; clean build/deploy acceptance in `VALIDATION.md` | **Implemented.** The target still requires Python 3; the course wording does not explicitly require a Python-free native binary. |
| Documentation and version-control workflow | Root, Gazebo, HIL, validation and requirements documentation identify the staged workflow and current baselines. Development uses dedicated branches. | `README.md`; `VALIDATION.md`; `REQUIREMENTS.md`; `edge-path-project/gazebo/README.md`; `edge-path-project/hil/README.md`; tag `r2100-hil-verified-2026-11-02` | **Established.** Fresh-checkout acceptance, raw route evidence and the verified-baseline tag are complete. |
| Multiple vehicle instances | The v6.4 Ruskotunturi SIL baseline uses two vehicles, independent routes, coordination and a common loading-zone objective. A hybrid two-vehicle HIL mission has also passed. | v6.4 result/trace; HIL-9D / later hybrid evidence in the HIL guide | **Implemented and demonstrated.** |
| Reproducibility and independent work packages | Offline tests, GitHub Actions, versioned deployment, configuration checks, target-side preflight, startup diagnostics, retained evidence, the verified baseline tag and an automated process-termination harness reduce hidden setup dependencies. A formal clean checkout was built, deployed and hardware-accepted on 2026-11-02. | `python3 -m unittest discover -s tests -v`; `.github/workflows/offline-validation.yml`; deployment workflow; `VALIDATION.md`; tag `r2100-hil-verified-2026-11-02`; `hil_process_termination_test.py` | **Verified baseline established.** The clean run exposed the R2100 CAN-interface initialization as an explicit host prerequisite; the tested build and deployed artifact hashes matched exactly. The later watchdog harness adds repeatable forced-process-failure validation. |
| Staged SIL -> HIL -> VIL methodology with less manual intervention | SIL and HIL now share the `emergency_braking` scenario definition, the same Gazebo world/Vehicle B acceptance case and one runner that selects either target. HIL deployment/readiness remains managed separately. | `edge-path-project/gazebo/scenarios/emergency_braking.json`; `edge-path-project/gazebo/scripts/run_staged_scenario.py`; 6 / 6 staged-scenario tests; SIL and HIL PASS on 2026-11-02 | **Implemented for the current SIL -> HIL AEB milestone.** Gazebo and R2100 lifecycle startup are still explicit operator steps, and result formats remain target-specific; those are optional automation extensions rather than blockers for the current scope. |

## Current verification baseline

Run the complete offline suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Formal clean-checkout baseline result from 2026-11-02:

```text
source revision: ed1bb2e8496c36965f51a2fcb70af938777f872f
Ran 152 tests
OK
```

That clean checkout used Python 3.12.3 / x86_64 WSL. The verified baseline was subsequently tagged as:

```text
r2100-hil-verified-2026-11-02
```

After adding the common staged emergency-braking scenario and the independent HIL command-watchdog tests, the current development suite is:

```text
Ran 163 tests
OK
```

The GitHub Actions workflow `.github/workflows/offline-validation.yml` runs the offline suite on Python 3.13 and independently builds and validates the R2100 deployment package.

The clean checkout also completed:

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

## Common SIL/HIL emergency-braking milestone

The common staged scenario is defined in:

```text
edge-path-project/gazebo/scenarios/emergency_braking.json
```

It selects the same HIL9E Gazebo world and Vehicle B for both execution targets and defines shared acceptance criteria for vehicle motion, valid AEB reasons, stop speed, stop confirmation and minimum clearance.

The common runner is:

```text
edge-path-project/gazebo/scripts/run_staged_scenario.py
```

SIL verification:

```bash
python3 edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target sil
```

Verified 2026-11-02 result:

```text
STAGED SCENARIO RESULT: PASS
End reason: aeb_stop_verified
AEB reason: ttc_threshold
Ground-truth clearance: 1.253 m
```

HIL verification, with the same world running and the R2100 runtime ready:

```bash
python3 edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target hil \
  --r2100-host <R2100_IP>
```

Verified 2026-11-02 result:

```text
HIL-10 RESULT: PASS
End reason: aeb_stop_verified
AEB reason: ttc_threshold
```

This closes the focused common SIL/HIL scenario-definition and target-selection milestone. It does not claim a fully autonomous orchestration framework: Gazebo startup, R2100 runtime startup and target-specific evidence formats remain explicit.

## Independent HIL process-termination safety extension

The HIL path includes an independent command watchdog:

```text
edge-path-project/gazebo/scripts/hil_command_watchdog.py
```

The watchdog arms from the normal Vehicle B command stream and latches a zero-command STOP if command freshness exceeds the default 0.30 s timeout. Its state-machine behaviour is covered by **5 / 5** offline tests.

The automated validation harness is:

```text
edge-path-project/gazebo/scripts/hil_process_termination_test.py
```

With the Phase 6 Gazebo world active and the R2100 runtime ready, run:

```bash
python3 -u edge-path-project/gazebo/scripts/hil_process_termination_test.py \
  --r2100-host <R2100_IP>
```

Verified 2026-11-02 result:

```text
Vehicle B moving before fault    0.850 m/s
HIL runner fault                 SIGKILL
watchdog STOP latency            0.314 s
final observed speed             0.000 m/s
stop confirmation samples        17
stop-and-hold verification       1.680 s after SIGKILL
PROCESS TERMINATION FAIL-SAFE    PASS
```

The use of `SIGKILL` is intentional: it prevents the killed HIL runner from executing its own cleanup STOP, so the harness explicitly verifies a new post-kill zero command from the independent watchdog before accepting the later stop. This closes the forced HIL-runner-termination gap documented by the historical HIL-10 material.

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

The verified baseline, common SIL/HIL emergency-braking scenario and independent process-termination fail-safe are complete. The current embedded AEB scope does not require a general-purpose local planner or additional showcase navigation.

Remaining work is optional refinement:

1. optional unified SIL/HIL result schema,
2. optional Gazebo/R2100 lifecycle orchestration around the common runner,
3. optional broader fault coverage such as physical CAN disconnection or PC power-loss behaviour.
