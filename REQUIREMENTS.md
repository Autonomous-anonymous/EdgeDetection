# Sanitised course and project requirements map

This file summarises the project objectives derived from the course material and CAMINO introduction without reproducing environment-specific connection details or credentials.

The purpose of this map is traceability: each objective is linked to the current implementation, a reproducible verification command where available, recorded evidence and any remaining gap.

## Requirements traceability

| Objective | Current implementation | Verification / evidence | Assessment |
| --- | --- | --- | --- |
| Existing vehicle and environment models | Gazebo vehicle models and the Ruskotunturi environment are used as the current simulation basis. | `edge-path-project/gazebo/README.md`; `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv` | **Implemented.** The course explicitly permits existing Gazebo models. |
| Virtual sensors and an ADAS controller | Gazebo LiDAR and RGB camera work, AEB, obstacle avoidance, camera + LiDAR fusion work and multi-vehicle coordination are implemented in SIL. | `python3 -m unittest discover -s tests -p 'test_final_sil_v6.py' -v`; committed v6.4 SIL result and trace | **Implemented in SIL.** Current embedded HIL AEB uses the LiDAR-based safety path rather than camera + LiDAR fusion. |
| Simple collision-avoidance / emergency-braking function | Static avoidance, moving-obstacle interaction, TTC logic and emergency braking are implemented across the staged SIL/HIL work. The shared `emergency_braking` scenario exercises equivalent SIL and HIL AEB acceptance conditions. | SIL v6.4 PASS evidence; HIL-10 hardware validation; 2026-11-02 fresh-checkout HIL acceptance; committed common HIL AEB JSON/CSV evidence; `tests/test_aeb_threshold_consistency.py` 5 / 5 PASS | **Implemented.** The required embedded scope is the safety/AEB path; a general-purpose WSMS or dynamic local path planner is outside the current core scope. SIL/HIL AEB threshold copies are protected by regression tests. |
| Embedded controller with physical CAN | The R2100 receives simulated vehicle/sensor data, maps it to physical CAN, evaluates the AEB path and returns brake state to Gazebo. An independent WSL-side command watchdog also protects Vehicle B if the HIL runner process disappears. | HIL-10 hardware scenarios; R2100 deployment/preflight evidence; fresh-checkout physical-CAN route acceptance; common HIL emergency-braking committed evidence; committed `hil_process_termination_20261102T115256Z.json` | **Implemented and hardware-validated.** Forced HIL runner termination is fail-safe validated with `SIGKILL`; PC power loss, physical CAN disconnection and bus-off are not claimed. |
| Automatic signal mapping and configuration between virtual signals and CAN | Shared JSON profiles configure CAN interfaces, CAN IDs, directions, encoding and scalar scaling. Gateway, perception and AEB consume the same mapping. | `python3 -m unittest discover -s tests -p 'test_signal*.py' -v`; alternate-ID and alternate-scaling hardware runs documented in the HIL guide | **Substantially implemented.** Automatic discovery of arbitrary simulator signals and fully automatic phase-to-phase transformation are not demonstrated, and this limitation is documented. |
| Executable deployment before transfer to embedded hardware | The R2100 runtime is built as a Python zipapp with external configuration and managed deployment scripts. | `python3 edge-path-project/hil/deployment/build_r2100_hil.py`; `python3 edge-path-project/hil/dist/r2100_hil/r2100_hil.pyz check`; `python3 -m unittest discover -s tests -p 'test_embedded_deployment.py' -v`; clean build/deploy acceptance in `VALIDATION.md` | **Implemented.** The target still requires Python 3; the course wording does not explicitly require a Python-free native binary. |
| Documentation and version-control workflow | Root, Gazebo, HIL, validation and requirements documentation identify the staged workflow and current baselines. Development uses dedicated branches. | `README.md`; `VALIDATION.md`; `REQUIREMENTS.md`; `edge-path-project/gazebo/README.md`; `edge-path-project/hil/README.md`; tag `r2100-hil-verified-2026-11-02` | **Established.** Fresh-checkout acceptance, raw route/AEB/watchdog evidence and the verified-baseline tag are complete. |
| Multiple vehicle instances | The v6.4 Ruskotunturi SIL baseline uses two vehicles, independent routes, coordination and a common loading-zone objective. A hybrid two-vehicle HIL mission has also passed. | v6.4 result/trace; HIL-9D / later hybrid evidence in the HIL guide | **Implemented and demonstrated.** |
| Reproducibility and independent work packages | Offline tests, GitHub Actions, versioned deployment, configuration checks, target-side preflight, startup diagnostics, retained evidence, the verified baseline tag and an automated process-termination harness reduce hidden setup dependencies. | `python3 -m unittest discover -s tests -v`; `.github/workflows/offline-validation.yml`; deployment workflow; `VALIDATION.md`; tag `r2100-hil-verified-2026-11-02`; committed raw AEB/watchdog evidence | **Verified baseline established.** The clean run exposed the R2100 CAN-interface initialization as an explicit host prerequisite; later work adds traceable forced-process-failure evidence and threshold-drift regression coverage. |
| Staged SIL -> HIL -> VIL methodology with less manual intervention | SIL and HIL share the `emergency_braking` scenario definition, the same Gazebo world/Vehicle B acceptance case and one runner that selects either target. HIL deployment/readiness remains managed separately. | `edge-path-project/gazebo/scenarios/emergency_braking.json`; `edge-path-project/gazebo/scripts/run_staged_scenario.py`; 6 / 6 staged-scenario tests; SIL and HIL PASS on 2026-11-02 | **Implemented for the current SIL -> HIL AEB milestone.** Gazebo and R2100 lifecycle startup remain explicit and result formats remain target-specific. A VIL stage is not implemented and is explicitly out of scope for the current deliverable. |

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

After the common staged emergency-braking scenario, independent HIL command-watchdog work and five SIL/HIL AEB threshold-consistency tests, the current development suite is:

```text
Ran 168 tests
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

The successful HIL run is retained as committed raw evidence:

```text
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.json
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.csv
```

The JSON records `passed=true`, `stop_verified=true`, `aeb_reason=ttc_threshold` and 1.189 m minimum recorded clearance against the 1.000 m requirement. Commit `c2c91d6` added the result pair and `b445c716` normalized the CSV line endings without changing measurement content.

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

Final committed 2026-11-02 result:

```text
Vehicle B moving before fault    0.850 m/s
HIL runner fault                 SIGKILL
watchdog STOP latency            0.261 s after SIGKILL
final observed speed             ~0.000 m/s
stop confirmation samples        17
stop-and-hold verification       1.616 s after SIGKILL
source commit                    269a1c4feb5ce91a5174f25243f21be71cce959f
source dirty                     false
PROCESS TERMINATION FAIL-SAFE    PASS
```

Machine-readable committed evidence:

```text
edge-path-project/gazebo/results/hil10/hil_process_termination_20261102T115256Z.json
```

The use of `SIGKILL` is intentional: it prevents the killed HIL runner from executing its own cleanup STOP, so the harness explicitly verifies a new post-kill zero command from the independent watchdog before accepting the later stop. The harness source was committed as `269a1c4`; the evidence was committed as `63cb16f`.

## SIL/HIL AEB threshold consistency

The SIL emergency-braking backend and the R2100 AEB controller intentionally remain separate runtime modules, but their duplicated AEB constants are now protected against drift by:

```text
tests/test_aeb_threshold_consistency.py
```

The five checks compare the TTC threshold, emergency distance, sensor timeout, control period and minimum speed used for TTC. Verified result:

```text
Ran 5 tests
OK
```

The test was added in commit `acc8712`.

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

The referenced baseline commit is not missing history: `52331fde756991988cbb94f01e6d33d615c8d7df` exists in the repository, is reachable from the current `hil-development` HEAD, and is the historical `Update project README for completed HIL-9 milestone` commit. It simply falls outside short recent-commit listings.

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

## Scope boundary

The current project deliverable validates the staged **SIL -> HIL** workflow. A Vehicle-in-the-Loop (VIL) stage is **not implemented and is explicitly out of scope** for this deliverable; no VIL completion claim is made.

## Remaining recommended milestones

The verified baseline, common SIL/HIL emergency-braking scenario, independent process-termination fail-safe and threshold-consistency regression are complete. The current embedded AEB scope does not require a general-purpose local planner or additional showcase navigation.

Remaining work is optional refinement:

1. optional unified SIL/HIL result schema,
2. optional Gazebo/R2100 lifecycle orchestration around the common runner,
3. optional second common SIL/HIL scenario to demonstrate runner generalisation,
4. optional broader fault coverage such as physical CAN disconnection or PC power-loss behaviour.
