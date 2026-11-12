# Validation and documentation index

The **CAMINO branch** intentionally includes Gazebo SIL and R2100 HIL only, with no earlier CARLA scripts, CSV evidence or videos in its checked-out tree.

This file identifies the current SIL/HIL baselines, the role of the different preflight checks, and the evidence that should be used for current reporting.

It is intentionally small and does not replace the detailed Gazebo or HIL guides. Course/project requirement traceability is maintained separately in `REQUIREMENTS.md`.

## Current baselines

### Software-in-the-Loop

Current SIL baseline: **Ruskotunturi final SIL v6.4**.

Committed evidence:

- `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv`
- `edge-path-project/gazebo/results/final_sil_v6_4_trace_20261026T133025_782767.csv`

The committed v6.4 result is `PASS` with `end_reason=final_demo_complete` and no failed checks. It records both vehicle mission states, camera-assisted static avoidance, moving-obstacle interaction, coordination and loading-zone safety checks.

### Hardware-in-the-Loop

Current platform name: **Ruskotunturi R2100 HIL**.

The historical functional suite name remains **HIL-10 Freshness and Fault Validation**. Later work packages extend the same platform with configurable signal mapping, configuration-driven scalar codecs, standalone native executable deployment, R2100 deployment preflight, WSL-side startup-readiness diagnostics, a common SIL/HIL emergency-braking scenario, an independent Vehicle B command watchdog for process-termination fail-safe protection and SIL/HIL AEB threshold-consistency regression coverage.

The verified R2100 baseline is tagged:

```text
r2100-hil-verified-2026-11-02
```

That tag closes the fresh-checkout build/deploy/physical-CAN acceptance baseline. The common staged emergency-braking scenario, process-termination watchdog, threshold-consistency tests and standalone native executable work were added afterwards on `hil-development`.

The standalone ARM64 deployment was physically accepted on the R2100 on 2026-11-03. The target identified `r2100_hil` as an ARM aarch64 ELF; `r2100_hil check` and `r2100_hil preflight` both passed on Debian 12 / glibc 2.36 with CAN0 and CAN1 UP, ERROR-ACTIVE and configured at 500 kbit/s. The detailed record is:

```text
edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md
```

### Offline validation and packaging

The [2026-11-12 GitHub Actions run](https://github.com/Autonomous-anonymous/EdgeDetection/actions/runs/43185855454) tested `camino` at `51f6f139`. The offline suite reported **422 passed and 3 skipped out of 425**. The three skips require local Mevea inputs for model regeneration or generator options. Deployment-design tests and the Debian 12 ARM64 package build and smoke test also passed.

These CI results cover software tests and packaging. The [2026-11-05 physical AEB acceptance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md#full-native-executable-hil-aeb-acceptance--2026-11-05) used release `46b51c5`.

The full offline suite includes `test_embedded_deployment.py`, which builds a standalone executable with PyInstaller. From the repository root, install the pinned build dependency in a virtual environment before running the suite:

```bash
python3 -m venv .venv-requirements
source .venv-requirements/bin/activate
python -m pip install -r edge-path-project/hil/deployment/requirements-build.txt
python -m unittest discover -s tests -v
```

A missing PyInstaller installation causes the deployment test class to fail at setup, so the test suite is offline at execution time but **not** standard-library-only for initial environment provisioning. If a previous Docker build left root-owned generated files under `edge-path-project/hil/dist/`, fix their ownership before rerunning the native build test. The CI workflow provisions PyInstaller explicitly.

The same command is part of `.github/workflows/offline-validation.yml`. The workflow also builds and validates the standalone R2100 deployment package and runs the deployment regression tests.

## Fresh-checkout reproducibility acceptance — 2026-11-02

A formal fresh checkout of `hil-development` at source revision
`ed1bb2e8496c36965f51a2fcb70af938777f872f` was validated end-to-end on 2026-11-02.

The verification used a separate clean checkout with no files copied from the development working tree. The clean checkout was on Python 3.12.3 / x86_64 WSL and had a clean Git worktree before the validation steps.

This acceptance predates the later native-executable deployment work. It used the historical Python zipapp package and is retained as reproducibility evidence for that baseline rather than as the description of the current deployment format.

The fresh checkout completed:

```text
full offline suite                152 / 152 PASS
R2100 deployment build           PASS
generated zipapp configuration   PASS
deployment regression tests      8 / 8 PASS
R2100 deployment                 PASS
R2100 target preflight           PASS
managed runtime startup          PASS
physical-CAN HIL route           PASS
end reason                       route_complete
```

The deployed release was:

```text
/home/recomputer/r2100_hil/releases/r2100_hil_20261102T085551Z
```

The fresh build and deployed target were verified to contain the same historical zipapp artifacts:

```text
manifest.json SHA-256
23a00a6eddfee877b9beda997ad3b6b0672d22db7c4a4a4cc676bcb4e13f35a4

r2100_hil.pyz SHA-256
e92b8f7a6a3367a4e0b01ee52e5ba8eb23046aa76fb9cf7f6af1264ad948ec37
```

The acceptance route produced:

```text
hil10_route_20261102T090358_69a081f9.json
HIL-10 RESULT: PASS
End reason: route_complete
```

The raw acceptance JSON and CSV are committed in the repository as the final fresh-checkout reproducibility evidence. Commit `e78868d` added the evidence pair, and commit `cee3445` normalized the CSV line endings without changing the measurement content.

### Host prerequisite discovered by the clean run

The deployment package intentionally checks CAN readiness but does not configure host CAN interfaces itself. On a fresh/restarted R2100 host, `can0` and `can1` therefore have to be configured before runtime startup.

The validated bench configuration is:

```text
can0 / can1       UP
CAN state         ERROR-ACTIVE
bitrate           500000 bit/s
restart-ms        100
txqueuelen        100
physical loop     CAN0_H <-> CAN1_H, CAN0_L <-> CAN1_L
```

This is now considered an explicit R2100 host prerequisite rather than an implicit setup assumption. The target-side preflight correctly failed while the interfaces were down and passed after this host setup was applied.

The clean run also demonstrated that stale PID files from an earlier managed run are harmless: `stop_hil.sh` removes stale PID state and the subsequent managed startup records the current config and log directory.

## Common SIL/HIL emergency-braking scenario — 2026-11-02

A shared staged scenario defines one equivalent emergency-braking acceptance case for both execution targets:

```text
edge-path-project/gazebo/scenarios/emergency_braking.json
```

Both targets use:

```text
world: edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
vehicle: aeb_vehicle_360_b
required clearance: 1.00 m
stop threshold: 0.05 m/s
stop confirmation: 3 samples
valid AEB reasons: emergency_distance, ttc_threshold
```

The common target-selecting runner is:

```text
edge-path-project/gazebo/scripts/run_staged_scenario.py
```

The SIL target starts `ruskotunturi_emergency_braking_sil_stage.py` with the generated mapping. The adapter applies it to `ruskotunturi_emergency_braking_sil.py`. The HIL target starts `ruskotunturi_hil10_b_test.py --mode aeb` and uses the already-running R2100 with its deployed CAN profile.

The 2026-11-02 operator record reports:

```text
SIL target
result                      PASS
end reason                  aeb_stop_verified
AEB reason                  ttc_threshold
trigger LiDAR clearance     1.374 m
trigger speed               0.850 m/s
trigger TTC                 1.617 s
ground-truth stop clearance 1.253 m

HIL target
result                      PASS
end reason                  aeb_stop_verified
AEB reason                  ttc_threshold
raw LiDAR transport         active
physical-CAN AEB path       exercised
```

The SIL values above come from the operator record. The HIL run has a committed JSON summary and CSV trace:

```text
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.json
edge-path-project/gazebo/results/hil10/hil10_aeb_20261102T102033_e21c05af.csv
```

The JSON records:

```text
passed                 true
end_reason             aeb_stop_verified
stop_verified          true
AEB reason             ttc_threshold
final speed            ~0.000 m/s
observer stop samples  4
recorded clearance     1.189 m
required clearance     1.000 m
fault                   null
```

Commit `c2c91d6` added the raw JSON/CSV evidence pair and `b445c716` normalized the CSV line endings without changing its measurements.

The first HIL attempt intentionally failed closed with `startup_brake_reply_missing` while the R2100 runtime was not returning brake state. After the managed R2100 runtime was started, the same scenario passed. This is consistent with the startup-readiness diagnostics design and is not used as PASS evidence.

The offline staged-scenario tests pass **6 / 6**.

The common runner selects the target, generates its local mapping and starts the backend. The native build generates and packages the default HIL profile from the same interface. The operator starts Gazebo and, for HIL, the deployed R2100 runtime. Each backend produces its own result format.

## Independent HIL process-termination fail-safe — 2026-11-02

The HIL path includes an independent WSL-side Vehicle B command watchdog:

```text
edge-path-project/gazebo/scripts/hil_command_watchdog.py
```

Its state logic is covered by five offline tests in:

```text
tests/test_hil_command_watchdog.py
```

The watchdog arms only after at least one Vehicle B command has been observed. If command updates then stop for the configured timeout, the watchdog latches STOP and continuously publishes zero speed and steering. The latch is not released by a later command during the same process run.

An automated hardware-in-the-loop process-termination harness is provided by:

```text
edge-path-project/gazebo/scripts/hil_process_termination_test.py
```

The harness starts the watchdog and the normal HIL route runner, independently observes Vehicle B commands and odometry, waits until the vehicle is moving, and then kills the HIL runner with `SIGKILL`. Because `SIGKILL` prevents the runner from executing its own cleanup/final STOP path, the harness requires a new post-kill zero command from the independent watchdog before the later vehicle stop can be counted as PASS.

Final committed operator run on 2026-11-02:

```text
Vehicle B moving before fault    PASS, 0.850 m/s
fault injection                  SIGKILL to HIL runner
runner termination               PASS
watchdog STOP latency            PASS, 0.261 s after SIGKILL
final observed speed             ~0.000 m/s
stop confirmation samples        17
stop-and-hold verification       PASS, 1.616 s after SIGKILL
source commit                    269a1c4feb5ce91a5174f25243f21be71cce959f
source dirty                     false
PROCESS TERMINATION FAIL-SAFE    PASS
```

Machine-readable committed evidence:

```text
edge-path-project/gazebo/results/hil10/hil_process_termination_20261102T115256Z.json
```

The evidence JSON was produced by the committed harness source at `269a1c4feb5ce91a5174f25243f21be71cce959f` with a clean working tree and was committed in `63cb16f`.

The default watchdog timeout is 0.30 s. The recorded 0.261 s value is measured from the `SIGKILL` injection to the first observed watchdog zero command; the watchdog's own timeout is measured from the last healthy Vehicle B command, so those two timings are intentionally different references.

The test verifies that the actual post-kill Vehicle B zero command comes from the watchdog and that the vehicle subsequently remains below the 0.05 m/s stop threshold for the required confirmation/hold period.

The watchdog state tests pass **5 / 5**.

This run covers loss of the HIL runner process while its WSL host and watchdog remain running. A separate [operator-reported CAN interruption and recovery trial](edge-path-project/hil/README.md#manual-physical-can-interruption-and-recovery-validation) covers one two-interface bench setup. PC power loss and broader CAN fault cases remain open.

## SIL/HIL AEB threshold consistency — 2026-11-02

The SIL emergency-braking adapter and the R2100 AEB controller intentionally remain separate runtime modules. Their duplicated safety constants are protected against silent drift by:

```text
tests/test_aeb_threshold_consistency.py
```

The five regression checks compare:

```text
TTC threshold
emergency distance
sensor timeout
control period
minimum speed used for TTC
```

Verified result:

```text
Ran 5 tests
OK
```

This test was added in commit `acc8712`, when the complete offline suite passed **168 / 168**. A local run on 2026-11-03 passed **190 / 190** after adding native deployment and requirements tests. See [offline validation and packaging](#offline-validation-and-packaging) for the dated `camino` CI result.

## Preflight terminology

There are three different checks with different purposes. They should not be treated as interchangeable.

### Historical baseline integrity check

```bash
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
```

`hil10_preflight.py` compares a small dependency set against the original HIL-9-derived snapshot used when HIL-10 freshness testing was introduced. It is retained as development-history evidence.

Because the runtime has since gained signal mapping, codecs, deployment and diagnostics changes, `DIFFERS` results are expected for changed dependencies. This script is **not** the current runtime-readiness preflight. References to this command in the historical HIL-10 offline-validation material describe the original milestone procedure rather than the current startup procedure.

### Current R2100 runtime readiness

On the R2100, substitute the verified native release directory:

```bash
cd "$HOME/r2100_hil/releases/<NATIVE_RELEASE>"
./r2100_hil preflight
```

This is the current target-side readiness check. It validates the selected mapping profile, configured CAN interfaces, live CAN state and bitrate, and the UDP ports needed by the HIL runtime. `start_hil.sh` runs this check automatically before launching the managed processes.

### Current WSL / end-to-end startup readiness

`ruskotunturi_hil10_b_test.py` uses `hil_startup_diagnostics.py` to report the observed Gazebo pose/odometry state, R2100 brake-return freshness, brake state/reason, runtime fault and local LiDAR sender error when startup readiness is not reached.

This complements the R2100 deployment preflight: the target preflight checks the embedded machine itself, while startup diagnostics report what the WSL-side HIL runner actually observes.

## Evidence catalogue

### SIL v6.4

| Evidence | Status | Notes |
| --- | --- | --- |
| `ruskotunturi_final_sil_demo_v6_4_results.csv` | PASS | Current final SIL result row; `final_demo_complete`. |
| `final_sil_v6_4_trace_20261026T133025_782767.csv` | committed | Matching trace named by the v6.4 result. |

### HIL-10 and later R2100 work

| Evidence | Status | Notes |
| --- | --- | --- |
| HIL-10 offline validation | 42 / 42 PASS | Historical focused HIL-10 suite at milestone completion. |
| HIL-10 hardware scenarios | 6 / 6 PASS | Route, hybrid, AEB and three fault modes are documented in `edge-path-project/hil/README.md`. |
| Signal regression | 30 / 30 PASS | Mapping and codec milestone documented in the HIL guide. |
| Deployment tests | 8 / 8 PASS | Embedded package build/check coverage documented in the HIL guide. |
| R2100 deployment preflight unit tests | 4 / 4 PASS | Target-side preflight coverage. |
| HIL startup diagnostics | 7 / 7 PASS | Dedicated diagnostics unit tests. |
| HIL integration regression | 18 / 18 PASS | Integration tests after startup diagnostics. |
| Common staged-scenario tests | 6 / 6 PASS | Scenario schema, target command construction, HIL host requirement and both dry-run paths. |
| SIL/HIL AEB threshold consistency | 5 / 5 PASS | TTC, emergency distance, timeout, period and TTC minimum-speed copies match. |
| Common emergency-braking SIL run | Operator-reported PASS on 2026-11-02 | `aeb_stop_verified`, `ttc_threshold`, 1.253 m ground-truth stop clearance. |
| Common emergency-braking HIL run | PASS / committed raw evidence | `hil10_aeb_20261102T102033_e21c05af.json/.csv`; R2100 and physical-CAN AEB path exercised. |
| Command-watchdog state tests | 5 / 5 PASS | Arming, freshness, timeout latch, invalid timeout and unarmed behaviour. |
| HIL runner `SIGKILL` fail-safe | PASS / committed raw evidence | `hil_process_termination_20261102T115256Z.json`; watchdog zero at 0.261 s after SIGKILL, 17 stop samples, source clean. |
| Standalone ARM64 deployment and native functional AEB | PASS / committed JSON/CSV on 2026-11-05 | Physical R2100: Debian 12 ARM64 CI executable, artifact SHA-256 matched on target, `check`/`preflight`/managed runtime PASS, full physical-CAN `emergency_braking` HIL-10 PASS, `aeb_stop_verified`, 1.236 m clearance, 4 stop samples, no fault. Committed evidence: `edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.json/.csv`; see `edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md`. |
| Manual CAN interruption and recovery | Operator-reported PASS | One two-interface bench trial. The [HIL guide](edge-path-project/hil/README.md#manual-physical-can-interruption-and-recovery-validation) records the sequence. The raw trace and exact release are unavailable in the branch. |
| GitHub Actions offline validation | 422 passed, 3 skipped on 2026-11-12 | [CI for `51f6f139`](#offline-validation-and-packaging) also passed the native ARM64 build, smoke test and deployment-design checks. |
| Fresh-checkout R2100 acceptance | PASS | Historical zipapp baseline: clean checkout -> build -> deploy -> preflight -> managed runtime -> physical-CAN route; `route_complete`. |
| Verified baseline tag | complete | `r2100-hil-verified-2026-11-02`; closes the pre-common-scenario R2100 acceptance baseline. |

### Standalone route evidence

The earlier committed file

`edge-path-project/gazebo/results/hil10/hil10_route_20261029T092701_b95d92cd.json`

is intentionally retained as development history. It records:

```text
passed: false
end_reason: interrupted
```

A later successful standalone route result is also committed:

`edge-path-project/gazebo/results/hil10/hil10_route_20261030T101421_6ac599e1.json`

It records:

```text
passed: true
end_reason: route_complete
stop_verified: true
fault: null
```

The recorded final speed is effectively zero (`3.1974423109204504e-12 m/s`), and the observer recorded five stop-confirmation samples. The raw evidence was added in commit `3f5642f` (`Add successful HIL route evidence`).

The JSON field `baseline_commit=52331fde756991988cbb94f01e6d33d615c8d7df` identifies the historical HIL-10 baseline used by the test format. It must **not** be interpreted as the source revision of this later route run.

The referenced baseline commit is not missing repository history. `52331fde756991988cbb94f01e6d33d615c8d7df` exists, is reachable from the current `hil-development` HEAD, and is the historical commit `Update project README for completed HIL-9 milestone`. It does not appear in short recent-commit listings simply because the branch history is longer than those listings.

The startup-diagnostics source exercised by the later route run was subsequently committed as `b5210da` (`Add HIL startup readiness diagnostics`). The exact deployed R2100 release-directory name and active target configuration for that older run are not encoded in the committed result and should not be inferred.

The 2026-11-02 fresh-checkout acceptance fixes that traceability gap by explicitly recording the source revision, deployed release and build/deployment hashes together with the committed route result pair.

## Scope boundary

The current deliverable validates the staged **SIL → HIL** development and acceptance workflow. Vehicle-in-the-Loop (VIL) is out of scope.

## Documentation index

- `README.md` — project overview and current reference points.
- `REQUIREMENTS.md` — sanitised course/CAMINO requirements map with implementation, verification, evidence and remaining gaps.
- `edge-path-project/gazebo/README.md` — detailed Gazebo SIL architecture, v6.4 evidence and run instructions.
- `edge-path-project/gazebo/results/README.md` — result-file conventions and current SIL reporting baseline.
- [JARCRAC SIL results](edge-path-project/gazebo/results/jarcrac/README.md) — experimental wheel-based and tracked-model results and their limits.
- `edge-path-project/hil/README.md` — detailed R2100 HIL architecture, deployment, preflight, diagnostics, tests and hardware evidence.
- `edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md` — physical R2100 acceptance record for the standalone ARM64 executable.
- `.github/workflows/offline-validation.yml` — automated offline test and deployment-build validation.
- `VALIDATION.md` — this cross-project baseline/evidence index.

## Requirements documentation

Public repository documentation uses the sanitised `REQUIREMENTS.md` summary rather than copying course source documents verbatim. Source documents may contain environment-specific connection details or credentials that are not requirements and must not be published as project documentation.

The requirements map links each objective to:

1. implementation,
2. reproducible test command or validation procedure,
3. recorded evidence,
4. current status or known gap.

## Remaining acceptance work

The verified R2100 baseline tag, common SIL/HIL emergency-braking scenario, independent HIL runner process-termination fail-safe, threshold-consistency regression and native ARM64 deployment acceptance are complete. No additional navigation or local-planning feature is required to close the current embedded AEB scope.

Remaining work is optional refinement:

1. optional unification of SIL/HIL result output into one comparable result schema,
2. optional lifecycle automation for Gazebo and R2100 runtime startup/cleanup,
3. optional second common SIL/HIL scenario to demonstrate runner generalisation,
4. optional PC power-loss and repeatable CAN fault testing beyond the reported bench trial.
