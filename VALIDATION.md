# Validation and documentation index

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

The historical functional suite name remains **HIL-10 Freshness and Fault Validation**. Later work packages extend the same platform with configurable signal mapping, configuration-driven scalar codecs, embedded deployment, R2100 deployment preflight, WSL-side startup-readiness diagnostics and a common SIL/HIL emergency-braking scenario.

The verified R2100 baseline is tagged:

```text
r2100-hil-verified-2026-11-02
```

That tag closes the fresh-checkout build/deploy/physical-CAN acceptance baseline. The common staged emergency-braking scenario was added afterwards on `hil-development`.

The current offline repository suite was rerun on 2026-11-02 after adding the staged-scenario configuration and runner tests:

```text
Ran 158 tests
OK
```

Run the current offline suite from the repository root with:

```bash
python3 -m unittest discover -s tests -v
```

The same command is part of `.github/workflows/offline-validation.yml`. The workflow also builds the R2100 zipapp, validates its generated configuration and runs the deployment regression tests.

## Fresh-checkout reproducibility acceptance — 2026-11-02

A formal fresh checkout of `hil-development` at source revision
`ed1bb2e8496c36965f51a2fcb70af938777f872f` was validated end-to-end on 2026-11-02.

The verification used a separate clean checkout with no files copied from the development working tree. The clean checkout was on Python 3.12.3 / x86_64 WSL and had a clean Git worktree before the validation steps.

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

The fresh build and deployed target were verified to contain the same artifacts:

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

A shared staged scenario now defines one equivalent emergency-braking acceptance case for both execution targets:

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

The SIL target uses `ruskotunturi_emergency_braking_sil.py`. The HIL target uses the existing `ruskotunturi_hil10_b_test.py --mode aeb`, preserving the validated embedded raw-LiDAR -> R2100 perception -> physical CAN -> R2100 AEB -> returned brake path.

Verified operator runs on 2026-11-02 produced:

```text
SIL target
result                     PASS
end reason                 aeb_stop_verified
AEB reason                 ttc_threshold
trigger LiDAR clearance    1.374 m
trigger speed              0.850 m/s
trigger TTC                1.617 s
ground-truth stop clearance 1.253 m

HIL target
result                     PASS
end reason                 aeb_stop_verified
AEB reason                 ttc_threshold
raw LiDAR transport        active
physical-CAN AEB path      exercised
```

The HIL run generated the target-specific result file `hil10_aeb_20261102T102033_e21c05af.json` in the operator checkout. It is not part of the earlier verified-baseline tag and should not be confused with the committed fresh-checkout route evidence.

The first HIL attempt intentionally failed closed with `startup_brake_reply_missing` while the R2100 runtime was not returning brake state. After the managed R2100 runtime was started, the same scenario passed. This is consistent with the startup-readiness diagnostics design and is not used as PASS evidence.

The new offline staged-scenario tests pass **6 / 6**, and the complete repository suite passes **158 / 158**.

The common runner currently selects the target, validates the shared scenario and starts the target-specific backend. Gazebo world lifecycle and R2100 runtime lifecycle remain explicit operator steps, and target-specific result formats have not yet been replaced by one unified result schema.

## Preflight terminology

There are three different checks with different purposes. They should not be treated as interchangeable.

### Historical baseline integrity check

```bash
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
```

`hil10_preflight.py` compares a small dependency set against the original HIL-9-derived snapshot used when HIL-10 freshness testing was introduced. It is retained as development-history evidence.

Because the runtime has since gained signal mapping, codecs, deployment and diagnostics changes, `DIFFERS` results are expected for changed dependencies. This script is **not** the current runtime-readiness preflight. References to this command in the historical HIL-10 offline-validation material describe the original milestone procedure rather than the current startup procedure.

### Current R2100 runtime readiness

On the deployed R2100 package:

```bash
~/r2100_hil/current/r2100_hil.pyz preflight
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
| Signal regression | 30 / 30 PASS | Current mapping/codec regression documented in the HIL guide. |
| Deployment tests | 8 / 8 PASS | Embedded package build/check coverage documented in the HIL guide. |
| R2100 deployment preflight unit tests | 4 / 4 PASS | Target-side preflight coverage. |
| HIL startup diagnostics | 7 / 7 PASS | Dedicated diagnostics unit tests. |
| HIL integration regression | 18 / 18 PASS | Integration tests after startup diagnostics. |
| Common staged-scenario tests | 6 / 6 PASS | Scenario schema, target command construction, HIL host requirement and both dry-run paths. |
| Common emergency-braking SIL run | PASS | `aeb_stop_verified`, `ttc_threshold`, 1.253 m ground-truth stop clearance. |
| Common emergency-braking HIL run | PASS | `aeb_stop_verified`, `ttc_threshold`; R2100 and physical-CAN AEB path exercised. |
| Full repository offline suite | 158 / 158 PASS | Verified after adding the common staged scenario on 2026-11-02. |
| GitHub Actions offline validation | PASS | Full offline suite plus deployment build/configuration check and deployment regression tests. |
| Fresh-checkout R2100 acceptance | PASS | Clean checkout -> build -> deploy -> preflight -> managed runtime -> physical-CAN route; `route_complete`. |
| Verified baseline tag | complete | `r2100-hil-verified-2026-11-02`; closes the pre-common-scenario R2100 acceptance baseline. |

### Standalone route evidence

The earlier committed file

`edge-path-project/gazebo/results/hil10/hil10_route_20261029T092701_b95d92cd.json`

is intentionally retained as development history. It records:

```text
passed: false
end_reason: interrupted
```

A later successful standalone route result is now also committed:

`edge-path-project/gazebo/results/hil10/hil10_route_20261030T101421_6ac599e1.json`

It records:

```text
passed: true
end_reason: route_complete
stop_verified: true
fault: null
```

The recorded final speed is effectively zero (`3.1974423109204504e-12 m/s`), and the observer recorded five stop-confirmation samples. The raw evidence was added in commit `3f5642f` (`Add successful HIL route evidence`).

The JSON field `baseline_commit=52331fde756991988cbb94f01e6d33d615c8d7df` identifies the historical HIL-10 baseline used by the test format. It must **not** be interpreted as the source revision of this later route run. The startup-diagnostics source exercised by the run was subsequently committed as `b5210da` (`Add HIL startup readiness diagnostics`). The exact deployed R2100 release-directory name and active target configuration for that specific run are not encoded in the committed result and should not be inferred.

The 2026-11-02 fresh-checkout acceptance fixes that traceability gap by explicitly recording the source revision, deployed release and build/deployment hashes together with the committed route result pair.

## Documentation index

- `README.md` — project overview and current reference points.
- `REQUIREMENTS.md` — sanitised course/CAMINO requirements map with implementation, verification, evidence and remaining gaps.
- `edge-path-project/gazebo/README.md` — detailed Gazebo SIL architecture, v6.4 evidence and run instructions.
- `edge-path-project/gazebo/results/README.md` — result-file conventions and current SIL reporting baseline.
- `edge-path-project/hil/README.md` — detailed R2100 HIL architecture, deployment, preflight, diagnostics, tests and hardware evidence.
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

The verified R2100 baseline tag and the first common SIL/HIL emergency-braking scenario are complete. No additional navigation or local-planning feature is required to close the current embedded AEB scope.

If time remains, the most useful extensions are:

1. independent command-watchdog / process-termination fail-safe tests on the HIL path,
2. optional unification of SIL/HIL result output into one comparable result schema,
3. optional lifecycle automation for Gazebo and R2100 runtime startup/cleanup.
