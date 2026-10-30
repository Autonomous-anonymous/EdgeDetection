# Validation and documentation index

This file identifies the current SIL/HIL baselines, the role of the different preflight checks, and the evidence that should be used for current reporting.

It is intentionally small and does not replace the detailed Gazebo or HIL guides.

## Current baselines

### Software-in-the-Loop

Current SIL baseline: **Ruskotunturi final SIL v6.4**.

Committed evidence:

- `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv`
- `edge-path-project/gazebo/results/final_sil_v6_4_trace_20261026T133025_782767.csv`

The committed v6.4 result is `PASS` with `end_reason=final_demo_complete` and no failed checks. It records both vehicle mission states, camera-assisted static avoidance, moving-obstacle interaction, coordination and loading-zone safety checks.

### Hardware-in-the-Loop

Current platform name: **Ruskotunturi R2100 HIL**.

The historical functional suite name remains **HIL-10 Freshness and Fault Validation**. Later work packages extend the same platform with configurable signal mapping, configuration-driven scalar codecs, embedded deployment, R2100 deployment preflight and WSL-side startup-readiness diagnostics.

The current offline repository suite was run on 2026-10-30 after updating the HIL-10 controller fixture for the configuration-driven signal mapping:

```text
Ran 152 tests
OK
```

Run the current offline suite from the repository root with:

```bash
python3 -m unittest discover -s tests -v
```

## Preflight terminology

There are three different checks with different purposes. They should not be treated as interchangeable.

### Historical baseline integrity check

```bash
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
```

`hil10_preflight.py` compares a small dependency set against the original HIL-9-derived snapshot used when HIL-10 freshness testing was introduced. It is retained as development-history evidence.

Because the runtime has since gained signal mapping, codecs, deployment and diagnostics changes, `DIFFERS` results are expected for changed dependencies. This script is **not** the current runtime-readiness preflight.

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
| Full repository offline suite | 152 / 152 PASS | Verified 2026-10-30 after updating the HIL-10 test fixture. |

### Standalone route evidence

The committed file

`edge-path-project/gazebo/results/hil10/hil10_route_20261029T092701_b95d92cd.json`

is intentionally retained as development history. It records:

```text
passed: false
end_reason: interrupted
```

A later route run was successfully validated with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

The originating local result filename is:

`hil10_route_20261030T101421_6ac599e1.json`

That raw JSON file is not currently present in GitHub. It must be committed from the machine that produced it before it can be treated as committed raw evidence. This catalogue records the known validation result without inventing missing JSON fields.

The startup-diagnostics source exercised by that validation was subsequently committed as `b5210da` (`Add HIL startup readiness diagnostics`). The exact deployed R2100 release-directory name used by that route run is not recorded in the repository evidence and should not be inferred.

## Documentation index

- `README.md` — project overview and current reference points.
- `edge-path-project/gazebo/README.md` — detailed Gazebo SIL architecture, v6.4 evidence and run instructions.
- `edge-path-project/gazebo/results/README.md` — result-file conventions and current SIL reporting baseline.
- `edge-path-project/hil/README.md` — detailed R2100 HIL architecture, deployment, preflight, diagnostics, tests and hardware evidence.
- `VALIDATION.md` — this cross-project baseline/evidence index.

## Requirements documentation

Public repository documentation should use a sanitised requirements summary rather than copying course source documents verbatim. Source documents may contain environment-specific connection details or credentials that are not requirements and must not be published as project documentation.

A later requirements map should link each sanitised course/project objective to:

1. implementation,
2. reproducible test command,
3. recorded evidence,
4. current status or known gap.
