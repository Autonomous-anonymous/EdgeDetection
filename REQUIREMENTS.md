# Sanitised course and project requirements map

This file summarises the project objectives derived from the course material and CAMINO introduction without reproducing environment-specific connection details or credentials.

The purpose of this map is traceability: each objective is linked to the current implementation, a reproducible verification command where available, recorded evidence and any remaining gap.

## Requirements traceability

| Objective | Current implementation | Verification / evidence | Assessment |
| --- | --- | --- | --- |
| Existing vehicle and environment models | Gazebo vehicle models and the Ruskotunturi environment are used as the current simulation basis. | `edge-path-project/gazebo/README.md`; `edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_v6_4_results.csv` | **Implemented.** The course explicitly permits existing Gazebo models. |
| Virtual sensors and an ADAS controller | Gazebo LiDAR and RGB camera work, AEB, obstacle avoidance, camera + LiDAR fusion work and multi-vehicle coordination are implemented in SIL. | `python3 -m unittest discover -s tests -p 'test_final_sil_v6.py' -v`; committed v6.4 SIL result and trace | **Implemented in SIL.** Current embedded HIL AEB uses the LiDAR-based safety path rather than camera + LiDAR fusion. |
| Simple collision-avoidance / emergency-braking function | Static avoidance, moving-obstacle interaction, TTC logic and emergency braking are implemented across the staged SIL/HIL work. | SIL v6.4 PASS evidence; HIL-10 hardware validation documented in `edge-path-project/hil/README.md` | **Implemented.** A dedicated oncoming-obstacle active-avoidance demo remains a useful next demo milestone. |
| Embedded controller with physical CAN | The R2100 receives simulated vehicle/sensor data, maps it to physical CAN, evaluates the AEB path and returns brake state to Gazebo. | HIL-10 hardware scenarios; R2100 deployment/preflight evidence in `edge-path-project/hil/README.md` | **Implemented and hardware-validated.** |
| Automatic signal mapping and configuration between virtual signals and CAN | Shared JSON profiles configure CAN interfaces, CAN IDs, directions, encoding and scalar scaling. Gateway, perception and AEB consume the same mapping. | `python3 -m unittest discover -s tests -p 'test_signal*.py' -v`; alternate-ID and alternate-scaling hardware runs documented in the HIL guide | **Substantially implemented.** Automatic discovery of arbitrary simulator signals and fully automatic phase-to-phase transformation are not yet demonstrated. |
| Executable deployment before transfer to embedded hardware | The R2100 runtime is built as a Python zipapp with external configuration and managed deployment scripts. | `python3 edge-path-project/hil/deployment/build_r2100_hil.py`; `python3 edge-path-project/hil/dist/r2100_hil/r2100_hil.pyz check`; `python3 -m unittest discover -s tests -p 'test_embedded_deployment.py' -v` | **Implemented.** The target still requires Python 3; the course wording does not explicitly require a Python-free native binary. |
| Documentation and version-control workflow | Root, Gazebo, HIL and validation documentation identify the staged workflow and current baselines. Development uses dedicated branches. | `README.md`; `VALIDATION.md`; `edge-path-project/gazebo/README.md`; `edge-path-project/hil/README.md` | **Established.** Final fresh-checkout acceptance and a verified tag remain recommended before final delivery. |
| Multiple vehicle instances | The v6.4 Ruskotunturi SIL baseline uses two vehicles, independent routes, coordination and a common loading-zone objective. A hybrid two-vehicle HIL mission has also passed. | v6.4 result/trace; HIL-9D / later hybrid evidence in the HIL guide | **Implemented and demonstrated.** |
| Reproducibility and independent work packages | Offline tests, versioned deployment, configuration checks, target-side preflight, startup diagnostics and retained evidence reduce hidden setup dependencies. | `python3 -m unittest discover -s tests -v`; deployment workflow; `VALIDATION.md` | **Strong baseline established.** A formal fresh-checkout acceptance run should still be recorded and tagged. |
| Staged SIL -> HIL -> VIL methodology with less manual intervention | SIL and HIL use related Ruskotunturi scenarios and shared signal semantics; HIL deployment/readiness is automated. | Current SIL/HIL guides and validation evidence | **Partially complete relative to the broader CAMINO goal.** A common scenario definition and runner that selects SIL or HIL is the next focused automation milestone. |

## Current verification baseline

Run the complete offline suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Verified local result on 2026-10-30:

```text
Ran 152 tests
OK
```

The GitHub Actions workflow `.github/workflows/offline-validation.yml` runs the same offline suite and independently builds and validates the R2100 deployment package on pushes and pull requests.

## Evidence limitations

The successful diagnostics-integrated HIL route was observed with:

```text
HIL-10 RESULT: PASS
End reason: route_complete
```

and the originating local filename is known as:

```text
hil10_route_20261030T101421_6ac599e1.json
```

That raw result file is not currently committed. The source content exercised by the run was subsequently committed as `b5210da` (`Add HIL startup readiness diagnostics`), but the exact R2100 release-directory name and active target configuration for that specific run are not encoded in current committed evidence. They must not be inferred after the fact.

Before final acceptance, the preferred evidence record for a hardware run should include:

- repository commit SHA,
- selected signal-mapping profile,
- deployed release identifier or manifest hash,
- test command and selected R2100 host,
- raw result file,
- PASS / FAIL outcome and end reason.

## Remaining recommended milestones

1. Commit the original successful standalone route result from the machine that produced it.
2. Perform a fresh-checkout acceptance run by another team member and record undocumented setup steps.
3. Tag the verified baseline after that acceptance run.
4. Add one common scenario definition/runner that selects SIL or HIL, applies the matching configuration, checks readiness and collects comparable results.
5. If time remains, add an independent command watchdog and process-termination tests.
