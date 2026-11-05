# R2100 native executable acceptance

This record documents the final target-hardware acceptance of the standalone R2100 HIL executable required for deployment before transfer to the embedded device.

## Acceptance date

2026-11-03

## Target platform

The physical Seeed Studio reComputer Industrial R2100 reported:

```text
architecture : aarch64
OS           : Debian GNU/Linux 12 (bookworm)
glibc        : 2.36
kernel       : Linux 6.12.93+rpt-rpi-2712
```

The deployment artifact was built as an ARM64 standalone Linux executable and was first exercised in a Debian 12 ARM64 container before transfer to the R2100.

## Executable identity on the R2100

The transferred runtime was inspected on the physical target with `file r2100_hil` and reported:

```text
ELF 64-bit LSB executable, ARM aarch64
interpreter /lib/ld-linux-aarch64.so.1
for GNU/Linux 3.7.0
stripped
```

This confirms that the deployed artifact is a native ARM64 ELF executable rather than the earlier Python zipapp deployment format.

## Deployment configuration check

The executable was run directly on the R2100:

```bash
./r2100_hil check
```

Result:

```text
Deployment configuration: PASS

Profile             : ruskotunturi_hil
Schema version      : 1
CAN bitrate         : 500000
Gateway interface   : can0
Controller interface: can1
```

The active signal mapping was loaded successfully:

```text
vehicle_speed        CAN=0x100  simulation_to_controller
obstacle_distance    CAN=0x101  perception_to_controller
brake_command        CAN=0x200  controller_to_simulation
```

## Target preflight

The native executable then ran the R2100 readiness check:

```bash
./r2100_hil preflight
```

Result:

```text
[PASS] config: ruskotunturi_hil
[PASS] CAN can0: UP, ERROR-ACTIVE, bitrate=500000
[PASS] CAN can1: UP, ERROR-ACTIVE, bitrate=500000
[PASS] UDP 15000: available for gateway
[PASS] UDP 15010: available for LiDAR perception

R2100 HIL preflight: PASS
```


## Build identity and reproducible artifact hashes

**Historical evidence limitation:** The physical acceptance on 2026-11-03 recorded the ELF identity and the target-side `check` / `preflight` results. It did **not** capture a source commit or SHA-256 digest for the precise binary and manifest inspected that day. The existing results are valid for native compatibility and readiness, but must not be described as a cryptographically traced full native HIL scenario.

The ARM64 GitHub Actions job now builds inside a **Debian 12 ARM64 container** to match the target's older glibc userspace. Its downloadable `r2100-hil-aarch64` artifact contains:

- `r2100_hil-aarch64.tar.gz` — the packaged native executable, config and managed scripts;
- `r2100-hil-aarch64.provenance.txt` — source Git commit, builder image, target architecture, and SHA-256 digests of the executable, manifest and archive.

After downloading the CI artifact, verify the archive SHA-256 against the provenance file before deployment. After extracting the release on the R2100, run `sha256sum r2100_hil manifest.json` inside its deployment directory and compare both values against `binary_sha256` and `manifest_sha256` in the corresponding provenance file. Record the matching source commit and run timestamp with the acceptance result.

These new CI checks prove that the build *runs in Debian 12* and produces a traceable artifact. They do **not** themselves prove that a particular CI-built binary has subsequently been exercised on the physical R2100.

## Full native-executable HIL AEB acceptance — 2026-11-05

**Operator-verified PASS on the physical R2100.** This test used the **CI-built Debian 12 ARM64 release**, rather than the historical zipapp deployment. The operator supplied the target terminal output, the stage-runner result, and the key fields from the generated HIL JSON.

### Source and exact deployed artifact

| Field | Verified value |
| --- | --- |
| Source commit / CI release | `46b51c5c772c2a4f37d54b8a83b1f8e5ce6862eb` |
| CI builder image / architecture | `debian:12` / `aarch64` |
| CI workflow run | [main push run 41980942845](https://github.com/Autonomous-anonymous/EdgeDetection/actions/runs/41980942845) |
| R2100 release directory | `~/r2100_hil/releases/r2100_hil_ci_46b51c5_20261105` |
| Native executable SHA-256 | `cc83d60385bdf0db5cf8490c54e87d026e66930a528f1d15ec6e91802fbef6cc` |
| Manifest SHA-256 | `4444db47705b97dce249aa17df439e6b4986830bb5f53f4171f1c8f3fe17cf10` |
| Archive SHA-256 | `7556aed3f7cd9ebc7d12a4506d75b50d170b3d99cb0623aa96b24bc9f94e9700` |

The operator verified the downloaded archive digest on WSL and the extracted binary and manifest against the provenance file. The R2100 then reported the **same binary and manifest hashes** from the deployed release directory.

### Native physical runtime and scenario

The new release passed `./r2100_hil check` and `./r2100_hil preflight` on the R2100 (Debian 12, `aarch64`; CAN0/CAN1 UP, ERROR-ACTIVE at 500 kbit/s; UDP ports 15000/15010 available). `./start_hil.sh` reported **R2100 HIL startup: PASS**, and `./status_hil.sh` showed all three components running from the native release:

```text
perception PID=2294 RUNNING
gateway    PID=2295 RUNNING
aeb        PID=2296 RUNNING
```

The R2100 runtime log directory was:

```text
/home/recomputer/hil_test_logs/r2100_hil_20261105T065838Z
```

On WSL, the operator ran the same shared HIL scenario used for the earlier zipapp tests:

```bash
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking \
  --target hil \
  --r2100-host <R2100_IP>
```

The stage runner reported raw LiDAR transport to the R2100, Vehicle B motion at 0.850 m/s, and an AEB zero-speed command with reason `ttc_threshold`.

### Operator-verified JSON acceptance values

| Acceptance field | Observed result |
| --- | --- |
| `passed` | `true` |
| `end_reason` | `aeb_stop_verified` |
| `aeb_reason` | `ttc_threshold` |
| `stop_verified` | `true` |
| `observer_stop_samples` | `4` (minimum 3) |
| `final_speed_mps` | `3.730349362740526e-12` (threshold 0.05 m/s) |
| `clearance_m` | `1.2356799457994787` (minimum 1.0 m) |
| `required_clearance_m` | `1.0` |
| `fault` | `null` |
| Terminal test result | `HIL-10 RESULT: PASS` |

The generated result file on the operator's WSL checkout was:

```text
edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.json
```

**Evidence retention status:** The result above is supported by operator-supplied terminal output and a field-by-field reading of the local JSON. This documentation update does **not** claim that the raw 2026-11-05 JSON/CSV or the R2100 process logs have been committed. Commit the original unmodified raw results as a separate evidence change; check the CSV filename and hash rather than reconstructing measurements from this summary. The older 2026-11-02 zipapp evidence remains committed and distinct.

This closes the previously missing **functional runtime demonstration with the CI-built standalone ARM64 executable**; it does not claim the full system would remain safe under PC power loss, physical CAN cable disconnection or bus-off.

## Acceptance conclusion

The standalone deployment requirement is **hardware-validated on the physical R2100**.

The final acceptance demonstrates that:

- the controller runtime is packaged before transfer as a standalone native executable,
- the artifact is ARM64/aarch64 and executes on the target Debian 12 userspace,
- the runtime does not depend on the project Python virtual environment on the target,
- the generated deployment configuration is valid,
- both physical CAN interfaces satisfy the expected state and bitrate,
- the required UDP ports are available,
- the target-side HIL preflight passes from the native executable.

This closes the remaining target-compatibility caveat for the executable-before-deployment course requirement. The existing SIL/HIL functional evidence remains separate and is indexed in `VALIDATION.md` and `REQUIREMENTS.md`.
