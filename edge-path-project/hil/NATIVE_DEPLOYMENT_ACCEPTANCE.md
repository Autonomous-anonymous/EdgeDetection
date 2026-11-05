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

## Follow-up: full native-executable HIL functional acceptance (not yet recorded)

The 2026-11-02 committed functional AEB and route results used the earlier managed Python zipapp package. To close the remaining *combined-evidence* gap, perform one end-to-end AEB run **after deploying a Debian-12-built ARM64 release** into the managed `~/r2100_hil/current` directory:

1. On the R2100, verify the downloaded binary/manifest SHA-256 digests against that build's provenance, then run `./r2100_hil check` and `./r2100_hil preflight`.
2. On the R2100, launch `./start_hil.sh` and inspect `./status_hil.sh`. The runtime must show active perception, CAN gateway and AEB processes rather than stale PIDs.
3. On WSL, start `edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf` and run the common scenario with `python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py --scenario emergency_braking --target hil --r2100-host <R2100_IP>`.
4. Require `aeb_stop_verified`, a permitted AEB intervention reason, stop confirmation, and the minimum-clearance acceptance condition. Archive the HIL JSON/CSV, runtime logs, timestamp, source commit, and both native artifact hashes.

Do not mark this follow-up as PASS until it has been executed with the native release and its evidence has been inspected and committed.

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
