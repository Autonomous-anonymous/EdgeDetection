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
