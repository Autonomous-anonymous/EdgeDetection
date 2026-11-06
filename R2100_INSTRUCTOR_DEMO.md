# R2100 instructor demo — physical CAN HIL

This is the **single walkthrough** for showing how the real Seeed Studio reComputer Industrial R2100 is used in CAMINO. The accepted HIL baseline uses the existing `aeb_vehicle_360_b` Gazebo vehicle, **not the experimental JARCRAC v2**. The JARCRAC v2 32-layer LiDAR AEB has been tested in **SIL only**.

## 1. Explain the architecture

- **WSL/Gazebo:** virtual vehicle, world, LiDAR and odometry. Steering stays on the PC.
- **UDP 15010:** raw virtual LiDAR sent to the real R2100.
- **R2100 perception/gateway:** extract nearest obstacle, encode speed and distance onto **physical CAN0**.
- **CAN0 ↔ CAN1:** physically wired H-to-H and L-to-L, configured for **500 kbit/s**. No real vehicle CAN network or actuator is required.
- **R2100 AEB controller:** receives physical CAN, applies time-to-collision/emergency-distance checks, publishes CAN brake command.
- **UDP 15000:** returned braking state applied to the virtual Gazebo vehicle.

Default CAN IDs are `0x100` speed, `0x101` obstacle distance, `0x200` brake command.

## 2. Show the physical R2100 and its native release

From a laptop terminal, connect using the device's currently reachable IP address:

```bash
ssh recomputer@<R2100_IP>
uname -m
cat /etc/os-release | head -6
cd ~/r2100_hil/releases/r2100_hil_ci_46b51c5_20261105
file ./r2100_hil
./r2100_hil check
```

The accepted 2026-11-05 release was a **Debian 12 ARM64/aarch64 standalone ELF**, not a Python zipapp. If that exact directory is absent, inspect the existing releases and consult [native acceptance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md); do **not** assume `~/r2100_hil/current` is the accepted native release.

Then, from the same release directory, before starting other HIL components:

```bash
./r2100_hil preflight
./start_hil.sh
./status_hil.sh
ip -details link show can0
ip -details link show can1
```

Expected: preflight PASS, both real CAN interfaces UP at 500000 bitrate, and the **perception, gateway and aeb** processes running. Optionally use `candump can0 can1` in a second R2100 terminal during the trial if `can-utils` is installed. Physical traffic is the distinguishing evidence; `vcan` traffic alone is not physical HIL.

## 3. Run the shared emergency-braking scenario through the R2100

Close other Gazebo sessions and controllers first. On **WSL terminal 1**:

```bash
cd ~/edge-path-project-git
git switch camino
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

On **WSL terminal 2**:

```bash
cd ~/edge-path-project-git
read -r -p "Reachable R2100 IP: " R2100_IP
python3 -u edge-path-project/gazebo/scripts/run_staged_scenario.py \
  --scenario emergency_braking --target hil --r2100-host "$R2100_IP"
```

Explain the comparison: `--target sil` keeps AEB in the PC; `--target hil` routes perception and AEB through the real R2100 and the physical CAN wiring. **Both** use the Gazebo vehicle and obstacle world. Restart the world before changing targets. The R2100 must actually be running; a Gazebo-only scene is not a hardware demo.

## 4. Show the previous hardware-validated result

The verified **2026-11-05 native physical-CAN HIL AEB** completed:

| Field | Evidence |
| --- | --- |
| Result | `HIL-10 RESULT: PASS` |
| End reason | `aeb_stop_verified` |
| Trigger reason | `ttc_threshold` |
| Final speed | approximately 0 m/s |
| Observed obstacle clearance | **1.2356799458 m** (requirement: ≥1 m) |
| Independent stop samples | 4 |
| Fault | `null` |

- [Accepted native executable and hash provenance](edge-path-project/hil/NATIVE_DEPLOYMENT_ACCEPTANCE.md)
- [Committed original HIL JSON](edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.json)
- [Committed matching HIL CSV](edge-path-project/gazebo/results/hil10/hil10_aeb_20261105T070006_b18e73a4.csv)
- [Full HIL instructions](edge-path-project/hil/README.md)

If the device/network is unavailable during the meeting, show these **existing** evidence files; do not claim a new live run passed. The accepted R2100 process logs remained on the device under `/home/recomputer/hil_test_logs/r2100_hil_20261105T065838Z`, not in Git.

## 5. Shut down

On the R2100 from the same deployed release directory:

```bash
./stop_hil.sh
./status_hil.sh
```

**Scope:** This is physical R2100 **HIL**, not Vehicle-in-the-Loop or real machinery operation. There is no connection to a real vehicle actuator. A separate JARCRAC-specific R2100 stage has not yet been implemented. The JARCRAC v2 trial currently demonstrates flat-ground **SIL** only.
