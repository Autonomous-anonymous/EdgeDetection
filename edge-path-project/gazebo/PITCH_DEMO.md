# Two vehicles, two obstacles

Use this short Gazebo scene during a five-minute CAMINO pitch. The projector shows the two branches of the Ruskotunturi route from one fixed viewpoint. The turquoise A steers around a stationary orange barrier. The violet B sees a yellow obstacle cross its path, stops, and drives on when the crossing clears. The Finnish status window can be dragged beside the scene.

## At the presentation computer

Use the team's Gazebo Harmonic installation on Ubuntu 24.04 or WSL with a working GUI and the system Python bindings `gz.transport13` and `gz.msgs10`. Install `python3-tk` for the optional presenter window. Run from the repository root.

**SIL rehearsal**

```bash
python3 edge-path-project/gazebo/scripts/run_pitch_demo.py sil
```

**Physical CAN rehearsal or presentation**

1. Use the team's wired R2100 bench. Check the configured CAN interfaces, termination, bitrate and the reachable Ethernet address in the [R2100 setup guide](../hil/README.md). Start a fresh deployed runtime on the **R2100** with the first command below. Its preflight checks both CAN interfaces and the UDP ports.
2. Run the second command on the **presentation computer**, using the board's actual IP address. The launcher owns B's raw LiDAR sender and the speed/brake link.
3. Let the world appear while introducing the problem. Press **Enter** when the audience is ready. Leave the final frame on screen for the closing sentence. Press **Enter** again to close. Starting the launcher again creates a fresh world and transport partition. For another HIL run, stop the previous bench runtime with `~/r2100_hil/current/stop_hil.sh` and start it again.

```bash
# R2100
~/r2100_hil/current/start_hil.sh
```

```bash
# Presentation computer. Replace this example address with your bench address.
python3 edge-path-project/gazebo/scripts/run_pitch_demo.py hil --r2100-host 192.168.50.2
```

The archived v6.4 SIL trace places B's first hold at about 8 simulated seconds, its restart at about 24 and A's complete pass at about 36. Allow about 40 seconds of stage time on a computer sustaining real time. Cold startup is separate. Rehearse on the projector computer to measure its actual duration.

## What the audience sees

| Projector cue | What to point out |
| --- | --- |
| The two labeled cars approach on separate branches | Both vehicles use the same virtual worksite. Each needs a different response. |
| **A · Väistää estettä** | A's camera confirms the orange object. Its LiDAR helps select the side for the detour. |
| **B · Odottaa** | A separate observer has seen B come to rest near the crossing. The ordinary yield decision uses B's own LiDAR. |
| **B · Jatkaa matkaa** | The obstacle crossed and the observer measured B moving again. In HIL, every B motion request also passes the R2100 brake gate. |
| **Ajo valmis** | A's whole footprint passed the far edge of the barrier. Both vehicles received a final zero command and their odometry confirmed standstill. |

**Suggested closing line in Finnish**

> Sama työmaa, kaksi eri päätöstä. A löysi reitin esteen ohi. B antoi tietä ja jatkoi, kun reitti vapautui. Tällaisia tilanteita voimme harjoitella ja mitata ennen ensimmäistä kenttäkoetta.

For the CAN run, add *“B:n ajolupa kulki tässä ajossa oikean ohjaimen ja fyysisen CAN-väylän kautta.”* Use this line after rehearsing with the wired bench.

## What CAN controls

Vehicle A stays in SIL. In HIL the computer sends B's measured odometry and raw LiDAR to the R2100 over Ethernet. With the default project mapping, the gateway and perception processes place speed (`0x100`) and distance (`0x101`) on `can0`. The AEB controller on `can1` returns the brake decision (`0x200`) across the physical bus. The gateway relays that decision to Gazebo. The panel's *ajolupa* and rolling counter come from accepted replies. A remains under its local controller while the existing independent watchdog supervises both cars.

A planned yield can resume after distinct clear scans and a fresh DRIVE. A hardware emergency brake, stale odometry, missing brake replies or a sender error ends the scene and requests zero for both cars. If a CAN run ends early, read the reported reason and the bench status. The launcher never changes mode during a run. For an independent wire-level check, run `candump -tz can1` on the R2100 and watch the three configured identifiers while presenting.

## Run evidence

Each run produces its own CSV of requested commands and observed poses, plus a JSON report under `edge-path-project/gazebo/results/pitch/`. The report includes the actual mode, event times, minimum observed clearance, both final stop checks, and, in HIL, the number of raw scans, speed packets and accepted brake replies. A win requires the measured pass, the crossing stop and restart, scene safety checks, and three fresh standstill samples per vehicle. A run times out after 70 simulation seconds or 100 wall seconds of driving. This result covers the opening scene. The full v6.4 result separately covers the merge and summit loading mission.

For a headless rehearsal on a machine with working OGRE2/EGL rendering, run the same launcher with `sil --headless --auto-start`. For a projected run using terminal labels only, add `--no-panel`.
