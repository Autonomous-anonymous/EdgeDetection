# Two vehicles, two obstacles

Use this Gazebo scene during a five-minute CAMINO pitch. The turquoise A steers around a stationary orange barrier while the violet B yields to a crossing obstacle. Their two branches meet at the orange junction. A leads onto the dashed exit lane and B follows. Both drive down the same road before the scene ends.

Both cars use a LiDAR/TTC command guard. Before its detour, A first brakes and stops. It then observes the LiDAR surface for at least two seconds while measured odometry confirms standstill. A constant nearest range alone is insufficient: the visible surface's longitudinal and lateral bounds must remain within 0.15 m, using at least 15 distinct fresh scans with no gap longer than 0.30 s of sensor time. Motion, missing returns, invalid scans or stale speed restart the observation. Camera confirmation remains part of the existing A encounter.

After this observation, A may use its existing detour only if the chosen side has at least 6.7 m of visible range. The independent clearance watchdog still supervises the maneuver. B waits for the moving obstacle to clear and then resumes. A stationary classification does not create a route: the short scene has an existing detour corridor for A; B remains stopped if its branch stays blocked. This is not a general obstacle-tracking or arbitrary-route planner.

TTC is estimated as bumper clearance divided by measured vehicle speed, consistent with the R2100 baseline's stationary-obstacle approximation. The local guard brakes at TTC <= 2.0 s or clearance <= 1.1 m, and limits requested speed to preserve a 3.0 s clearance/speed margin. The R2100's independent 1.7 s TTC and 0.8 m emergency-distance limits keep their existing latched behavior. Neither a stationary classification nor a detour request can override a CAN brake.

## At the presentation computer

Use the team's Gazebo Harmonic installation on Ubuntu 24.04 or WSL with a working GUI and the system Python bindings `gz.transport13` and `gz.msgs10`. Install `python3-tk` for the optional presenter window. Run from the repository root.

If the terminal reports `No module named 'tkinter'`, install the panel dependency with `sudo apt-get install python3-tk` and launch the scene again. Driving and terminal status still work without the panel.

**SIL rehearsal**

```bash
python3 edge-path-project/gazebo/scripts/run_pitch_demo.py sil
```

**Physical CAN rehearsal or presentation**

1. Use the team's wired R2100 bench. Check the configured CAN interfaces, termination, bitrate and the reachable Ethernet address in the [R2100 setup guide](../hil/README.md). Start a fresh deployed runtime on the **R2100** with the first command below. Its preflight checks both CAN interfaces and the UDP ports.
2. Run the second command on the **presentation computer**, using the board's actual IP address. The launcher owns B's raw LiDAR sender and the speed/brake link.
3. Start the launcher before your speaking slot. Wait for **Valmis lähtöön**. The world, sensors, safety watchdog and, in HIL, CAN feedback are checked before this cue. Both vehicles stay parked. Adjust the view, then press **Enter** to start driving. The final frame stays on screen until you press **Enter** again. Starting the launcher again creates a fresh world. For another HIL run, stop the previous bench runtime with `~/r2100_hil/current/stop_hil.sh` and start it again.

```bash
# R2100
~/r2100_hil/current/start_hil.sh
```

```bash
# Presentation computer. Replace this example address with your bench address.
python3 edge-path-project/gazebo/scripts/run_pitch_demo.py hil --r2100-host 192.168.50.2
```

**Framing.** The opening view covers the approaches and the shared exit. Drag with the left mouse button to pan, drag with the middle button to orbit, and use the wheel to zoom. The same controls work while driving and on the final frame. Drag the Finnish status window beside the scene if it covers a car.

**Timing.** The live counter uses the presentation computer's wall clock. The scene has a 55-second live budget from the start cue through verified standstill. A missed budget is recorded as a failed rehearsal. World preparation and the presenter's wait are measured separately. Use the `live_wall_s` from a rehearsal on the projector computer when planning the pitch.

## What the audience sees

| Projector cue | What to point out |
| --- | --- |
| The two labeled cars approach on separate branches | Both vehicles use the same virtual worksite. Each needs a different response. |
| **A · Jarruttaa · LiDAR/TTC**, then **Tutkii esteen liikettä** | A stops first. New LiDAR scans, measured standstill and surface stability determine whether it can start the existing detour. |
| **A · Väistää estettä** | A's camera confirms the orange object. Its LiDAR helps select the side for the detour. |
| **B · Odottaa** | A separate observer has seen B come to rest near the crossing. The ordinary yield decision uses B's own LiDAR. |
| **B · Jatkaa matkaa** | The obstacle crossed and the observer measured B moving again. In HIL, every B motion request also passes the R2100 brake gate. |
| **Yhteisellä tiellä / Seuraa A:ta** | A takes priority at the junction. B enters behind A. Both cars travel along the dashed lane in the same direction with measured headway. |
| **Ajo valmis** | The observer measured both maneuvers and at least 1.2 m of movement by each car together on the exit. Both vehicles' odometry confirmed final standstill. |

**Suggested closing line in Finnish**

> A löysi reitin esteen ohi. B antoi tietä. Katsokaa nyt risteystä. Autot tulivat eri suunnista, ja nyt B seuraa A:ta samalla työmaatiellä. Tämän haluamme voida testata jo ennen kenttäkoetta.

For the CAN run, add *“B:n ajolupa kulki tässä ajossa oikean ohjaimen ja fyysisen CAN-väylän kautta.”* Use this line after rehearsing with the wired bench.

## What CAN controls

Vehicle A stays in SIL. In HIL the computer sends B's measured odometry and raw LiDAR to the R2100 over Ethernet. With the default project mapping, the gateway and perception processes place speed (`0x100`) and distance (`0x101`) on `can0`. The AEB controller on `can1` returns the brake decision (`0x200`) across the physical bus. The gateway relays that decision to Gazebo. The panel's *ajolupa* and rolling counter come from accepted replies. A remains under its local controller while the existing independent watchdog supervises both cars.

A planned yield can resume after distinct clear scans and a fresh DRIVE. A hardware emergency brake, stale odometry in HIL, missing brake replies or a sender error ends the scene and requests zero for both cars. If a CAN run ends early, read the reported reason and the bench status. The launcher never changes mode during a run. For an independent wire-level check, run `candump -tz can1` on the R2100 and watch the three configured identifiers while presenting.

In SIL, a brief odometry delivery gap sends zero to both cars immediately. Recovery requires three new standstill samples from each car plus fresh pose, LiDAR and simulation-clock data. If recovery is not established within two seconds, the run fails. The 0.5 s freshness limit and independent watchdog are unchanged, and the live clock continues during the stop. The report records each pause and the last odometry age for each car; a missing sample is never treated as zero speed. HIL communication faults remain latched and require a fresh run.

## Run evidence

Each run produces its own CSV of requested commands and observed poses, plus a JSON report under `edge-path-project/gazebo/results/pitch/`. The report separates `preparation_wall_s`, `presenter_wait_s`, `live_wall_s` and `simulation_seconds`. It includes event times in both clocks, final positions along the shared lane and headway, measured clearance, both stop checks and, in HIL, raw scan, speed and brake counts. A pass requires the camera-confirmed detour, B's observed stop and restart, both vehicles driving in the same exit corridor with A ahead, the live budget, and three fresh standstill samples per car. The full v6.4 mission continues uphill to its loading zone.

For a headless rehearsal on a machine with working OGRE2/EGL rendering, run the same launcher with `sil --headless --auto-start`. For a projected run using terminal labels only, add `--no-panel`.

The GitHub rehearsal workflow first runs SIL, then launches the same scene with the production R2100 perception, gateway and AEB programs connected to a Linux virtual CAN bus. Its artifacts keep the two results separate and include the virtual bus capture. A physical bench rehearsal adds the board, Ethernet link, transceivers and wiring used at the presentation.

## Previous rehearsal before stop-and-observe

The [5 November 2026 Gazebo run](https://github.com/Autonomous-anonymous/EdgeDetection/actions/runs/41980665706) tested runtime commit `b172c29` on Ubuntu 24.04 with software rendering, before the added stop-and-observe phase. Both runs passed every then-existing scene, clearance, timing and final-standstill check. Use a new rehearsal's report for the revised scene's timing.

| Rehearsal | Preparation before the cue | Actual time from cue to standstill | Shared-lane headway |
| --- | ---: | ---: | ---: |
| Gazebo GUI, SIL | 7.7 s | 39.0 s | 8.00 m |
| Production R2100 programs, virtual CAN, headless Gazebo | 4.6 s | 38.4 s | 8.08 m |

The GUI rehearsal sent the presenter's Enter cue, moved the camera with an actual mouse drag and captured the starting and final views. The CAN rehearsal recorded IDs `0x100`, `0x101` and `0x200`, accepted 771 brake replies and sent 390 raw LiDAR scans. The workflow artifact contains both JSON reports, pose/command traces, screenshots and the bus capture. The same runtime commit passed all 203 offline tests and the ARM64 executable build checks.

These times are measured wall time on that runner, separate from simulation time and preparation. Rehearse on the presentation computer with its display and, when used, the physical R2100/CAN bench; that combination was not tested by this workflow.
