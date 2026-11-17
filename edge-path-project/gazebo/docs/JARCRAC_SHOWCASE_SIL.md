# CAMINO five-minute Gazebo showcase

JARCRAC and the original Gazebo Vehicle B share a short course on Ruskotunturi. The tracked vehicle starts at the junction and proceeds first. B yields on the side approach, merges when A clears the junction, and visibly follows A on the shared lane. When A brakes, B stops behind it. Once A has started its bypass and moved far enough ahead, B advances slowly to an observed waiting point. Each vehicle holds at a measured stop for at least five simulated seconds before its own bypass. B waits for the whole of A to clear the maneuver area before turning out. Both rejoin before parking in their marked loading bays.

The course demonstrates the SIL topics in Course Topics.docx that can be seen in a simulation: existing vehicle and environment models, virtual sensors, a Python ADAS controller with collision braking, and multiple vehicles in one Gazebo world. The run can be reproduced locally with one command and produces an independent pose report. The embedded controller, CAN mapping, executable deployment, and phase documentation are evidenced separately through the project's SIL/HIL work.

## Install in a fresh worktree

The integrated patch includes the convoy controller and applies directly to `camino` commit `c5e8a857813628171aadaea653257b56e839a939`. Use the Gazebo Harmonic installation and system Python bindings from the [compact SIL guide](JARCRAC_COMPACT_SIL.md). If the integrated showcase is already installed, run only the last two commands from that worktree.

```bash
git worktree add --detach ../camino-integrated-showcase c5e8a85
cd ../camino-integrated-showcase
git apply --check /path/to/camino-jarcrac-integrated-showcase.patch
git apply /path/to/camino-jarcrac-integrated-showcase.patch
git apply --check /path/to/camino-jarcrac-showcase-headstart.patch
git apply /path/to/camino-jarcrac-showcase-headstart.patch
```

## Run once or loop

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario showcase --out /tmp/camino-showcase-first
```

The GUI opens to an overview of the course. The junction outline is yellow, the crate is orange, and the green and blue loading bays are JARCRAC's and B's goals. The paint follows the terrain and has no collision geometry. In the **Visualize Lidar** panel, select the ray display and choose `/camino/cooperative/a/lidar` for JARCRAC's front 180° scan or `/camino/cooperative/b/lidar` for B's 360° scan. The panel uses Gazebo's topic selector. Check the selection when a fresh window opens.

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py loop \
  --out /tmp/camino-showcase-loop
```

Use a new output path for each invocation. Add `--laps 2` to rehearse two loops. Each successful lap closes and relaunches Gazebo in its own partition. Press Ctrl+C to end the loop. A failed lap stops the loop and leaves its reports for inspection.

The default wall-clock budget is **300 seconds per lap**, but for diagnostics you can explicitly set `--wall-timeout 900`. `result.json` records `passed` and `wall_elapsed_s`. `control.jsonl` records position-independent controller stages, yielding and both stationary holds. `evaluation.json` records merge order, dual occupancy, measured route rejoining, clearances and stopped goals. Use `--headless` for a server-only timing check. The on-screen run is the timing check for the exhibition.

## Diagnostic speed trials

The original Gazebo world requests RTF `1.0` (100%), with a 1 ms physics step. A request is not guaranteed: actual RTF depends on the laptop's physics, GPU-LiDAR and rendering workloads. The launcher permits a **diagnostic** wall-timeout override so a slow run can finish or produce a complete failure report:

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario showcase --wall-timeout 900 --out /tmp/camino-showcase-debug
```

For an *experimental* performance comparison, `--physics-step 0.002` doubles the maximum simulation step. Keep the RTF target at `1.0` initially:

```bash
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario showcase --physics-step 0.002 --target-rtf 1.0 \
  --wall-timeout 900 --out /tmp/camino-showcase-2ms-test
```

This may reduce physics work but can change contact, braking, track dynamics, sensor synchronization and collision behavior. It has **not** been accepted as safe and must not replace the normal 1 ms configuration until the independent pose and hold tests pass in live Gazebo. A `--headless` trial can distinguish GUI rendering cost; it does not show LiDAR rays. `--no-lidar-view` hides only the ray viewer, not the sensor used by the controller. The integrated demo retains a 171 simulated-second mission timeout: increasing the wall-clock budget does not remove a genuine navigation deadlock. Do not use a 500% RTF request as a shortcut to safe, fast driving.


## How to verify the run

The independent evaluator requires A then B through the merge, zero samples with both footprints inside the junction, B yielding before entry, and both bypasses after the merge. It also checks that B follows along the shared lane for at least 2.5 m after merging and advances at least 0.5 m between A leaving the lane and A's measured rejoin. It projects the actual Gazebo collision footprints and requires at least 1 m of sampled obstacle and vehicle clearance. After each bypass it measures at least 1.5 m of aligned progress over five or more seconds and confirms both stopped goals.

The fixture spawner and pose evaluator receive the obstacle coordinates. The controller uses LiDAR returns, a terrain map and explicit vehicle state. A confirms stationarity from its front scan before planning. The waiting point comes from A's measured footprint and LiDAR estimate of the crate's leading edge, with room reserved for B's Ackermann turn. B verifies its swept approach with its own 360° scan and stays short of the reserved bypass area. The trace records permitted movement as `approaching_waiting_point`. B caps curvature to 0.23 m⁻¹ during its bypass and verifies the stopping space and next swept section on its return. The trial uses Gazebo odometry. An explicit external pose source is available for a single run.

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac_showcase.py' -v
python3 -m unittest discover -s tests -p 'test_jarcrac_convoy_v2.py' -v
```

The terrain ray-cast replays use kinematic vehicle movement and model geometry.

| Replay | Simulated time | B's longest stop before detouring | Minimum obstacle gap | Minimum vehicle gap |
| --- | ---: | ---: | ---: | ---: |
| Previous integrated showcase | 158.00 s | 77.00 s | 1.37 m | 2.10 m |
| Head-start layout, default crate | 157.50 s | 45.25 s | 1.27 m | 3.57 m |
| Head-start layout, crate shifted 0.50 m forward and 0.10 m sideways | 159.50 s | 46.00 s | 1.31 m | 3.54 m |

With the default crate, B advanced 3.86 m on the shared lane before A rejoined, including 1.90 m while A's bypass was underway. The live Gazebo result and five-minute wall-clock target still require a run on the presentation machine.
