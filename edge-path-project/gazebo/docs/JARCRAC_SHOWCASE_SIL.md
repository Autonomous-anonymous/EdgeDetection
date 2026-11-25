# CAMINO five-minute Gazebo showcase

JARCRAC and the original Gazebo Vehicle B share a short course on Ruskotunturi. The tracked vehicle starts at the junction and proceeds first. B yields on the side approach, merges when A clears the junction, and visibly follows A on the shared lane. When A brakes, B stops behind it. Once A has started its bypass and moved far enough ahead, B advances slowly to an observed waiting point. Each vehicle holds at a measured stop for at least five simulated seconds before its own bypass. B waits for the whole of A to clear the maneuver area before turning out. Both rejoin before parking in their marked loading bays.

The course demonstrates the SIL topics in Course Topics.docx that can be seen in a simulation: existing vehicle and environment models, virtual sensors, a Python ADAS controller with collision braking, and multiple vehicles in one Gazebo world. The run can be reproduced locally with one command and produces an independent pose report. The embedded controller, CAN mapping, executable deployment, and phase documentation are evidenced separately through the project's SIL/HIL work.

## Run once or loop

Start at the root of the `main` checkout with Gazebo Harmonic and the Python bindings `gz.transport13` and `gz.msgs10`. For one lap, use the settings selected on the presentation machine.

```bash
run_dir="$(mktemp -d)"
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario showcase --physics-step 0.002 --target-rtf 1.5 \
  --wall-timeout 900 --out "$run_dir/showcase"
```

The GUI opens to an overview of the course. The junction outline is yellow, the crate is orange, and the green and blue loading bays are JARCRAC's and B's goals. The paint follows the terrain and has no collision geometry. In the **Visualize Lidar** panel, select the ray display and choose `/camino/cooperative/a/lidar` for JARCRAC's front 180° scan or `/camino/cooperative/b/lidar` for B's 360° scan. The panel uses Gazebo's topic selector. Check the selection when a fresh window opens.

To repeat the showcase with the same settings, run this from the repository root.

```bash
run_dir="$(mktemp -d)"
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py loop \
  --physics-step 0.002 --target-rtf 1.5 \
  --wall-timeout 900 --out "$run_dir/showcase-loop"
```

Use a new output path for each invocation. Add `--laps 2` to rehearse two loops. Each successful lap closes and relaunches Gazebo in its own partition. Press Ctrl+C to end the loop. A failed lap stops the loop and leaves its reports for inspection.

## Recorded presentation settings and results

On 17 November 2026 the presentation machine completed two consecutive laps at a 2 ms physics step and target real-time factor (RTF) of 1.5. Their wall-clock times were **183.50 s and 182.58 s**, under the five-minute presentation target. The simulated times were 164.14 s and 165.27 s. At target RTF 2.0, B failed to rejoin before the simulation timeout. The Finnish guide records those trials and the two passes at target RTF 1.0. [Presentation-machine results](../../../demos/JARCRAC_SHOWCASE_FI.md#esityskoneella-mitatut-testiajot-17112026)

The launcher's defaults are a 1 ms physics step, target RTF 1.0 and a 300 s wall-clock limit per Showcase lap. The presentation commands above set the wall-clock deadline to 900 s. Compare the measured `wall_elapsed_s` with 300 s to check the five-minute target. The mission also has a 171 simulated-second timeout. Actual running time depends on the machine and rendering workload.

`result.json` records `passed` and `wall_elapsed_s`. `control.jsonl` records controller stages, yielding and stationary holds. `evaluation.json` records merge order, dual occupancy, route rejoining, clearances and stopped goals. `--headless` runs the server without its GUI. `--no-lidar-view` hides the ray display while keeping the controller's LiDAR active.

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

With the default crate, B advanced 3.86 m on the shared lane before A rejoined, including 1.90 m while A's bypass was underway.
