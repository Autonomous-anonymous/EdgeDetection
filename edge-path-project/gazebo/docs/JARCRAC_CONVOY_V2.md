# JARCRAC and Vehicle B convoy SIL

Use `convoy` to test two consecutive detours around a persistent obstacle on a shared lane. JARCRAC and the original Gazebo Vehicle B start in order on a surveyed stretch of Ruskotunturi downstream of a merge. The compact showcase exercises the merge separately. Use `b-private` to isolate Vehicle B's planner on a parallel approach with room to turn before rejoining.

## Run a trial

Start at the root of the `main` checkout with Gazebo Harmonic and the Python bindings `gz.transport13` and `gz.msgs10`. Use a separate output path for each scenario.

```bash
run_dir="$(mktemp -d)"
python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario convoy --out "$run_dir/convoy"

python3 edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py demo \
  --scenario b-private --obstacle 5.0 -0.12 --out "$run_dir/b-private"
```

Use a fresh output directory for each run. Add `--headless` for server-only rendering on a compatible setup. `--obstacle X Y` moves the box in route-local metres, and `prepare` writes a world without launching Gazebo. The default wall-clock timeout is 600 seconds. Both commands write `world.sdf`, `control.jsonl`, `evaluation.json` and `result.json` during a complete run. Read `passed` and `wall_elapsed_s` in `result.json` to check the outcome and presentation time.

## What the controller checks

JARCRAC receives its existing front 180° LiDAR scan. Vehicle B uses its roof-mounted 360° planar LiDAR. The fixture spawner and the independent evaluator have the box coordinates. The controller receives sensed range and terrain elevation data, vehicle state, and the designated lane and goal. The trial uses two-metre boxes within the usable height of B's LiDAR plane. The known clear start area gives JARCRAC room to start with a front-only field of view.

1. JARCRAC brakes, confirms it is stationary, and waits five simulated seconds before planning a bypass. B stops behind it while the maneuver corridor is reserved.
2. B may start a bypass once A is aligned and its entire footprint clears the swept corridor. B plans a forward curve from its measured pose with curvature at most 0.23 m⁻¹, including after headway reduces its speed. It verifies the padded body along the curve and screens candidates for room to return to the lane.
3. B uses communicated poses and model footprints to follow A without treating A as an obstacle to pass. Range returns still veto an unsafe path. After a moving obstruction clears, B checks the lane again and resumes. When neither side is verified, it stays stopped and reports `no_safe_detour`. If a new obstacle invalidates a committed path, B brakes and waits for a freshly verified continuation.
4. The independent evaluator requires at least 1 m of sampled obstacle and vehicle clearance and checks full-body corridor separation, route order, stopped goals, and at least 1.5 m of measured aligned motion over at least five seconds after the bypass. It uses Gazebo world poses for evaluation and cross-checks the five-second stationary holds against the controller trace.

The default `--state-source gazebo-odometry` uses the simulation's explicit odometry topics. `--state-source external` uses only `/camino/cooperative/a/state` and `/camino/cooperative/b/state` as appropriate for the scenario. The messages must use the `camino_map` parent frame, the model name as child frame, and timestamps synchronized with the simulation clock. The controller rejects stale or inconsistent pose pairs. Gazebo's world-pose evaluator is independent of this choice.

## Validation status

Run the convoy regression tests from the repository root.

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac_convoy_v2.py' -v
```

Offline acceptance covers the B-only detour, the two sequential convoy detours, a stopped leader, a moving crossing, blocked bypasses, and an obstruction inserted during B's detour. The inserted obstruction clears before B verifies its replacement path. Additional checks cover stale localization, Ackermann curvature after slowing, and the generated sensor configuration.

Terrain ray-cast replays drove the controller with simulated ranges and kinematic vehicle motion. The pose scorer recorded these results.

| Trial | Simulated duration | Minimum obstacle footprint gap | Minimum vehicle footprint gap | Measured rejoin and stopped goals |
| --- | ---: | ---: | ---: | --- |
| Shared convoy | 277.25 s | 1.33 m | 5.13 m | A and B |
| B private approach, relocated obstacle | 152.50 s | 1.52 m | Solo | B |

The convoy is the longer experiment. Its cruise limits are 0.30 m/s for JARCRAC and 0.36 m/s for B, with B capped at 0.30 m/s during a bypass. The original compact SIL showcase is the short run. Live Gazebo completion and wall time for the convoy still need to be measured.
