# Compact Ruskotunturi SIL demo

The compact demo uses tracked JARCRAC, the original Gazebo Vehicle B and the
Ruskotunturi terrain. The vehicles approach the original v6.4 merge at
`(-4, 28)` and park at separate goals on the shared lane before the summit
climb. JARCRAC detours around a static obstacle and Vehicle B stops for the
moving crossing fixture. Both waits start after the vehicle is stationary.

## Run on Gazebo Harmonic

Start at the root of the `main` checkout with the Python bindings `gz.transport13` and `gz.msgs10`. The two-vehicle trial on the reference laptop took 305.905 wall-clock seconds. Give it a longer budget than the 285-second default when reproducing that run.

```bash
run_dir="$(mktemp -d)"
python3 edge-path-project/gazebo/scripts/jarcrac_compact_sil.py demo \
  --scenario compact --wall-timeout 600 --out "$run_dir/compact"
```

Add `--headless` to run the OGRE2 server with EGL rendering. The compact
world targets a real-time factor of 1.0. Both scenarios default to 285 wall-clock seconds for setup and the mission. `--wall-timeout` sets the budget for a run. The result records wall time in `wall_elapsed_s` and simulated time in `duration_sim_s`.

| Scenario | JARCRAC command ceiling | Vehicle B command ceiling | Goals |
| --- | --- | --- | --- |
| `compact` | 0.30 m/s on straight sections | 0.85 m/s from the original v6.4 cruise setting | Both park beyond the merge |
| `rejoin` | 0.20 m/s | JARCRAC is the only vehicle | Park on A's approach after rejoining |

The compact controller reduces JARCRAC's speed when the differential track
commands would exceed the model's track-speed limit. Vehicle B slows for
braking, holds and its goal. The moving fixture keeps the original crossing
trajectory and uses a 1.85 m box for visibility to B's planar LiDAR.

For the isolated detour trial, start with a fresh output path.

```bash
run_dir="$(mktemp -d)"
python3 edge-path-project/gazebo/scripts/jarcrac_compact_sil.py demo \
  --scenario rejoin --out "$run_dir/rejoin"
```

Append `--static-obstacle 8.8 34.4` to exercise another placement. Both
coordinates are in Gazebo world metres. The generated world can also be
inspected with the `prepare` subcommand and an output SDF path.

## Control and acceptance

JARCRAC uses the existing chassis-mounted 180° LiDAR. Ranges are checked
against the ground mesh using timestamped localisation. The two-vehicle
mode reserves separate approach lanes using the known routes. The planner
selects detours using the observed swept footprint and leaves room to steer
back before the merge. On excessive rejoin error, it checks a new curve
starting at the measured pose. Unverified stopping space or a blocked return
curve commands a stop.

The merge coordinator uses actual model footprints, direction and stopping
distance. It may withdraw a pending grant to B. If B has committed, A yields
until the whole of B clears. A missing or stale vehicle state denies entry.
The command relay sends zeros if the pair of controller commands expires.

The default `gazebo-odometry` mode uses ideal simulation localisation.
For external localisation, select `--state-source external --partition NAME`
and publish timestamped `gz.msgs.Odometry` in Gazebo world axes on
`/camino/cooperative/a/state` and, for the two-vehicle run,
`/camino/cooperative/b/state`. Use parent frame `camino_map` and child
frames `camino_jarcrac` and `camino_vehicle_b`. Include absolute XYZ,
orientation and Gazebo-time stamps. The controller receives LiDAR and
vehicle localisation. The fixture driver and evaluator use exact scene
poses in their separate process.

The evaluator measures JARCRAC's departure and confirms that its rear
clears the obstacle. Rejoining requires at least 1.50 m of forward travel
over at least five simulated seconds within 0.35 m and 0.20 rad of the
route. The scoring also checks stopped goals, at least 1.0 m sampled
footprint separation, and, in the compact run, B's crossing stop and
restart plus the merge order. The controller trace provides the five-second
hold evidence and records replan counts and stop reasons.

```bash
python3 -m unittest discover -s tests -p 'test_jarcrac_compact_sil.py' -v
```

Offline tests use the committed models and terrain with synthetic ranges
and kinematic motion. They include a relocated box and an injected
lateral and heading error.

## Recorded live Gazebo trials

Both laptop runs used Gazebo odometry. The evaluator recorded the detour, measured JARCRAC's aligned travel after rejoining, and verified stopped goals.

| Scenario and report | Wall time | Simulated time | Aligned travel after bypass | Stopped goals |
| --- | ---: | ---: | ---: | --- |
| [Solo rejoin](../results/jarcrac/jarcrac_compact_rejoin_laptop_pass.json) | 190.708 s | 145.389 s | 1.502 m over 7.622 s | JARCRAC |
| [Two-vehicle compact](../results/jarcrac/jarcrac_compact_two_vehicle_laptop_pass.json) | 305.905 s | 168.342 s | 1.504 m over 6.145 s | JARCRAC and Vehicle B |
