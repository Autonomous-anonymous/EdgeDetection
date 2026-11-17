#!/usr/bin/env python3
"""Launch the CAMINO showcase or convoy regression trials in Gazebo Harmonic."""
import argparse
from functools import partial
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from final_sil_v6_checks import polygon_distance
from jarcrac_compact_sil import B_LIDAR, REQUEST_PAIR, reap, relay
from jarcrac_convoy_score import BOX_NAME, BOX_SIZE, WORLD, evaluate, hold_evidence
from jarcrac_convoy_v2 import (
    TILT_LIMIT, TRIALS, ConvoyMission, launch_space, mapped_terrain, planar_scan, stationary,
)
from jarcrac_cooperative_control import STOP
from jarcrac_cooperative_geometry import GAZEBO, MODELS, model_footprints
from jarcrac_cooperative_sil import EXTERNAL_STATE, LIDAR, SIM_STATE, Inputs, add, insert_vehicles
from jarcrac_cooperative_terrain import WORLD_FRAME, scan_from_message
from jarcrac_showcase_mission import SHOWCASE, ShowcaseMission
from jarcrac_showcase_scene import showcase_scene

SCENARIOS = {**TRIALS, "showcase": SHOWCASE}
DEFAULT_OBSTACLES = {"convoy": (10., 0.), "b-private": (4., 0.), "showcase": (8.5, .35)}
SHOWCASE_WALL_LIMIT = 300.


def prepare(output, trial, obstacle, *, show_lidar=True, physics_step=0.001, target_rtf=1.0):
    tree = ET.parse(GAZEBO / "worlds/ruskotunturi_final_sil_demo_v6_world.sdf")
    world = tree.getroot().find("world")
    world.set("name", WORLD)
    # Performance experiments are explicit; the validated 1 ms step remains default.
    world.find("physics/max_step_size").text = str(physics_step)
    world.find("physics/real_time_factor").text = str(target_rtf)
    for name in ("aeb_vehicle_360", "aeb_vehicle_360_b"):
        world.remove(world.find(f"include[name='{name}']"))
    for model in world.findall("model"):
        world.remove(model)
    terrain = mapped_terrain()
    indices = (1,) if trial.solo else (0, 1)
    starts = tuple(trial.start(i) for i in indices)
    yaws = tuple(trial.start_yaw[i] for i in indices)
    insert_vehicles(world, terrain, starts, yaws, trial.frame, B_LIDAR, vehicle_indices=indices)

    x, y = obstacle
    length, width, height = BOX_SIZE
    box = tuple((x+dx*length/2, y+dy*width/2) for dx, dy in ((-1,-1), (1,-1), (1,1), (-1,1)))
    for x0, y0 in starts:
        launch = ((x0-3.1,y0-3.), (x0+3.1,y0-3.), (x0+3.1,y0+3.), (x0-3.1,y0+3.))
        if polygon_distance(box, launch) <= 1e-6:
            raise ValueError("Obstacle meets the clear start envelope")
    ground = [terrain.height_at(*trial.frame.world(*point)) for point in box]
    if any(z is None for z in ground):
        raise ValueError("Obstacle is outside the mapped terrain")
    ox, oy = trial.frame.world(x, y)
    model = add(world, "model", name=BOX_NAME)
    add(model, "static", "true")
    add(model, "pose", f"{ox} {oy} {min(ground)+height/2-.015} 0 0 {trial.frame.yaw}")
    link = add(model, "link", name="body")
    for label in ("visual", "collision"):
        item = add(link, label, name=label)
        geometry = add(item, "geometry")
        add(add(geometry, "box"), "size", " ".join(map(str, BOX_SIZE)))
        if label == "visual":
            material = add(item, "material")
            add(material, "ambient", ".95 .36 .11 1")
            add(material, "diffuse", ".95 .36 .11 1")
    if trial.merge_zone:
        showcase_scene(world, terrain, trial, show_lidar=show_lidar)
    ET.indent(tree, space="  ")
    with output.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)


def control(trial, source, trace, processes, deadline):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    node, inputs = Node(), Inputs(WORLD_FRAME, 2, TILT_LIMIT)
    pub = node.advertise(REQUEST_PAIR, Double_V)
    indices = (1,) if trial.solo else (0, 1)
    topics = SIM_STATE if source == "gazebo-odometry" else EXTERNAL_STATE
    subscriptions = [(Clock, f"/world/{WORLD}/clock", inputs.clock_sample)]
    subscriptions.extend((Odometry, topics[i], partial(inputs.vehicle_sample, i)) for i in indices)
    subscriptions.extend((LaserScan, (LIDAR, B_LIDAR)[i], partial(inputs.lidar_sample, index=i)) for i in indices)
    for kind, topic, callback in subscriptions:
        if not node.subscribe(kind, topic, callback):
            raise RuntimeError(f"Cannot subscribe to {topic}")

    def send(a, b):
        message = Double_V()
        message.data.extend((a.speed, a.yaw_rate, b.speed, b.yaw_rate))
        pub.publish(message)

    mission_type = ShowcaseMission if trial.merge_zone else ConvoyMission
    terrain, mission = mapped_terrain(), mission_type(model_footprints(), trial)
    space = None if trial.solo else launch_space(trial)
    seen, cache = [None, None], [None, None]
    started = last_clock = time.monotonic()
    start_sim = previous = None
    try:
        while time.monotonic() < deadline:
            tick = time.monotonic()
            for name, process in processes.items():
                status = process.poll()
                if status is not None and (name != "evaluator" or status != 0):
                    raise RuntimeError(f"{name} exited with status {status}")
            now, a, b, frames, reset = inputs.snapshot_all()
            if reset:
                raise RuntimeError("Gazebo simulation time reset")
            if now is not None:
                last_clock = tick
            for i in indices:
                frame = frames[i]
                if frame is None or frame[1] == seen[i]:
                    continue
                message, stamp, pose = frame
                if i == 0:
                    if start_sim is None and not space.launch_clear:
                        space = launch_space(trial)
                    scan = scan_from_message(message, stamp, pose, terrain, WORLD_FRAME)
                    cache[i] = space.update(scan, pose) if scan else None
                else:
                    cache[i] = planar_scan(message, stamp, pose, terrain)
                seen[i] = stamp
            a_scan, b_scan = (item if raw else None for item, raw in zip(cache, frames))
            if start_sim is None and now is not None and b_scan and (trial.solo or (a_scan and space.launch_clear)):
                poses = (a, b)
                if all(poses[i] and poses[i].fresh(now, TILT_LIMIT) and stationary(poses[i]) for i in indices):
                    for i in indices:
                        here = trial.frame.state(poses[i])
                        if (math.dist((here.x, here.y), trial.start(i)) > .30
                                or abs(here.yaw-trial.start_yaw[i]) > .16):
                            raise RuntimeError("Vehicle is outside its fresh start envelope")
                    start_sim = now
            if start_sim is None:
                if tick-started > 45.:
                    raise RuntimeError("Missing fresh initial clock, poses or ranges")
                decision = mission.fail_closed("waiting_for_start")
            else:
                if tick-last_clock > 8.0 or (now is not None and now-start_sim > trial.sim_timeout):
                    raise RuntimeError("Gazebo clock stalled or simulated trial timed out")
                decision = mission.step(now, a, b, a_scan, b_scan)
            send(decision.a, decision.b)
            navigators = mission.leader, mission.follower
            status = (tuple(n.phase if n else None for n in navigators), decision.owner)
            if status != previous:
                print(f"sim {now}   phases {status[0]}   maneuver owner {status[1]}", flush=True)
                previous = status
            row = {"sim_s": now, "owner": decision.owner, "reason": decision.reason}
            if trial.merge_zone:
                row["merge_completed"] = mission.merged
            for label, nav, cmd in zip("ab", navigators, (decision.a, decision.b)):
                row[label] = ({"phase": nav.phase, "hold_s": nav.completed_hold_seconds, "cmd_speed": cmd.speed,
                               "cmd_yaw_rate": cmd.yaw_rate, "stop_reason": nav.stop_reason} if nav else None)
            row["follower_replans"] = mission.follower.replans+mission.follower.rejoin_replans
            print(json.dumps(row, allow_nan=False), file=trace, flush=True)
            if mission.done:
                return
            time.sleep(max(0., .05-(time.monotonic()-tick)))
        raise RuntimeError("Wall-clock trial timeout")
    finally:
        for _ in range(6):
            send(STOP, STOP)
            time.sleep(.05)


def demo(args, trial, obstacle):
    start = time.monotonic()
    deadline = start+args.wall_timeout
    drive_deadline = deadline-min(5., args.wall_timeout*.1) if trial.merge_zone else deadline
    gz = shutil.which("gz")
    if gz is None:
        raise RuntimeError("Gazebo Harmonic and the gz command are required")
    __import__("gz.msgs10.odometry_pb2")
    __import__("gz.transport13")
    out = args.out.resolve()
    out.mkdir(parents=True)
    world, trace, report = (out / name for name in ("world.sdf", "control.jsonl", "evaluation.json"))
    prepare(world, trial, obstacle, show_lidar=not args.no_lidar_view,
            physics_step=args.physics_step, target_rtf=args.target_rtf)
    os.environ["GZ_PARTITION"] = args.partition or "camino_convoy_"+uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        filter(None, (str(MODELS), os.environ.get("GZ_SIM_RESOURCE_PATH"))))
    print(f"Gazebo partition {os.environ['GZ_PARTITION']}", flush=True)
    print(f"Second-terminal topic check while this run is active: export GZ_PARTITION={os.environ['GZ_PARTITION']}", flush=True)
    print(f"Physics step {args.physics_step:g} s, requested RTF {args.target_rtf:g}; achieved RTF depends on hardware", flush=True)
    processes, failure = {}, None

    def start_worker(name, command):
        with (out / f"{name}.log").open("xb") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = process
        return process

    try:
        for name in ("relay", "evaluator"):
            ready = out / f".{name}-ready"
            child = start_worker(name, [sys.executable, str(Path(__file__).resolve()), name, args.scenario,
                                        str(ready), str(report), str(max(0., deadline-time.monotonic()))])
            until = min(deadline, time.monotonic()+15.)
            while not ready.exists() and child.poll() is None and time.monotonic() < until:
                time.sleep(.05)
            if not ready.exists():
                raise RuntimeError(f"{name} failed to start. See {out / (name+'.log')}")
            ready.unlink()
        mode = ["-s", "--headless-rendering"] if args.headless else []
        start_worker("gazebo", [gz, "sim", "-r", *mode, str(world)])
        with trace.open("x", encoding="utf-8") as stream:
            control(trial, args.state_source, stream, processes, drive_deadline)
        settle_until = deadline-1. if trial.merge_zone else deadline
        processes["evaluator"].wait(timeout=max(0., min(8., settle_until-time.monotonic())))
    except (RuntimeError, OSError, ValueError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        if trial.merge_zone:
            for name in ("gazebo", "evaluator", "relay"):
                reap(processes.get(name), grace=max(0., min(4., deadline-time.monotonic())))
        else:
            for name in ("evaluator", "relay", "gazebo"):
                reap(processes.get(name))
    geometry = json.loads(report.read_text()) if report.exists() else {}
    holds = hold_evidence(trace, (1,) if trial.solo else (0, 1)) if trace.exists() else {}
    elapsed = time.monotonic()-start
    if trial.merge_zone and elapsed > args.wall_timeout and failure is None:
        failure = "Showcase exceeded the wall-clock limit"
    passed = failure is None and geometry.get("geometry_passed", False) and bool(holds) and all(holds.values())
    result = {"passed": passed, "failure": failure, "scenario": args.scenario, "state_source": args.state_source,
              "wall_elapsed_s": round(elapsed, 3), **geometry, **holds}
    (out / "result.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(f"{'PASS' if passed else 'FAIL'}  {out / 'result.json'}", flush=True)
    return passed


def play_loop(args):
    root = args.out.resolve()
    root.mkdir(parents=True)
    index = 1
    while args.laps is None or index <= args.laps:
        destination = root / f"lap-{index:03d}"
        command = [sys.executable, str(Path(__file__).resolve()), "demo", "--scenario", "showcase",
                   "--out", str(destination), "--wall-timeout", str(args.wall_timeout),
                   "--physics-step", str(getattr(args, "physics_step", .001)),
                   "--target-rtf", str(getattr(args, "target_rtf", 1.0))]
        for flag, enabled in (("--headless", args.headless), ("--no-lidar-view", args.no_lidar_view)):
            if enabled:
                command.append(flag)
        child = subprocess.Popen(command, start_new_session=True)
        try:
            status = child.wait()
        except KeyboardInterrupt:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            child.wait()
            return 130
        if status:
            print(f"Loop stopped at lap {index}. See {destination}", file=sys.stderr)
            return status
        index += 1
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    setup = sub.add_parser("prepare")
    setup.add_argument("world", type=Path)
    run = sub.add_parser("demo")
    loop = sub.add_parser("loop", help="Repeat the integrated showroom scenario")
    loop.add_argument("--laps", type=int, help="Stop after this many successful laps")
    run.add_argument("--partition")
    run.add_argument("--state-source", choices=("gazebo-odometry", "external"), default="gazebo-odometry")
    for command in (run, loop):
        command.add_argument("--out", required=True, type=Path)
        command.add_argument("--headless", action="store_true")
        command.add_argument("--wall-timeout", type=float)
        command.add_argument("--physics-step", type=float, default=.001,
                             help="Experimental physics step in seconds (default 0.001)")
        command.add_argument("--target-rtf", type=float, default=1.0,
                             help="Requested maximum real-time factor; not guaranteed (default 1.0)")
    for command in (setup, run, loop):
        command.add_argument("--no-lidar-view", action="store_true", help="Hide the GUI LiDAR plugin")
    for command in (setup, run):
        command.add_argument("--scenario", choices=SCENARIOS, default="convoy")
        command.add_argument("--obstacle", nargs=2, metavar=("X", "Y"), type=float,
                             help="Set the fixture box in local route metres")
    for name in ("relay", "evaluator"):
        worker = sub.add_parser(name)
        worker.add_argument("scenario", choices=SCENARIOS)
        worker.add_argument("ready", type=Path)
        worker.add_argument("report", type=Path)
        worker.add_argument("timeout", type=float)
    args = parser.parse_args()
    if args.action == "loop":
        args.scenario = "showcase"
        if args.laps is not None and args.laps <= 0:
            parser.error("The lap count must be positive")
    trial = SCENARIOS[args.scenario]
    if args.action in ("demo", "loop"):
        if args.wall_timeout is None:
            args.wall_timeout = SHOWCASE_WALL_LIMIT if trial.merge_zone else 600.
        if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0.:
            parser.error("Wall timeout must be positive and finite")
        if not math.isfinite(args.physics_step) or not .0005 <= args.physics_step <= .002:
            parser.error("Physics step must be between 0.0005 and 0.002 seconds")
        if not math.isfinite(args.target_rtf) or not .1 <= args.target_rtf <= 5.0:
            parser.error("Requested RTF must be between 0.1 and 5.0")
    try:
        if args.action == "relay":
            relay(args.ready, trial, vehicle_indices=(1,) if trial.solo else (0, 1))
        elif args.action == "evaluator":
            return 0 if evaluate(model_footprints(), trial, args.report, args.ready, args.timeout) else 1
        elif args.action == "loop":
            return play_loop(args)
        else:
            obstacle = args.obstacle if args.obstacle is not None else DEFAULT_OBSTACLES[args.scenario]
            if not all(map(math.isfinite, obstacle)):
                parser.error("Obstacle coordinates must be finite")
            if args.action == "prepare":
                prepare(args.world, trial, obstacle, show_lidar=not args.no_lidar_view)
                print(args.world)
            else:
                return 0 if demo(args, trial, obstacle) else 1
    except (RuntimeError, OSError, ValueError, ImportError) as exc:
        parser.exit(1, str(exc)+"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
