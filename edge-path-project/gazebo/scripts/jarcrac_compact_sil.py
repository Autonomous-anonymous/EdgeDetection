#!/usr/bin/env python3
"""Run the compact Ruskotunturi SIL demo or an isolated JARCRAC rejoin trial."""
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
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from final_sil_v6_checks import polygon_distance
from jarcrac_compact_evaluate import (
    MOVING_NAME, MOVING_RAISE, MOVING_SIZE, STATIC_NAME, STATIC_SIZE, WORLD, evaluate, trace_evidence,
)
from jarcrac_compact_mission import (
    COMPACT, PRIVATE_FRAME, SCENARIOS, START_YAWS, TILT_LIMIT, Mission, b_scan_blocked, fresh,
    launch_space, mission_terrain, stationary,
)
from jarcrac_cooperative_control import STOP, Motion, angle_error
from jarcrac_cooperative_geometry import GAZEBO, MODELS, model_footprints
from jarcrac_cooperative_sil import ACTUATORS, EXTERNAL_STATE, LIDAR, PREFIX, SIM_STATE, Inputs, insert_vehicles
from jarcrac_cooperative_terrain import WORLD_FRAME, scan_from_message

B_LIDAR = f"{PREFIX}/b/lidar"
REQUEST_PAIR = f"{PREFIX}/requests"
DEFAULT_OBSTACLE = (8.0, 34.0)


def prepare(output, obstacle, scenario=COMPACT):
    tree = ET.parse(GAZEBO / "worlds/ruskotunturi_final_sil_demo_v6_world.sdf")
    world = tree.getroot().find("world")
    world.set("name", WORLD)
    for name in ("aeb_vehicle_360", "aeb_vehicle_360_b"):
        world.remove(world.find(f"include[name='{name}']"))
    world.remove(world.find("model[@name='loading_zone_marker']"))
    terrain = mission_terrain()
    insert_vehicles(world, terrain, scenario.starts, START_YAWS, WORLD_FRAME, B_LIDAR)
    x, y = obstacle
    length, width, height = STATIC_SIZE
    box = tuple((x+u*length/2, y+v*width/2) for u, v in ((-1,-1), (1,-1), (1,1), (-1,1)))
    bounds = launch_space(scenario)
    a, b = bounds.launch_x
    c, d = bounds.launch_y
    launch = tuple(PRIVATE_FRAME.world(u, v) for u, v in ((a,c), (b,c), (b,d), (a,d)))
    if polygon_distance(launch, box) <= 1e-6:
        raise ValueError("Static obstacle overlaps the designated clear launch area")
    ground = [terrain.height_at(*p) for p in box]
    if any(z is None for z in ground):
        raise ValueError("Static obstacle footprint is outside the mapped terrain")
    static = world.find(f"model[@name='{STATIC_NAME}']")
    static.find("pose").text = f"{x} {y} {min(ground)+height/2-.015} 0 0 0"
    moving = world.find(f"model[@name='{MOVING_NAME}']")
    if scenario.solo:
        world.remove(moving)
    else:
        origin = list(map(float, moving.findtext("pose").split()))
        origin[2] += MOVING_RAISE
        moving.find("pose").text = " ".join(map(str, origin))
        for size in moving.findall(".//box/size"):
            size.text = " ".join(map(str, MOVING_SIZE))
    ET.indent(tree, space="  ")
    with output.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)


def relay(ready, scenario, *, vehicle_indices=None):
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    node, lock, ending = Node(), threading.Lock(), threading.Event()
    indices = range(len(scenario.starts)) if vehicle_indices is None else vehicle_indices
    pubs = {i: node.advertise(ACTUATORS[i], Twist) for i in indices}
    commands, receipt = (STOP, STOP), -math.inf

    def receive(message):
        nonlocal commands, receipt
        values = message.data
        allowed = (len(values) == 4 and 0 <= values[0] <= scenario.a_speed and 0 <= values[2] <= scenario.b_speed
                   and abs(values[1]) <= .25 and abs(values[3]) <= .25)
        with lock:
            commands = (Motion(*values[:2]), Motion(*values[2:])) if allowed else (STOP, STOP)
            receipt = time.monotonic()

    def send(pair):
        for i, pub in pubs.items():
            command = pair[i]
            msg = Twist()
            msg.linear.x, msg.angular.z = command.speed, command.yaw_rate
            pub.publish(msg)

    if not node.subscribe(Double_V, REQUEST_PAIR, receive):
        raise RuntimeError("Cannot subscribe to paired motion requests")
    signal.signal(signal.SIGTERM, lambda *_: ending.set())
    ready.touch()
    try:
        while not ending.is_set():
            with lock:
                pair = commands if time.monotonic()-receipt <= .30 else (STOP, STOP)
            send(pair)
            ending.wait(.04)
    finally:
        for _ in range(6):
            send((STOP, STOP))
            time.sleep(.04)


def control(scenario, source, trace, processes, deadline):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    count = len(scenario.starts)
    node, inputs = Node(), Inputs(WORLD_FRAME, count, TILT_LIMIT)
    pub = node.advertise(REQUEST_PAIR, Double_V)
    topics = SIM_STATE if source == "gazebo-odometry" else EXTERNAL_STATE
    subscriptions = [(Clock, f"/world/{WORLD}/clock", inputs.clock_sample)]
    subscriptions.extend((Odometry, t, partial(inputs.vehicle_sample, i)) for i, t in enumerate(topics[:count]))
    subscriptions.extend((LaserScan, t, partial(inputs.lidar_sample, index=i))
                         for i, t in enumerate((LIDAR, B_LIDAR)[:count]))
    for kind, topic, callback in subscriptions:
        if not node.subscribe(kind, topic, callback):
            raise RuntimeError(f"Cannot subscribe to {topic}")

    def send(a, b):
        msg = Double_V()
        msg.data.extend((a.speed, a.yaw_rate, b.speed, b.yaw_rate))
        pub.publish(msg)

    shapes, terrain, space = model_footprints(), mission_terrain(), launch_space(scenario)
    mission = Mission(shapes, scenario)
    seen = [None]*count
    cache = [None]*count
    start_wall = last_clock = time.monotonic()
    start_sim = previous = None
    try:
        while time.monotonic() < deadline:
            tick = time.monotonic()
            for name, process in processes.items():
                status = process.poll()
                if status is not None and (name != "evaluator" or status != 0):
                    raise RuntimeError(f"{name} exited ({status})")
            now, a, b, scans, reset = inputs.snapshot_all()
            if reset:
                raise RuntimeError("Gazebo clock reset")
            for i, raw in enumerate(scans):
                if raw is None or raw[1] == seen[i]:
                    continue
                msg, stamp, pose = raw
                if i == 0:
                    if start_sim is None and not space.launch_clear:
                        space = launch_space(scenario)
                    scan = scan_from_message(msg, stamp, pose, terrain, WORLD_FRAME)
                    cache[i] = space.update(scan, pose) if scan is not None else None
                else:
                    cache[i] = b_scan_blocked(msg, pose, terrain, shapes[1], mission.merge.brakes["B"])
                seen[i] = stamp
            a_view = cache[0] if scans[0] is not None else None
            b_blocked = cache[1] if count == 2 and scans[1] is not None else None
            if now is not None:
                last_clock = tick
            if start_sim is None and now is not None and a_view and (scenario.solo or b_blocked is not None):
                vehicles = (a, b)[:count]
                if all(fresh(p, now) and stationary(p) for p in vehicles):
                    if not all(math.dist((p.x, p.y), start) <= .25 and abs(angle_error(p.yaw, yaw)) <= .15
                               for p, start, yaw in zip(vehicles, scenario.starts, START_YAWS)):
                        raise RuntimeError("Vehicles are outside the fresh start")
                    if space.launch_clear:
                        start_sim = now
            if start_sim is None:
                if tick-start_wall > 45.0:
                    raise RuntimeError("Could not get initial clock, localisation and scans")
                decision = mission.merge.fail_closed("awaiting_fresh_start")
            else:
                if (tick-last_clock > 10.0 or (now is not None and now-start_sim > scenario.sim_timeout)):
                    raise RuntimeError("Clock stalled or simulated mission timed out")
                decision = mission.step(now, a, b, a_view, b_blocked)
            send(decision.a, decision.b)
            phase = mission.a_phase, mission.holds[1].phase, decision.owner
            if phase != previous:
                print(f"t={now} A={phase[0]} B={phase[1]} merge={phase[2]}", flush=True)
                previous = phase
            row = {"sim_s": now, "a_phase": phase[0], "b_phase": phase[1],
                   "merge_owner": decision.owner, "committed": decision.committed, "reason": decision.reason,
                   "a_rejoin_replans": mission.navigator.rejoin_replans,
                   "a_stop_reason": mission.navigator.stop_reason if mission.private else None,
                   "a_hold_s": mission.navigator.completed_hold_seconds,
                   "b_hold_s": mission.holds[1].completed_seconds,
                   "a_scan_valid": a_view is not None, "b_scan_valid": b_blocked is not None}
            for label, pose, command in (("a", a, decision.a), ("b", b, decision.b)):
                row[f"{label}_pose_xy_yaw"] = [pose.x, pose.y, pose.yaw] if pose else None
                row[f"{label}_speed"] = pose.speed if pose else None
                row[f"{label}_cmd"] = [command.speed, command.yaw_rate]
            print(json.dumps(row, allow_nan=False), file=trace, flush=True)
            if mission.done:
                return
            time.sleep(max(0.0, .05-(time.monotonic()-tick)))
        raise RuntimeError("Wall-clock mission timeout")
    finally:
        for _ in range(6):
            send(STOP, STOP)
            time.sleep(.05)


def reap(process, *, grace=4.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        process.wait()


def launch(args, scenario):
    start = time.monotonic()
    deadline = start+(args.wall_timeout if args.wall_timeout is not None else scenario.wall_timeout)
    gz = shutil.which("gz")
    if gz is None:
        raise RuntimeError("Gazebo Harmonic and its gz command are required")
    __import__("gz.msgs10.odometry_pb2")
    __import__("gz.transport13")
    output = args.out.resolve()
    output.mkdir(parents=True)
    world, trace, report = (output / name for name in ("world.sdf", "control.jsonl", "evaluation.json"))
    prepare(world, args.static_obstacle, scenario)
    os.environ["GZ_PARTITION"] = args.partition or "camino_compact_"+uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        filter(None, (str(MODELS), os.environ.get("GZ_SIM_RESOURCE_PATH"))))
    print(f"Gazebo partition {os.environ['GZ_PARTITION']}", flush=True)
    processes, failure = {}, None

    def start_process(name, command):
        with (output / f"{name}.log").open("xb") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = process
        return process

    try:
        for name in ("relay", "evaluator"):
            ready = output / f".{name}-ready"
            worker = start_process(name, [sys.executable, str(Path(__file__).resolve()), name, args.scenario,
                                          str(ready), str(report), str(max(0., deadline-time.monotonic()))])
            wait_until = min(deadline, time.monotonic()+15.0)
            while not ready.exists() and worker.poll() is None and time.monotonic() < wait_until:
                time.sleep(.05)
            if not ready.exists():
                raise RuntimeError(f"{name} did not start. See {output / (name+'.log')}")
            ready.unlink()
        mode = ["-s", "--headless-rendering"] if args.headless else []
        start_process("gazebo", [gz, "sim", "-r", *mode, str(world)])
        with trace.open("x", encoding="utf-8") as stream:
            control(scenario, args.state_source, stream, processes, deadline)
        processes["evaluator"].wait(timeout=max(0., min(8., deadline-time.monotonic())))
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        for name in ("evaluator", "relay", "gazebo"):
            reap(processes.get(name))
    geometry = json.loads(report.read_text()) if report.exists() else {}
    evidence = trace_evidence(trace) if trace.exists() else {}
    passed = (failure is None and geometry.get("geometry_passed", False)
              and evidence.get("a_stop_wait_start_detour", False)
              and (scenario.solo or evidence.get("b_stop_wait_resume", False)))
    result = {"passed": passed, "failure": failure, "scenario": args.scenario, "state_source": args.state_source,
              "wall_elapsed_s": round(time.monotonic()-start, 3), **geometry, **evidence}
    (output / "result.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(f"{'PASS' if passed else 'FAIL'}  {output / 'result.json'}", flush=True)
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    setup = sub.add_parser("prepare")
    setup.add_argument("world", type=Path)
    run = sub.add_parser("demo")
    run.add_argument("--out", required=True, type=Path)
    run.add_argument("--headless", action="store_true")
    run.add_argument("--state-source", default="gazebo-odometry", choices=("gazebo-odometry", "external"))
    run.add_argument("--partition")
    run.add_argument("--wall-timeout", type=float)
    for command in (setup, run):
        command.add_argument("--scenario", choices=SCENARIOS, default="compact")
        command.add_argument("--static-obstacle", nargs=2, metavar=("X", "Y"), type=float, default=DEFAULT_OBSTACLE)
    for name in ("relay", "evaluator"):
        worker = sub.add_parser(name)
        worker.add_argument("scenario", choices=SCENARIOS)
        worker.add_argument("ready", type=Path)
        worker.add_argument("report", type=Path)
        worker.add_argument("timeout", type=float)
    args = parser.parse_args()
    scenario = SCENARIOS[args.scenario]
    try:
        if args.action == "relay":
            relay(args.ready, scenario)
        elif args.action == "evaluator":
            return 0 if evaluate(model_footprints(), scenario, args.report, args.ready, args.timeout) else 1
        elif all(map(math.isfinite, args.static_obstacle)):
            if args.action == "prepare":
                prepare(args.world, args.static_obstacle, scenario)
                print(args.world)
            else:
                if args.wall_timeout is not None and (not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0):
                    parser.error("Wall timeout must be finite and positive")
                return 0 if launch(args, scenario) else 1
        else:
            parser.error("Obstacle coordinates must be finite")
    except (OSError, RuntimeError, ValueError, ImportError) as exc:
        parser.exit(1, str(exc)+"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
