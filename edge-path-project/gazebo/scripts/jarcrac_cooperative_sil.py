#!/usr/bin/env python3
"""Prepare and run the separate JARCRAC cooperative Ruskotunturi SIL experiment."""
import argparse
from collections import deque
from dataclasses import replace
import json
import math
from pathlib import Path
import signal
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_cooperative_control import (
    A_GOAL, A_START, AGE_LIMIT, B_ROUTE, B_START, PAIR_LIMIT, STOP,
    MergeCoordinator, ObstacleNavigator, State, VehicleBRoute, angle_error,
)
from jarcrac_cooperative_geometry import GAZEBO, MODELS, MODEL_NAMES, model_footprints
from jarcrac_cooperative_terrain import (
    HALF_STEP, LAUNCH_X, LAUNCH_Y, LOCAL_FRAME, MAP_YAW, ObservedSpace, height_at,
    scan_from_message, spawn_pose, terrain_index, world_xy,
)
from jarcrac_lidar_map_assisted import quaternion_tuple

WORLD = "camino_jarcrac_cooperative"
NAMES = ("camino_jarcrac", "camino_vehicle_b")
PREFIX = "/camino/cooperative"
ACTUATORS = tuple(f"{PREFIX}/{name}/cmd_vel" for name in ("a", "b"))
SIM_STATE = tuple(f"{PREFIX}/{name}/sim_odometry" for name in ("a", "b"))
EXTERNAL_STATE = tuple(f"{PREFIX}/{name}/state" for name in ("a", "b"))
LIDAR = f"{PREFIX}/a/lidar"
FRAME = "camino_map"
OBSTACLE_SIZE = (0.8, 1.2, 1.5)
CLOCK_TOPIC = f"/world/{WORLD}/clock"


def add(element, key, value=None, **attributes):
    item = ET.SubElement(element, key, attributes)
    if value is not None:
        item.text = str(value)
    return item


def insert_vehicles(world, terrain, starts, yaws, frame=LOCAL_FRAME, b_lidar=None):
    footprints = model_footprints()
    for index, (source, name, start, yaw) in enumerate(zip(
            MODEL_NAMES, NAMES, starts, yaws)):
        model = ET.parse(MODELS / source / "model.sdf").getroot().find("model")
        model.set("name", name)
        add(model, "pose", " ".join(map(str, spawn_pose(terrain, footprints[index], start, yaw, frame))))
        for link in model.findall("link"):
            for sensor in link.findall("sensor"):
                if index == 0 and sensor.get("name") == "gpu_lidar_front":
                    sensor.find("topic").text = LIDAR
                    horizontal = sensor.find("ray/scan/horizontal")
                    for angle in ("min_angle", "max_angle"):
                        value = horizontal.find(angle)
                        value.text = str(float(value.text) + HALF_STEP)
                elif index == 1 and b_lidar and sensor.get("name") == "gpu_lidar_360":
                    sensor.find("topic").text = b_lidar
                else:
                    link.remove(sensor)
        driver = next(plugin for plugin in model.findall("plugin")
                      if plugin.get("name").endswith(("TrackedVehicle", "AckermannSteering")))
        driver.find("topic").text = ACTUATORS[index]
        driver.find("odom_topic").text = f"{ACTUATORS[index].rsplit('/', 1)[0]}/drive_odometry"
        odom = add(model, "plugin", filename="gz-sim-odometry-publisher-system",
                   name="gz::sim::systems::OdometryPublisher")
        for key, value in (("odom_frame", FRAME), ("robot_base_frame", name),
                           ("odom_topic", SIM_STATE[index]),
                           ("odom_covariance_topic", f"{SIM_STATE[index]}_with_covariance"),
                           ("tf_topic", f"{SIM_STATE[index]}_tf"),
                           ("dimensions", 3), ("odom_publish_frequency", 30)):
            add(odom, key, value)
        world.append(model)


def prepare(output, obstacles):
    tree = ET.parse(GAZEBO / "worlds/jarcrac_terrain_obstacle_probe_world.sdf")
    world = tree.getroot().find("world")
    world.set("name", WORLD)
    world.remove(world.find("include[name='jarcrac']"))
    world.remove(world.find("model[@name='jarcrac_terrain_test_obstacle']"))
    terrain = terrain_index()
    yaw_b = math.atan2(B_ROUTE[1][1]-B_START[1], B_ROUTE[1][0]-B_START[0])
    insert_vehicles(world, terrain, (A_START, B_START), (0.0, yaw_b))

    length, width, height = OBSTACLE_SIZE
    for i, (x, y) in enumerate(obstacles):
        if (LAUNCH_X[0]-length/2 <= x <= LAUNCH_X[1]+length/2 and
                LAUNCH_Y[0]-width/2 <= y <= LAUNCH_Y[1]+width/2):
            raise ValueError("Obstacle intersects JARCRAC's designated clear launch area")
        ground = [height_at(terrain, x+dx, y+dy)
                  for dx in (-length/2, length/2) for dy in (-width/2, width/2)]
        if any(z is None for z in ground):
            raise ValueError("Obstacle footprint is outside mapped terrain")
        wx, wy = world_xy(x, y)
        obstacle = add(world, "model", name=f"cooperative_obstacle_{i}")
        add(obstacle, "static", "true")
        add(obstacle, "pose", f"{wx} {wy} {min(ground)+height/2-0.015} 0 0 {MAP_YAW}")
        body = add(obstacle, "link", name="body")
        for tag in ("collision", "visual"):
            box = add(add(add(body, tag, name=tag), "geometry"), "box")
            add(box, "size", " ".join(map(str, OBSTACLE_SIZE)))
    ET.indent(tree, space="  ")
    with output.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)


def stamp_of(message):
    if not message.HasField("header") or not message.header.HasField("stamp"):
        return None
    t = message.header.stamp
    if t.sec < 0 or not 0 <= t.nsec < 1_000_000_000:
        return None
    return t.sec + t.nsec / 1e9


def angles(orientation):
    x, y, z, w = quaternion_tuple(orientation)
    return (
        math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
        math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))),
        math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)),
    )



class Inputs:
    def __init__(self, frame=LOCAL_FRAME, scan_count=1, tilt_limit=None):
        self.lock = threading.Lock()
        self.clock = None
        self.clock_receipt = -math.inf
        self.clock_reset = False
        self.samples = tuple(deque(maxlen=20) for _ in NAMES)
        self.frame, self.tilt_limit = frame, tilt_limit
        self.scans = [None]*scan_count

    def clock_sample(self, msg):
        now = msg.sim.sec + msg.sim.nsec / 1e9
        with self.lock:
            if self.clock is not None and now < self.clock:
                self.clock_reset = True
            if self.clock != now:
                self.clock = now
                self.clock_receipt = time.monotonic()

    def vehicle_sample(self, index, message):
        stamp = stamp_of(message)
        if stamp is None:
            return
        frames = {entry.key: tuple(entry.value) for entry in message.header.data}
        if frames.get("frame_id") != (FRAME,) or frames.get("child_frame_id") != (NAMES[index],):
            return
        p = message.pose.position
        if not all(map(math.isfinite, (p.x, p.y, p.z))):
            return
        try:
            roll, pitch, yaw = angles(message.pose.orientation)
        except ValueError:
            return
        x, y = self.frame.local(p.x, p.y)
        yaw = self.frame.local_yaw(yaw)
        sample = State(stamp, x, y, yaw, roll=roll, pitch=pitch, z=p.z)
        with self.lock:
            history = self.samples[index]
            previous = history[-1][0] if history else None
            if previous is not None and stamp <= previous.stamp:
                return
            continuous = previous is not None and stamp - previous.stamp <= AGE_LIMIT
            if continuous:
                dt = stamp - previous.stamp
                sample = replace(sample, vx=(x-previous.x)/dt, vy=(y-previous.y)/dt,
                                 omega=angle_error(yaw, previous.yaw)/dt)
            history.append((sample, time.monotonic(), continuous))

    def lidar_sample(self, message, index=0):
        stamp = stamp_of(message)
        if stamp is None:
            return
        with self.lock:
            old = self.scans[index]
            if old is None or stamp > old[1]:
                self.scans[index] = message, stamp, time.monotonic()

    def snapshot_all(self):
        wall = time.monotonic()
        with self.lock:
            now, reset = self.clock, self.clock_reset
            if reset or now is None or wall - self.clock_receipt > AGE_LIMIT:
                return None, None, None, (None,)*len(self.scans), reset
            states = []
            for history in self.samples:
                sample = history[-1] if history else None
                states.append(sample[0] if sample and sample[2] and wall-sample[1] <= AGE_LIMIT else None)
            scans = tuple(self.pair_scan(raw, history, now, wall)
                          for raw, history in zip(self.scans, self.samples))
            return now, states[0], states[1], scans, reset

    def pair_scan(self, raw, history, now, wall):
        if raw is None:
            return None
        message, stamp, receipt = raw
        if wall-receipt > AGE_LIMIT or not -.02 <= now-stamp <= AGE_LIMIT:
            return None
        paired = min((s for s, _, _ in history), key=lambda s: abs(s.stamp-stamp), default=None)
        limits = () if self.tilt_limit is None else (self.tilt_limit,)
        if paired and abs(paired.stamp-stamp) <= PAIR_LIMIT and paired.fresh(now, *limits):
            return message, stamp, paired
        return None

    def snapshot(self):
        now, a, b, scans, reset = self.snapshot_all()
        return now, a, b, scans[0], reset


def run(source, trace):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    a_shape, b_shape = model_footprints()
    terrain, space = terrain_index(), ObservedSpace()
    node = Node()
    inputs = Inputs()
    topics = SIM_STATE if source == "gazebo-odometry" else EXTERNAL_STATE
    pubs = [node.advertise(topic, Twist) for topic in ACTUATORS]

    def receive_a(msg):
        inputs.vehicle_sample(0, msg)

    def receive_b(msg):
        inputs.vehicle_sample(1, msg)

    subscriptions = ((Clock, CLOCK_TOPIC, inputs.clock_sample),
                     (Odometry, topics[0], receive_a), (Odometry, topics[1], receive_b),
                     (LaserScan, LIDAR, inputs.lidar_sample))
    for kind, topic, callback in subscriptions:
        if not node.subscribe(kind, topic, callback):
            raise RuntimeError(f"Could not subscribe to {topic}")

    def publish(pub, command):
        msg = Twist()
        msg.linear.x = command.speed
        msg.angular.z = command.yaw_rate
        pub.publish(msg)

    print(f"Localisation source {source} on {', '.join(topics)}", flush=True)
    navigator, route = ObstacleNavigator(a_shape), VehicleBRoute()
    merge = MergeCoordinator(a_shape, b_shape)
    previous, scan, last_stamp = None, None, None

    def terminate(signum, frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        while True:
            now, a, b, raw, reset = inputs.snapshot()
            if reset:
                raise RuntimeError("Simulation clock reset. Restart this controller with the world.")
            if raw is None:
                scan, last_stamp = None, None
            elif raw[1] != last_stamp:
                observation = scan_from_message(*raw, terrain)
                scan = space.update(observation, raw[2]) if observation is not None else None
                last_stamp = raw[1]
            if now is None:
                navigator.stopped_since = None
                d = merge.fail_closed("clock_unavailable")
            else:
                b_request = route.tick(b) if b and b.fresh(now) else STOP
                d = merge.step(now, a, b, navigator.tick(now, a, scan), b_request)
            publish(pubs[0], d.a)
            publish(pubs[1], d.b)
            if now is not None and trace:
                print(json.dumps({"sim_s": now, "phase": navigator.phase, "owner": d.owner,
                                  "committed": d.committed, "occupied": d.occupied,
                                  "grant": d.granted, "reason": d.reason,
                                  "a_speed_mps": a.speed if a else None,
                                  "a_yaw_rate_radps": a.omega if a else None,
                                  "a_state_s": a.stamp if a else None,
                                  "scan_s": scan.stamp if scan else None,
                                  "ground_returns": scan.current.ground_returns if scan else None,
                                  "elevated_returns": scan.current.elevated_returns if scan else None,
                                  "a_cmd": [d.a.speed, d.a.yaw_rate],
                                  "b_cmd": [d.b.speed, d.b.yaw_rate]}), file=trace, flush=True)
            status = navigator.phase, d.owner, d.committed
            if status != previous:
                print(f"t={now} A={status[0]} owner={status[1]} committed={status[2]}", flush=True)
                previous = status
            if (a and b and a.fresh(now) and b.fresh(now) and navigator.phase == "arrived"
                    and math.dist((b.x, b.y), B_ROUTE[-1]) <= 0.25):
                if max(a.speed, b.speed, abs(a.omega), abs(b.omega)) <= 0.025:
                    print("Both goals observed by controller", flush=True)
                    return
            time.sleep(0.05)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        for _ in range(6):
            for pub in pubs:
                publish(pub, STOP)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    setup = sub.add_parser("prepare", help="Generate the isolated world")
    setup.add_argument("output", type=Path)
    setup.add_argument("--obstacle", nargs=2, metavar=("X", "Y"), type=float,
                       action="append", default=[], help="Obstacle centre in local route metres")
    control = sub.add_parser("run", help="Run the SIL controller")
    control.add_argument("--state-source", required=True, choices=("gazebo-odometry", "external"))
    control.add_argument("--trace", type=Path)
    args = parser.parse_args()
    if args.action == "prepare":
        if any(not all(map(math.isfinite, p)) for p in args.obstacle):
            parser.error("Obstacle coordinates must be finite")
        try:
            prepare(args.output, args.obstacle)
        except (ValueError, FileExistsError) as error:
            parser.error(str(error))
        print(args.output)
    elif args.trace:
        with args.trace.open("x", encoding="utf-8") as trace:
            run(args.state_source, trace)
    else:
        run(args.state_source, None)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
