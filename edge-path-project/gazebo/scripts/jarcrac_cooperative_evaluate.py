#!/usr/bin/env python3
"""Read-only Gazebo pose evaluator for the JARCRAC cooperative SIL experiment."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

from final_sil_v6_checks import hull, polygon_distance
from jarcrac_cooperative_control import A_GOAL, A_START, B_GOAL, B_START, CLEARANCE, CONFLICT, State
from jarcrac_cooperative_geometry import model_footprints, rotate
from jarcrac_cooperative_sil import NAMES, OBSTACLE_SIZE, WORLD, angles, stamp_of
from jarcrac_cooperative_terrain import route_xy, route_yaw

OBSTACLE_CLEARANCE = 0.55


def obstacle_box(pose, size=OBSTACLE_SIZE):
    length, width, height = size
    corners = (rotate((x, y, z), pose.roll, pose.pitch, pose.yaw)
               for x in (-length/2, length/2) for y in (-width/2, width/2)
               for z in (-height/2, height/2))
    return hull((pose.x + x, pose.y + y) for x, y, _ in corners)


class Score:
    def __init__(self, footprints, obstacle_count):
        self.footprints = footprints
        self.obstacle_names = tuple(f"cooperative_obstacle_{i}" for i in range(obstacle_count))
        self.samples = 0
        self.start_verified = False
        self.min_vehicle = math.inf
        self.min_obstacle = math.inf
        self.dual_occupancy = 0
        self.occupants = []
        self.last_time = None
        self.coverage_gap = False
        self.goals_since = None
        self.finished = False

    def observe(self, poses, stamp):
        if self.last_time is not None and stamp <= self.last_time:
            return
        if any(n not in poses for n in (*NAMES, *self.obstacle_names)):
            return
        if self.last_time is not None and stamp - self.last_time > 0.35:
            self.coverage_gap = True
        self.last_time = stamp
        a, b = (poses[name] for name in NAMES)
        if not self.samples:
            self.start_verified = all(math.dist((state.x, state.y), expected) <= 0.35
                                      for state, expected in zip((a, b), (A_START, B_START)))
        a_poly, b_poly = (shape.at(state) for shape, state in zip(self.footprints, (a, b)))
        self.samples += 1
        self.min_vehicle = min(self.min_vehicle, polygon_distance(a_poly, b_poly))
        for name in self.obstacle_names:
            obstacle = obstacle_box(poses[name])
            self.min_obstacle = min(self.min_obstacle, polygon_distance(a_poly, obstacle),
                                    polygon_distance(b_poly, obstacle))

        occupied = tuple(label for label, footprint in zip(("A", "B"), (a_poly, b_poly))
                         if polygon_distance(footprint, CONFLICT) <= 1e-6)
        self.dual_occupancy += len(occupied) == 2
        if len(occupied) == 1 and (not self.occupants or self.occupants[-1] != occupied[0]):
            self.occupants.append(occupied[0])

        if math.dist((a.x, a.y), A_GOAL) <= 0.32 and math.dist((b.x, b.y), B_GOAL) <= 0.32:
            if self.goals_since is None:
                self.goals_since = stamp
            self.finished = stamp-self.goals_since >= 0.5
        else:
            self.goals_since = None

    def result(self):
        checked = (self.start_verified and self.finished and self.samples > 1 and not self.coverage_gap
                   and self.dual_occupancy == 0 and self.min_vehicle >= CLEARANCE
                   and self.min_obstacle >= OBSTACLE_CLEARANCE and self.occupants == ["A", "B"])
        return {"geometry_passed": checked, "start_verified": self.start_verified,
                "goals_observed": self.finished,
                "pose_samples": self.samples, "coverage_gap": self.coverage_gap,
                "minimum_vehicle_footprint_gap_m": self.min_vehicle if math.isfinite(self.min_vehicle) else None,
                "minimum_obstacle_footprint_gap_m": self.min_obstacle if math.isfinite(self.min_obstacle) else None,
                "dual_occupancy_samples": self.dual_occupancy, "occupancy_order": self.occupants}


def observe(score, wall_timeout):
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    lock = threading.Lock()
    obstacle_poses = {}
    node = Node()

    def callback(msg):
        stamp = stamp_of(msg)
        if stamp is None:
            return
        with lock:
            poses = {}
            for p in msg.pose:
                name = p.name
                if name not in (*NAMES, *score.obstacle_names):
                    continue
                if not math.isfinite(p.position.x) or not math.isfinite(p.position.y):
                    continue
                try:
                    roll, pitch, yaw = angles(p.orientation)
                except ValueError:
                    continue
                x, y = route_xy(p.position.x, p.position.y)
                state = State(stamp, x, y, route_yaw(yaw), roll=roll, pitch=pitch, z=p.position.z)
                if name in score.obstacle_names:
                    obstacle_poses[name] = state
                else:
                    poses[name] = state
            score.observe({**obstacle_poses, **poses}, stamp)

    if not node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", callback):
        raise RuntimeError("Gazebo pose evaluator could not subscribe")
    start = time.monotonic()
    while time.monotonic() - start < wall_timeout:
        with lock:
            if score.finished:
                break
        time.sleep(0.1)
    with lock:
        return score.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obstacles", type=int, default=0, help="Number of boxes inserted by prepare")
    parser.add_argument("--wall-timeout", type=float, default=1200.0)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.obstacles < 0 or not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error("Provide a nonnegative obstacle count and a positive finite timeout")
    result = observe(Score(model_footprints(), args.obstacles), args.wall_timeout)
    text = json.dumps(result, indent=2, allow_nan=False)
    if args.report:
        with args.report.open("x", encoding="utf-8") as file:
            file.write(text + "\n")
    print(text)
    if not result["geometry_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
