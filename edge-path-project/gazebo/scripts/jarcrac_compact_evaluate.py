"""Gazebo fixture motion and pose-only scoring for the shortened SIL mission."""
from dataclasses import replace
import json
import math
import signal
import threading
import time

from final_sil_v6_checks import polygon_distance
from jarcrac_compact_mission import GOAL_RADIUS, PRIVATE_FRAME, START_YAWS, TILT_LIMIT, WORLD_CONFLICT, stationary
from jarcrac_cooperative_control import CLEARANCE, HOLD_SECONDS, State, angle_error
from jarcrac_cooperative_evaluate import obstacle_box
from jarcrac_cooperative_sil import NAMES, angles, stamp_of

WORLD = "ruskotunturi_jarcrac_compact"
STATIC_NAME, MOVING_NAME = "final_static_obstacle", "final_moving_obstacle"
STATIC_SIZE, MOVING_SIZE = (1.60, 1.60, 1.50), (1.40, 1.40, 1.85)
MOVING_RAISE = (MOVING_SIZE[2]-1.40)/2


class RejoinEvidence:
    def __init__(self):
        self.departed = self.verified = False
        self.previous = self.aligned_since = None
        self.progress_m = self.progress_s = 0.0

    def observe(self, a, body, obstacle):
        if self.verified:
            return
        local = PRIVATE_FRAME.state(a)
        previous, self.previous = self.previous, local
        xs = tuple(PRIVATE_FRAME.local(*p)[0] for p in obstacle)
        self.departed |= min(xs)-7.0 <= local.x <= max(xs)+3.0 and abs(local.y) >= 1.0
        rear = min(PRIVATE_FRAME.local(*p)[0] for p in body)
        aligned = (self.departed and rear > max(xs)+.20
                   and abs(local.y) <= .35 and abs(local.yaw) <= .20)
        dt = local.stamp-previous.stamp if previous else 0.0
        advancing = 0 < dt <= .35 and .020 <= (local.x-previous.x)/dt <= .43
        if not aligned or not advancing:
            self.aligned_since = None
            return
        if self.aligned_since is None:
            self.aligned_since = local
        elapsed, progress = local.stamp-self.aligned_since.stamp, local.x-self.aligned_since.x
        if progress > self.progress_m:
            self.progress_m, self.progress_s = progress, elapsed
        self.verified = progress >= 1.50 and elapsed >= 5.0

    def result(self):
        return {"a_detour_departure_observed": self.departed, "a_rejoin_verified": self.verified,
                "a_aligned_progress_m": self.progress_m, "a_aligned_progress_s": self.progress_s}


class Score:
    def __init__(self, shapes, scenario):
        self.shapes, self.scenario = shapes, scenario
        self.previous = self.start_time = self.goal_since = self.b_stopped_since = None
        self.samples = self.dual_occupancy = 0
        self.min_vehicle = self.min_obstacle = math.inf
        self.order = []
        self.start = self.finished = self.coverage_gap = self.tilt_violation = False
        self.b_held = self.b_resumed = False
        self.rejoin = RejoinEvidence()

    def observe(self, vehicles, static, moving=None, fixture_done=False):
        stamp = vehicles[0].stamp
        if self.previous is not None:
            dt = stamp-self.previous[0].stamp
            if dt < .08:
                return
            self.coverage_gap |= dt > .35
            vehicles = tuple(replace(p, vx=(p.x-q.x)/dt, vy=(p.y-q.y)/dt,
                                     omega=angle_error(p.yaw, q.yaw)/dt)
                             for p, q in zip(vehicles, self.previous))
        else:
            self.start_time = stamp
            self.start = all(math.dist((p.x, p.y), start) <= .55 and abs(angle_error(p.yaw, yaw)) <= .25
                             for p, start, yaw in zip(vehicles, self.scenario.starts, START_YAWS))
        self.samples += 1
        self.previous = vehicles
        polygons = tuple(shape.at(p) for shape, p in zip(self.shapes, vehicles))
        static_poly = obstacle_box(static, STATIC_SIZE)
        self.rejoin.observe(vehicles[0], polygons[0], static_poly)
        self.min_obstacle = min(self.min_obstacle, *(polygon_distance(poly, static_poly) for poly in polygons))
        if not self.scenario.solo:
            self.observe_crossing_and_merge(vehicles[1], polygons, moving, fixture_done)
        self.tilt_violation |= any(abs(p.roll) > TILT_LIMIT or abs(p.pitch) > TILT_LIMIT for p in vehicles)
        at_goals = all(math.dist((p.x, p.y), goal) <= GOAL_RADIUS+.10 and stationary(p)
                       for p, goal in zip(vehicles, self.scenario.goals))
        if at_goals:
            if self.goal_since is None:
                self.goal_since = stamp
            self.finished |= stamp-self.goal_since >= .50
        else:
            self.goal_since = None

    def observe_crossing_and_merge(self, b, polygons, moving, fixture_done):
        obstacle = obstacle_box(moving, MOVING_SIZE)
        self.min_vehicle = min(self.min_vehicle, polygon_distance(*polygons))
        self.min_obstacle = min(self.min_obstacle, *(polygon_distance(poly, obstacle) for poly in polygons))
        near = (not fixture_done and 8 < b.x < 16.2 and abs(b.y-24) <= 1.0 and abs(moving.y-24) <= 2.7
                and stationary(b) and polygon_distance(polygons[1], obstacle) <= 6.0)
        if near:
            if self.b_stopped_since is None:
                self.b_stopped_since = b.stamp
            self.b_held |= b.stamp-self.b_stopped_since >= .35
        else:
            self.b_stopped_since = None
        if fixture_done and self.b_held and b.x > 6.0 and b.speed > .10:
            self.b_resumed = True
        inside = [label for label, poly in zip(("A", "B"), polygons)
                  if polygon_distance(poly, WORLD_CONFLICT) <= 1e-6]
        self.dual_occupancy += len(inside) == 2
        if len(inside) == 1 and (not self.order or self.order[-1] != inside[0]):
            self.order.append(inside[0])

    def result(self, fixture_done=False, fixture_error=None):
        passed = (self.start and self.finished and not self.coverage_gap and not self.tilt_violation
                  and self.rejoin.verified and self.min_obstacle >= CLEARANCE)
        if not self.scenario.solo:
            passed &= (self.order == ["A", "B"] and self.dual_occupancy == 0 and self.min_vehicle >= CLEARANCE
                       and self.b_held and self.b_resumed and fixture_done and fixture_error is None)
        return {
            "geometry_passed": passed, "started_at_spawns": self.start, "pose_samples": self.samples,
            "goals_stopped": self.finished, "coverage_gap": self.coverage_gap, "tilt_violation": self.tilt_violation,
            "duration_sim_s": self.previous[0].stamp-self.start_time if self.previous else None,
            "minimum_vehicle_footprint_gap_m": self.min_vehicle if math.isfinite(self.min_vehicle) else None,
            "minimum_obstacle_footprint_gap_m": self.min_obstacle if math.isfinite(self.min_obstacle) else None,
            "merge_order": self.order, "dual_merge_occupancy_samples": self.dual_occupancy,
            "b_held_near_crossing": self.b_held, "b_resumed_after_crossing": self.b_resumed,
            "moving_obstacle_completed": fixture_done, "moving_obstacle_error": fixture_error,
            **self.rejoin.result(),
        }


def trace_evidence(trace):
    previous = {"a": None, "b": None}
    held = {"a": 0.0, "b": 0.0}
    with trace.open(encoding="utf-8") as stream:
        for row in map(json.loads, stream):
            for label, next_phase in (("a", "detour"), ("b", "follow")):
                phase = row[f"{label}_phase"]
                if (previous[label] == "wait" and phase == next_phase
                        and row[f"{label}_cmd"][0] > 0 and row[f"{label}_scan_valid"]):
                    held[label] = max(held[label], row[f"{label}_hold_s"])
                previous[label] = phase
    return {"a_stop_wait_start_detour": held["a"] >= HOLD_SECONDS,
            "b_stop_wait_resume": held["b"] >= HOLD_SECONDS,
            "a_stationary_hold_s": held["a"], "b_stationary_hold_s": held["b"]}


def evaluate(shapes, scenario, report, ready, timeout):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from final_sil_v6_harness import (
        MAX_ATTEMPTS, MOVING_PATH, MOVING_POSE_UPDATE_PERIOD_S, RETRY_INTERVAL_S, REQUEST_TIMEOUT_MS,
        VERIFY_DEADLINE_S, ScenarioDriver, target_for_time,
    )

    score, node, lock = Score(shapes, scenario), Node(), threading.Lock()
    latest, ending = {}, threading.Event()
    vehicle_names = NAMES[:len(scenario.starts)]
    watched = (*vehicle_names, STATIC_NAME) if scenario.solo else (*vehicle_names, STATIC_NAME, MOVING_NAME)

    def sample(msg):
        stamp = stamp_of(msg)
        if stamp is None:
            return
        found = {}
        for p in msg.pose:
            name = p.name.rsplit("::", 1)[-1]
            pos = p.position.x, p.position.y, p.position.z
            if name not in watched or not all(map(math.isfinite, pos)):
                continue
            try:
                roll, pitch, yaw = angles(p.orientation)
            except ValueError:
                continue
            found[name] = State(stamp, *pos[:2], yaw, roll=roll, pitch=pitch, z=pos[2])
        with lock:
            latest.update(found)

    if not node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", sample):
        raise RuntimeError("Cannot subscribe to Gazebo evaluation poses")
    signal.signal(signal.SIGTERM, lambda *_: ending.set())
    ready.touch()
    fixture = ScenarioDriver()
    deadline = time.monotonic()+timeout
    sim_start, pending, error = None, None, None
    confirmed_time = target_time = 0.0
    confirmed_point = MOVING_PATH[0]
    done = False
    while not ending.wait(.02) and time.monotonic() < deadline:
        with lock:
            poses = tuple(latest.get(name) for name in watched)
        if not all(poses):
            continue
        if scenario.solo:
            a, static = poses
            score.observe((a,), static)
            if score.finished:
                break
            continue
        a, b, static, moving = poses
        if abs(a.stamp-b.stamp) <= .05 and abs(b.stamp-moving.stamp) <= .20:
            score.observe((a, b), static, moving, done)
        if score.finished:
            break
        if sim_start is None and b.x <= 16.5:
            sim_start = b.stamp
        if sim_start is None or done or error:
            continue
        now = time.monotonic()
        point = moving.x, moving.y, moving.z
        if pending and moving.stamp > before and math.dist(point, pending) <= .12:
            confirmed_time, confirmed_point = target_time, (moving.x, moving.y, moving.z-MOVING_RAISE)
            pending, done = None, final
        if pending and now-target_wall > VERIFY_DEADLINE_S:
            error = "Gazebo did not confirm the moving obstacle pose"
        if not (done or error or pending) and b.stamp-sim_start-target_time >= MOVING_POSE_UPDATE_PERIOD_S:
            target_time = b.stamp-sim_start
            try:
                (x, y, z), final = target_for_time(fixture, target_time, confirmed_time, confirmed_point)
                pending = x, y, z+MOVING_RAISE
            except ValueError as exc:
                error = str(exc)
            attempts, before, target_wall, last_send = 0, moving.stamp, now, -math.inf
        if pending and not error and attempts < MAX_ATTEMPTS and now-last_send >= RETRY_INTERVAL_S:
            request = Pose_V()
            pose = request.pose.add()
            pose.name = MOVING_NAME
            pose.position.x, pose.position.y, pose.position.z = pending
            pose.orientation.z = pose.orientation.w = math.sqrt(.5)
            attempts, last_send = attempts+1, now
            try:
                node.request(f"/world/{WORLD}/set_pose_vector", request, Pose_V, Boolean, REQUEST_TIMEOUT_MS)
            except RuntimeError:
                # A delayed acknowledgement can still yield a confirmed pose.
                pass
    result = score.result(done, error)
    report.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result["geometry_passed"]
