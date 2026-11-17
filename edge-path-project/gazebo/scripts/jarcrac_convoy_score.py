"""Independent pose scoring for the convoy and private-lane SIL fixtures."""
from dataclasses import replace
import json
import math
import signal
import threading
import time

from final_sil_v6_checks import polygon_distance
from jarcrac_convoy_v2 import GOAL_POSITION_M, TILT_LIMIT, stationary
from jarcrac_cooperative_control import CLEARANCE, HOLD_SECONDS, State, angle_error
from jarcrac_cooperative_evaluate import obstacle_box
from jarcrac_cooperative_sil import NAMES, angles, stamp_of

WORLD = "camino_ruskotunturi_convoy_v2"
BOX_NAME = "convoy_test_box"
BOX_SIZE = (1.35, 1.10, 2.0)


class BypassEvidence:
    def __init__(self):
        self.departed, self.rejoined = False, False
        self.first_departure = None
        self.aligned_since = self.hold_since = None
        self.aligned_m = self.aligned_s = self.hold_s = 0.0
        self.swept_front = -math.inf

    def observe(self, state, body, box_min, box_max, dt, cruise):
        if not self.departed:
            near = box_min-17.0 <= state.x < box_min and abs(state.y) <= .38
            if near and dt is not None and dt <= .35 and stationary(state):
                if self.hold_since is None:
                    self.hold_since = state.stamp
                self.hold_s = max(self.hold_s, state.stamp-self.hold_since)
            else:
                self.hold_since = None
            # Witness the beginning of the lateral departure, independently of
            # the controller's detour phase, and keep measuring the envelope.
            if abs(state.y) > .55 and state.x <= box_max+3.0:
                self.departed, self.first_departure = True, state.stamp
        if not self.departed or self.rejoined:
            return
        along = [x for x, _ in body]
        if abs(state.y) > .35 or abs(state.yaw) > .20:
            self.swept_front = max(self.swept_front, max(along))
        clear_and_aligned = (min(along) > box_max+.20 and abs(state.y) <= .35 and abs(state.yaw) <= .20
                             and dt is not None and dt <= .35 and .02 <= state.vx <= max(.49, cruise+.10))
        if not clear_and_aligned:
            self.aligned_since = None
            return
        if self.aligned_since is None:
            self.aligned_since = state
        distance = state.x-self.aligned_since.x
        elapsed = state.stamp-self.aligned_since.stamp
        if distance > self.aligned_m:
            self.aligned_m, self.aligned_s = distance, elapsed
        self.rejoined = distance >= 1.50 and elapsed >= 5.0


class Score:
    def __init__(self, shapes, trial):
        self.shapes, self.trial = shapes, trial
        self.indices = (1,) if trial.solo else (0, 1)
        self.witnesses = {i: BypassEvidence() for i in self.indices}
        self.previous = self.start_time = self.goal_since = None
        self.samples = 0
        self.min_peer = self.min_obstacle = math.inf
        self.started = self.coverage_gap = self.finished = self.tilt_violation = False
        self.order_violation = self.corridor_violation = False
        self.merge_order, self.merge_exited = [], set()
        self.b_merge_exit_stamp = None
        self.b_merge_exit_x = self.b_stage_start_x = None
        self.b_follow_m = self.b_advance_during_a_bypass_m = 0.
        self.merge_hold, self.merge_yield_s = None, 0.
        self.dual_merge_samples = 0

    def observe(self, a, b, obstacle):
        active = [((a, b)[i], i) for i in self.indices]
        stamp = b.stamp
        dt = stamp-self.previous[1].stamp if self.previous else None
        if dt is not None and dt < .075:
            return
        if dt is not None:
            self.coverage_gap |= dt > .35
            active = [(replace(p, vx=(p.x-self.previous[i].x)/dt, vy=(p.y-self.previous[i].y)/dt,
                               omega=angle_error(p.yaw, self.previous[i].yaw)/dt), i) for p, i in active]
        else:
            self.start_time = stamp
            self.started = all(math.dist(self.trial.frame.local(p.x, p.y), self.trial.start(i)) <= .40
                               and abs(angle_error(self.trial.frame.local_yaw(p.yaw),
                                                   self.trial.start_yaw[i])) <= .18 for p, i in active)
        self.previous = {i: p for p, i in active}
        self.samples += 1
        self.tilt_violation |= any(max(abs(p.roll), abs(p.pitch)) > TILT_LIMIT for p, _ in active)
        box = tuple(self.trial.frame.local(*p) for p in obstacle_box(obstacle, BOX_SIZE))
        box_xs = [p[0] for p in box]
        local = {i: self.trial.frame.state(p) for p, i in active}
        body = {i: self.shapes[i].at(local[i]) for i in self.indices}
        self.min_obstacle = min(self.min_obstacle, *(polygon_distance(shape, box) for shape in body.values()))
        if self.trial.merge_zone:
            self.observe_merge(local, body, dt)
        for i, witness in self.witnesses.items():
            if not self.trial.merge_zone or i in self.merge_exited:
                witness.observe(local[i], body[i], min(box_xs), max(box_xs), dt,
                                (self.trial.a_speed, self.trial.b_speed)[i])
        if not self.trial.solo:
            leading_rear = min(x for x, _ in body[0])
            self.min_peer = min(self.min_peer, polygon_distance(body[0], body[1]))
            if not self.trial.merge_zone or self.merge_exited == {0, 1}:
                self.order_violation |= max(x for x, _ in body[1]) >= leading_rear
            leader, follower = self.witnesses[0], self.witnesses[1]
            if self.b_merge_exit_x is not None and not leader.rejoined and not follower.departed:
                self.b_follow_m = max(self.b_follow_m, local[1].x-self.b_merge_exit_x)
                if leader.departed:
                    if self.b_stage_start_x is None:
                        self.b_stage_start_x = local[1].x
                    self.b_advance_during_a_bypass_m = max(self.b_advance_during_a_bypass_m,
                                                          local[1].x-self.b_stage_start_x)
            if follower.departed and not follower.rejoined:
                self.corridor_violation |= not leader.rejoined or leading_rear <= leader.swept_front+.15
        parked = all(math.dist((p.x, p.y), (self.trial.goals[i], 0.)) <= GOAL_POSITION_M+.12
                     and abs(p.yaw) <= .24 and stationary(p) for i, p in local.items())
        if parked and dt is not None and dt <= .35:
            if self.goal_since is None:
                self.goal_since = stamp
            self.finished |= stamp-self.goal_since >= .50
        else:
            self.goal_since = None

    def observe_merge(self, local, body, dt):
        zone = self.trial.merge_zone
        exit_x = max(x for x, _ in zone)
        gaps = {i: polygon_distance(body[i], zone) for i in self.indices}
        occupied = [i for i, gap in gaps.items() if gap <= 1e-6]
        self.dual_merge_samples += int(len(occupied) > 1)
        for i in occupied:
            if i not in self.merge_order:
                self.merge_order.append(i)
        for i in self.merge_order:
            if (min(x for x, _ in body[i]) > exit_x+.15
                    and abs(local[i].y) <= .4 and abs(local[i].yaw) <= .22):
                self.merge_exited.add(i)
                if i == 1 and self.b_merge_exit_stamp is None:
                    self.b_merge_exit_stamp = local[i].stamp
                    self.b_merge_exit_x = local[i].x
        if (1 not in self.merge_order and 0. < gaps[1] <= 3.25 and
                dt is not None and dt <= .35 and stationary(local[1])):
            if self.merge_hold is None:
                self.merge_hold = local[1].stamp
            self.merge_yield_s = max(self.merge_yield_s, local[1].stamp-self.merge_hold)
        else:
            self.merge_hold = None

    def result(self):
        checks = self.started and self.finished and not (self.coverage_gap or self.tilt_violation
                                                         or self.order_violation or self.corridor_violation)
        # Pose broadcaster sampling can trim either edge of the five-second
        # interval. The controller trace separately gates on all five seconds.
        checks &= all(p.rejoined and p.hold_s >= HOLD_SECONDS-.25 for p in self.witnesses.values())
        checks &= self.min_obstacle >= CLEARANCE and self.min_peer >= CLEARANCE
        report = {
            "geometry_passed": bool(checks), "starts_observed": self.started,
            "goals_stopped": self.finished, "pose_samples": self.samples,
            "coverage_gap": self.coverage_gap, "tilt_violation": self.tilt_violation,
            "maneuver_corridor_violation": self.corridor_violation, "order_violation": self.order_violation,
            "duration_sim_s": self.previous[1].stamp-self.start_time if self.previous else None,
            "minimum_obstacle_footprint_gap_m": self.min_obstacle if math.isfinite(self.min_obstacle) else None,
            "minimum_vehicle_footprint_gap_m": self.min_peer if math.isfinite(self.min_peer) else None,
        }
        for i, p in self.witnesses.items():
            name = "ab"[i]
            report[f"{name}_rejoin_verified"] = p.rejoined
            report[f"{name}_aligned_progress_m"] = p.aligned_m
            report[f"{name}_aligned_progress_s"] = p.aligned_s
            report[f"{name}_stationary_pose_hold_s"] = p.hold_s
        if self.trial.merge_zone:
            leader_departed = self.witnesses[0].first_departure
            bypass_after_merge = (self.b_merge_exit_stamp is not None and leader_departed is not None
                                  and leader_departed >= self.b_merge_exit_stamp)
            followed_into_queue = self.b_follow_m >= 2.5 and self.b_advance_during_a_bypass_m >= .50
            passed = (self.merge_order == [0, 1] and self.merge_exited == {0, 1}
                      and self.dual_merge_samples == 0 and self.merge_yield_s >= 1.
                      and bypass_after_merge and followed_into_queue)
            report.update(merge_order=["AB"[i] for i in self.merge_order],
                          merge_completed=self.merge_exited == {0, 1},
                          dual_merge_samples=self.dual_merge_samples, b_yielded_s=self.merge_yield_s,
                          bypass_after_merge=bypass_after_merge, b_follow_distance_m=self.b_follow_m,
                          b_followed_into_queue=followed_into_queue,
                          b_advance_during_a_bypass_m=self.b_advance_during_a_bypass_m)
            report["geometry_passed"] &= passed
        return report


def hold_evidence(trace, indices):
    completed = {i: False for i in indices}
    with trace.open(encoding="utf-8") as src:
        for row in map(json.loads, src):
            for i in completed:
                data = row["ab"[i]]
                if data is not None and data["phase"] == "detour" and data["cmd_speed"] > 0.:
                    completed[i] |= data["hold_s"] >= HOLD_SECONDS
    return {f"{'ab'[i]}_controller_hold_verified": done for i, done in completed.items()}


def evaluate(shapes, trial, report, ready, timeout):
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    score, node = Score(shapes, trial), Node()
    watched = (NAMES[1], BOX_NAME) if trial.solo else (*NAMES, BOX_NAME)
    latest, lock, ending = {}, threading.Lock(), threading.Event()

    def sample(message):
        stamp = stamp_of(message)
        if stamp is None:
            return
        found = {}
        for p in message.pose:
            name = p.name.rsplit("::", 1)[-1]
            if name not in watched:
                continue
            xyz = p.position.x, p.position.y, p.position.z
            if not all(map(math.isfinite, xyz)):
                continue
            try:
                roll, pitch, yaw = angles(p.orientation)
            except ValueError:
                continue
            found[name] = State(stamp, *xyz[:2], yaw, roll=roll, pitch=pitch, z=xyz[2])
        with lock:
            latest.update(found)

    if not node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", sample):
        raise RuntimeError("Cannot subscribe to convoy evaluation poses")
    signal.signal(signal.SIGTERM, lambda *_: ending.set())
    ready.touch()
    deadline = time.monotonic()+timeout
    while not ending.wait(.02) and time.monotonic() < deadline:
        with lock:
            a, b, box = (latest.get(name) for name in (*NAMES, BOX_NAME))
        if b is None or box is None or (not trial.solo and (a is None or abs(a.stamp-b.stamp) > .05)):
            continue
        score.observe(a, b, box)
        if score.finished:
            break
    result = score.result()
    report.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result["geometry_passed"]
