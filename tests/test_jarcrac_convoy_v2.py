"""Offline sensor, control and independent pose-scoring cases for the convoy."""
from dataclasses import dataclass, replace
from functools import lru_cache
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

from test_jarcrac_cooperative_sil import integrate

from final_sil_v6_checks import polygon_distance
from jarcrac_convoy_score import BOX_NAME, BOX_SIZE, Score
from jarcrac_convoy_trial import DEFAULT_OBSTACLES, prepare
from jarcrac_convoy_v2 import (
    ACKERMANN_CURVATURE, B_PRIVATE, CONVOY, LANE_HALF_WIDTH,
    ConvoyMission, mapped_terrain, planar_scan, speed_cap, stationary,
)
from jarcrac_cooperative_control import CLEARANCE, HOLD_SECONDS, Motion, STOP, State
from jarcrac_cooperative_geometry import model_footprints, rotate
from jarcrac_cooperative_sil import NAMES


DT = .25
BEAMS = tuple((math.cos(i*math.tau/720-math.pi), math.sin(i*math.tau/720-math.pi)) for i in range(721))
SHAPES = model_footprints()


@lru_cache(maxsize=1)
def terrain():
    return mapped_terrain()


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    width: float = BOX_SIZE[1]

    def limits(self, frame):
        dx, dy = BOX_SIZE[0]/2, self.width/2
        z = min(terrain().height_at(*frame.world(self.x+u, self.y+v))
                for u in (-dx, dx) for v in (-dy, dy))-.015
        return (self.x-dx, self.y-dy, z), (self.x+dx, self.y+dy, z+BOX_SIZE[2])

    def pose(self, frame):
        return State(1., *frame.world(self.x, self.y), frame.yaw, z=self.limits(frame)[0][2]+BOX_SIZE[2]/2)


class FrontFixture:
    # A's 180 degree sensor projection is exercised by the existing cooperative
    # regression. This fixture supplies A with an ideal free-space envelope.
    def __init__(self, stamp, frame, boxes):
        self.stamp, self.frame, self.boxes = stamp, frame, boxes

    def covers(self, x, y):
        x, y = self.frame.local(x, y)
        return not any(abs(x-box.x) <= BOX_SIZE[0]/2 and abs(y-box.y) <= box.width/2 for box in self.boxes)


def range_to_box(origin, direction, bounds):
    near, far = 0., 30.
    lower, upper = bounds
    for p, v, lo, hi in zip(origin, direction, lower, upper):
        if abs(v) <= 1e-12:
            if not lo <= p <= hi:
                return math.inf
            continue
        a, b = (lo-p)/v, (hi-p)/v
        near, far = max(near, min(a, b)), min(far, max(a, b))
        if near > far:
            return math.inf
    return near if near >= .10 else -math.inf


def b_ranges(vehicle, frame, boxes, leader):
    bounds = [box.limits(frame) for box in boxes]
    if leader:
        a = frame.state(leader)
        xs, ys = zip(*SHAPES[0].at(a))
        _, _, (lo, hi) = SHAPES[0].bounds(a.roll, a.pitch)
        bounds.append(((min(xs), min(ys), a.z+lo), (max(xs), max(ys), a.z+hi)))
    b = frame.state(vehicle)
    offset = rotate((0., 0., 1.40), b.roll, b.pitch, b.yaw)
    origin = b.x+offset[0], b.y+offset[1], b.z+offset[2]
    ax = rotate((1., 0., 0.), b.roll, b.pitch, b.yaw)
    ay = rotate((0., 1., 0.), b.roll, b.pitch, b.yaw)
    ranges = []
    for c, s in BEAMS:
        direction = tuple(c*x+s*y for x, y in zip(ax, ay))
        ranges.append(min((range_to_box(origin, direction, box) for box in bounds), default=math.inf))
    return SimpleNamespace(count=721, vertical_count=1, angle_min=-math.pi, angle_step=math.tau/720,
                           range_min=.1, range_max=30., ranges=ranges)


def surface(state):
    c, s = math.cos(state.yaw), math.sin(state.yaw)

    def height(x, y):
        return terrain().height_at(state.x+c*x-s*y, state.y+s*x+c*y)

    return replace(state, z=height(0., 0.)+.035, roll=math.atan2(height(0., .8)-height(0., -.8), 1.6),
                   pitch=-math.atan2(height(1., 0.)-height(-1., 0.), 2.))


class Rollout:
    def __init__(self, trial):
        self.trial = trial
        self.mission, self.score = ConvoyMission(SHAPES, trial), Score(SHAPES, trial)
        self.a, self.b = (surface(State(1., *trial.frame.world(x, 0.), trial.frame.yaw)) if x is not None else None
                          for x in trial.starts)
        self.phases = set()
        self.max_curvature = 0.

    def step(self, boxes=(), *, leader_stopped=False, scored_box=None):
        frame, b, a = self.trial.frame, self.b, self.a
        b_scan = planar_scan(b_ranges(b, frame, boxes, a), b.stamp, b, terrain())
        a_scan = FrontFixture(a.stamp, frame, boxes) if a else None
        result = self.mission.step(b.stamp, a, b, a_scan, b_scan)
        self.phases.add(self.mission.follower.phase)
        if result.b.speed > 0.:
            self.max_curvature = max(self.max_curvature, abs(result.b.yaw_rate/result.b.speed))
        if scored_box:
            self.score.observe(a, b, scored_box.pose(frame))
        if a:
            self.a = surface(integrate(a, STOP if leader_stopped else result.a, self.mission.leader.brakes, DT))
        self.b = surface(integrate(b, result.b, self.mission.follower.brakes, DT))
        return result


class ConvoyAcceptance(unittest.TestCase):
    def finish(self, run, obstacle):
        while run.b.stamp < run.trial.sim_timeout+1. and not run.mission.done:
            run.step((obstacle,), scored_box=obstacle)
        report = run.score.result()
        self.assertTrue(run.mission.done and report["geometry_passed"], report)
        self.assertGreaterEqual(run.mission.follower.completed_hold_seconds, HOLD_SECONDS)
        self.assertLessEqual(run.max_curvature, ACKERMANN_CURVATURE+1e-10)
        return report

    def test_b_private_detours_after_hold_and_measurably_rejoins(self):
        run = Rollout(B_PRIVATE)
        report = self.finish(run, Box(5., -.12))
        self.assertTrue(report["b_rejoin_verified"])
        self.assertGreaterEqual(report["b_aligned_progress_m"], 1.5)

    def test_shared_obstacle_is_bypassed_in_order_with_full_clearance(self):
        run = Rollout(CONVOY)
        report = self.finish(run, Box(*DEFAULT_OBSTACLES["convoy"]))
        self.assertTrue(report["a_rejoin_verified"] and report["b_rejoin_verified"])
        self.assertFalse(report["maneuver_corridor_violation"])
        self.assertGreaterEqual(report["minimum_vehicle_footprint_gap_m"], CLEARANCE)
        self.assertGreaterEqual(run.mission.leader.completed_hold_seconds, HOLD_SECONDS)
        self.assertLess(run.score.witnesses[0].first_departure, run.score.witnesses[1].first_departure)

    def test_b_stops_behind_communicating_teammate_without_passing(self):
        run = Rollout(CONVOY)
        travel = run.b.x, run.b.y
        min_gap = math.inf
        while run.b.stamp <= 42.:
            decision = run.step(leader_stopped=run.b.stamp >= 13.)
            min_gap = min(min_gap, polygon_distance(SHAPES[0].at(run.a), SHAPES[1].at(run.b)))
            self.assertFalse(run.mission.static.confirmed)
            self.assertNotEqual(decision.owner, "B")
        self.assertGreater(math.dist((run.b.x, run.b.y), travel), 2.0)
        self.assertTrue(stationary(run.b))
        self.assertGreaterEqual(min_gap, CLEARANCE)
        self.assertNotIn("detour", run.phases)

    def test_b_brakes_for_moving_crossing_and_resumes_without_detouring(self):
        trial = replace(B_PRIVATE, starts=(None, -7.))
        run = Rollout(trial)
        held_before_resume = 0.
        min_gap = math.inf
        while run.b.stamp < 170. and not run.mission.done:
            elapsed = run.b.stamp-16.
            moving = (Box(4., -1.6+.31*elapsed),) if 0. <= elapsed < 16. else ()
            if moving:
                footprint = SHAPES[1].at(trial.frame.state(run.b))
                min_gap = min(min_gap, polygon_distance(footprint, hazard_polygon(moving[0])))
            decision = run.step(moving)
            if "wait" in run.phases and decision.b.speed > 0.:
                held_before_resume = run.mission.follower.completed_hold_seconds
        self.assertTrue(run.mission.done)
        self.assertIn("wait", run.phases)
        self.assertGreaterEqual(held_before_resume, HOLD_SECONDS)
        self.assertGreaterEqual(min_gap, CLEARANCE)
        self.assertNotIn("detour", run.phases)
        self.assertNotIn("rejoin", run.phases)

    def test_b_reports_no_safe_detour_when_both_sides_are_blocked(self):
        run = Rollout(B_PRIVATE)
        obstruction = Box(*DEFAULT_OBSTACLES["b-private"], width=2*LANE_HALF_WIDTH)
        while run.b.stamp < 40.:
            decision = run.step((obstruction,))
        self.assertEqual(decision.b, STOP)
        self.assertTrue(stationary(run.b))
        self.assertEqual(decision.reason, "no_safe_detour")
        self.assertNotIn("detour", run.phases)

    def test_b_brakes_mid_detour_then_verifies_a_new_plan(self):
        run = Rollout(B_PRIVATE)
        first = Box(*DEFAULT_OBSTACLES["b-private"])
        hazard = None
        stopped_at, old_curve = None, None
        replanned = False
        while run.b.stamp < 200. and not run.mission.done:
            nav = run.mission.follower
            b = B_PRIVATE.frame.state(run.b)
            if hazard is None and nav.phase == "detour" and abs(b.y) > .80:
                x, y, _, _ = nav.curve.at(b.x+3.60)
                hazard, old_curve = Box(x, y), nav.curve
            boxes = (first, hazard) if hazard and (stopped_at is None or b.stamp-stopped_at < 6.) else (first,)
            decision = run.step(boxes, scored_box=first)
            if len(boxes) == 2 and nav.phase in ("brake", "wait"):
                self.assertEqual(decision.b, STOP)
                self.assertEqual(decision.owner, "B")
                self.assertGreaterEqual(polygon_distance(SHAPES[1].at(b), hazard_polygon(hazard)), CLEARANCE)
                if stopped_at is None:
                    stopped_at = b.stamp
            if stopped_at and nav.replans and decision.b.speed > 0.:
                self.assertGreaterEqual(b.stamp-stopped_at, HOLD_SECONDS)
                self.assertIsNot(nav.curve, old_curve)
                replanned = True
        self.assertIsNotNone(stopped_at)
        report = run.score.result()
        self.assertTrue(replanned and run.mission.done and report["geometry_passed"], report)

    def test_stale_localization_stops_both_and_resets_hold(self):
        run = Rollout(CONVOY)
        run.mission.follower.stopped_since = 1.
        decision = run.mission.step(run.b.stamp+.31, run.a, run.b, None, None)
        self.assertEqual((decision.a, decision.b), (STOP, STOP))
        self.assertIsNone(decision.owner)
        self.assertIsNone(run.mission.follower.stopped_since)

    def test_prepared_worlds_place_the_correct_sensors_and_relocatable_box(self):
        with TemporaryDirectory() as directory:
            for name, trial in (("convoy", CONVOY), ("b-private", B_PRIVATE)):
                output = Path(directory) / f"{name}.sdf"
                obstacle = DEFAULT_OBSTACLES[name][0]+1., -.12
                prepare(output, trial, obstacle)
                world = ET.parse(output).getroot().find("world")
                a, b = (world.find(f"model[@name='{name}']") for name in NAMES)
                self.assertEqual(trial.solo, a is None)
                if a is not None:
                    horizontal = a.find(".//sensor[@type='gpu_lidar']/lidar/scan/horizontal")
                    if horizontal is None:
                        horizontal = a.find(".//sensor[@type='gpu_lidar']/ray/scan/horizontal")
                    self.assertAlmostEqual(float(horizontal.findtext("max_angle"))-
                                           float(horizontal.findtext("min_angle")), math.pi, places=5)
                scan = b.find(".//sensor[@name='gpu_lidar_360']/ray/scan/horizontal")
                self.assertAlmostEqual(float(scan.findtext("max_angle"))-float(scan.findtext("min_angle")),
                                       math.tau, places=6)
                pose = world.findtext(f"model[@name='{BOX_NAME}']/pose").split()
                for actual, expected in zip(trial.frame.local(*map(float, pose[:2])), obstacle):
                    self.assertAlmostEqual(actual, expected)

    def test_headway_speed_cap_scales_yaw_with_speed(self):
        command = Motion(.36, .36*ACKERMANN_CURVATURE)
        reduced = speed_cap(command, .12)
        self.assertAlmostEqual(reduced.yaw_rate/reduced.speed, ACKERMANN_CURVATURE)
        self.assertEqual(speed_cap(command, 0.), STOP)


def hazard_polygon(box):
    return tuple((box.x+x*BOX_SIZE[0]/2, box.y+y*box.width/2)
                 for x,y in ((-1,-1), (1,-1), (1,1), (-1,1)))
