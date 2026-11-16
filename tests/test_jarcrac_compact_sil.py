"""Offline tests with the committed Ruskotunturi mesh and vehicle footprints."""
import ast
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

SCRIPTS = Path(__file__).resolve().parents[1] / "edge-path-project/gazebo/scripts"
sys.path.insert(0, str(SCRIPTS))

from jarcrac_compact_evaluate import MOVING_NAME, MOVING_RAISE, MOVING_SIZE, STATIC_NAME, STATIC_SIZE, Score
from jarcrac_compact_mission import (
    B_SENSOR_HEIGHT, COMPACT, MERGE_FRAME, PRIVATE_FRAME, REJOIN, START_YAWS, Mission,
    b_scan_blocked, launch_space, mission_terrain,
)
from jarcrac_compact_sil import B_LIDAR, DEFAULT_OBSTACLE, prepare
from jarcrac_cooperative_control import HOLD_SECONDS, STOP, State
from jarcrac_cooperative_geometry import MODELS, model_footprints, rotate
from jarcrac_cooperative_sil import LIDAR, NAMES
from jarcrac_cooperative_terrain import FRONT_SENSOR, FrontScan
from test_jarcrac_cooperative_sil import ANGLES, integrate

SHAPES = model_footprints()
TERRAIN = mission_terrain()
BASELINE = ast.parse((SCRIPTS / "ruskotunturi_final_sil_demo_v6.py").read_text())


def original(name):
    for expression in BASELINE.body:
        if isinstance(expression, ast.Assign) and getattr(expression.targets[0], "id", "") == name:
            return ast.literal_eval(expression.value)
    raise ValueError(name)


def surface(state):
    c, s = math.cos(state.yaw), math.sin(state.yaw)

    def h(x, y):
        return TERRAIN.height_at(state.x+c*x-s*y, state.y+s*x+c*y)

    return replace(state, z=h(0., 0.)+.035, roll=math.atan2(h(0., .8)-h(0., -.8), 1.6),
                   pitch=-math.atan2(h(1., 0.)-h(-1., 0.), 2.0))


def box_range(origin, direction, center, dimensions):
    near, far = 0., math.inf
    for p, v, c, size in zip(origin, direction, center, dimensions):
        if abs(v) < 1e-10:
            if abs(p-c) > size/2:
                return math.inf
            continue
        lo, hi = sorted(((c-size/2-p)/v, (c+size/2-p)/v))
        near, far = max(lo, near), min(hi, far)
    return near if near <= far else math.inf


def a_scan(a, center):
    x, y, _ = rotate(FRONT_SENSOR, a.roll, a.pitch, a.yaw)
    x, y = x+a.x, y+a.y
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    ranges = tuple(box_range((x, y), (c*u-s*v, s*u+c*v), center, STATIC_SIZE[:2])
                   for u, v in ANGLES) if math.dist((x, y), center) <= 14 else (math.inf,)*len(ANGLES)
    return FrontScan(a.stamp, x, y, a.yaw, tuple(min(9., r) for r in ranges), ranges)


def b_scan(b, moving, a):
    offset = rotate((0, 0, B_SENSOR_HEIGHT), b.roll, b.pitch, b.yaw)
    origin = tuple(v+w for v, w in zip((b.x, b.y, b.z), offset))
    step = math.pi/360
    # Use a conservative body box as a synthetic return for the other vehicle.
    a_frame = replace(MERGE_FRAME, origin=(a.x, a.y), yaw=a.yaw)
    ax, ay = a_frame.local(*origin[:2])
    boxes = []
    if math.dist((b.x, b.y), (moving.x, moving.y)) <= 10:
        boxes.append((origin, (moving.x, moving.y, moving.z), MOVING_SIZE, None))
    if math.dist((b.x, b.y), (a.x, a.y)) <= 10:
        boxes.append(((ax, ay, origin[2]), (0., 0., a.z+1.10), (3.05, 1.80, 2.20), a_frame))
    ranges = []
    for i in range(721) if boxes else ():
        theta = -math.pi+i*step
        ray = rotate((math.cos(theta), math.sin(theta), 0.), b.roll, b.pitch, b.yaw)
        ranges.append(min(box_range(o, (*frame.local_vector(*ray[:2]), ray[2]) if frame else ray, p, size)
                          for o, p, size, frame in boxes))
    return SimpleNamespace(vertical_count=1, count=721, ranges=ranges or [math.inf]*721,
                           angle_min=-math.pi, angle_step=step, range_min=.10, range_max=30.)


class MovingBox:
    def __init__(self):
        self.points = original("MOVING_PATH")
        self.speed = original("MOVING_SPEED_MPS")
        self.dwell = original("MOVING_CENTER_DWELL_S")
        self.start = None
        self.done = False

    def pose(self, b):
        if self.start is None and b.x <= 16.5:
            self.start = b.stamp
        elapsed = 0.0 if self.start is None else b.stamp-self.start
        for i, (p, q) in enumerate(zip(self.points, self.points[1:])):
            leg = math.dist(p, q)/self.speed
            if elapsed <= leg:
                point = tuple(a+(c-a)*elapsed/leg for a, c in zip(p, q))
                break
            elapsed -= leg
            if i == 1:
                if elapsed <= self.dwell:
                    point = q
                    break
                elapsed -= self.dwell
        else:
            point, self.done = self.points[-1], True
        return State(b.stamp, *point[:2], math.pi/2, z=point[2]+MOVING_RAISE)


def run_scenario(scenario, center=DEFAULT_OBSTACLE, drift=False):
    vehicle = tuple(surface(State(1., *p, yaw)) for p, yaw in zip(scenario.starts, START_YAWS))
    mission, score, space = Mission(SHAPES, scenario), Score(SHAPES, scenario), launch_space(scenario)
    obstacle = State(1., *center, z=TERRAIN.height_at(*center)+.72)
    mover = MovingBox()
    best = [0., 0.]
    injected = False
    for _ in range(round(scenario.sim_timeout/.25)):
        a = vehicle[0]
        b = vehicle[-1] if not scenario.solo else None
        if drift and not injected and mission.a_phase == "rejoin":
            local = PRIVATE_FRAME.state(a)
            side = math.copysign(1., local.y)
            x, y = PRIVATE_FRAME.world(local.x, local.y+side*.225)
            a = surface(replace(a, x=x, y=y, yaw=a.yaw+side*.230))
            vehicle = (a, *vehicle[1:])
            injected = True
        moving = mover.pose(b) if b else None
        blocked = b_scan_blocked(b_scan(b, moving, a), b, TERRAIN, SHAPES[1], mission.merge.brakes["B"]) if b else None
        space.update(a_scan(a, center), a)
        decision = mission.step(a.stamp, a, b, space, blocked)
        score.observe(vehicle, obstacle, moving, mover.done)
        commands = decision.a, decision.b
        best = [max(last, cmd.speed) for last, cmd in zip(best, commands)]
        if mission.done and score.finished:
            break
        vehicle = tuple(surface(integrate(p, cmd, mission.merge.brakes[label], .25))
                        for p, cmd, label in zip(vehicle, commands, ("A", "B")))
    return mission, score.result(mover.done), vehicle, best


class CompactDemoTests(unittest.TestCase):
    def test_compact_mission_finishes_with_sensor_braking_detour_merge_and_two_goals(self):
        mission, result, poses, fastest = run_scenario(COMPACT)
        self.assertTrue(mission.done and result["geometry_passed"], (result, poses, mission.a_phase))
        self.assertEqual(result["merge_order"], ["A", "B"])
        self.assertLessEqual(result["duration_sim_s"], 180.)
        self.assertAlmostEqual(fastest[0], .30)
        self.assertAlmostEqual(fastest[1], .85)
        self.assertGreaterEqual(mission.navigator.completed_hold_seconds, HOLD_SECONDS)
        self.assertGreaterEqual(mission.holds[1].completed_seconds, HOLD_SECONDS)

    def test_rejoin_trial_accepts_a_relocated_static_box(self):
        mission, result, poses, fastest = run_scenario(REJOIN, (8.8, 34.4))
        self.assertTrue(mission.done and result["geometry_passed"], (result, poses))
        self.assertAlmostEqual(fastest[0], .20)
        self.assertLessEqual(result["duration_sim_s"], 180.)

    def test_rejoin_trial_refits_and_completes_after_lateral_and_heading_drift(self):
        mission, result, poses, _ = run_scenario(REJOIN, drift=True)
        self.assertTrue(mission.done and result["geometry_passed"], (result, poses))
        self.assertGreaterEqual(mission.navigator.rejoin_replans, 1)
        self.assertGreaterEqual(result["a_aligned_progress_m"], 1.50)
        self.assertGreaterEqual(result["a_aligned_progress_s"], 5.)

    def test_rejoin_brakes_for_unobserved_replacement_sweep(self):
        nav = Mission(SHAPES, REJOIN).navigator
        nav.phase = "rejoin"
        nav.goal = (-9.5, 0.)
        nav.curve = nav.curve_to(State(1., -16., 2.7, -.15), nav.goal)
        x, y, yaw, _ = nav.curve.at(-14.)
        state = State(1., x, y+.23, yaw+.24, vx=.12)
        blocked = SimpleNamespace(stamp=1., covers=lambda x, y: x < -11.)
        self.assertTrue(nav.stopping_space_clear(state, blocked))
        self.assertEqual(nav.tick(1., state, blocked), STOP)
        self.assertEqual(nav.stop_reason, "rejoin_unverified")

    def test_verified_return_can_rejoin_and_scan_the_continuation_from_the_lane(self):
        nav = Mission(SHAPES).navigator
        state = State(1., -12.23, -4.30)
        view = SimpleNamespace(stamp=1., covers=lambda x, y: x <= -4.30)
        goal = nav.rejoin_goal(state, view)
        self.assertIsNotNone(goal)
        self.assertTrue(nav.clear_segment(state, view, goal))
        joined = State(1., *goal)
        self.assertFalse(nav.clear_segment(joined, view, nav.route_goal(joined, 1.4)))

    def test_rejoin_score_rejects_a_vehicle_parked_on_the_route(self):
        score = Score(SHAPES, REJOIN)
        box = State(1., *DEFAULT_OBSTACLE)
        off = State(1., *PRIVATE_FRAME.world(-14., 2.4), PRIVATE_FRAME.yaw)
        score.observe((off,), box)
        for i in range(50):
            on = State(1.2+i*.20, *PRIVATE_FRAME.world(-10., 0.), PRIVATE_FRAME.yaw)
            score.observe((on,), box)
        self.assertTrue(score.rejoin.departed)
        self.assertFalse(score.rejoin.verified)

    def test_generated_worlds_use_original_models_and_front_mount(self):
        with tempfile.TemporaryDirectory() as directory:
            for i, scenario in enumerate((COMPACT, REJOIN)):
                world = Path(directory) / f"trial{i}.sdf"
                prepare(world, DEFAULT_OBSTACLE, scenario)
                root = ET.parse(world).getroot().find("world")
                self.assertEqual(root.findtext("include/uri"), "model://ruskotunturi_map")
                self.assertIsNotNone(root.find(f"model[@name='{STATIC_NAME}']"))
                self.assertEqual(root.find(f"model[@name='{MOVING_NAME}']") is None, scenario.solo)
                a = root.find(f"model[@name='{NAMES[0]}']")
                sensor = a.find("link/sensor")
                self.assertEqual(sensor.get("name"), "gpu_lidar_front")
                self.assertEqual(sensor.findtext("topic"), LIDAR)
                hor = sensor.find("ray/scan/horizontal")
                self.assertAlmostEqual(float(hor.findtext("max_angle"))-float(hor.findtext("min_angle")), math.pi)
                b = root.find(f"model[@name='{NAMES[1]}']")
                if scenario.solo:
                    self.assertIsNone(b)
                else:
                    self.assertEqual(b.findtext("link/sensor/topic"), B_LIDAR)
                for uri in root.findall(".//mesh/uri"):
                    self.assertTrue((MODELS / uri.text.removeprefix("model://")).is_file())


if __name__ == "__main__":
    unittest.main()
