"""Offline acceptance cases for the JARCRAC SIL policy and its Gazebo boundary."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

SCRIPTS = Path(__file__).resolve().parents[1] / "edge-path-project/gazebo/scripts"
sys.path.insert(0, str(SCRIPTS))

from final_sil_v6_checks import polygon_distance
from jarcrac_cooperative_control import (
    A_GOAL, A_START, B_GOAL, B_ROUTE, B_START, CLEARANCE, CONFLICT, HOLD_SECONDS, STOP,
    MergeCoordinator, Motion, ObstacleNavigator, State, VehicleBRoute,
)
from jarcrac_cooperative_geometry import model_footprints, rotate
from jarcrac_cooperative_sil import (
    ACTUATORS, FRAME, LIDAR, NAMES, SIM_STATE, WORLD, Inputs, prepare,
)
from jarcrac_cooperative_evaluate import OBSTACLE_CLEARANCE, Score
from jarcrac_cooperative_terrain import (
    BIN_COUNT, BIN_STEP, FRONT_SENSOR, HALF_STEP, HORIZONTAL_SAMPLES, MAP_YAW,
    VERTICAL_SAMPLES, FrontScan, ObservedSpace, beam_vectors, height_at, route_xy,
    scan_from_message, spawn_pose, terrain_index, world_xy,
)

A_SHAPE, B_SHAPE = model_footprints()
TERRAIN = terrain_index()
A_COMMAND, B_COMMAND = Motion(.19), Motion(.31)


def state(x, y, yaw=0.0, vx=0.0, *, stamp=1.0):
    return State(stamp, x, y, yaw, vx)


def box(x, y):
    return ((x-.4, y-.6), (x+.4, y-.6), (x+.4, y+.6), (x-.4, y+.6))


ANGLES = tuple((math.cos(i*BIN_STEP-math.pi/2), math.sin(i*BIN_STEP-math.pi/2))
               for i in range(BIN_COUNT))


def ray_range(vehicle, direction, obstacle):
    if obstacle is None:
        return 25.0
    lower, upper = 0.0, 25.0
    for origin, component, edge, half in zip((vehicle.x, vehicle.y), direction, obstacle, (.4, .6)):
        if abs(component) < 1e-12:
            if not edge - half <= origin <= edge + half:
                return 25.0
        else:
            endpoints = ((edge-half-origin)/component, (edge+half-origin)/component)
            lower = max(lower, min(endpoints))
            upper = min(upper, max(endpoints))
    return lower if lower <= upper else 25.0


def projected_scan(vehicle, obstacle=None):
    # The mock sensor has the obstacle coordinates. The controller gets only front-range evidence.
    rx, ry, _ = rotate(FRONT_SENSOR, vehicle.roll, vehicle.pitch, vehicle.yaw)
    sensor = replace(vehicle, x=vehicle.x+rx, y=vehicle.y+ry)
    c, d = math.cos(vehicle.yaw), math.sin(vehicle.yaw)
    distance = tuple(ray_range(sensor, (c*u-d*v, d*u+c*v), obstacle) for u, v in ANGLES)
    return FrontScan(vehicle.stamp, sensor.x, sensor.y, vehicle.yaw,
                     tuple(min(r, 9.0) for r in distance),
                     tuple(r if r < 25 else math.inf for r in distance))


def ray_fixture(pose, obstacle=None):
    # Cast representative vertical layers through the committed terrain heightfield.
    # Test geometry is private to this fixture, including the optional box centre.
    first, step = -math.pi/2+HALF_STEP, math.pi/(HORIZONTAL_SAMPLES-1)
    vfirst, vstep = -math.pi/12, (math.pi/6)/(VERTICAL_SAMPLES-1)
    rx, ry, rz = rotate(FRONT_SENSOR, pose.roll, pose.pitch, pose.yaw)
    sensor = replace(pose, x=pose.x+rx, y=pose.y+ry, z=pose.z+rz)
    ranges = [math.inf] * (HORIZONTAL_SAMPLES*VERTICAL_SAMPLES)
    if obstacle is not None:
        floor = height_at(TERRAIN, *obstacle)
    for i, vector in enumerate(beam_vectors(first, step, vfirst, vstep)):
        if i//HORIZONTAL_SAMPLES % 4:
            continue
        dx, dy, dz = rotate(vector, pose.roll, pose.pitch, pose.yaw)
        length = math.inf
        for metre in range(1, 11):
            height = height_at(TERRAIN, sensor.x+dx*metre, sensor.y+dy*metre)
            if height is not None and sensor.z+dz*metre <= height:
                lo, hi = metre-1.0, float(metre)
                for _ in range(7):
                    mid=(lo+hi)/2
                    surface = height_at(TERRAIN, sensor.x+dx*mid, sensor.y+dy*mid)
                    if surface is None or sensor.z+dz*mid > surface:
                        lo = mid
                    else:
                        hi = mid
                length = hi
                break
        hit = ray_range(sensor, (dx, dy), obstacle)
        if obstacle is not None and hit < 10.0 and floor <= sensor.z+dz*hit <= floor+1.5:
            length = min(length, hit)
        ranges[i] = length if length >= .30 else -math.inf
    return SimpleNamespace(count=HORIZONTAL_SAMPLES, vertical_count=VERTICAL_SAMPLES,
                           angle_min=first, angle_step=step, vertical_angle_min=vfirst,
                           vertical_angle_step=vstep, range_min=.30, range_max=30., ranges=ranges)


def surface_state(vehicle):
    h = height_at(TERRAIN, vehicle.x, vehicle.y)
    c, s = math.cos(vehicle.yaw), math.sin(vehicle.yaw)
    longitudinal = (height_at(TERRAIN, vehicle.x+c, vehicle.y+s) -
                    height_at(TERRAIN, vehicle.x-c, vehicle.y-s))
    lateral = (height_at(TERRAIN, vehicle.x-s, vehicle.y+c) -
               height_at(TERRAIN, vehicle.x+s, vehicle.y-c))
    return replace(vehicle, roll=math.atan2(lateral, 2), pitch=-math.atan2(longitudinal, 2), z=h+.026)


def integrate(vehicle, command, braking, dt):
    def approach(value, desired, max_step):
        return value + math.copysign(min(abs(desired - value), max_step), desired - value)

    forward = vehicle.vx * math.cos(vehicle.yaw) + vehicle.vy * math.sin(vehicle.yaw)
    speed = approach(forward, command.speed, braking.deceleration * dt)
    omega = approach(vehicle.omega, command.yaw_rate, braking.angular_deceleration * dt)
    yaw = vehicle.yaw + omega*dt
    vx, vy = speed*math.cos(yaw), speed*math.sin(yaw)
    return State(vehicle.stamp+dt, vehicle.x+vx*dt, vehicle.y+vy*dt, yaw, vx, vy, omega)


class MergeAcceptance(unittest.TestCase):
    def setUp(self):
        self.coordinator = MergeCoordinator(A_SHAPE, B_SHAPE)
        self.a_out = state(11.0, 0, vx=.19)
        self.b_approach = state(-5.6, -7.4, .55)

    def step(self, a, b, a_command=A_COMMAND, b_command=B_COMMAND, now=1.0):
        return self.coordinator.step(now, a, b, a_command, b_command)

    def test_a_returning_revokes_b_before_entry(self):
        initial = self.step(self.a_out, self.b_approach)
        self.assertEqual(initial.granted, "B")
        self.assertFalse(initial.committed)
        returning = state(11.0, 0, math.pi, -.14)
        for _ in range(2):
            result = self.step(returning, self.b_approach)
            self.assertNotEqual(result.granted, "B")
            self.assertEqual(result.b, STOP)
        self.assertEqual(self.step(self.a_out, self.b_approach).granted, "B")

    def test_a_yields_to_committed_b_until_entire_footprint_clears(self):
        self.step(self.a_out, self.b_approach)
        occupied = state(.0, -.9, .25, .21)
        result = self.step(state(5.88, 0, math.pi, -.17), occupied)
        self.assertEqual((result.owner, result.occupied, result.committed), ("B", ("B",), True))
        self.assertEqual(result.a, STOP)
        self.assertGreater(result.b.speed, 0)
        # The centre has crossed x=4, and the rear wheels remain in the zone.
        result = self.step(state(-3, 6.0, -math.pi/2), state(4.7, 0, vx=.2))
        self.assertEqual((result.owner, result.occupied, result.committed), ("B", ("B",), True))
        result = self.step(state(-3, 5.4, -math.pi/2), state(7.1, 0, vx=.2))
        self.assertEqual(result.granted, "A")
        self.assertGreater(result.a.speed, 0)

    def test_reentry_motion_keeps_a_out_until_committed_b_has_left(self):
        a = state(11.2, 0.0, math.pi, -.19)
        b = state(1.8, 0, vx=.24)
        held_a = cleared_b = False
        minimum_gap = math.inf
        for _ in range(300):
            result = self.step(a, b, now=a.stamp)
            a_poly, b_poly = A_SHAPE.at(a), B_SHAPE.at(b)
            a_inside = polygon_distance(a_poly, CONFLICT) <= 1e-6
            b_inside = polygon_distance(b_poly, CONFLICT) <= 1e-6
            self.assertFalse(a_inside and b_inside)
            minimum_gap = min(minimum_gap, polygon_distance(a_poly, b_poly))
            if b_inside and result.a == STOP:
                held_a = True
            if b_inside:
                self.assertEqual(result.owner, "B")
            elif min(x for x, _ in b_poly) > 4.0:
                cleared_b = True
            if held_a and cleared_b:
                break
            a = integrate(a, result.a, self.coordinator.brakes["A"], .10)
            b = integrate(b, result.b, self.coordinator.brakes["B"], .10)
        else:
            self.fail(f"B did not clear the junction with A yielding. A={a}; B={b}; {result}")
        self.assertTrue(held_a and cleared_b)
        self.assertGreaterEqual(minimum_gap, CLEARANCE)

    def test_b_inside_stopping_distance_has_commitment_before_geometric_entry(self):
        self.assertEqual(self.step(self.a_out, self.b_approach).granted, "B")
        front_at_boundary = state(-3.2, -6.43, math.pi/2, 0.0)
        front_at_boundary = replace(front_at_boundary, vy=.31)
        result = self.step(state(7.0, 0, math.pi, -.1), front_at_boundary)
        self.assertEqual(result.occupied, ())
        self.assertEqual(result.owner, "B")
        self.assertTrue(result.committed)

    def test_off_route_a_cannot_release_b(self):
        result = self.step(state(8.0, 3.3), self.b_approach)
        self.assertNotEqual(result.granted, "B")
        self.assertEqual(result.b, STOP)

    def test_b_brakes_for_stopped_a_on_shared_route(self):
        a = state(13.0, 0)
        b = state(9.0, 0, vx=.31)
        separation = polygon_distance(A_SHAPE.at(a), B_SHAPE.at(b))
        self.assertGreater(separation, CLEARANCE)
        self.assertEqual(self.step(a, b).b, STOP)
        self.assertGreater(self.step(replace(a, x=15.5), b).b.speed, 0)

    def test_closing_vehicle_stops_with_footprint_margin(self):
        a = state(14.0, 0)
        b = state(8.8, 0, vx=.31)
        saw_stop = False
        for _ in range(80):
            decision = self.step(replace(a, stamp=b.stamp), b, STOP, B_COMMAND, now=b.stamp)
            self.assertGreaterEqual(polygon_distance(A_SHAPE.at(a), B_SHAPE.at(b)), CLEARANCE)
            if decision.b == STOP:
                saw_stop = True
            b = integrate(b, decision.b, self.coordinator.brakes["B"], .10)
        self.assertTrue(saw_stop)
        self.assertLessEqual(b.speed, .03)

    def test_stale_or_missing_pose_denies_new_entry_and_preserves_commitment(self):
        self.assertEqual(self.step(self.a_out, self.b_approach).granted, "B")
        for missing in (self.a_out, None):
            result = self.step(missing, replace(self.b_approach, stamp=1.5), now=1.5)
            self.assertEqual((result.a, result.b, result.granted), (STOP, STOP, None))
            self.assertFalse(result.committed)
        in_merge = state(0, 0, vx=.25)
        self.step(self.a_out, in_merge)
        result = self.step(None, replace(in_merge, stamp=1.4), now=1.4)
        self.assertEqual((result.a, result.b, result.granted), (STOP, STOP, None))
        self.assertEqual((result.owner, result.committed), ("B", True))

    def test_dual_occupancy_stops_both(self):
        result = self.step(state(-1, 0), state(0, -2.7, .6))
        self.assertEqual((result.a, result.b, result.granted), (STOP, STOP, None))
        self.assertEqual((result.occupied, result.reason), (("A", "B"), "dual_occupancy"))


class ObstacleAcceptance(unittest.TestCase):
    def setUp(self):
        self.planner = ObstacleNavigator(A_SHAPE)
        initial = state(*A_START, stamp=.8)
        self.view = ObservedSpace().update(projected_scan(initial), initial)

    def tick(self, pose, obstacle=None):
        self.view.update(projected_scan(pose, obstacle), pose)
        return self.planner.tick(pose.stamp, pose, self.view)

    def test_hold_has_five_fresh_stationary_seconds_before_resuming(self):
        a = state(-21.7, 0.0, vx=.10)
        self.assertEqual(self.tick(a, (-16.4, 0)), STOP)
        self.assertEqual(self.planner.phase, "brake")
        a = replace(a, vx=0.0)
        for i in range(51):
            now = 1.1 + i/10
            a = replace(a, stamp=now)
            command = self.tick(a)
            if i < 50:
                self.assertEqual(command, STOP, now)
        self.assertEqual(self.planner.phase, "follow")
        self.assertGreater(self.tick(replace(a, stamp=6.2)).speed, 0)

    def test_missing_scan_restarts_stationary_wait(self):
        a = state(-21.7, 0.)
        self.tick(a, (-16.4, 0))
        self.tick(replace(a, stamp=1.1))
        self.assertEqual(self.planner.tick(1.4, replace(a, stamp=1.4), None), STOP)
        self.assertEqual(self.tick(replace(a, stamp=6.2)), STOP)
        self.assertEqual(self.planner.phase, "wait")

    def test_no_observed_corridor_stays_stopped(self):
        a = state(-21.7, 0)
        for index in range(62):
            a = replace(a, stamp=1 + index/10)
            observation = replace(projected_scan(a), clearance=(0.,)*BIN_COUNT,
                                  hazard=(2.,)*BIN_COUNT)
            self.view.update(observation, a)
            self.assertEqual(self.planner.tick(a.stamp, a, self.view), STOP)
        self.assertEqual(self.planner.phase, "wait")

class IntegratedAcceptance(unittest.TestCase):
    def run_world(self, obstacle):
        planner = ObstacleNavigator(A_SHAPE)
        b_route = VehicleBRoute()
        coordinator = MergeCoordinator(A_SHAPE, B_SHAPE)
        a = surface_state(State(1.0, *A_START))
        b = surface_state(State(1.0, *B_START,
                        math.atan2(B_ROUTE[1][1]-B_START[1], B_ROUTE[1][0]-B_START[0])))
        view = ObservedSpace()
        obstacle_shape = box(*obstacle) if obstacle else None
        min_pair = min_obstacle = math.inf
        phases, entries = set(), []
        wait_start = leave_wait = None
        old_phase = planner.phase
        for trial_step in range(3200):
            if trial_step % 2 == 0:
                view.update(projected_scan(a, obstacle), a)
            cmd_a = planner.tick(a.stamp, a, view)
            decision = coordinator.step(a.stamp, a, b, cmd_a, b_route.tick(b))
            self.assertLessEqual(len(decision.occupied), 1)
            if decision.occupied and (not entries or decision.occupied[0] != entries[-1]):
                entries.append(decision.occupied[0])
            poses = A_SHAPE.at(a), B_SHAPE.at(b)
            min_pair = min(min_pair, polygon_distance(*poses))
            if obstacle_shape:
                min_obstacle = min(min_obstacle, polygon_distance(poses[0], obstacle_shape))
            phases.add(planner.phase)
            if wait_start is None and planner.phase == "wait":
                wait_start = a.stamp
            if leave_wait is None and old_phase == "wait" and planner.phase != "wait":
                leave_wait = a.stamp
            old_phase = planner.phase
            if (planner.phase == "arrived" and math.dist((b.x, b.y), B_GOAL) <= .25
                    and max(a.speed, b.speed) <= .025):
                break
            a = surface_state(integrate(a, decision.a, coordinator.brakes["A"], .2))
            b = surface_state(integrate(b, decision.b, coordinator.brakes["B"], .2))
        else:
            self.fail(f"Goals unfinished. A={a}, {planner.phase}, {planner.goal}; B={b}; last {decision}; min obstacle {min_obstacle}")
        self.assertEqual(entries, ["A", "B"])
        self.assertGreaterEqual(min_pair, CLEARANCE)
        if obstacle:
            self.assertTrue({"brake", "wait", "detour", "rejoin", "arrived"} <= phases, phases)
            self.assertGreaterEqual(leave_wait-wait_start, HOLD_SECONDS)
            self.assertGreaterEqual(min_obstacle, OBSTACLE_CLEARANCE)
        return min_pair, min_obstacle

    def test_normal_merge_reaches_both_goals(self):
        self.run_world(None)

    def test_detour_and_merge_for_unknown_obstacle_locations(self):
        for obstacle in ((-16.4, 0), (-15.8, -.55)):
            with self.subTest(obstacle=obstacle):
                self.run_world(obstacle)


class GazeboWorldTests(unittest.TestCase):
    @patch("jarcrac_cooperative_sil.time.monotonic")
    def test_external_state_checks_frames_freshness_and_duplicate_source_time(self, clock):
        def odometry(t, x, parent=FRAME):
            sec, nsec = divmod(round(t * 1e9), 1_000_000_000)
            header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nsec=nsec),
                                     HasField=lambda field: field == "stamp",
                                     data=[SimpleNamespace(key="frame_id", value=[parent]),
                                           SimpleNamespace(key="child_frame_id", value=[NAMES[0]])])
            wx, wy = world_xy(x, 0.)
            return SimpleNamespace(header=header, HasField=lambda field: field == "header",
                                   pose=SimpleNamespace(position=SimpleNamespace(x=wx, y=wy, z=-10.5),
                                                        orientation=SimpleNamespace(x=0, y=0, z=math.sin(MAP_YAW/2),
                                                                                    w=math.cos(MAP_YAW/2))))

        inputs = Inputs()
        clock.return_value = 2.0
        inputs.clock_sample(SimpleNamespace(sim=SimpleNamespace(sec=1, nsec=100_000_000)))
        inputs.vehicle_sample(0, odometry(1.0, 0.0))
        inputs.vehicle_sample(0, odometry(1.1, 0.01, "wrong_frame"))
        self.assertIsNone(inputs.snapshot()[1])
        inputs.vehicle_sample(0, odometry(1.1, 0.01))
        self.assertAlmostEqual(inputs.snapshot()[1].vx, .10)
        clock.return_value = 2.25
        inputs.clock_sample(SimpleNamespace(sim=SimpleNamespace(sec=1, nsec=200_000_000)))
        inputs.vehicle_sample(0, odometry(1.1, 10.0))
        clock.return_value = 2.31
        self.assertIsNone(inputs.snapshot()[1])

    def test_native_scan_uses_ground_mesh_and_localized_sensor_pose(self):
        x,y,z,roll,pitch,_ = spawn_pose(TERRAIN, A_SHAPE, A_START, 0.)
        a = State(1.0, *route_xy(x, y), roll=roll, pitch=pitch, z=z)
        scan = ray_fixture(a)
        clear = scan_from_message(scan, a.stamp, a, TERRAIN)
        self.assertIsNotNone(clear)
        self.assertGreater(clear.ground_returns, 100)
        self.assertEqual(clear.elevated_returns, 0)
        self.assertAlmostEqual(clear.x, a.x+rotate(FRONT_SENSOR, roll, pitch, 0)[0])
        self.assertTrue(clear.verdict(a.x+4, 0))
        self.assertIsNone(clear.verdict(a.x-3, 0))
        self.assertIsNone(scan_from_message(scan, a.stamp, replace(a, z=a.z+3), TERRAIN))
        self.assertIsNone(scan_from_message(SimpleNamespace(**{**vars(scan), "ranges": scan.ranges[:-1]}),
                                            a.stamp, a, TERRAIN))

    def test_raised_obstacle_blocks_clearance_and_no_return_is_unknown(self):
        a = surface_state(State(1.0, *A_START))
        obstacle = (a.x+5.0, 0.0)
        scan = ray_fixture(a, obstacle)
        detected = scan_from_message(scan, a.stamp, a, TERRAIN)
        self.assertIsNotNone(detected)
        self.assertGreater(detected.elevated_returns, 0)
        self.assertFalse(detected.verdict(*obstacle))
        planner = ObstacleNavigator(A_SHAPE)
        self.assertEqual(planner.tick(a.stamp, a, ObservedSpace().update(detected, a)), STOP)
        self.assertEqual(planner.phase, "brake")
        scan.ranges[:] = [math.inf] * len(scan.ranges)
        self.assertIsNone(scan_from_message(scan, a.stamp, a, TERRAIN))

    def test_observed_history_only_covers_previously_seen_blind_space(self):
        a = state(-13, 0)
        space = ObservedSpace().update(projected_scan(a), a)
        self.assertFalse(space.covers(-14, 0))
        shifted = replace(a, x=-10, stamp=31.0)
        space.update(projected_scan(shifted), shifted)
        self.assertTrue(space.covers(-10.5, 0))
        self.assertFalse(space.covers(-14, 0))
        later = replace(shifted, stamp=80.0)
        space.update(projected_scan(later), later)
        self.assertFalse(space.covers(-10.5, 0))

    def test_pose_evaluator_requires_both_footprints_to_stay_out_of_shared_conflict(self):
        score = Score((A_SHAPE, B_SHAPE), 0)
        score.observe(dict(zip(NAMES, (state(0, 0), state(-2, -1)))), 1.0)
        self.assertEqual(score.result()["dual_occupancy_samples"], 1)
        self.assertFalse(score.result()["geometry_passed"])

    def test_generated_world_uses_dedicated_topics_and_model_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "test.sdf"
            prepare(output, [(-16.4, 0)])
            world = ET.parse(output).getroot().find("world")
            self.assertEqual(world.get("name"), WORLD)
            for i, name in enumerate(NAMES):
                model = world.find(f"model[@name='{name}']")
                self.assertIn(ACTUATORS[i], [p.findtext("topic") for p in model.findall("plugin")])
                self.assertIn(SIM_STATE[i], [p.findtext("odom_topic") for p in model.findall("plugin")])
            self.assertEqual(world.findtext("include/uri"), "model://ruskotunturi_map")
            sensor = world.find(f"model[@name='{NAMES[0]}']/link/sensor")
            self.assertEqual(sensor.get("name"), "gpu_lidar_front")
            self.assertEqual(sensor.findtext("topic"), LIDAR)
            self.assertEqual(tuple(map(float, sensor.findtext("pose").split()[:3])), (1.68, 0., -.171))
            hor = sensor.find("ray/scan/horizontal")
            self.assertAlmostEqual(float(hor.findtext("max_angle"))-float(hor.findtext("min_angle")), math.pi)
            self.assertAlmostEqual(float(hor.findtext("min_angle"))+math.pi/2, HALF_STEP)
            self.assertEqual(int(hor.findtext("samples")), HORIZONTAL_SAMPLES)
            self.assertEqual(int(sensor.findtext("ray/scan/vertical/samples")), VERTICAL_SAMPLES)
            for i, route_start in enumerate((A_START, B_START)):
                model_pose = tuple(map(float, world.findtext(f"model[@name='{NAMES[i]}']/pose").split()))
                for got, expected in zip(route_xy(*model_pose[:2]), route_start):
                    self.assertAlmostEqual(got, expected)
                self.assertAlmostEqual(model_pose[2], height_at(TERRAIN, *route_start), delta=.12)
            with self.assertRaises(ValueError):
                prepare(Path(tmp) / "blocked-start.sdf", [A_START])
            a_bounds, b_bounds = A_SHAPE.bounds(), B_SHAPE.bounds()
            self.assertLess(a_bounds[1][0], -.88)
            self.assertGreater(a_bounds[2][1], 2.2)
            self.assertGreater(b_bounds[1][1], 1.0)
            self.assertLess(b_bounds[2][1], .8)
