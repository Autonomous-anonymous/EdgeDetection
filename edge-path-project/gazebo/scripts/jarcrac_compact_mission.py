"""Navigation and reservations for the shortened Ruskotunturi SIL mission."""
from bisect import bisect_right
from dataclasses import dataclass
import math

from final_sil_v6_checks import scan_valid
from jarcrac_cooperative_control import (
    AGE_LIMIT, CLEARANCE, HOLD_SECONDS, PAIR_LIMIT, STOP, STOP_SPEED, TRACK_SPEED_LIMIT,
    Decision, MergeCoordinator, Motion, ObstacleNavigator, angle_error, tracked_command,
)
from jarcrac_cooperative_geometry import rotate
from jarcrac_cooperative_terrain import GROUND_TOLERANCE, ObservedSpace, RouteFrame, terrain_index

MERGE = (-4.0, 28.0)
PRIVATE_FRAME = RouteFrame(MERGE, math.atan2(-2., -4.))
MERGE_FRAME = RouteFrame(MERGE, -3*math.pi/4)
START_YAWS = (PRIVATE_FRAME.yaw, math.pi)
COMPACT_START_A = PRIVATE_FRAME.world(-23.8, 0.0)
COMPACT_GOAL_A = MERGE_FRAME.world(16.0, 0.0)
COMPACT_GOAL_B = MERGE_FRAME.world(9.0, 0.0)
REJOIN_GOAL = PRIVATE_FRAME.world(-2.8, 0.0)
CONFLICT = ((-5.0, -2.6), (4.0, -2.6), (4.0, 5.7), (-5.0, 5.7))
WORLD_CONFLICT = tuple(MERGE_FRAME.world(*point) for point in CONFLICT)
TILT_LIMIT = math.radians(12.0)
GOAL_RADIUS = .44
B_SENSOR_HEIGHT = 1.40
A_APPROACH_HALF_WIDTH = 2.50


@dataclass(frozen=True)
class Scenario:
    starts: tuple
    goals: tuple
    a_speed: float
    b_speed: float
    sim_timeout: float
    wall_timeout: float

    @property
    def solo(self):
        return len(self.starts) == 1


COMPACT = Scenario((COMPACT_START_A, (18., 24.)), (COMPACT_GOAL_A, COMPACT_GOAL_B),
                   TRACK_SPEED_LIMIT, .85, 220., 285.)
REJOIN = Scenario(((20., 40.),), (REJOIN_GOAL,), .20, 0.0, 205., 285.)
SCENARIOS = {"compact": COMPACT, "rejoin": REJOIN}


def mission_terrain():
    return terrain_index((4.0, 28.0), 35.0)


def heading(p, q):
    return math.atan2(q[1]-p[1], q[0]-p[0])


def rounded(points, radius):
    yield points[0]
    for p, q, r in zip(points, points[1:], points[2:]):
        into, out = heading(p, q), heading(q, r)
        turn = angle_error(out, into)
        cut = min(radius*math.tan(abs(turn)/2), .46*math.dist(p, q), .46*math.dist(q, r))
        signed = math.copysign(cut/math.tan(abs(turn)/2), turn)
        x = q[0]-cut*math.cos(into)-signed*math.sin(into)
        y = q[1]-cut*math.sin(into)+signed*math.cos(into)
        steps = math.ceil(abs(turn)/.09)
        for i in range(steps+1):
            angle = into+turn*i/steps
            yield x+signed*math.sin(angle), y-signed*math.cos(angle)
    yield points[-1]


@dataclass(frozen=True)
class Segment:
    start: tuple
    ux: float
    uy: float
    length: float
    station: float
    yaw: float

    def at(self, along):
        return self.start[0]+along*self.ux, self.start[1]+along*self.uy


class Lane:
    def __init__(self, points, radius):
        vertices = tuple(rounded(points, radius))
        self.segments = []
        self.length = 0.0
        for p, q in zip(vertices, vertices[1:]):
            distance = math.dist(p, q)
            self.segments.append(Segment(p, (q[0]-p[0])/distance, (q[1]-p[1])/distance,
                                         distance, self.length, heading(p, q)))
            self.length += distance
        self.stations = tuple(segment.station for segment in self.segments)

    def at(self, station):
        s = max(0.0, min(station, self.length))
        segment = self.segments[bisect_right(self.stations, s)-1]
        return *segment.at(s-segment.station), segment.yaw

    def project(self, x, y):
        gap = math.inf
        for segment in self.segments:
            along = (x-segment.start[0])*segment.ux+(y-segment.start[1])*segment.uy
            along = max(0., min(segment.length, along))
            spot = segment.at(along)
            distance = (x-spot[0])**2+(y-spot[1])**2
            if distance < gap:
                gap, station = distance, segment.station+along
        return station, math.sqrt(gap)

    def follow(self, state, station, cruise, aim, max_curvature, stop_at, tracked=False):
        x, y, _ = self.at(min(station+aim, stop_at))
        dx, dy = x-state.x, y-state.y
        error = angle_error(math.atan2(dy, dx), state.yaw)
        if abs(error) > 1.0 or stop_at-station <= .18:
            return STOP
        turn = 2*math.sin(error)/max(math.hypot(dx, dy), .35)
        curvature = max(-max_curvature, min(max_curvature, turn))
        speed = min(cruise, .52*(stop_at-station))
        return tracked_command(speed, curvature) if tracked else Motion(speed, speed*curvature)


A_LANE = Lane((COMPACT_START_A, MERGE, COMPACT_GOAL_A), 4.1)
B_LANE = Lane((COMPACT.starts[1], (4., 24.), MERGE, COMPACT_GOAL_B), 5.5)


def coordinator(shapes):
    return MergeCoordinator(*shapes, conflict=CONFLICT, exit_half_width=2.30, tilt_limit=TILT_LIMIT)


def stationary(state):
    return state.speed <= STOP_SPEED and abs(state.omega) <= STOP_SPEED


def fresh(state, now):
    return state is not None and state.fresh(now, TILT_LIMIT)


def launch_space(scenario):
    sx, sy = PRIVATE_FRAME.local(*scenario.starts[0])
    return ObservedSpace(PRIVATE_FRAME, (sx, sy), (sx-3.1, sx+3.1), (-3.0, 3.0))


class PrivateScan:
    def __init__(self, space, reserve_b_approach):
        self.space, self.stamp = space, space.stamp
        self.reserve_b_approach = reserve_b_approach

    def covers(self, x, y):
        point = PRIVATE_FRAME.world(x, y)
        # Keep A's swept footprint on its side of the approaches while B uses
        # its own lane. Leave the nominal approach open as the lanes converge.
        if self.reserve_b_approach and y > A_APPROACH_HALF_WIDTH and B_LANE.project(*point)[1] < y+.35:
            return False
        return self.space.covers(*point)


class StopHold:
    def __init__(self):
        self.phase = "follow"
        self.since = self.last = None
        self.completed_seconds = 0.0

    def step(self, state, blocked, command):
        if blocked is None:
            self.since = None
            return STOP
        if self.phase == "follow" and not blocked:
            return command
        if not stationary(state):
            self.phase, self.since = "brake", None
            return STOP
        if self.since is None or state.stamp-self.last > AGE_LIMIT:
            self.since = state.stamp
        self.last, self.phase = state.stamp, "wait"
        held = state.stamp-self.since
        if not blocked and held >= HOLD_SECONDS:
            self.phase, self.since, self.completed_seconds = "follow", None, held
            return command
        return STOP


class Mission:
    def __init__(self, shapes, scenario=COMPACT):
        self.shapes, self.scenario = shapes, scenario
        self.merge = coordinator(shapes)
        self.navigator = ObstacleNavigator(shapes[0], destination=(-4.8, 0.), approach_end=-4.3,
                                           cruise=scenario.a_speed, padding=.50, tilt_limit=TILT_LIMIT)
        self.a_goal_station = A_LANE.project(*scenario.goals[0])[0]
        self.holds = (StopHold(), StopHold())
        self.private = True
        self.a_phase = "follow"
        self.done = False

    def a_drive(self, now, a, scan):
        if self.private:
            local = PRIVATE_FRAME.state(a)
            aligned = local.x >= -5.65 and abs(local.y) <= .25 and abs(local.yaw) <= .20
            if not (aligned and self.navigator.phase in ("follow", "rejoin")):
                view = PrivateScan(scan, not self.scenario.solo) if scan else None
                request = self.navigator.tick(now, local, view)
                self.a_phase = self.navigator.phase
                if self.a_phase != "arrived":
                    return request
            self.private = False
        if math.dist((a.x, a.y), self.scenario.goals[0]) <= GOAL_RADIUS:
            self.a_phase = "arrived"
            return STOP
        station, gap = A_LANE.project(a.x, a.y)
        request = A_LANE.follow(a, station, self.scenario.a_speed, 1.15, .85,
                                self.a_goal_station, tracked=True)
        samples = self.shapes[0].clearance_samples(a.roll, a.pitch, .22)
        blocked = None if scan is None else (
            gap > .75 or not self.navigator.stopping_space_clear(a, scan, samples=samples)
            or not all(self.navigator.clear_at(scan, *A_LANE.at(min(station+s, self.a_goal_station)), samples=samples)
                       for s in (.26, .58, 1.1)))
        result = self.holds[0].step(a, blocked, request)
        self.a_phase = self.holds[0].phase
        return result

    def b_drive(self, b, blocked):
        station, gap = B_LANE.project(b.x, b.y)
        if math.dist((b.x, b.y), COMPACT_GOAL_B) <= GOAL_RADIUS or gap > .85:
            return STOP
        request = B_LANE.follow(b, station, self.scenario.b_speed, 2.25, .235, B_LANE.length)
        return self.holds[1].step(b, blocked, request)

    def step(self, now, a, b, a_scan, b_blocked):
        if (now is None or not fresh(a, now)
                or (not self.scenario.solo and (not fresh(b, now) or abs(a.stamp-b.stamp) > PAIR_LIMIT))):
            self.navigator.stopped_since = None
            for hold in self.holds:
                hold.since = None
            return self.merge.fail_closed("stale_vehicle_state")
        if a_scan and not -.02 <= now-a_scan.stamp <= AGE_LIMIT:
            a_scan = None
        a_request = self.a_drive(now, a, a_scan)
        a_done = self.a_phase == "arrived" and stationary(a)
        if self.scenario.solo:
            self.done = a_done
            return Decision(a_request, STOP, None, False, (), "rejoin_trial", None)
        self.done = a_done and math.dist((b.x, b.y), COMPACT_GOAL_B) <= GOAL_RADIUS and stationary(b)
        return self.merge.step(now, MERGE_FRAME.state(a), MERGE_FRAME.state(b),
                               a_request, self.b_drive(b, b_blocked))


def b_scan_blocked(msg, pose, terrain, shape, brakes):
    """Check the original planar LiDAR against the upcoming stopping sweep."""
    if msg.vertical_count not in (0, 1) or msg.count != len(msg.ranges) or not scan_valid(msg):
        return None
    station, _ = B_LANE.project(pose.x, pose.y)
    stopping = brakes.distance(pose)
    extent = max(stopping, min(max(4.4, stopping+CLEARANCE), B_LANE.length-station+.25))
    steps = max(1, math.ceil(extent/.55))
    preview = []
    for i in range(steps+1):
        s = station+extent*i/steps
        x, y, yaw = B_LANE.at(s)
        c, d = math.cos(yaw), math.sin(yaw)
        overshoot = max(0., s-B_LANE.length)
        preview.append((x+overshoot*c, y+overshoot*d, c, d))
    (lo_x, hi_x), (lo_y, hi_y), _ = shape.bounds(pose.roll, pose.pitch)
    offset = rotate((0, 0, B_SENSOR_HEIGHT), pose.roll, pose.pitch, pose.yaw)
    x_axis, y_axis = (rotate(axis, pose.roll, pose.pitch, pose.yaw) for axis in ((1, 0, 0), (0, 1, 0)))
    for i, distance in enumerate(msg.ranges):
        angle = msg.angle_min+i*msg.angle_step
        if distance == -math.inf and abs(angle_error(angle, 0.0)) <= math.pi/3:
            return True
        if not msg.range_min <= distance <= extent+shape.radius+CLEARANCE:
            continue
        u, v = distance*math.cos(angle), distance*math.sin(angle)
        px, py, pz = (base+shift+u*ex+v*ey for base, shift, ex, ey in
                      zip((pose.x, pose.y, pose.z), offset, x_axis, y_axis))
        ground = terrain.height_at(px, py)
        if ground is not None and abs(pz-ground) <= GROUND_TOLERANCE:
            continue
        for x, y, c, s in preview:
            dx, dy = px-x, py-y
            along, across = dx*c+dy*s, -dx*s+dy*c
            if (lo_x-CLEARANCE <= along <= hi_x+CLEARANCE
                    and lo_y-CLEARANCE <= across <= hi_y+CLEARANCE):
                return True
    return False
