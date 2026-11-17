"""Sensor-driven convoy navigation on a surveyed straight Ruskotunturi strip."""
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
import math
from statistics import median

from final_sil_v6_checks import polygon_distance, scan_valid
from jarcrac_cooperative_control import (
    AGE_LIMIT, CLEARANCE, GEOMETRY_MARGIN, PAIR_LIMIT, STOP, STOP_SPEED, Braking, Decision,
    Motion, ObstacleNavigator, angle_error,
)
from jarcrac_cooperative_geometry import rotate
from jarcrac_cooperative_terrain import GROUND_TOLERANCE, LOCAL_FRAME, ObservedSpace, RouteFrame, terrain_index

TILT_LIMIT = math.radians(8.0)
LANE_HALF_WIDTH = 6.0
ACKERMANN_CURVATURE = .23
PLANAR_MIN_HEIGHT = .08
PLANAR_MAX_HEIGHT = 1.80
GOAL_POSITION_M = .68
PLANAR_HORIZON = 21.0


@dataclass(frozen=True)
class Trial:
    frame: RouteFrame
    starts: tuple
    goals: tuple
    a_speed: float = .30
    b_speed: float = .36
    sim_timeout: float = 295.0
    b_detour_speed: float = .30
    start_y: tuple = (0., 0.)
    start_yaw: tuple = (0., 0.)
    merge_zone: tuple = ()
    half_width: float = LANE_HALF_WIDTH

    @property
    def solo(self):
        return self.starts[0] is None

    def start(self, index):
        return self.starts[index], self.start_y[index]


CONVOY = Trial(LOCAL_FRAME, (-10., -18.), (45., 34.))
B_PRIVATE = Trial(RouteFrame(LOCAL_FRAME.world(0., -8.), LOCAL_FRAME.yaw),
                  (None, -16.), (None, 27.), a_speed=0., sim_timeout=210.)
TRIALS = {"convoy": CONVOY, "b-private": B_PRIVATE}


def mapped_terrain():
    return terrain_index(LOCAL_FRAME.origin, 68.0)


def launch_space(trial):
    start = trial.starts[0]
    return ObservedSpace(trial.frame, (start, 0.), (start-3.1, start+3.1), (-3., 3.))


def stationary(state):
    return state.speed <= STOP_SPEED and abs(state.omega) <= STOP_SPEED


def aligned(state):
    return abs(state.y) <= .30 and abs(state.yaw) <= .20


def point_gap(point, polygon):
    """Unsigned point-to-convex-polygon distance, including points on the boundary."""
    signs, smallest = set(), math.inf
    x, y = point
    for p, q in zip(polygon, (*polygon[1:], polygon[0])):
        dx, dy = q[0]-p[0], q[1]-p[1]
        side = dx*(y-p[1])-dy*(x-p[0])
        if abs(side) > 1e-9:
            signs.add(side > 0)
        length2 = dx*dx+dy*dy
        u = min(1., max(0., ((x-p[0])*dx+(y-p[1])*dy)/length2)) if length2 else 0.
        smallest = min(smallest, math.hypot(x-p[0]-u*dx, y-p[1]-u*dy))
    return smallest if len(signs) > 1 else 0.


@dataclass(frozen=True)
class Beam:
    angle: float
    run: float
    rise: float
    hit: bool


class PlanarEvidence:
    """Horizontal projection of the LiDAR plane for the specified obstacle height."""
    def __init__(self, stamp, origin, beams, contacts, terrain):
        self.stamp, self.origin, self.terrain = stamp, origin, terrain
        self.beams = tuple(sorted(beams, key=lambda b: b.angle))
        self.angles = tuple(b.angle for b in self.beams)
        self.contacts = tuple(contacts)

    def covers(self, x, y):
        ox, oy, oz = self.origin
        ground = self.terrain.height_at(x, y)
        if ground is None:
            return False
        dx, dy = x-ox, y-oy
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        i = bisect_right(self.angles, angle)
        left, right = self.beams[i-1], self.beams[i % len(self.beams)]
        # A lost column or a scanner with a narrower real FoV leaves a gap.
        if (right.angle-left.angle) % math.tau > .035:
            return False
        if distance < .12:
            return not any(b.hit and b.run <= .12 for b in (left, right))
        for beam in (left, right):
            plane_height = oz+distance*beam.rise-ground
            if not PLANAR_MIN_HEIGHT <= plane_height <= PLANAR_MAX_HEIGHT:
                return False
            if distance+.15 > beam.run:
                return False
        return True


def planar_scan(msg, stamp, pose, terrain):
    if (msg.vertical_count not in (0, 1) or msg.count != len(msg.ranges) or not scan_valid(msg)):
        return None
    ox, oy, oz = rotate((0., 0., 1.40), pose.roll, pose.pitch, pose.yaw)
    origin = pose.x+ox, pose.y+oy, pose.z+oz
    axis_x, axis_y = (rotate(v, pose.roll, pose.pitch, pose.yaw) for v in ((1., 0., 0.), (0., 1., 0.)))
    beams, contacts = [], []
    for i, value in enumerate(msg.ranges):
        theta = msg.angle_min+i*msg.angle_step
        c, s = math.cos(theta), math.sin(theta)
        dx, dy, dz = (c*u+s*v for u, v in zip(axis_x, axis_y))
        horizontal = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        if value == math.inf:
            beams.append(Beam(angle, min(PLANAR_HORIZON, msg.range_max*horizontal), dz/horizontal, False))
            continue
        if not msg.range_min <= value <= msg.range_max:
            beams.append(Beam(angle, 0., dz/horizontal, True))
            continue
        px, py, pz = (p+v*value for p, v in zip(origin, (dx, dy, dz)))
        h = terrain.height_at(px, py)
        is_ground = h is not None and abs(pz-h) <= GROUND_TOLERANCE
        beams.append(Beam(angle, min(PLANAR_HORIZON, value*horizontal), dz/horizontal, not is_ground))
        if h is not None and pz-h > GROUND_TOLERANCE and value*horizontal <= PLANAR_HORIZON:
            contacts.append((px, py))
    return PlanarEvidence(stamp, origin, beams, contacts, terrain)


class StaticEvidence:
    """Keep distinct, fresh observations of a bounded cluster across simulated time."""
    def __init__(self):
        self.points = deque()
        self.stamp = -math.inf

    def reset(self):
        self.points.clear()
        self.stamp = -math.inf

    def update(self, scan, state, frame, leader):
        if scan.stamp <= self.stamp:
            return self.confirmed
        if scan.stamp-self.stamp > AGE_LIMIT:
            self.points.clear()
        self.stamp = scan.stamp
        contacts = []
        for point in scan.contacts:
            x, y = frame.local(*point)
            if not 0.8 < x-state.x < 15.5 or abs(y-state.y) > 1.6:
                continue
            if leader and point_gap((x, y), leader) <= .55:
                continue  # Keep raw returns in the clearance check.
            contacts.append((x, y))
        if not contacts:
            self.points.clear()
            return False
        front = min(x for x, _ in contacts)
        face = [(x, y) for x, y in contacts if x <= front+.30]
        position = tuple(median(p[i] for p in face) for i in (0, 1))
        self.points.append((self.stamp, position))
        while self.points and self.stamp-self.points[0][0] > 1.50:
            self.points.popleft()
        return self.confirmed

    @property
    def confirmed(self):
        if len(self.points) < 4 or self.points[-1][0]-self.points[0][0] < 1.10:
            return False
        xs, ys = zip(*(point for _, point in self.points))
        return math.hypot(max(xs)-min(xs), max(ys)-min(ys)) <= .23


class LaneView:
    def __init__(self, scan, frame, stop_at, half_width=LANE_HALF_WIDTH, *, start_at=-24.):
        self.scan, self.frame, self.stop_at = scan, frame, stop_at
        self.half_width = half_width
        self.start_at = start_at
        self.stamp = scan.stamp

    def covers(self, x, y):
        return (self.start_at <= x <= self.stop_at and abs(y) <= self.half_width
                and self.scan.covers(*self.frame.world(x, y)))


class TrackedNavigator(ObstacleNavigator):
    def __init__(self, shape, destination, cruise):
        super().__init__(shape, destination=(destination, 0.), approach_end=destination+1.0,
                         cruise=cruise, padding=.62, tilt_limit=TILT_LIMIT, follow_preview=3.2)

    def tick(self, now, state, scan):
        if self.phase == "arrived":
            return STOP
        if (self.phase not in ("brake", "wait") and state.fresh(now, self.tilt_limit)
                and scan is not None and -0.02 <= now-scan.stamp <= AGE_LIMIT
                and aligned(state) and math.dist((state.x, state.y), self.destination) <= GOAL_POSITION_M
                and self.stopping_space_clear(state, scan)):
            self.phase = "arrived"
            return STOP
        return super().tick(now, state, scan)


class AckermannNavigator(ObstacleNavigator):
    """Plan forward, curvature-limited outward, passing and rejoining segments."""
    arrival_radius = GOAL_POSITION_M

    def __init__(self, shape, destination, cruise, detour_speed=.30):
        super().__init__(shape, destination=(destination, 0.), approach_end=destination+1.0,
                         cruise=cruise, padding=.98, tilt_limit=TILT_LIMIT, follow_preview=12.1)
        self.brakes = Braking(.35, .65, shape.radius)
        self.detour_speed = detour_speed
        self.detour_allowed = False
        self.wait_for_leader = False
        self.static_ready = False
        self.detour_committed = False
        self.interrupted_segment = None
        self.replans = 0
        self.follow_target_ahead = 4.0

    def route_goal(self, state, ahead=None):
        if self.wait_for_leader:
            return state.x, state.y
        return super().route_goal(state, self.follow_target_ahead if ahead is None else ahead)

    def verified_end(self, scan, curve, start, end):
        steps = max(1, math.ceil((end-start)/.10))
        for i in range(steps+1):
            x, y, yaw, bend = curve.at(start+(end-start)*i/steps)
            if abs(bend) > ACKERMANN_CURVATURE or not self.clear_at(scan, x, y, yaw):
                return start+(end-start)*(i-1)/steps if i else None
        return end

    def latest_detour_end(self, lateral):
        need = math.sqrt(6*abs(lateral)/ACKERMANN_CURVATURE)
        return self.approach_end-self.padded_bounds[0][1]-need-.55

    def detour_goal(self, state, scan):
        candidates = [(state.x+dx, y) for dx in (10., 11.5, 13., 15.)
                      for y in (-3.60, 3.60, -4.20, 4.20)]
        if abs(state.y) >= 1.7:
            candidates.extend((state.x+dx, state.y) for dx in (2.4, 4.0, 5.8))
        candidates = sorted(set((min(x, self.latest_detour_end(y)), y) for x, y in candidates),
                            key=lambda p: (math.dist((state.x, state.y), p), p[1], -p[0]))
        for goal in candidates:
            curve = self.curve_to(state, goal)
            if curve and self.verified_end(scan, curve, state.x, curve.end_x) == curve.end_x:
                return goal, curve
        return None

    def rejoin_goal(self, state, scan):
        limit = self.approach_end-self.padded_bounds[0][1]-.20
        for ahead in (3.8, 5.5, 7.5, 9.0, 10.5, 12.0, 14.0):
            goal = min(state.x+ahead, limit), 0.
            if self.clear_segment(state, scan, goal):
                return goal
        return None

    def resume_segment(self, state, scan):
        if self.interrupted_segment is None:
            return False
        phase, goal = self.interrupted_segment
        curve = self.curve_to(state, goal)
        if curve is None:
            return False
        seen = self.verified_end(scan, curve, state.x, curve.end_x)
        if seen == curve.end_x:
            self.interrupted_segment = None
        elif phase == "detour" and seen is not None and seen-state.x >= 1.25:
            # Publish only the verified portion and reconsider from its end.
            goal = curve.at(seen)[:2]
        else:
            return False
        self.phase, self.goal, self.curve = phase, goal, curve
        return True

    def choose_path(self, state, scan):
        if not self.detour_allowed:
            self.stop_reason = "leader_corridor_reserved"
            return False
        if not (self.static_ready or self.detour_committed):
            self.stop_reason = "waiting_for_clear_or_stationary_obstacle"
            return False
        if not self.resume_segment(state, scan):
            if not super().choose_path(state, scan):
                self.stop_reason = "no_safe_detour"
                return False
            self.interrupted_segment = None
        if self.detour_committed:
            self.replans += 1
        self.detour_committed = True
        return True

    def brake(self, reason):
        if self.interrupted_segment is None and self.phase in ("detour", "rejoin") and self.goal:
            self.interrupted_segment = self.phase, self.goal
        return super().brake(reason)

    def tracked_motion(self, state, curve, end):
        x, y, _, _ = curve.at(min(end, state.x+1.40))
        dx, dy = x-state.x, y-state.y
        turn = 2*math.sin(angle_error(math.atan2(dy, dx), state.yaw))/max(.55, math.hypot(dx, dy))
        curvature = max(-ACKERMANN_CURVATURE, min(ACKERMANN_CURVATURE, turn))
        cap = min(self.cruise, self.detour_speed) if self.phase != "follow" else self.cruise
        speed = min(cap, .39*(end-state.x))
        return Motion(speed, speed*curvature)

    def tick(self, now, state, scan):
        if self.phase == "arrived":
            return STOP
        if self.wait_for_leader and self.phase not in ("brake", "wait"):
            return self.brake("leader_corridor_reserved")
        if (self.phase not in ("brake", "wait") and state.fresh(now, self.tilt_limit)
                and scan is not None and -0.02 <= now-scan.stamp <= AGE_LIMIT
                and aligned(state) and math.dist((state.x, state.y), self.destination) <= self.arrival_radius
                and self.stopping_space_clear(state, scan)):
            self.phase = "arrived"
            return STOP
        return super().tick(now, state, scan)


def speed_cap(command, cap):
    if command.speed <= 0. or cap <= 0.:
        return STOP
    speed = min(command.speed, cap)
    return Motion(speed, command.yaw_rate*speed/command.speed)


class ConvoyMission:
    def __init__(self, shapes, trial):
        self.shapes, self.trial = shapes, trial
        self.leader = None if trial.solo else TrackedNavigator(shapes[0], trial.goals[0], trial.a_speed)
        self.follower = AckermannNavigator(shapes[1], trial.goals[1], trial.b_speed, trial.b_detour_speed)
        self.static = StaticEvidence()
        self.a_corridor_end = None
        self.b_corridor_end = None
        self.done_since = None
        self.done = False

    def fail_closed(self, reason):
        if self.leader:
            self.leader.stopped_since = None
        self.follower.stopped_since, self.done_since = None, None
        self.static.reset()
        return Decision(STOP, STOP, None, False, (), reason, None)

    def waiting_point(self, leader, leader_rear):
        """Return a candidate waiting point when the leader owns the bypass."""
        return None

    def step(self, now, a, b, a_scan, b_scan, *, following=True):
        if (now is None or b is None or not b.fresh(now, TILT_LIMIT)
                or (self.leader and (a is None or not a.fresh(now, TILT_LIMIT) or abs(a.stamp-b.stamp) > PAIR_LIMIT))):
            return self.fail_closed("stale_or_excessive_tilt")
        frame = self.trial.frame
        b = frame.state(b)
        b_poly = self.shapes[1].at(b)
        b_rear = min(x for x, _ in b_poly)
        a_command, a_poly, headway_stop = STOP, None, False
        leader_ready, leader_reserved = self.trial.solo, False
        max_b_station = self.trial.goals[1]+2.85
        if self.leader:
            a = frame.state(a)
            a_poly = self.shapes[0].at(a)
            a_rear = min(x for x, _ in a_poly)
            if following and (a.x <= b.x or a.vx < -STOP_SPEED):
                return self.fail_closed("leader_return_or_wrong_order")
            gap = polygon_distance(a_poly, b_poly)
            if gap <= CLEARANCE:
                return self.fail_closed("vehicle_clearance")
            leader_view = LaneView(a_scan, frame, self.trial.goals[0]+3.1, self.trial.half_width) if a_scan else None
            a_command = self.leader.tick(now, a, leader_view)
            if self.leader.phase in ("detour", "rejoin") and (not self.trial.merge_zone or not aligned(a)):
                front = max(x for x, _ in a_poly)
                self.a_corridor_end = front if self.a_corridor_end is None else max(self.a_corridor_end, front)
            leader_ready = (self.a_corridor_end is not None and a_rear > self.a_corridor_end+.25
                            and self.leader.phase in ("follow", "arrived") and aligned(a))
            leader_reserved = (self.leader.phase in ("brake", "wait", "detour", "rejoin")
                               or self.a_corridor_end is not None and not leader_ready)
            if not following:
                return Decision(a_command, STOP, None, False, (), "merge_approach", None)
            max_b_station = min(max_b_station, a_rear-CLEARANCE-GEOMETRY_MARGIN)
            headway_stop = (b.vx >= a.vx-STOP_SPEED
                            and gap <= CLEARANCE+.35+self.follower.brakes.distance(b, Motion(self.trial.b_speed)))
        if self.follower.detour_committed and self.follower.phase in ("follow", "arrived") and aligned(b):
            if self.b_corridor_end is not None and b_rear > self.b_corridor_end+.15:
                self.follower.detour_committed = False
                self.follower.interrupted_segment = None
                self.b_corridor_end = None

        sense_ready = b_scan is not None and -.02 <= now-b_scan.stamp <= AGE_LIMIT
        if sense_ready:
            self.follower.static_ready = self.static.update(b_scan, b, frame, a_poly)
        else:
            self.static.reset()
            self.follower.static_ready = False
        self.follower.detour_allowed = leader_ready or self.follower.detour_committed
        reserved = leader_reserved and not self.follower.detour_committed
        waiting_point = self.waiting_point(a, a_rear) if reserved else None
        self.follower.wait_for_leader = reserved and waiting_point is None
        long_preview = self.follower.static_ready
        padded_front = self.follower.padded_bounds[0][1]
        room = max_b_station-b.x-padded_front-.18
        self.follower.follow_target_ahead = 13.0 if long_preview else min(4.0, max(1.0, room))
        self.follower.follow_preview = 12.1 if long_preview else min(3.1, self.follower.follow_target_ahead)
        if waiting_point is not None:
            # The approach and the bypass have separate permissions. Keep the
            # verified swept body on the near side of the reserved corridor.
            room = min(room, waiting_point-b.x)
            self.follower.wait_for_leader = room <= .18
            self.follower.follow_target_ahead = min(3.1, max(0., room))
            self.follower.follow_preview = self.follower.follow_target_ahead
            max_b_station = min(max_b_station, waiting_point+padded_front+.18)
        route_end = self.trial.goals[1]+1.0
        self.follower.approach_end = route_end if self.trial.merge_zone else min(route_end, max_b_station)
        view = LaneView(b_scan, frame, max_b_station, self.trial.half_width) if sense_ready else None
        b_command = self.follower.tick(now, b, view)
        if self.follower.phase in ("detour", "rejoin"):
            front = max(x for x, _ in b_poly)
            self.b_corridor_end = front if self.b_corridor_end is None else max(self.b_corridor_end, front)
        reason = self.follower.stop_reason or "clear"
        if self.leader:
            # Keep room for the longer turning approach when A pauses.
            parking = self.trial.merge_zone and self.leader.phase == "arrived"
            headway = 2.8 if parking else 6.0
            rate = .25 if parking else .13
            cap = max(0., a.vx+rate*max(0., a_rear-max(x for x, _ in b_poly)-headway))
            b_command = speed_cap(b_command, cap)
            if headway_stop:
                b_command, reason = STOP, "leader_headway"
        if waiting_point is not None and b_command.speed > 0.:
            reason = "approaching_waiting_point"
        settled = (self.follower.phase == "arrived" and stationary(b)
                   and (not self.leader or self.leader.phase == "arrived" and stationary(a)))
        if settled:
            if self.done_since is None:
                self.done_since = now
            self.done = now-self.done_since >= .70
        else:
            self.done_since = None
        owner = "B" if self.follower.detour_committed else "A" if leader_reserved else None
        return Decision(a_command, b_command, owner, owner == "B", (), reason, owner)
