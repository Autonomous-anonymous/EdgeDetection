"""JARCRAC terrain obstacle and bidirectional merge control, independent of Gazebo."""
from dataclasses import dataclass
from functools import cached_property
import math

from final_sil_v6_checks import polygon_distance

CONFLICT = ((-5.0, -5.0), (4.0, -5.0), (4.0, 3.0), (-5.0, 3.0))
EXIT_X = 4.0
A_START = (-22.0, 0.0)
B_START = (-15.0, -12.0)
B_ROUTE = (B_START, (-3.0, -2.4), (1.5, 0.0), (17.0, 0.0))
A_GOAL = (24.0, 0.0)
B_GOAL = B_ROUTE[-1]
AGE_LIMIT = 0.30
PAIR_LIMIT = 0.10
CLEARANCE = 1.0
STOP_SPEED = 0.025
HOLD_SECONDS = 5.0
GEOMETRY_MARGIN = 0.20
APPROACH = 3.0
A_CRUISE = 0.10
B_CRUISE = 0.31
TILT_LIMIT = math.radians(8.0)
TRACK_SPEED_LIMIT = 0.30
TRACK_SEPARATION = 1.27
STEERING_EFFICIENCY = 0.50
PATH_CURVATURE_LIMIT = 1.1


@dataclass(frozen=True)
class State:
    stamp: float
    x: float
    y: float
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    z: float = 0.0

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def fresh(self, now, tilt_limit=TILT_LIMIT):
        return (-0.02 <= now - self.stamp <= AGE_LIMIT and
                all(map(math.isfinite, (self.x, self.y, self.z, self.yaw, self.vx, self.vy, self.omega))) and
                abs(self.roll) <= tilt_limit and abs(self.pitch) <= tilt_limit)


@dataclass(frozen=True)
class Motion:
    speed: float = 0.0
    yaw_rate: float = 0.0


STOP = Motion()


@dataclass(frozen=True)
class Braking:
    deceleration: float
    angular_deceleration: float
    radius: float
    # Includes the permitted state age and a control/actuation allowance.
    reaction: float = 0.50

    def translation(self, speed):
        return speed * self.reaction + speed*speed / (2 * self.deceleration)

    def rotation(self, rate):
        return rate * self.reaction + rate*rate / (2 * self.angular_deceleration)

    def distance(self, state, command=STOP):
        v = max(state.speed, abs(command.speed))
        w = max(abs(state.omega), abs(command.yaw_rate))
        return self.translation(v) + self.radius * self.rotation(w)


@dataclass(frozen=True)
class Decision:
    a: Motion
    b: Motion
    owner: str | None
    committed: bool
    occupied: tuple
    reason: str
    granted: str | None


class MergeCoordinator:
    def __init__(self, a_footprint, b_footprint, *, conflict=CONFLICT, exit_half_width=1.65,
                 tilt_limit=TILT_LIMIT, exit_x=EXIT_X, exit_length=None):
        self.footprints = {"A": a_footprint, "B": b_footprint}
        self.brakes = {"A": Braking(0.18, 0.45, a_footprint.radius),
                       "B": Braking(0.35, 0.65, b_footprint.radius)}
        self.conflict, self.tilt_limit = conflict, tilt_limit
        self.exit_x = exit_x
        # Leave room for the longer body and its clearance beyond the junction.
        lengths = (high-low for (low, high), _, _ in (a_footprint.bounds(), b_footprint.bounds()))
        exit_end = exit_x + (max(lengths) + CLEARANCE + 0.50 if exit_length is None else exit_length)
        self.exit_corridor = ((exit_x, -exit_half_width), (exit_end, -exit_half_width),
                              (exit_end, exit_half_width), (exit_x, exit_half_width))
        self.owner = None
        self.committed = False
        self.entered = False

    def reserve(self, vehicle, *, committed=False, entered=False):
        self.owner, self.committed, self.entered = vehicle, committed, entered

    def outbound(self, state, polygon):
        return (min(x for x, _ in polygon) > self.exit_x + GEOMETRY_MARGIN and
                abs(state.y) <= 0.65 and math.cos(state.yaw) >= 0.85 and
                state.vx >= -STOP_SPEED and abs(state.vy) <= 0.10 and abs(state.omega) <= 0.08)

    def fail_closed(self, reason):
        if not self.committed:
            self.reserve(None)
        return Decision(STOP, STOP, self.owner, self.committed, (), reason, None)

    def step(self, now, a, b, a_request, b_request):
        if (a is None or b is None or not a.fresh(now, self.tilt_limit) or not b.fresh(now, self.tilt_limit)
                or abs(a.stamp - b.stamp) > PAIR_LIMIT):
            return self.fail_closed("stale_vehicle_state")

        states = {"A": a, "B": b}
        commands = {"A": a_request, "B": b_request}
        polygons = {k: self.footprints[k].at(state) for k, state in states.items()}
        gap = {k: polygon_distance(polygon, self.conflict) for k, polygon in polygons.items()}
        occupied = tuple(k for k, distance in gap.items() if distance <= 1e-6)
        if len(occupied) > 1:
            return Decision(STOP, STOP, self.owner, self.committed, occupied, "dual_occupancy", None)

        exit_gap = {k: polygon_distance(poly, self.exit_corridor) for k, poly in polygons.items()}
        a_clear = self.outbound(a, polygons["A"]) and exit_gap["A"] > GEOMETRY_MARGIN
        previous_owner = self.owner
        if self.owner and not self.committed:
            state = states[self.owner]
            stopping = self.brakes[self.owner].distance(state)
            moving = max(state.speed, abs(state.omega)) > STOP_SPEED
            if moving and gap[self.owner] <= stopping + GEOMETRY_MARGIN:
                self.committed = True

        if occupied:
            if self.committed and self.owner != occupied[0]:
                return Decision(STOP, STOP, self.owner, True, occupied, "reservation_violation", None)
            self.reserve(occupied[0], committed=True, entered=True)
        elif self.committed:
            state = states[self.owner]
            braking = self.brakes[self.owner].distance(state)
            if (gap[self.owner] > GEOMETRY_MARGIN + braking
                    and (self.entered or max(state.speed, abs(state.omega)) <= STOP_SPEED)):
                self.reserve(None)

        if not self.committed:
            requested = "B" if a_clear and gap["B"] <= APPROACH else None
            if requested is None and gap["A"] <= APPROACH and not a_clear:
                requested = "A"
            self.reserve(requested)
            if self.owner:
                state = states[self.owner]
                stopping = self.brakes[self.owner].distance(state)
                moving = max(state.speed, abs(state.omega)) > STOP_SPEED
                if moving and gap[self.owner] <= stopping + GEOMETRY_MARGIN:
                    self.committed = True

        reason = "clear"
        for label, state in states.items():
            if label == self.owner:
                continue
            if (self.outbound(state, polygons[label]) and commands[label].speed >= 0
                    and abs(commands[label].yaw_rate) < 0.08):
                # The complete body is downstream and its forward command takes it farther away.
                continue
            stopping = self.brakes[label].distance(state, commands[label])
            hold_before_b = label == "B" and not a_clear and (previous_owner == "B" or gap[label] <= APPROACH)
            entry_gap = min(gap[label], exit_gap[label]) if self.owner else gap[label]
            if hold_before_b or entry_gap <= stopping + GEOMETRY_MARGIN + 0.25:
                commands[label] = STOP
                reason = f"{label}_yields"

        clearance = polygon_distance(polygons["A"], polygons["B"])
        if clearance <= CLEARANCE:
            return Decision(STOP, STOP, self.owner, self.committed, occupied, "vehicle_clearance", None)

        # Apply a separate moving/stopped-leader guard, including when the cars are outside the zone.
        dx, dy = b.x - a.x, b.y - a.y
        separation = math.hypot(dx, dy)
        closing = {}
        for k, sign in (("A", 1), ("B", -1)):
            state = states[k]
            measured = (state.vx * dx + state.vy * dy) * sign / separation
            commanded = commands[k].speed * (math.cos(state.yaw) * dx + math.sin(state.yaw) * dy) * sign / separation
            closing[k] = max(measured, commanded) > STOP_SPEED
        for k in states:
            other = "A" if k == "B" else "B"
            budget = self.brakes[k].distance(states[k], commands[k])
            if closing[other]:
                budget += self.brakes[other].distance(states[other], commands[other])
            if closing[k] and clearance <= CLEARANCE + GEOMETRY_MARGIN + budget:
                commands[k] = STOP
                reason = f"{k}_brakes_for_vehicle"

        return Decision(commands["A"], commands["B"], self.owner, self.committed, occupied,
                        reason, self.owner)


def angle_error(target, current):
    return math.remainder(target - current, math.tau)


@dataclass(frozen=True)
class Curve:
    """Forward cubic with the current heading at the start and lane heading at the end."""
    x: float
    y: float
    slope: float
    end_x: float
    end_y: float

    @property
    def length(self):
        return self.end_x-self.x

    def at(self, x):
        t = min(1.0, max(0.0, (x-self.x)/self.length))
        dy = self.end_y-self.y
        start = self.slope * self.length
        y = self.y + start*t*(1-t)**2 + dy*t*t*(3-2*t)
        slope = (start*(1-4*t+3*t*t) + 6*dy*t*(1-t))/self.length
        bend = ((start*(-4+6*t) + dy*(6-12*t))/self.length**2
                / (1+slope*slope)**1.5)
        return x, y, math.atan(slope), bend


class ObstacleNavigator:
    """Stop and dwell, then search observable local segments and return to the route."""
    def __init__(self, footprint, *, destination=A_GOAL, approach_end=CONFLICT[0][0],
                 cruise=A_CRUISE, padding=.32, tilt_limit=TILT_LIMIT, follow_preview=2.80):
        self.footprint = footprint
        self.brakes = Braking(0.18, 0.45, footprint.radius)
        self.destination, self.approach_end, self.cruise = destination, approach_end, cruise
        self.padding, self.tilt_limit = padding, tilt_limit
        self.follow_preview = follow_preview
        self.phase = "follow"
        self.goal = None
        self.curve = None
        self.stopped_since = None
        self.last_hold_time = None
        self.completed_hold_seconds = 0.0
        self.rejoin_replans = 0
        self.stop_reason = None

    @cached_property
    def padded_bounds(self):
        # Enclose the body with the maximum projected dimensions over the permitted tilt.
        (x0, x1), (y0, y1), (z0, z1) = self.footprint.bounds()
        pad = self.padding + max(abs(z0), abs(z1))*math.sin(self.tilt_limit)*1.1
        return (x0-pad, x1+pad), (y0-pad, y1+pad)

    @cached_property
    def padded_samples(self):
        (x0, x1), (y0, y1) = self.padded_bounds
        # Sample the padded body across its width and length. Edge and interior samples
        # catch narrow features in the free-range envelope.
        return tuple((u, v) for u in (x0, (x0+x1)/2, 0.0, x1/2, x1)
                     for v in (y0, y0/2, 0.0, y1/2, y1))

    def clear_at(self, scan, x, y, yaw, *, samples=None):
        c, s = math.cos(yaw), math.sin(yaw)
        return all(scan.covers(x+c*u-s*v, y+s*u+c*v)
                   for u, v in (self.padded_samples if samples is None else samples))

    @staticmethod
    def curve_to(state, goal):
        if goal[0]-state.x < 0.10 or abs(state.yaw) > 1.0:
            return None
        return Curve(state.x, state.y, math.tan(state.yaw), *goal)

    def verified_end(self, scan, curve, start, end):
        extent = end-start
        steps = max(1, math.ceil(extent / 0.10))
        for i in range(steps+1):
            x, y, heading, bend = curve.at(start + extent*i/steps)
            if abs(bend) > PATH_CURVATURE_LIMIT or not self.clear_at(scan, x, y, heading):
                return start + extent*(i-1)/steps if i else None
        return end

    def clear_segment(self, state, scan, goal):
        curve = self.curve_to(state, goal)
        return (curve is not None
                and self.verified_end(scan, curve, state.x, curve.end_x) == curve.end_x)

    def tracked_motion(self, state, curve, end):
        x, y, _, _ = curve.at(min(end, state.x+0.75))
        angle = angle_error(math.atan2(y-state.y, x-state.x), state.yaw)
        curvature = 2*math.sin(angle) / max(0.25, math.hypot(x-state.x, y-state.y))
        speed = min(self.cruise, 0.65*(end-state.x))
        return tracked_command(speed, max(-PATH_CURVATURE_LIMIT, min(PATH_CURVATURE_LIMIT, curvature)))

    def choose_path(self, state, scan):
        rejoin = self.rejoin_goal(state, scan) if abs(state.y) > 0.28 else None
        if rejoin:
            self.phase = "rejoin"
            self.goal, self.curve = rejoin, self.curve_to(state, rejoin)
            return True
        option = self.detour_goal(state, scan)
        if option is None:
            return False
        self.phase = "detour"
        self.goal, self.curve = option
        return True

    def stopping_space_clear(self, state, scan, *, samples=None):
        distance = self.brakes.translation(state.speed)
        angle = math.copysign(self.brakes.rotation(abs(state.omega)), state.omega)
        heading = math.atan2(state.vy, state.vx)
        dx, dy = distance * math.cos(heading), distance * math.sin(heading)
        steps = max(1, math.ceil(max(distance, abs(angle)) / 0.16))
        # Check coasting as well as the requested segment when changing direction.
        # Independent ranges cover either translation or yaw stopping first.
        return all(self.clear_at(scan, state.x+dx*i/steps, state.y+dy*i/steps,
                                 state.yaw+angle*j/steps, samples=samples)
                   for i in range(steps+1) for j in range(steps+1))

    def route_goal(self, state, ahead=3.0):
        return min(self.destination[0], state.x+ahead), self.destination[1]

    def rejoin_goal(self, state, scan):
        limit = self.approach_end-self.padded_bounds[0][1]-.10
        visible_return = None
        for ahead in (3.2, 4.0, 4.8, 5.6):
            x, y = self.route_goal(state, ahead)
            goal = min(x, limit), y
            if self.clear_segment(state, scan, goal):
                if visible_return is None:
                    visible_return = goal
                origin = State(state.stamp, *goal)
                if self.clear_segment(origin, scan, self.route_goal(origin, 1.4)):
                    return goal
        # The verified return may reveal the lane from a better viewing
        # angle. The follower rechecks clearance before going past it.
        return visible_return

    def recover_rejoin(self, state, scan):
        """Verify a replacement curve starting at the measured pose."""
        goal = self.goal if self.goal and self.clear_segment(state, scan, self.goal) else self.rejoin_goal(state, scan)
        if goal is None:
            return None
        self.goal, self.curve = goal, self.curve_to(state, goal)
        self.rejoin_replans += 1
        return self.curve

    def latest_detour_end(self, lateral):
        # A lane-parallel cubic needs forward room to undo its lateral offset.
        run = math.sqrt(6*abs(lateral-self.destination[1])/PATH_CURVATURE_LIMIT)
        return self.approach_end-self.padded_bounds[0][1]-max(3.2, run)-.40

    def detour_goal(self, state, scan):
        candidates = [(state.x + dx, y) for dx in (3.4, 4.3, 5.2)
                      for y in (2.5, -2.5, 3.3, -3.3, 4.4, -4.4)]
        if abs(state.y) > 1.7:
            candidates.append((state.x+1.2, state.y))
        candidates = sorted(set((min(x, self.latest_detour_end(y)), y) for x, y in candidates),
                            key=lambda p: (math.dist((state.x, state.y), p), p[0], -p[1]))
        partial = None
        best_progress = 0.0
        for goal in candidates:
            curve = self.curve_to(state, goal)
            if curve is None:
                continue
            seen = self.verified_end(scan, curve, state.x, curve.end_x)
            if seen is None:
                continue
            if seen == curve.end_x:
                origin = State(state.stamp, *goal)
                if self.clear_segment(origin, scan, (goal[0]+0.9, goal[1])):
                    return goal, curve
            # Move only through a supported part of an outward curve, then look
            # again from the new position. The unknown continuation stays unused.
            progress = seen-state.x
            x, y, _, _ = curve.at(seen)
            if (progress >= 1.05 and progress > best_progress and x <= self.latest_detour_end(y)
                    and abs(y) >= abs(state.y)+0.30):
                partial, best_progress = ((x, y), curve), progress
        return partial

    def brake(self, reason):
        self.phase, self.goal, self.curve, self.stopped_since = "brake", None, None, None
        self.stop_reason = reason
        return STOP

    def tick(self, now, state, scan):
        if (state is None or not state.fresh(now, self.tilt_limit) or scan is None or
                not -0.02 <= now - scan.stamp <= AGE_LIMIT):
            self.stopped_since = None
            self.stop_reason = "state_or_scan_unavailable"
            return STOP
        if self.phase not in ("brake", "wait") and math.dist((state.x, state.y), self.destination) <= .25:
            self.phase = "arrived"
            return STOP

        if self.phase in ("brake", "wait"):
            if state.speed > STOP_SPEED or abs(state.omega) > STOP_SPEED:
                self.phase, self.stopped_since = "brake", None
                return STOP
            self.phase = "wait"
            if self.stopped_since is None or state.stamp-self.last_hold_time > AGE_LIMIT:
                self.stopped_since = state.stamp
            self.last_hold_time = state.stamp
            held = state.stamp-self.stopped_since
            if held < HOLD_SECONDS:
                return STOP
            route = self.route_goal(state)
            if abs(state.y) <= 0.28 and self.clear_segment(state, scan, route):
                self.phase, self.goal, self.curve = "follow", None, None
            elif not self.choose_path(state, scan):
                return STOP
            self.completed_hold_seconds = held
            self.stopped_since = None

        if self.goal is not None and math.dist((state.x, state.y), self.goal) <= 0.25:
            if self.phase == "rejoin" and abs(state.yaw) <= .22:
                self.phase, self.goal, self.curve = "follow", None, None
            elif self.phase != "rejoin" and not self.choose_path(state, scan):
                return self.brake("detour_continuation_unverified")

        curve = self.curve or self.curve_to(state, self.route_goal(state))
        end = self.goal[0] if self.goal else (curve.end_x if curve else state.x)
        if not self.stopping_space_clear(state, scan):
            return self.brake("stopping_space_unverified")
        if curve is None:
            return self.brake("path_unavailable")
        _, planned_y, heading, _ = curve.at(state.x)
        deviated = (state.x >= end or state.x < curve.x-.15 or abs(state.y-planned_y) > .20
                    or abs(angle_error(heading, state.yaw)) > .22)
        if deviated:
            curve = self.recover_rejoin(state, scan) if self.phase == "rejoin" else None
            if curve is None:
                return self.brake("rejoin_unverified" if self.phase == "rejoin" else "path_tracking_error")
            end = self.goal[0]
        progress = max(curve.x, state.x)
        command = self.tracked_motion(state, curve, end)
        stopping = self.brakes.distance(state, command)
        # Retain the slow trial's preview and add the extra coasting distance
        # when travelling faster on the approach.
        preview = .65
        if self.phase == "follow":
            excess = self.brakes.translation(max(state.speed, command.speed))-self.brakes.translation(.20)
            preview = self.follow_preview+max(0., excess)
        lookahead = min(end, progress + max(stopping+.30, preview))
        if self.verified_end(scan, curve, progress, lookahead) != lookahead:
            return self.brake("path_clearance_unverified")
        self.stop_reason = None
        return command


def tracked_command(speed, curvature):
    # This is the differential-speed relationship in Gazebo TrackedVehicle.
    ratio = abs(curvature)*TRACK_SEPARATION/(2*STEERING_EFFICIENCY)
    speed = min(speed, TRACK_SPEED_LIMIT/(1+ratio))
    return Motion(speed, speed*curvature)


class VehicleBRoute:
    def __init__(self):
        self.target = 1

    def tick(self, state):
        if state is None or math.dist((state.x, state.y), B_GOAL) <= 0.25:
            return STOP
        goal = B_ROUTE[self.target]
        if self.target < len(B_ROUTE)-1 and math.dist((state.x, state.y), goal) <= 1.65:
            self.target += 1
            goal = B_ROUTE[self.target]
        dx, dy = goal[0] - state.x, goal[1] - state.y
        error = angle_error(math.atan2(dy, dx), state.yaw)
        speed = min(B_CRUISE, math.hypot(dx, dy) * 0.6)
        curvature = max(-0.21, min(0.21, 2 * math.sin(error) / max(2.0, math.hypot(dx, dy))))
        return Motion(speed, speed * curvature)
