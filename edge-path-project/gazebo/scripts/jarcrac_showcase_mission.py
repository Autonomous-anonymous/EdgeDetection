"""Merge both vehicles into the lane, then use the sensor-driven convoy controller."""
import math
from types import SimpleNamespace

from final_sil_v6_checks import polygon_distance
from jarcrac_convoy_v2 import (
    ACKERMANN_CURVATURE, TILT_LIMIT, AckermannNavigator, ConvoyMission, LaneView,
    StaticEvidence, TrackedNavigator, Trial, aligned, speed_cap,
)
from jarcrac_cooperative_control import (
    AGE_LIMIT, CLEARANCE, GEOMETRY_MARGIN, PAIR_LIMIT, STOP,
    Curve, MergeCoordinator, Motion, angle_error,
)
from jarcrac_cooperative_terrain import BIN_STEP, LOCAL_FRAME

MERGE_EXIT = -8.6
MERGE_ZONE = ((-13.5, -2.15), (MERGE_EXIT, -2.15), (MERGE_EXIT, 2.15), (-13.5, 2.15))
SHOWCASE = Trial(LOCAL_FRAME, (-7.85, -19.7), (29.8, 23.6), b_speed=.78, sim_timeout=171.,
                 b_detour_speed=.70, start_y=(0., -3.65), start_yaw=(0., .26), merge_zone=MERGE_ZONE,
                 half_width=6.8)
MERGE_APPROACH_END = MERGE_EXIT+.25
# Keep the minimum 10 m outward curve, plus allowance for range projection.
TURN_IN_ROOM = 10.40


class ShowcaseLeader(TrackedNavigator):
    def __init__(self, shape, destination, cruise):
        super().__init__(shape, destination, cruise)
        self.static_ready = False
        self.bypassing = False

    def choose_path(self, state, scan):
        if not self.static_ready and not self.bypassing:
            self.stop_reason = "waiting_for_clear_or_stationary_obstacle"
            return False
        ready = super().choose_path(state, scan)
        self.bypassing |= ready
        return ready

    def detour_goal(self, state, scan):
        if abs(state.y) < .35:
            # A longer outward curve needs less differential track speed.
            for y in (2.7, -2.7, 3.3, -3.3):
                for length in (5.6, 6.4, 7.2):
                    goal = min(state.x+length, self.latest_detour_end(y)), y
                    curve = self.curve_to(state, goal)
                    if curve and self.verified_end(scan, curve, state.x, goal[0]) == goal[0]:
                        return goal, curve
        return super().detour_goal(state, scan)

    def tick(self, now, state, scan):
        if self.phase == "follow" and aligned(state):
            self.bypassing = False
        return super().tick(now, state, scan)


class ShowcaseFollower(AckermannNavigator):
    arrival_radius = .75

    def __init__(self, shape, trial, evidence):
        super().__init__(shape, trial.goals[1], trial.b_speed, trial.b_detour_speed)
        self.evidence = evidence
        self.obstruction_front = None

    def tracked_motion(self, state, curve, end):
        command = super().tracked_motion(state, curve, end)
        # Pace the queue while the leader is ahead on the same approach.
        if self.phase == "follow" and not self.detour_allowed:
            return speed_cap(command, .22)
        return command

    def rejoin_goal(self, state, scan):
        if self.obstruction_front is None or state.x <= self.obstruction_front+.45:
            return super().rejoin_goal(state, scan)
        end_of_lane = self.approach_end-self.padded_bounds[0][1]-.20
        stopping = self.brakes.distance(state, Motion(self.detour_speed))
        for length in (9.0, 10.5, 12.0, 14.0):
            goal = min(state.x+length, end_of_lane), 0.
            curve = self.curve_to(state, goal)
            if curve is None:
                continue
            steps = math.ceil(curve.length/.10)
            if any(abs(curve.at(state.x+curve.length*i/steps)[3]) > ACKERMANN_CURVATURE
                   for i in range(steps+1)):
                continue
            # Only authorize the observed part of the return. The normal
            # navigation tick rechecks stopping space and the next swept
            # section before each command, including the moving leader bound.
            preview = min(goal[0], state.x+max(2.8, stopping+.60))
            if self.verified_end(scan, curve, state.x, preview) == preview:
                return goal
        return None

    def tick(self, now, state, scan):
        if self.evidence.confirmed and not self.detour_committed:
            self.obstruction_front = self.evidence.points[-1][1][0]
        if (self.phase == "detour" and self.obstruction_front is not None
                and state.x > self.obstruction_front+.45 and abs(state.y) > 2.1
                and scan is not None and -.02 <= now-scan.stamp <= AGE_LIMIT):
            goal = self.rejoin_goal(state, scan)
            if goal is not None:
                self.phase, self.goal, self.curve = "rejoin", goal, self.curve_to(state, goal)
        return super().tick(now, state, scan)


class ShowcaseMission(ConvoyMission):
    def __init__(self, shapes, trial=SHOWCASE):
        super().__init__(shapes, trial)
        self.leader = ShowcaseLeader(shapes[0], trial.goals[0], trial.a_speed)
        self.follower = ShowcaseFollower(shapes[1], trial, self.static)
        self.leader_static = StaticEvidence()
        self.merge = MergeCoordinator(*shapes, conflict=trial.merge_zone, exit_x=MERGE_EXIT,
                                      exit_half_width=2.15, tilt_limit=TILT_LIMIT, exit_length=1.15)
        self.merge_curve = Curve(*trial.start(1), math.tan(trial.start_yaw[1]), MERGE_APPROACH_END, 0.)
        self.merged = False
        self.b_entered_merge = False
        self.observed_obstruction_front = None
        self.reserved_entry = None
        self.approach_open = False

    def waiting_point(self, leader, leader_rear):
        self.reserved_entry = (leader_rear if self.reserved_entry is None
                               else min(self.reserved_entry, leader_rear))
        self.approach_open |= (self.leader.phase in ("detour", "rejoin") and abs(leader.y) >= .60
                               and leader_rear >= self.reserved_entry+.70)
        if not self.approach_open or self.observed_obstruction_front is None:
            return None
        short_of_leader = self.reserved_entry-CLEARANCE-GEOMETRY_MARGIN-self.follower.padded_bounds[0][1]-.18
        before_turn_in = self.observed_obstruction_front-TURN_IN_ROOM
        return min(short_of_leader, before_turn_in)

    def approach_command(self, now, b, scan):
        if scan is None or not -.02 <= now-scan.stamp <= AGE_LIMIT:
            return STOP
        _, lane_y, lane_yaw, _ = self.merge_curve.at(b.x)
        if abs(b.y-lane_y) > .32 or abs(angle_error(lane_yaw, b.yaw)) > .24:
            return STOP
        view = LaneView(scan, self.trial.frame, MERGE_EXIT+7.0, half_width=7.5,
                        start_at=self.trial.starts[1]-4.)
        target_x, target_y, _, _ = self.merge_curve.at(b.x+1.65)
        dx, dy = target_x-b.x, target_y-b.y
        angle = angle_error(math.atan2(dy, dx), b.yaw)
        curvature = max(-ACKERMANN_CURVATURE, min(ACKERMANN_CURVATURE, 2*math.sin(angle)/math.hypot(dx, dy)))
        speed = min(self.trial.b_speed, .32+.28*max(0., MERGE_EXIT-b.x))
        request = Motion(speed, speed*curvature)
        remaining = self.follower.brakes.distance(b, request)+.35
        if not self.follower.stopping_space_clear(b, view):
            return STOP
        end = b.x+max(1.4, remaining)
        if self.follower.verified_end(view, self.merge_curve, b.x, end) != end:
            return STOP
        return request

    def step(self, now, a, b, a_scan, b_scan):
        if (now is None or a is None or b is None or not a.fresh(now, TILT_LIMIT)
                or not b.fresh(now, TILT_LIMIT) or abs(a.stamp-b.stamp) > PAIR_LIMIT):
            self.leader_static.reset()
            super().fail_closed("stale_or_excessive_tilt")
            return self.merge.fail_closed("stale_or_excessive_tilt")
        a_local, b_local = (self.trial.frame.state(state) for state in (a, b))
        b_poly = self.shapes[1].at(b_local)
        front = a_scan.current if a_scan is not None else None
        if front is not None and -.02 <= now-front.stamp <= AGE_LIMIT:
            contacts = []
            for index, distance in enumerate(front.hazard):
                if math.isfinite(distance):
                    bearing = front.yaw-math.pi/2+index*BIN_STEP
                    contacts.append((front.x+distance*math.cos(bearing), front.y+distance*math.sin(bearing)))
            self.leader.static_ready = self.leader_static.update(
                SimpleNamespace(stamp=front.stamp, contacts=contacts), a_local, self.trial.frame, b_poly)
            if self.leader.static_ready:
                first = self.leader_static.points[-1][1][0]
                self.observed_obstruction_front = (first if self.observed_obstruction_front is None
                                                   else min(self.observed_obstruction_front, first))
        else:
            self.leader.static_ready = False
            self.leader_static.reset()
        if self.merged:
            return super().step(now, a, b, a_scan, b_scan)
        decision = super().step(now, a, b, a_scan, b_scan, following=False)
        b_request = self.approach_command(now, b_local, b_scan)
        outcome = self.merge.step(now, a_local, b_local, decision.a, b_request)
        self.b_entered_merge |= "B" in outcome.occupied
        # Preserve the reservation until the whole trailing vehicle has left the junction.
        if (self.b_entered_merge and self.merge.outbound(b_local, b_poly)
                and aligned(b_local) and polygon_distance(b_poly, self.trial.merge_zone) > .20):
            self.merged = True
        return outcome
