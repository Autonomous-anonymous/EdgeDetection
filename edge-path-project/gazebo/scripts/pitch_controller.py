"""Live obstacle encounters and a measured two-vehicle merge with optional CAN authority."""

import json
import math
from pathlib import Path
import select
import sys
import time

# Register the v6 message types before hil10_runtime loads native transport.
# Ubuntu's C++ protobuf backend otherwise loses the nested Pose class in Pose_V.
from ruskotunturi_final_sil_demo_v6 import (
    FinalSilDemoLargeArea, MODEL_A, MODEL_B, A_ROUTE, A_SHARED_INDEX, B_SHARED_INDEX,
    MERGE_POINT, SHARED_HEADWAY_SLOW_M, SHARED_HEADWAY_STOP_M,
)
from gz.msgs10.odometry_pb2 import Odometry
from final_sil_v6_checks import footprint, HALF_LENGTH, HALF_WIDTH, HEIGHT, margins_ok, starting_scene_issue
from hil10_runtime import ManagedLidarSender, odometry_values
from hil10_safety import HILLink, OdometryMonitor, REASONS

LIVE_BUDGET_S = 55.0
EXIT_YAW = math.atan2(A_ROUTE[A_SHARED_INDEX][1] - MERGE_POINT[1], A_ROUTE[A_SHARED_INDEX][0] - MERGE_POINT[0])
EXIT_X, EXIT_Y = math.cos(EXIT_YAW), math.sin(EXIT_YAW)


def exit_position(vehicle):
    dx, dy = vehicle["x"] - MERGE_POINT[0], vehicle["y"] - MERGE_POINT[1]
    return dx*EXIT_X + dy*EXIT_Y, -dx*EXIT_Y + dy*EXIT_X


def convoy_positions(a, b):
    """Positions along the marked exit, only while both cars occupy it in order."""
    a_s, a_side = exit_position(a)
    b_s, b_side = exit_position(b)
    if (b_s < 2.3 or a_s > 17 or a_s - b_s < SHARED_HEADWAY_STOP_M
            or max(abs(a_side), abs(b_side)) > 1.5
            or any(math.cos(v["yaw"] - EXIT_YAW) < .84 for v in (a, b))):
        return None
    return a_s, b_s


def past_barrier(vehicle, barrier, direction):
    """Even A's rear corner must have passed the far edge along the route."""
    ux, uy = direction
    car = footprint(vehicle, HALF_LENGTH, HALF_WIDTH, 0, HEIGHT)
    box = footprint(barrier, .80, .80, -.80, .80)
    return min(x*ux + y*uy for x, y in car) > max(x*ux + y*uy for x, y in box)


def odometry_callback(monitor):
    def sample(msg):
        monitor.update(*odometry_values(msg))
    return sample


class PitchLink(HILLink):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.replies = 0

    def accept_brake(self, packet):
        if not super().accept_brake(packet):
            return False
        self.replies += 1
        if self.armed and self.brake_active:
            # Keep a veto even if a later DRIVE arrives in the same UDP drain.
            self.fault = self.fault or REASONS[self.brake_reason]
        return True


class PitchDemo(FinalSilDemoLargeArea):
    mission_timeout = 70.0
    harness_args = ("--pitch",)

    def __init__(self, host, results, panel, launch_wall, interactive):
        super().__init__()
        self.mode = "hil" if host else "sil"
        self.trace_directory = Path(results)
        self.trace_prefix = f"pitch_{self.mode}_"
        self.panel = panel
        self.host = host
        self.link = None
        self.lidar = None
        self.a_odom = OdometryMonitor()
        self.b_odom = OdometryMonitor()
        self.passed_barrier = False
        self.b_waited = False
        self.b_resumed = False
        self.shared_entry = None
        self.shared_travel = None
        self.events = {}
        self.wall_events = {}
        self.launch_wall = launch_wall
        self.ready_wall = self.motion_wall = None
        self.interactive = interactive
        self.can_at_events = {}
        self.last_panel = -math.inf
        self.sensor_callbacks = []

    def print_plan(self):
        print(f"CAMINO | kahden auton liveajo | {self.mode.upper()}", flush=True)

    def event(self, name, elapsed):
        if name not in self.events:
            self.events[name] = round(elapsed, 3)
            self.wall_events[name] = round(time.monotonic() - self.motion_wall, 3)
            if self.link:
                self.can_at_events[name] = dict(counter=self.link.brake_counter, accepted_replies=self.link.replies,
                                                brake_active=self.link.brake_active, reason=REASONS[self.link.brake_reason])
            print(f"[live {self.wall_events[name]:5.1f} s / sim {elapsed:5.1f} s] {name}", flush=True)

    def run(self):
        try:
            if self.host:
                self.link = PitchLink(self.host)
                self.b_odom = self.link.odom
                self.lidar = ManagedLidarSender(self.host)
            for name, monitor in ((MODEL_A, self.a_odom), (MODEL_B, self.b_odom)):
                sample = odometry_callback(monitor)
                self.sensor_callbacks.append(sample)
                if not self.node.subscribe(Odometry, f"/model/{name}/odometry", sample):
                    raise RuntimeError(f"Missing odometry subscription for {name}")
            return super().run()
        finally:
            if self.lidar is not None:
                self.lidar.close()
            if self.link is not None:
                self.link.close()

    def can_state(self):
        if self.link is None:
            return "SIL · simulaatio"
        brake, reason = self.link.decision()
        if brake:
            return "CAN · SEIS · " + reason
        return f"CAN · ajolupa · laskuri {self.link.brake_counter}"

    def prepare_fresh_world(self):
        # A cold OGRE shader cache can take longer than the settled-world
        # check. Wait at zero for the first real image before starting it.
        deadline = time.monotonic() + 45
        while not self.camera_frames and time.monotonic() < deadline:
            self.stop_both()
            if self.watchdog.poll() is not None:
                self.end_reason = "watchdog_unavailable"
                return False
            time.sleep(.05)
        if not super().prepare_fresh_world():
            return False
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            self.stop_both()
            if self.link:
                self.link.pump()
                if self.lidar.error or self.link.fault:
                    self.end_reason = self.link.fault or "lidar_send_error"
                    return False
            if self.a_odom.fresh() and self.b_odom.fresh():
                if not self.link or (self.lidar.sequence and self.link.replies >= 2 and not self.link.decision()[0]):
                    return self.start_when_ready()
            time.sleep(.05)
        gate = self.link.decision()[1] if self.link else "none"
        self.end_reason = "startup_" + (gate if gate != "none" else "sensors_not_ready")
        return False

    def start_when_ready(self):
        self.ready_wall = time.monotonic()
        self.panel.update("Lähtöpaikalla", "Lähtöpaikalla", self.can_state(), "Valmis lähtöön", live=False)
        if self.interactive:
            print("Gazebo ja anturit valmiina. Säädä kuvakulma ja aloita ajo painamalla Enter.", flush=True)
        while True:
            self.stop_both()
            if self.link:
                self.link.pump()
                brake, gate = self.link.decision()
                if brake or self.lidar.error:
                    self.end_reason = "start_" + (gate if brake else "lidar_send_error")
                    return False
            if not self.interactive or select.select([sys.stdin], [], [], .05)[0]:
                if self.interactive and not sys.stdin.readline():
                    self.end_reason = "start_cancelled"
                    return False
                # Recheck after the presenter's wait. No positive command has been sent.
                now = time.monotonic()
                with self.lock:
                    ready = (self.watchdog.poll() is None and now - self.clock_wall < .5
                             and starting_scene_issue(self.observed) is None
                             and all(v["scan_valid"] and now-v["last_pose_time"] < .5 and now-v["last_lidar_time"] < .5
                                     for v in (self.a, self.b)))
                if not ready or not self.a_odom.fresh() or not self.b_odom.fresh():
                    self.end_reason = "start_scene_not_ready"
                    return False
                self.motion_wall = now
                return True

    def pitch_speed(self, vehicle, command, scale, limit=math.inf):
        speed, yaw_rate = command
        if speed <= 0:
            return command
        faster = self.terrain_limited_speed(vehicle, min(speed * scale, limit))
        # Ackermann takes velocity and yaw rate. Preserve the requested turn radius.
        return faster, yaw_rate * faster / speed

    def a_control(self, now):
        cmd = super().a_control(now)
        if not self.static_avoidance_started:
            return cmd
        scale = 3.4
        if self.a_detour_phase == "OUT":
            # Build up speed only after the wheels have turned away from the
            # barrier. Entering this turn at cruise can cut the near corner.
            _, offset = self.static_line_project(self.a["x"], self.a["y"])
            scale = min(scale, 1.45 + abs(offset))
        limit = 3.0
        if self.a["target_index"] >= A_SHARED_INDEX:
            # Lead through the junction and let B close the gap on the marked exit.
            a_s, _ = exit_position(self.a)
            b_s, _ = exit_position(self.b)
            limit = min(limit, max(0.0, 1.5 * (8.8 + max(b_s, 0) - a_s)))
        return self.pitch_speed(self.a, cmd, scale, limit)

    def b_control(self, now):
        cmd = super().b_control(now)
        spacing = math.hypot(self.a["x"]-self.b["x"], self.a["y"]-self.b["y"])
        if (not self.b_dynamic_resume_events or self.coordinator_profile_active
                or (self.b_released and spacing <= SHARED_HEADWAY_SLOW_M)):
            return cmd
        return self.pitch_speed(self.b, cmd, 2.9, 2.35)

    def observe_merge(self, elapsed):
        if not (self.b_resumed and self.passed_barrier and self.b_released
                and self.b["target_index"] >= B_SHARED_INDEX):
            return
        positions = convoy_positions(self.a, self.b)
        if positions is None:
            self.shared_entry = None
            return
        if self.shared_entry is None:
            self.shared_entry = positions
            self.event("b_entered_shared_lane", elapsed)
        if min(p-q for p, q in zip(positions, self.shared_entry)) >= 1.2:
            self.shared_travel = dict(a_metres=round(positions[0], 2), b_metres=round(positions[1], 2),
                                      headway_m=round(positions[0]-positions[1], 2))
            self.event("both_drove_shared_lane", elapsed)

    def observe(self, elapsed):
        with self.lock:
            a = self.a
            if self.camera_used_for_control and self.static_avoidance_started:
                self.event("a_camera_detour", elapsed)
                barrier = next((p for n, p in self.observed.items() if self.pose_matches(n, "final_static_obstacle")), None)
                offset = self.static_line_project(a["x"], a["y"])[1]
                pose = (a["x"], a["y"], a["yaw"], math.radians(a["roll_deg"]), math.radians(a["pitch_deg"]))
                if barrier and abs(offset) >= 2.5 and past_barrier(pose, barrier, (self.static_line_ux, self.static_line_uy)):
                    self.passed_barrier = True
                    self.event("a_passed_static_obstacle", elapsed)
            if self.b_dynamic_hold and self.moving_started and self.encounter_stop_verified:
                self.b_waited = True
                self.event("b_stopped_for_crossing", elapsed)
            if self.b_waited and self.moving_completed and self.encounter_resume_verified and self.b_dynamic_resume_events and not self.b_dynamic_hold:
                self.b_resumed = True
                self.event("b_resumed_after_crossing", elapsed)
            self.observe_merge(elapsed)
            if time.monotonic() - self.last_panel >= .20:
                a_text = "Ohitti kiinteän esteen" if self.passed_barrier else "Väistää estettä" if self.static_avoidance_started else "Lähestyy estettä"
                b_text = "Odottaa" if self.b_dynamic_hold and self.b_waited else "Jarruttaa" if self.b_dynamic_hold else "Jatkaa matkaa" if self.b_resumed else "Ajossa"
                if self.b_released:
                    a_text = "Yhteisellä tiellä"
                    b_text = "Seuraa A:ta" if self.shared_entry else "Liittyy yhteiselle tielle"
                elif self.b_waiting:
                    b_text = "Antaa A:n liittyä ensin"
                self.panel.update(a_text, b_text, self.can_state(), f"Live  ·  {time.monotonic()-self.motion_wall:.0f} s", live=True)
                self.last_panel = time.monotonic()

    def send_commands(self, elapsed, a_command, b_command):
        reason = None
        if self.link:
            self.link.pump()
            speed, turn, gate = self.link.apply(*b_command)
            b_command = (speed, turn)
            if self.lidar.error or gate != "none":
                reason = "lidar_send_error" if self.lidar.error else gate
        if not self.a_odom.fresh() or not self.b_odom.fresh():
            reason = reason or "odometry_timeout"
        if time.monotonic() - self.motion_wall >= LIVE_BUDGET_S:
            reason = reason or "wall_timeout"
        if reason:
            self.end_reason = reason
            self.stop_both()
            return False
        self.observe(elapsed)
        if self.shared_travel:
            self.end_reason = "pitch_complete"
            self.stop_both()
            return False
        return super().send_commands(elapsed, a_command, b_command)

    def print_result(self, duration):
        # Verify both actual vehicles after sending zero. Keep exchanging HIL
        # data while stopping so a last-second veto cannot become a PASS.
        self.stop_both()
        for monitor in (self.a_odom, self.b_odom):
            monitor.begin_stop()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.stop_both()
            if self.link:
                self.link.pump()
            if self.a_odom.stopped() and self.b_odom.stopped():
                break
            time.sleep(.05)
        with self.lock:
            clearance_ok = margins_ok(self.a["minimum_clearance_seen"], self.b["minimum_clearance_seen"], self.ground_min)
            now = time.monotonic()
            scenario_ok = (self.end_reason == "pitch_complete" and self.safety_ok() and not self.moving_pose_failures
                           and self.watchdog.poll() is None and self.harness.poll() is None
                           and now - self.clock_wall < .5 and now - self.harness_wall < 1
                           and all(now - v[key] < .5 for v in (self.a, self.b) for key in ("last_pose_time", "last_lidar_time")))
        can_ok = not self.link or (self.link.armed and self.link.replies >= 2 and not self.link.decision()[0] and not self.lidar.error)
        live_wall = None if self.motion_wall is None else round(time.monotonic() - self.motion_wall, 3)
        checks = dict(scene_finished=self.end_reason == "pitch_complete", a_passed=self.passed_barrier,
                      b_stopped=self.b_waited, b_resumed=self.b_resumed, shared_lane=bool(self.shared_travel),
                      within_live_budget=live_wall is not None and live_wall <= LIVE_BUDGET_S, a_stopped_at_end=self.a_odom.stopped(),
                      b_stopped_at_end=self.b_odom.stopped(), scene_healthy=scenario_ok, clearance=clearance_ok)
        if self.link:
            checks["r2100_drive_fresh_at_stop"] = can_ok
        passed = all(checks.values())
        if not passed and self.end_reason == "pitch_complete":
            self.end_reason = next(k for k, ok in checks.items() if not ok)
        report = dict(mode=self.mode, passed=passed, reason=self.end_reason, simulation_seconds=round(duration, 3),
                      events_s=self.events, events_wall_s=self.wall_events, live_wall_s=live_wall, shared_lane=self.shared_travel,
                      preparation_wall_s=None if self.ready_wall is None else round(self.ready_wall-self.launch_wall, 3),
                      presenter_wait_s=None if self.motion_wall is None else round(self.motion_wall-self.ready_wall, 3),
                      checks=checks, trace=self.trace_path,
                      minimum_clearance_m=self.ground_min if math.isfinite(self.ground_min) else None)
        if self.link:
            report["r2100"] = dict(host=self.host, speed_packets=self.link.sent_count, brake_replies=self.link.replies,
                                   rejected_replies=self.link.rejected_replies, last_counter=self.link.brake_counter,
                                   last_reason=REASONS[self.link.brake_reason], fault=self.link.fault, raw_scans=self.lidar.sequence,
                                   at_scene_events=self.can_at_events)
        if self.trace_path:
            output = (self.trace_directory / self.trace_path).with_suffix(".json")
            output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(f"Evidence: {output}", flush=True)
        status = "Ajo valmis" if passed else "Ajo keskeytyi"
        self.panel.update("Johtaa yhteisellä tiellä" if self.shared_travel else "Ohitti esteen" if self.passed_barrier else "–",
                          "Seuraa A:ta" if self.shared_travel else "Pysähtyi ja jatkoi" if self.b_resumed else "–",
                          self.can_state(), status if passed else f"{status} · {self.end_reason}", live=False)
        print(f"{status} | {self.end_reason} | live {live_wall} s | {'PASS' if passed else 'FAIL'}", flush=True)
        return passed
