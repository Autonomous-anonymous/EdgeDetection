"""Short, measured opening scene of the v6.4 mission with optional CAN authority."""

import json
import math
from pathlib import Path
import time

# Register the v6 message types before hil10_runtime loads native transport.
# Ubuntu's C++ protobuf backend otherwise loses the nested Pose class in Pose_V.
from ruskotunturi_final_sil_demo_v6 import FinalSilDemoLargeArea, MODEL_A, MODEL_B
from gz.msgs10.odometry_pb2 import Odometry
from final_sil_v6_checks import footprint, HALF_LENGTH, HALF_WIDTH, HEIGHT, margins_ok
from hil10_runtime import ManagedLidarSender, odometry_values
from hil10_safety import HILLink, OdometryMonitor, REASONS


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

    def __init__(self, host, results, panel):
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
        self.events = {}
        self.can_at_events = {}
        self.last_panel = -math.inf
        self.sensor_callbacks = []

    def print_plan(self):
        print(f"CAMINO | kahden auton liveajo | {self.mode.upper()}", flush=True)

    def event(self, name, elapsed):
        if name not in self.events:
            self.events[name] = round(elapsed, 3)
            if self.link:
                self.can_at_events[name] = dict(counter=self.link.brake_counter, accepted_replies=self.link.replies,
                                                brake_active=self.link.brake_active, reason=REASONS[self.link.brake_reason])
            print(f"[{elapsed:5.1f} s] {name}", flush=True)

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
                    self.motion_wall = time.monotonic()
                    return True
            time.sleep(.05)
        gate = self.link.decision()[1] if self.link else "none"
        self.end_reason = "startup_" + (gate if gate != "none" else "sensors_not_ready")
        return False

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
            if time.monotonic() - self.last_panel >= .20:
                a_text = "Ohitti kiinteän esteen" if self.passed_barrier else "Väistää estettä" if self.static_avoidance_started else "Lähestyy estettä"
                b_text = "Odottaa" if self.b_dynamic_hold and self.b_waited else "Jarruttaa" if self.b_dynamic_hold else "Jatkaa matkaa" if self.b_resumed else "Ajossa"
                self.panel.update(a_text, b_text, self.can_state(), f"Live  ·  {elapsed:.0f} s", live=True)
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
        if time.monotonic() - self.motion_wall >= 100:
            reason = reason or "wall_timeout"
        if reason:
            self.end_reason = reason
            self.stop_both()
            return False
        self.observe(elapsed)
        if self.passed_barrier and self.b_resumed:
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
        checks = dict(scene_finished=self.end_reason == "pitch_complete", a_passed=self.passed_barrier,
                      b_stopped=self.b_waited, b_resumed=self.b_resumed, a_stopped_at_end=self.a_odom.stopped(),
                      b_stopped_at_end=self.b_odom.stopped(), scene_healthy=scenario_ok, clearance=clearance_ok)
        if self.link:
            checks["r2100_drive_fresh_at_stop"] = can_ok
        passed = all(checks.values())
        if not passed and self.end_reason == "pitch_complete":
            self.end_reason = next(k for k, ok in checks.items() if not ok)
        report = dict(mode=self.mode, passed=passed, reason=self.end_reason, simulation_seconds=round(duration, 3),
                      events_s=self.events, checks=checks, trace=self.trace_path,
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
        self.panel.update("Ohitti esteen" if self.passed_barrier else "–", "Pysähtyi ja jatkoi" if self.b_resumed else "–",
                          self.can_state(), status if passed else f"{status} · {self.end_reason}", live=False)
        print(f"{status} | {self.end_reason} | {'PASS' if passed else 'FAIL'}", flush=True)
        return passed
