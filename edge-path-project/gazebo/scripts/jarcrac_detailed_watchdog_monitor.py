#!/usr/bin/env python3
"""Read-only, latched telemetry watchdog for a stationary JARCRAC Gazebo trial.

Independent from the TTC controller; NEVER publishes /cmd_vel.
This is a monitoring/fault-detection experiment, not a safety-rated stop system.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

WORLD = 'jarcrac_detailed_terrain_halfstep'
STALE_WALL_S = 2.0             # diagnostic threshold; not a braking guarantee
CHANNELS = ('pose', 'imu', 'odom', 'lidar', 'clock')
CONTACT_TOPIC = (f'/world/{WORLD}/model/jarcrac/link/chassis/'
                 'sensor/detailed_chassis_contact/contact')
REPORT = Path('/tmp/jarcrac_detailed_watchdog_monitor.json')


class Watchdog:
    def __init__(self):
        self.lock = threading.RLock()
        self.seen = {}
        self.counts = {k: 0 for k in CHANNELS}
        self.armed = False
        self.fault = None
        self.sim_time = None
        self.clock_progress_at = None
        self.contact_messages = 0

    def trip(self, reason):
        with self.lock:
            if self.fault is None:
                self.fault = reason

    def observe(self, channel, *, at=None, sim_time=None):
        if at is None:
            at = time.monotonic()
        with self.lock:
            if channel == 'clock':
                if sim_time is None or not math.isfinite(sim_time) or sim_time < 0:
                    self.trip('invalid_clock')
                    return
                if self.sim_time is not None:
                    if sim_time < self.sim_time - 1e-4:
                        self.trip('clock_regression')
                    elif sim_time > self.sim_time + 1e-6:
                        self.clock_progress_at = at
                self.sim_time = sim_time
            self.seen[channel] = at
            self.counts[channel] += 1

    def state(self, *, at=None):
        if at is None:
            at = time.monotonic()
        with self.lock:
            if self.fault:
                return 'TRIPPED', self.fault
            missing = [k for k in CHANNELS if k not in self.seen]
            if missing:
                if self.armed:
                    self.trip('missing_' + missing[0])
                    return 'TRIPPED', self.fault
                return 'WAITING', 'missing_' + ','.join(missing)
            for k in CHANNELS:
                if at - self.seen[k] > STALE_WALL_S:
                    if self.armed:
                        self.trip('stale_' + k)
                        return 'TRIPPED', self.fault
                    return 'WAITING', 'stale_' + k
            if self.clock_progress_at is None:
                return 'WAITING', 'clock_not_advancing_yet'
            if at - self.clock_progress_at > STALE_WALL_S:
                if self.armed:
                    self.trip('clock_not_advancing')
                    return 'TRIPPED', self.fault
                return 'WAITING', 'clock_not_advancing'
            self.armed = True
            return 'HEALTHY_MONITOR_ONLY', 'all_required_streams_fresh'

    def summary(self):
        with self.lock:
            return dict(counts=dict(self.counts), contact_messages=self.contact_messages,
                        sim_time_s=self.sim_time, latched_fault=self.fault,
                        armed_once=self.armed)


def primed():
    w = Watchdog()
    for k in CHANNELS:
        if k == 'clock':
            w.observe(k, at=10.0, sim_time=1.0)
            w.observe(k, at=10.1, sim_time=1.1)
        else:
            w.observe(k, at=10.1)
    assert w.state(at=10.11)[0] == 'HEALTHY_MONITOR_ONLY'
    return w


def selftest():
    w = Watchdog()
    assert w.state(at=10.0)[0] == 'WAITING'
    print('PASS: missing inputs do not arm watchdog')
    for absent in CHANNELS:
        w = primed()
        for k in CHANNELS:
            if k != absent:
                if k == 'clock':
                    w.observe(k, at=12.5, sim_time=2.0)
                else:
                    w.observe(k, at=12.5)
        phase, reason = w.state(at=12.5)
        assert (phase, reason) == ('TRIPPED', 'stale_' + absent), (phase, reason)
        assert w.state(at=12.6) == ('TRIPPED', reason)
        print(f'PASS: injected stale {absent} -> latched {reason}')
    w = primed()
    w.trip('invalid_lidar_shape')
    assert w.state(at=10.2) == ('TRIPPED', 'invalid_lidar_shape')
    print('PASS: malformed LiDAR -> latched fault')
    w = primed()
    w.trip('chassis_contact')
    assert w.state(at=10.2) == ('TRIPPED', 'chassis_contact')
    print('PASS: chassis contact -> latched fault')
    w = primed()
    w.observe('clock', at=10.2, sim_time=0.3)
    assert w.state(at=10.3) == ('TRIPPED', 'clock_regression')
    print('PASS: clock regression -> latched fault')
    w = primed()
    for k in CHANNELS:
        if k == 'clock':
            w.observe(k, at=12.5, sim_time=1.1)
        else:
            w.observe(k, at=12.5)
    assert w.state(at=12.5) == ('TRIPPED', 'clock_not_advancing')
    print('PASS: stalled simulation time -> latched fault')
    print('WATCHDOG CORE SELFTEST: PASS (NO Gazebo, NO command publisher)')


def live(args):
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V

    w = Watchdog()
    node = Node()

    def on_pose(msg):
        try:
            for p in msg.pose:
                if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
                    q = p.orientation
                    values = (p.position.x, p.position.y, p.position.z,
                              q.x, q.y, q.z, q.w)
                    norm = sum(v*v for v in values[3:])
                    if not all(math.isfinite(v) for v in values) or not .8 < norm < 1.2:
                        w.trip('invalid_pose')
                    else:
                        w.observe('pose')
                    return
        except Exception as exc:
            w.trip('pose_callback_' + type(exc).__name__)

    def on_imu(msg):
        try:
            q = msg.orientation
            values = (q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values) or not .8 < sum(v*v for v in values) < 1.2:
                w.trip('invalid_imu')
            else:
                w.observe('imu')
        except Exception as exc:
            w.trip('imu_callback_' + type(exc).__name__)

    def on_odom(msg):
        try:
            v = msg.twist.linear
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            if not math.isfinite(speed):
                w.trip('invalid_odometry')
            elif speed > 0.06:
                w.trip('unexpected_motion_in_stationary_test')
            else:
                w.observe('odom')
        except Exception as exc:
            w.trip('odom_callback_' + type(exc).__name__)

    def on_lidar(msg):
        try:
            if (msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                    or msg.count != 721 or msg.vertical_count != 32
                    or len(msg.ranges) != 23072):
                w.trip('invalid_lidar_shape')
            elif not any(math.isfinite(r) and r > 0 for r in msg.ranges):
                w.trip('no_finite_lidar_ranges')
            else:
                w.observe('lidar')
        except Exception as exc:
            w.trip('lidar_callback_' + type(exc).__name__)

    def on_clock(msg):
        try:
            w.observe('clock', sim_time=msg.sim.sec + msg.sim.nsec * 1e-9)
        except Exception as exc:
            w.trip('clock_callback_' + type(exc).__name__)

    def on_contact(msg):
        try:
            with w.lock:
                w.contact_messages += 1
            if msg.contact:
                w.trip('chassis_contact')
        except Exception as exc:
            w.trip('contact_callback_' + type(exc).__name__)

    topics = (
        (Pose_V, f'/world/{WORLD}/pose/info', on_pose),
        (IMU, '/jarcrac/imu', on_imu),
        (Odometry, '/model/jarcrac/detailed_odometry', on_odom),
        (LaserScan, '/jarcrac/lidar', on_lidar),
        (Clock, f'/world/{WORLD}/clock', on_clock),
        (Contacts, CONTACT_TOPIC, on_contact),
    )
    for typ, topic, callback in topics:
        if not node.subscribe(typ, topic, callback):
            raise RuntimeError('Cannot subscribe: ' + topic)
        print('SUBSCRIBED:', topic, flush=True)

    started = time.monotonic()
    healthy_started = None
    passed = False
    reason = 'not_started'
    phase = 'WAITING'
    print('READ ONLY: no velocity publisher, no motion authorization.', flush=True)
    try:
        last_print = 0.0
        while True:
            now = time.monotonic()
            phase, reason = w.state(at=now)
            if phase == 'TRIPPED':
                print('WATCHDOG TRIPPED:', reason, flush=True)
                break
            if phase == 'HEALTHY_MONITOR_ONLY' and healthy_started is None:
                healthy_started = now
                print('ALL INPUTS FRESH: starting stationary observation', flush=True)
            if healthy_started is None and now - started > args.startup_timeout:
                reason = 'startup_timeout_' + reason
                break
            if healthy_started is not None and now - healthy_started >= args.seconds:
                passed = True
                reason = 'stationary_streams_remained_fresh'
                break
            if now - last_print >= 2.0:
                print(f'{phase}: {reason} | counts={w.summary()["counts"]}', flush=True)
                last_print = now
            time.sleep(.1)
    except KeyboardInterrupt:
        reason = 'interrupted'

    report = dict(test='jarcrac_independent_watchdog_monitor_only',
                  world=WORLD, passed=passed, final_phase=phase,
                  reason=reason, duration_requested_wall_s=args.seconds,
                  stale_threshold_wall_s=STALE_WALL_S,
                  publishes_velocity=False, motion_authorized=False,
                  contact_silence_does_not_prove_sensor_health=True,
                  source_timestamps_not_synchronized=True,
                  simulation_safety_rated=False, **w.summary())
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print('WATCHDOG MONITOR:', 'PASS' if passed else 'FAIL', flush=True)
    print('Report:', args.report, flush=True)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if not passed:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    sub.add_parser('selftest')
    p = sub.add_parser('live')
    p.add_argument('--seconds', type=float, default=15.0)
    p.add_argument('--startup-timeout', type=float, default=25.0)
    p.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args()
    if args.mode == 'selftest':
        selftest()
    else:
        if not 1 <= args.seconds <= 120 or not 5 <= args.startup_timeout <= 90:
            parser.error('Invalid monitoring duration or startup timeout')
        live(args)


if __name__ == '__main__':
    main()
