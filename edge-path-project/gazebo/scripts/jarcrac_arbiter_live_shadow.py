#!/usr/bin/env python3
"""CAMINO Phase 2a: JARCRAC live shadow arbiter, stationary only.

Subscribes to the actual Gazebo Harmonic vehicle telemetry, runs the already
unit-tested Watchdog and CommandArbiterPolicy on every local cycle, and asserts
that the resulting command is ZERO. NEVER advertises/publishes a Gazebo velocity
command or arms the policy. Optional dropped-LiDAR-callback fault injection.

This is not a vehicle stop system, not safety-rated, and not a driving trial.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from jarcrac_detailed_watchdog_monitor import Watchdog, WORLD, CONTACT_TOPIC, STALE_WALL_S
from jarcrac_command_arbiter_offline import CommandArbiterPolicy, ZERO

REPORT = Path('/tmp/jarcrac_arbiter_live_shadow.json')


def run(args):
    # Imports delayed to permit offline import and selftest without Gazebo bindings.
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V

    watchdog = Watchdog()
    arbiter = CommandArbiterPolicy(watchdog)
    node = Node()
    healthy_at = None
    dropped_lidar = 0

    def pose(msg):
        try:
            for p in msg.pose:
                if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
                    xyz = (p.position.x, p.position.y, p.position.z)
                    q = (p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w)
                    if (not all(math.isfinite(v) for v in xyz + q)
                            or not .8 < sum(v*v for v in q) < 1.2):
                        watchdog.trip('invalid_pose')
                    else:
                        watchdog.observe('pose')
                    return
        except Exception as exc:
            watchdog.trip('pose_callback_' + type(exc).__name__)

    def imu(msg):
        try:
            q = (msg.orientation.x, msg.orientation.y,
                 msg.orientation.z, msg.orientation.w)
            if (not all(math.isfinite(v) for v in q)
                    or not .8 < sum(v*v for v in q) < 1.2):
                watchdog.trip('invalid_imu')
            else:
                watchdog.observe('imu')
        except Exception as exc:
            watchdog.trip('imu_callback_' + type(exc).__name__)

    def odom(msg):
        try:
            v = msg.twist.linear
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            if not math.isfinite(speed):
                watchdog.trip('invalid_odometry')
            elif speed > .06:
                watchdog.trip('unexpected_motion_stationary_test')
            else:
                watchdog.observe('odom')
        except Exception as exc:
            watchdog.trip('odom_callback_' + type(exc).__name__)

    def lidar(msg):
        nonlocal dropped_lidar
        try:
            # Test fault is injected AFTER receipt/shape checking, by deliberately
            # withholding one input stream from the watchdog core.
            if (msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                    or msg.count != 721 or msg.vertical_count != 32
                    or len(msg.ranges) != 721*32):
                watchdog.trip('invalid_lidar_shape')
            elif not any(math.isfinite(r) and r > 0 for r in msg.ranges):
                watchdog.trip('no_finite_lidar_ranges')
            elif (args.inject_lidar_drop_after is not None
                  and healthy_at is not None
                  and time.monotonic() - healthy_at >= args.inject_lidar_drop_after):
                dropped_lidar += 1
            else:
                watchdog.observe('lidar')
        except Exception as exc:
            watchdog.trip('lidar_callback_' + type(exc).__name__)

    def clock(msg):
        try:
            watchdog.observe('clock', sim_time=msg.sim.sec + msg.sim.nsec*1e-9)
        except Exception as exc:
            watchdog.trip('clock_callback_' + type(exc).__name__)

    def contact(msg):
        try:
            with watchdog.lock:
                watchdog.contact_messages += 1
            if msg.contact:
                watchdog.trip('chassis_contact')
        except Exception as exc:
            watchdog.trip('contact_callback_' + type(exc).__name__)

    topics = (
        (Pose_V, f'/world/{WORLD}/pose/info', pose),
        (IMU, '/jarcrac/imu', imu),
        (Odometry, '/model/jarcrac/detailed_odometry', odom),
        (LaserScan, '/jarcrac/lidar', lidar),
        (Clock, f'/world/{WORLD}/clock', clock),
        (Contacts, CONTACT_TOPIC, contact),
    )
    for msg_type, topic, cb in topics:
        if not node.subscribe(msg_type, topic, cb):
            raise RuntimeError('Failed to subscribe: ' + topic)
        print('SUBSCRIBED:', topic, flush=True)

    print('SHADOW MODE: never arms, never advertises cmd_vel, never publishes.', flush=True)
    if args.inject_lidar_drop_after is not None:
        print('FAULT INJECTION: drop LiDAR observations after %.1fs healthy'
              % args.inject_lidar_drop_after, flush=True)

    start = time.monotonic()
    last_print = 0.
    phase, reason = 'WAITING', 'not_started'
    passed = False
    try:
        while True:
            now = time.monotonic()
            decision, decision_reason = arbiter.tick(at=now)
            if decision != ZERO or arbiter.enabled:
                raise AssertionError('Shadow mode must never authorize motion')
            phase, reason = watchdog.state(at=now)
            if healthy_at is None and phase == 'HEALTHY_MONITOR_ONLY':
                healthy_at = now
                print('HEALTHY: watchdog live; arbiter output ZERO', flush=True)
            if phase == 'TRIPPED':
                print('FAULT LATCHED:', reason, '| arbiter:', decision_reason, flush=True)
                passed = (args.inject_lidar_drop_after is not None
                          and arbiter.latched_fault == 'watchdog_stale_lidar'
                          and dropped_lidar > 0)
                break
            if healthy_at is None and now-start >= args.startup_timeout:
                reason = 'startup_timeout_' + reason
                break
            if healthy_at is not None and now-healthy_at >= args.seconds:
                passed = (args.inject_lidar_drop_after is None
                          and phase == 'HEALTHY_MONITOR_ONLY'
                          and arbiter.latched_fault is None)
                if args.inject_lidar_drop_after is not None:
                    reason = 'injected_fault_not_detected_in_time'
                break
            if now-last_print >= 2.0:
                print('%s: %s | output=ZERO | counts=%s' %
                      (phase, decision_reason, watchdog.summary()['counts']), flush=True)
                last_print = now
            time.sleep(.05)
    except KeyboardInterrupt:
        reason = 'interrupted'
    except Exception as exc:
        reason = type(exc).__name__ + ': ' + str(exc)
        watchdog.trip('shadow_exception')

    summary = watchdog.summary()
    report = dict(test='camino_shadow_arbiter_stationary', world=WORLD,
                  passed=bool(passed), phase=phase, reason=reason,
                  arbiter_latched_fault=arbiter.latched_fault,
                  watchdog_latched_fault=watchdog.fault,
                  injector_after_healthy_wall_s=args.inject_lidar_drop_after,
                  dropped_lidar_observations=dropped_lidar,
                  policy_enabled=arbiter.enabled, publishes_velocity=False,
                  motion_authorized=False, output_always_zero=True,
                  duration_requested_wall_s=args.seconds,
                  freshness_threshold_wall_s=STALE_WALL_S,
                  source_timestamps_not_synchronized=True,
                  contact_silence_proves_sensor_health=False,
                  simulation_safety_rated=False, **summary)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print('SHADOW ARBITER:', 'PASS' if passed else 'FAIL', flush=True)
    print('Report:', args.report, flush=True)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if not passed:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--startup-timeout', type=float, default=25.)
    parser.add_argument('--inject-lidar-drop-after', type=float, default=None)
    parser.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args()
    if not (3 <= args.seconds <= 120 and 5 <= args.startup_timeout <= 90):
        parser.error('Invalid duration / startup timeout')
    if args.inject_lidar_drop_after is not None and not (
            1 <= args.inject_lidar_drop_after < args.seconds - STALE_WALL_S - .5):
        parser.error('Injection must leave enough time for stale-data detection')
    run(args)


if __name__ == '__main__':
    main()
