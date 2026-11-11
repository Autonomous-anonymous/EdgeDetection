#!/usr/bin/env python3
"""Bounded JARCRAC forward-and-stop test, simulation only."""
import argparse
import json
import math
import threading
import time
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

SPEED = 0.10
TARGET = 0.12
MAX_FORWARD = 0.20
MAX_REVERSE = 0.015
MAX_LATERAL = 0.06
MAX_YAW = 5.0
MAX_BRAKING = 0.06
MAX_DRIVE_TIME = 4.0
STOP_OBSERVATION = 1.2


def yaw_deg(q):
    return math.degrees(math.atan2(
        2 * (q.w*q.z + q.x*q.y),
        1 - 2*(q.y*q.y + q.z*q.z)
    ))


def wrap(angle):
    return (angle + 180) % 360 - 180


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--case', choices=('flat', 'terrain'),
        required=True
    )
    args = parser.parse_args()

    world = 'jarcrac_detailed_' + args.case
    node = Node()
    lock = threading.RLock()

    state = dict(
        pose=None, odom=None, sim=None, scan=None,
        pose_at=0., odom_at=0.,
        clock_at=0., scan_at=0.,
        odom_seq=0, scan_count=0,
        contact=False, contact_messages=0
    )

    def on_pose(msg):
        for p in msg.pose:
            if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
                with lock:
                    state['pose'] = (
                        float(p.position.x),
                        float(p.position.y),
                        yaw_deg(p.orientation)
                    )
                    state['pose_at'] = time.monotonic()
                break

    def on_odom(msg):
        v = msg.twist.linear
        with lock:
            state['odom'] = (
                math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z),
                float(msg.twist.angular.z)
            )
            state['odom_at'] = time.monotonic()
            state['odom_seq'] += 1

    def on_clock(msg):
        with lock:
            state['sim'] = msg.sim.sec + msg.sim.nsec*1e-9
            state['clock_at'] = time.monotonic()

    def on_scan(msg):
        with lock:
            state['scan'] = (
                int(msg.count),
                int(msg.vertical_count),
                len(msg.ranges)
            )
            state['scan_at'] = time.monotonic()
            state['scan_count'] += 1

    def on_contact(msg):
        with lock:
            state['contact_messages'] += 1
            state['contact'] |= bool(len(msg.contact))

    subscriptions = (
        (Pose_V, f'/world/{world}/pose/info', on_pose),
        (Odometry, '/model/jarcrac/detailed_odometry', on_odom),
        (Clock, f'/world/{world}/clock', on_clock),
        (LaserScan, '/jarcrac/lidar', on_scan),
        (
            Contacts,
            '/jarcrac/detailed/chassis_contacts',
            on_contact
        ),
    )

    for message_type, topic, callback in subscriptions:
        if not node.subscribe(message_type, topic, callback):
            raise RuntimeError('Subscription failed: ' + topic)

    pub = node.advertise(
        '/model/jarcrac/detailed_cmd_vel', Twist
    )

    def command(speed):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.
        pub.publish(msg)

    def snapshot():
        with lock:
            return dict(state)

    def fresh(s):
        now = time.monotonic()
        return (
            all(
                s[k] is not None
                for k in ('pose', 'odom', 'sim', 'scan')
            )
            and all(
                0 <= now-s[k] <= 0.9
                for k in (
                    'pose_at', 'odom_at',
                    'clock_at', 'scan_at'
                )
            )
            and s['scan'] == (721, 32, 23072)
        )

    report = dict(
        test='jarcrac_detailed_forward_stop',
        world=world,
        requested_speed_m_s=SPEED,
        target_forward_m=TARGET,
        passed=False,
        reason='not_started',
        independent_watchdog=False
    )

    origin = None
    stop_start = None
    braking_distance = None
    settling_motion = None
    stopped_samples = set()
    drive_samples = []

    maximum_forward = 0.
    maximum_lateral = 0.
    maximum_yaw = 0.

    def displacement(s):
        x0, y0, heading_deg, _ = origin
        dx = s['pose'][0] - x0
        dy = s['pose'][1] - y0
        heading = math.radians(heading_deg)

        forward = (
            dx*math.cos(heading) +
            dy*math.sin(heading)
        )
        lateral = (
            -dx*math.sin(heading) +
            dy*math.cos(heading)
        )
        yaw = wrap(s['pose'][2] - heading_deg)

        return forward, lateral, yaw

    def inspect(s):
        nonlocal maximum_forward
        nonlocal maximum_lateral
        nonlocal maximum_yaw

        if not fresh(s):
            raise RuntimeError('Stale or invalid telemetry')

        if s['contact']:
            raise RuntimeError('Chassis contact reported')

        forward, lateral, yaw = displacement(s)

        values = (forward, lateral, yaw, *s['odom'])
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Nonfinite telemetry')

        maximum_forward = max(maximum_forward, forward)
        maximum_lateral = max(
            maximum_lateral, abs(lateral)
        )
        maximum_yaw = max(maximum_yaw, abs(yaw))

        if forward < -MAX_REVERSE:
            raise RuntimeError(
                'Reverse motion: track direction may be inverted'
            )

        if (
            forward > MAX_FORWARD
            or abs(lateral) > MAX_LATERAL
            or abs(yaw) > MAX_YAW
        ):
            raise RuntimeError(
                'Forward/lateral/yaw guard exceeded'
            )

        return forward, lateral, yaw

    try:
        deadline = time.monotonic() + 20

        while time.monotonic() < deadline:
            command(0.)
            s = snapshot()
            if fresh(s):
                break
            time.sleep(.08)
        else:
            raise RuntimeError('Telemetry preflight timeout')

        expected = (
            (0., 0.)
            if args.case == 'flat'
            else (20., 40.)
        )

        if (
            math.dist(s['pose'][:2], expected) > .6
            or s['odom'][0] > .04
            or abs(s['odom'][1]) > .035
            or s['contact']
        ):
            raise RuntimeError('Unexpected initial vehicle state')

        origin = (*s['pose'], s['sim'])

        print(
            f'PREFLIGHT PASS: {world}; LiDAR 721x32',
            flush=True
        )
        print(
            'Command +0.10 m/s; target +0.12 m',
            flush=True
        )

        deadline = time.monotonic() + 20
        next_print = -1.

        while time.monotonic() < deadline:
            s = snapshot()
            forward, lateral, yaw = inspect(s)
            elapsed = s['sim'] - origin[3]

            if elapsed < 0 or elapsed > MAX_DRIVE_TIME:
                raise RuntimeError('Drive time limit')

            if elapsed >= next_print:
                print(
                    f'sim={elapsed:.2f}s '
                    f'forward={forward:.4f}m '
                    f'lateral={lateral:.4f}m '
                    f'yaw={yaw:+.2f}deg '
                    f'odom_v={s["odom"][0]:.3f}m/s',
                    flush=True
                )
                next_print = elapsed + .25

            drive_samples.append((s['sim'], forward))

            if forward >= TARGET:
                stop_start = (s['sim'], forward)
                break

            command(SPEED)
            time.sleep(.035)
        else:
            raise RuntimeError('Drive wall-time limit')

        command(0.)
        print('Target reached: zero speed commanded', flush=True)

        deadline = time.monotonic() + 20
        late_positions = []

        while time.monotonic() < deadline:
            command(0.)
            s = snapshot()
            forward, lateral, yaw = inspect(s)
            elapsed = s['sim'] - stop_start[0]

            if elapsed < 0:
                raise RuntimeError('Clock moved backwards')

            braking_distance = max(
                0., forward - stop_start[1]
            )

            if braking_distance > MAX_BRAKING:
                raise RuntimeError(
                    'Stopping distance exceeded 0.06 m'
                )

            if (
                s['odom'][0] < .025
                and abs(s['odom'][1]) < .04
            ):
                stopped_samples.add(s['odom_seq'])

            if elapsed >= .60:
                late_positions.append(forward)

            if elapsed >= STOP_OBSERVATION:
                break

            time.sleep(.04)
        else:
            raise RuntimeError('Stop observation timeout')

        settling_motion = (
            max(late_positions) - min(late_positions)
            if len(late_positions) >= 2 else None
        )

        if len(stopped_samples) < 3:
            raise RuntimeError(
                'Insufficient distinct zero-speed samples'
            )

        if (
            settling_motion is None
            or settling_motion > .01
        ):
            raise RuntimeError(
                'Physical motion continued after stop'
            )

        report['reason'] = 'forward_and_stop_confirmed'

    except KeyboardInterrupt:
        report['reason'] = 'interrupted'
    except Exception as exc:
        report['reason'] = (
            f'{type(exc).__name__}: {exc}'
        )
    finally:
        for _ in range(25):
            command(0.)
            time.sleep(.06)

        s = snapshot()
        final = (
            displacement(s)
            if origin and s['pose']
            else None
        )

        avg_speed = None
        if (
            len(drive_samples) >= 2
            and drive_samples[-1][0] >
                drive_samples[0][0] + .1
        ):
            avg_speed = (
                (drive_samples[-1][1] - drive_samples[0][1])
                /
                (drive_samples[-1][0] - drive_samples[0][0])
            )

        passed = (
            report['reason'] == 'forward_and_stop_confirmed'
            and final is not None
            and TARGET <= final[0] <= MAX_FORWARD
            and maximum_lateral <= MAX_LATERAL
            and maximum_yaw <= MAX_YAW
            and len(stopped_samples) >= 3
            and not s['contact']
            and fresh(s)
        )

        report.update(
            passed=bool(passed),
            final_displacement_forward_lateral_yaw=final,
            measured_average_forward_speed_m_s=avg_speed,
            max_forward_m=maximum_forward,
            max_lateral_m=maximum_lateral,
            max_yaw_deg=maximum_yaw,
            post_zero_forward_distance_m=braking_distance,
            late_settling_motion_m=settling_motion,
            stop_odometry_samples=len(stopped_samples),
            lidar_scans_seen=s['scan_count'],
            chassis_contact_observed=s['contact'],
            contact_messages=s['contact_messages']
        )

        output = Path(
            f'/tmp/jarcrac_detailed_{args.case}_forward_stop.json'
        )
        output.write_text(
            json.dumps(report, indent=2) + '\n'
        )

        print(
            'FORWARD/STOP:',
            'PASS' if passed else 'FAIL',
            flush=True
        )
        print(json.dumps(report, indent=2), flush=True)
        print('Report:', output, flush=True)

        if not passed:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
