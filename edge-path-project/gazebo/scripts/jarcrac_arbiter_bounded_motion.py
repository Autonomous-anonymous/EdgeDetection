#!/usr/bin/env python3
"""CAMINO JARCRAC: first bounded, watchdog-gated Gazebo motion experiment.

prepare: copy the known-obstacle half-step terrain world into a new isolated
         world and switch only its native tracked-vehicle velocity topic.
selftest: offline policy and isolation checks; publishes nothing.
drive:    WITH --arm only, own one transport publisher and request straight motion
         at <=0.10m/s, for <=0.12m nominal forward progress. "inject-lidar"
         withholds observations inside this process and demonstrates a latched
         fast LiDAR freshness failure followed by zero commands.

Not an autonomous TTC controller, an independent watchdog process, a safety-
rated E-stop, obstacle avoidance, or certified clearance. It uses Gazebo truth
pose, a synthetic *internal* trial motion request, and reception-time freshness,
not synchronized timestamps. Do not operate a second world or command publisher.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_command_arbiter_offline import (
    CommandArbiterPolicy, EXPECTED_SOURCE, ZERO, MAX_FORWARD_MPS,
)
from jarcrac_detailed_watchdog_monitor import Watchdog

HOME = Path.home() / 'jarcrac_imu_experiment'
SOURCE_NAME = 'jarcrac_detailed_terrain_halfstep'
WORLD = 'jarcrac_detailed_arbiter_bounded_motion'
SOURCE = HOME / (SOURCE_NAME + '.sdf')
OUTPUT = HOME / (WORLD + '.sdf')
CMD = '/model/jarcrac/arbiter_bounded_motion_cmd_vel'
ODOM = '/model/jarcrac/detailed_odometry'
CONTACT = (f'/world/{WORLD}/model/jarcrac/link/chassis/'
           'sensor/detailed_chassis_contact/contact')
REPORT = Path('/tmp/jarcrac_arbiter_bounded_motion.json')

# Conservative SIMULATION-ONLY limits. Stale sensor cutoff is stricter than
# the general monitor's initial 2s, but is not a validated stopping deadline.
SPEED_MPS = 0.10
SOFT_PROGRESS_M = 0.12
HARD_PROGRESS_M = 0.23
MAX_LATERAL_M = 0.10
FAST_STALE_WALL_S = 0.55
MAX_MOTION_WALL_S = 8.0
STATIONARY_ODOM_MPS = 0.035
STOP_TIMEOUT_WALL_S = 4.0


def prepare(source=SOURCE, output=OUTPUT):
    source, output = Path(source), Path(output)
    if not source.is_file():
        raise FileNotFoundError('Generate the terrain half-step world first: ' + str(source))
    if output.exists():
        raise FileExistsError('Refusing to overwrite existing world: ' + str(output))
    old_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None or world.get('name') != SOURCE_NAME:
        raise RuntimeError('Wrong source world')
    model = world.find("./model[@name='jarcrac']")
    if model is None:
        raise RuntimeError('No JARCRAC model')
    drivers = model.findall("./plugin[@name='gz::sim::systems::TrackedVehicle']")
    controllers = model.findall("./plugin[@name='gz::sim::systems::TrackController']")
    if len(drivers) != 1 or len(controllers) != 2:
        raise RuntimeError('Expected one tracked driver and two controllers')
    driver = drivers[0]
    if driver.findtext('topic') != '/model/jarcrac/detailed_cmd_vel' or driver.findtext('odom_topic') != ODOM:
        raise RuntimeError('Unexpected command or odometry topic')
    if any(p.findtext('max_command_age') != '0.3' for p in controllers):
        raise RuntimeError('0.3-second native track command expiry missing')
    chassis = model.find("./link[@name='chassis']")
    lidar = chassis.find("./sensor[@name='gpu_lidar_front']") if chassis is not None else None
    if (lidar is None or lidar.findtext('topic') != '/jarcrac/lidar'
            or lidar.findtext('ray/scan/horizontal/samples') != '721'
            or lidar.findtext('ray/scan/vertical/samples') != '32'):
        raise RuntimeError('Expected 721 x 32 LiDAR missing')
    lo = float(lidar.findtext('ray/scan/horizontal/min_angle'))
    hi = float(lidar.findtext('ray/scan/horizontal/max_angle'))
    offset = math.radians(.125)
    if abs(lo - (-math.pi/2 + offset)) > 1e-6 or abs(hi - (math.pi/2 + offset)) > 1e-6:
        raise RuntimeError('LiDAR half-step sampling offset missing')
    if chassis.findtext("./sensor[@type='imu']/topic") != '/jarcrac/imu':
        raise RuntimeError('IMU missing')
    if chassis.findtext("./sensor[@type='contact']/topic") != '/jarcrac/detailed/chassis_contacts':
        raise RuntimeError('Chassis contact sensor missing')
    if world.find("./model[@name='jarcrac_terrain_test_obstacle']") is None:
        raise RuntimeError('Known static obstacle missing')
    driver.find('topic').text = CMD
    world.set('name', WORLD)
    ET.indent(tree, space='  ')
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='utf-8', xml_declaration=True)
    if hashlib.sha256(source.read_bytes()).hexdigest() != old_hash:
        raise RuntimeError('Source world unexpectedly changed')
    print('ISOLATED BOUNDED-MOTION WORLD: PASS')
    print('Generated:', output)
    print('Single experiment command topic:', CMD)
    print('Source world and original vehicle: unchanged.')
    return output


def yaw_deg(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def progress_lateral(start, xy, yaw):
    dx, dy = xy[0]-start[0], xy[1]-start[1]
    theta = math.radians(yaw)
    return (math.cos(theta)*dx+math.sin(theta)*dy,
            -math.sin(theta)*dx+math.cos(theta)*dy)


def fast_input_check(w, now):
    """Extra motion gate; trip latched on first missed freshness bound."""
    with w.lock:
        for channel in ('pose', 'imu', 'odom', 'lidar', 'clock'):
            seen = w.seen.get(channel)
            if seen is None or now - seen > FAST_STALE_WALL_S:
                w.trip('motion_stale_' + channel)
                return 'motion_stale_' + channel
        if w.clock_progress_at is None or now-w.clock_progress_at > FAST_STALE_WALL_S:
            w.trip('motion_clock_not_advancing')
            return 'motion_clock_not_advancing'
    return None


def selftest():
    from jarcrac_command_arbiter_offline import Twist2
    w = Watchdog()
    p = CommandArbiterPolicy(w)
    assert p.tick(at=10)[0] == ZERO
    assert not p.arm(at=10)
    for k in ('pose', 'imu', 'odom', 'lidar', 'clock'):
        w.observe(k, at=11.0, **({'sim_time':1.0} if k=='clock' else {}))
    w.observe('clock', at=11.1, sim_time=1.1)
    assert p.arm(at=11.11)
    assert p.receive_request(source=EXPECTED_SOURCE, seq=1, speed=.10,
                             turn=0.0, at=11.12)
    assert p.tick(at=11.13)[0] == Twist2(.10, 0.0)
    assert fast_input_check(w, 11.14) is None
    for k in ('pose', 'imu', 'odom', 'clock'):
        w.observe(k, at=11.8, **({'sim_time':1.8} if k=='clock' else {}))
    assert fast_input_check(w, 11.8) == 'motion_stale_lidar'
    assert p.tick(at=11.81)[0] == ZERO
    assert p.latched_fault == 'watchdog_motion_stale_lidar'
    assert not p.arm(at=11.82)
    f, lat = progress_lateral((20,40),(19.9,39.94),-153.4349488)
    assert abs(f-.116) < .01 and abs(lat) < .01, (f, lat)
    print('PASS: no motion before explicit arm')
    print('PASS: bounded synthetic request accepted only with fresh telemetry')
    print('PASS: injected stale LiDAR -> latched ZERO, rearm refused')
    print('PASS: signed progress and lateral geometry')
    print('BOUNDED-MOTION SELFTEST: PASS (offline; no Gazebo publisher)')


def drive(args):
    if not args.arm:
        raise RuntimeError('Motion requires explicit --arm. Nothing was published.')
    if SPEED_MPS > MAX_FORWARD_MPS:
        raise RuntimeError('Speed exceeds offline policy')
    if not OUTPUT.is_file():
        raise FileNotFoundError('Run prepare first: ' + str(OUTPUT))
    world = ET.parse(OUTPUT).getroot().find('world')
    driver = world.find("./model[@name='jarcrac']/plugin[@name='gz::sim::systems::TrackedVehicle']")
    if world.get('name') != WORLD or driver is None or driver.findtext('topic') != CMD:
        raise RuntimeError('Expected isolated motion world missing or modified')

    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist

    w = Watchdog()
    p = CommandArbiterPolicy(w)
    lock = threading.RLock()
    state = {'xy':None, 'yaw':None, 'speed':None, 'speed_count':0,
             'last_scan_nearest':None, 'accepted_scans':0,
             'dropped_scans':0, 'pose_updates':0, 'odom_updates':0}
    injecting = threading.Event()
    node = Node()

    def on_pose(msg):
        try:
            for item in msg.pose:
                if item.name == 'jarcrac' or item.name.endswith('::jarcrac'):
                    xyz = (item.position.x, item.position.y, item.position.z)
                    q = tuple(getattr(item.orientation, a) for a in ('x','y','z','w'))
                    if not all(math.isfinite(x) for x in (*xyz,*q)) or not .8 < sum(v*v for v in q) < 1.2:
                        w.trip('invalid_pose')
                        return
                    with lock:
                        state['xy'] = xyz[:2]
                        state['yaw'] = yaw_deg(q)
                        state['pose_updates'] += 1
                    w.observe('pose')
                    return
        except Exception as exc:
            w.trip('pose_callback_' + type(exc).__name__)

    def on_imu(msg):
        try:
            q = tuple(getattr(msg.orientation, a) for a in ('x','y','z','w'))
            if not all(math.isfinite(x) for x in q) or not .8 < sum(x*x for x in q) < 1.2:
                w.trip('invalid_imu')
            else:
                w.observe('imu')
        except Exception as exc:
            w.trip('imu_callback_' + type(exc).__name__)

    def on_odom(msg):
        try:
            v = msg.twist.linear
            speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            if not math.isfinite(speed) or speed > .17:
                w.trip('invalid_or_excessive_odometry_speed')
                return
            with lock:
                state['speed'] = speed
                state['speed_count'] += 1
                state['odom_updates'] += 1
            w.observe('odom')
        except Exception as exc:
            w.trip('odom_callback_' + type(exc).__name__)

    def on_scan(msg):
        try:
            if msg.frame != 'jarcrac::chassis::gpu_lidar_front' or msg.count != 721 or msg.vertical_count != 32 or len(msg.ranges) != 23072:
                w.trip('invalid_lidar_shape')
                return
            values = [float(v) for v in msg.ranges if math.isfinite(v) and v > 0.0]
            if len(values) < 100:
                w.trip('insufficient_lidar_returns')
                return
            near = min(values)
            if near < 1.0:
                w.trip('lidar_range_below_one_metre')
                return
            if injecting.is_set():
                with lock:
                    state['dropped_scans'] += 1
                return
            with lock:
                state['last_scan_nearest'] = near
                state['accepted_scans'] += 1
            w.observe('lidar')
        except Exception as exc:
            w.trip('lidar_callback_' + type(exc).__name__)

    def on_clock(msg):
        try:
            w.observe('clock', sim_time=msg.sim.sec+msg.sim.nsec*1e-9)
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
        (Odometry, ODOM, on_odom),
        (LaserScan, '/jarcrac/lidar', on_scan),
        (Clock, f'/world/{WORLD}/clock', on_clock),
        (Contacts, CONTACT, on_contact),
    )
    for typ, topic, cb in topics:
        if not node.subscribe(typ, topic, cb):
            raise RuntimeError('Subscription failed: ' + topic)
        print('SUBSCRIBED:', topic, flush=True)

    pub = node.advertise(CMD, Twist)
    if pub is None:
        raise RuntimeError('Could not advertise isolated command topic')

    published_total = published_nonzero = 0
    start_xy = start_yaw = None
    end_xy = None
    motion_start = None
    motion_stop = None
    trigger_reason = 'not_started'
    trip_reason = None
    max_forward = max_lateral = max_observed_speed = 0.0
    final_speed = None
    zero_sent_after_stop = 0
    arming_ok = False
    stop_samples = 0
    passed = False
    seq = 0

    def publish(v):
        nonlocal published_total, published_nonzero, zero_sent_after_stop
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = 0.0
        pub.publish(msg)
        published_total += 1
        if abs(v) > 0:
            published_nonzero += 1
        if motion_stop is not None and v == 0:
            zero_sent_after_stop += 1

    def snap():
        with lock:
            return dict(state)

    print('BOUNDED MOTION: isolated command topic:', CMD, flush=True)
    print('EXPLICIT --arm confirmed | speed <=0.10m/s | target <=0.12m', flush=True)
    print('Mode:', args.mode, '| motion freshness:', FAST_STALE_WALL_S, 'wall seconds', flush=True)
    print('NO TTC INTEGRATION / NO DETOUR / NOT SAFETY RATED', flush=True)
    try:
        start_wall = time.monotonic()
        last_print = 0.0
        while time.monotonic()-start_wall < args.startup_timeout:
            publish(0)
            phase, why = w.state()
            s = snap()
            if phase == 'TRIPPED':
                trigger_reason = 'preflight_watchdog_' + why
                break
            if (phase == 'HEALTHY_MONITOR_ONLY' and s['xy'] is not None
                    and s['speed'] is not None and s['accepted_scans'] >= 3):
                start_xy = s['xy']; start_yaw = s['yaw']
                if math.hypot(start_xy[0]-20.0, start_xy[1]-40.0) > .60:
                    trigger_reason = 'unexpected_start_position'
                    break
                if abs((start_yaw + 153.76 + 180.) % 360.-180.) > 12.:
                    trigger_reason = 'unexpected_start_heading'
                    break
                if s['speed'] > STATIONARY_ODOM_MPS:
                    trigger_reason = 'moving_before_arm'
                    break
                arming_ok = p.arm(at=time.monotonic())
                if not arming_ok:
                    trigger_reason = 'arbiter_refused_arm'
                else:
                    print('PREFLIGHT PASS: fresh sensors, >=3 scans, stationary, expected pose.', flush=True)
                break
            if time.monotonic()-last_print > 2:
                print('PREFLIGHT:', phase, why, '| scans:', s['accepted_scans'], flush=True)
                last_print = time.monotonic()
            time.sleep(.1)
        else:
            trigger_reason = 'startup_timeout'

        if arming_ok:
            motion_start = time.monotonic()
            while time.monotonic()-motion_start < MAX_MOTION_WALL_S:
                now = time.monotonic()
                s = snap()
                phase, why = w.state(at=now)
                if phase == 'TRIPPED':
                    trigger_reason = 'watchdog_' + why
                    break
                if s['xy'] is None or s['speed'] is None:
                    w.trip('missing_motion_pose_or_speed')
                    trigger_reason = 'missing_motion_pose_or_speed'
                    break
                progress, lateral = progress_lateral(start_xy, s['xy'], start_yaw)
                max_forward = max(max_forward, progress)
                max_lateral = max(max_lateral, abs(lateral))
                max_observed_speed = max(max_observed_speed, s['speed'])
                if (not math.isfinite(progress) or progress < -.03 or progress > HARD_PROGRESS_M
                        or abs(lateral) > MAX_LATERAL_M or
                        abs((s['yaw'] - start_yaw + 180) % 360 - 180) > 5.0):
                    w.trip('motion_pose_envelope')
                    trigger_reason = 'motion_pose_envelope'
                    break
                if s['speed'] > .15:
                    w.trip('motion_excess_speed')
                    trigger_reason = 'motion_excess_speed'
                    break
                stale = fast_input_check(w, now)
                if stale:
                    trigger_reason = stale
                    break
                if args.mode == 'inject-lidar' and now-motion_start >= .45:
                    injecting.set()
                if args.mode == 'normal' and progress >= SOFT_PROGRESS_M:
                    trigger_reason = 'distance_target_reached'
                    break
                seq += 1
                if not p.receive_request(source=EXPECTED_SOURCE, seq=seq,
                        speed=SPEED_MPS, turn=0.0, at=now):
                    trigger_reason = 'arbiter_request_rejected'
                    break
                output, why = p.tick(at=now)
                if output != ZERO and output.linear_x != SPEED_MPS:
                    w.trip('incorrect_arbiter_speed')
                    trigger_reason = 'incorrect_arbiter_speed'
                    break
                if output == ZERO:
                    trigger_reason = 'arbiter_inhibited_' + why
                    break
                publish(output.linear_x)
                if now-last_print >= 1.0:
                    print('MOVING: progress=%.3fm, speed=%.3fm/s, scans=%d' %
                          (progress,s['speed'],s['accepted_scans']), flush=True)
                    last_print = now
                time.sleep(.10)
            else:
                trigger_reason = 'motion_wall_timeout'

        # Stop issuing nonzero commands immediately on all exit paths.
        # Let the arbiter observe a watchdog fault BEFORE operator disarm, so
        # its own fault latch preserves the root cause rather than masking it.
        motion_stop = time.monotonic()
        p.tick(at=motion_stop)
        p.disarm()
        print('STOP:', trigger_reason, '| issuing repeated zero commands', flush=True)
        deadline = time.monotonic()+STOP_TIMEOUT_WALL_S
        last_count = snap()['speed_count']  # require NEW odometry updates after STOP
        while time.monotonic() < deadline:
            publish(0)
            s = snap()
            if (s['speed'] is not None and s['speed'] <= STATIONARY_ODOM_MPS
                    and s['speed_count'] > last_count):
                stop_samples += 1
                last_count = s['speed_count']
                if stop_samples >= 3:
                    break
            elif s['speed_count'] > last_count:
                stop_samples = 0
                last_count = s['speed_count']
            time.sleep(.075)
        # Hold zero for a bounded time; monitor vehicle pose after zero.
        hold_end = time.monotonic()+1.1
        while time.monotonic() < hold_end:
            publish(0)
            time.sleep(.075)
        s = snap()
        end_xy = s['xy']
        final_speed = s['speed']
        final_forward = progress_lateral(start_xy, end_xy, start_yaw)[0] if start_xy and end_xy else None
        held_progress_ok = (final_forward is not None
                            and -.03 <= final_forward <= HARD_PROGRESS_M
                            and max_lateral <= MAX_LATERAL_M)
        if args.mode == 'normal':
            passed = (arming_ok and trigger_reason == 'distance_target_reached'
                      and max_forward >= .08 and max_observed_speed >= .02
                      and held_progress_ok and final_speed is not None
                      and final_speed <= STATIONARY_ODOM_MPS
                      and stop_samples >= 3 and not w.fault)
        else:
            passed = (arming_ok and trigger_reason == 'motion_stale_lidar'
                      and w.fault == 'motion_stale_lidar'
                      and p.latched_fault == 'watchdog_motion_stale_lidar'
                      and s['dropped_scans'] > 0 and published_nonzero > 0
                      and max_observed_speed >= .02 and held_progress_ok
                      and stop_samples >= 3 and final_speed is not None
                      and final_speed <= STATIONARY_ODOM_MPS)
        # Explicit fault identity is useful even if operator disarms after trip.
        trip_reason = w.fault
    except KeyboardInterrupt:
        trigger_reason = 'interrupted'
    except Exception as exc:
        trigger_reason = type(exc).__name__+': '+str(exc)
        w.trip('motion_exception')
    finally:
        # Best-effort neutral stream even on exceptions. Native track controller
        # also has a configured 0.3s max command age. Neither is safety rated.
        for _ in range(20):
            try:
                publish(0)
            except Exception:
                pass
            time.sleep(.05)
    report = dict(test='camino_bounded_motion_arbiter', world=WORLD,
        mode=args.mode, passed=bool(passed), reason=trigger_reason,
        commanded_speed_mps=SPEED_MPS, nominal_progress_limit_m=SOFT_PROGRESS_M,
        hard_progress_limit_m=HARD_PROGRESS_M, max_forward_progress_m=max_forward,
        final_forward_progress_m=final_forward if 'final_forward' in locals() else None,
        max_lateral_m=max_lateral, max_observed_odom_speed_mps=max_observed_speed,
        final_odom_speed_mps=final_speed, stop_distinct_odom_samples=stop_samples,
        published_command_calls=published_total,
        published_nonzero_commands=published_nonzero,
        published_zero_after_stop=zero_sent_after_stop,
        accepted_scans=snap()['accepted_scans'], dropped_scans=snap()['dropped_scans'],
        watchdog_latched_fault=w.fault, arbiter_latched_fault=p.latched_fault,
        explicit_operator_arm=args.arm, policy_source='internal_synthetic_trial_request',
        real_ttc_controller_connected=False, independent_process_watchdog=False,
        contact_silence_proves_sensor_health=False,
        transport_exclusivity_enforced=False, safety_rated=False,
        simulation_pose_ground_truth_used=True, timestamp_sync_verified=False)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print('BOUNDED-MOTION GAZEBO:', 'PASS' if passed else 'FAIL', flush=True)
    print('Report:', args.report, flush=True)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if not passed:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('selftest', help='Offline tests: no Gazebo required')
    sub.add_parser('prepare', help='Generate an isolated, non-overwriting motion world')
    p = sub.add_parser('drive', help='Simulation-only bounded motion; requires --arm')
    p.add_argument('--arm', action='store_true', help='Explicitly authorize this isolated low-speed trial')
    p.add_argument('--mode', choices=['normal','inject-lidar'], default='normal')
    p.add_argument('--startup-timeout', type=float, default=25)
    p.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args()
    if args.action == 'selftest':
        selftest()
    elif args.action == 'prepare':
        prepare()
    else:
        if not 5 <= args.startup_timeout <= 60:
            parser.error('startup-timeout must be 5..60s')
        drive(args)


if __name__ == '__main__':
    main()
