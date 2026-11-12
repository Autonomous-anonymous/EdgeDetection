#!/usr/bin/env python3
"""CAMINO JARCRAC Phase 2b: isolated Gazebo ZERO-only command publisher.

prepare: make a NEW experimental world with an isolated command topic.
selftest: offline checks for the zero-output invariant and SDF isolation.
live: subscribe to five real streams, evaluate the watchdog + offline arbiter,
      and publish only Twist(0,0) at ~10 Hz, also after a latched fault.

No arming, nonzero request handling or driving. Not safety-rated. No guarantee
that this Python process can enforce exclusive Gazebo topic ownership.
Close other Gazebo worlds and command publishers before using 'live'.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

from jarcrac_detailed_watchdog_monitor import Watchdog, STALE_WALL_S
from jarcrac_command_arbiter_offline import CommandArbiterPolicy, ZERO

WORLD = 'jarcrac_detailed_arbiter_zero'
SOURCE_WORLD = 'jarcrac_detailed_terrain_halfstep'
FOLDER = Path.home() / 'jarcrac_imu_experiment'
SOURCE = FOLDER / (SOURCE_WORLD + '.sdf')
WORLD_FILE = FOLDER / (WORLD + '.sdf')
CMD = '/model/jarcrac/arbiter_zero_cmd_vel'
ODOM = '/model/jarcrac/detailed_odometry'
CONTACT = (f'/world/{WORLD}/model/jarcrac/link/chassis/'
           'sensor/detailed_chassis_contact/contact')
REPORT = Path('/tmp/jarcrac_arbiter_zero_owner.json')


def prepare_world(source=SOURCE, dest=WORLD_FILE):
    source, dest = Path(source), Path(dest)
    if not source.is_file():
        raise FileNotFoundError(f'Missing half-step terrain world: {source}')
    if dest.exists():
        raise FileExistsError(f'Refusing to overwrite existing world: {dest}')
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None or world.get('name') != SOURCE_WORLD:
        raise RuntimeError('Unexpected source world')
    model = world.find("./model[@name='jarcrac']")
    if model is None:
        raise RuntimeError('Missing JARCRAC model')
    tracked = model.findall("./plugin[@name='gz::sim::systems::TrackedVehicle']")
    controllers = model.findall("./plugin[@name='gz::sim::systems::TrackController']")
    if len(tracked) != 1 or len(controllers) != 2:
        raise RuntimeError('Expected one native tracked driver and two controllers')
    if tracked[0].findtext('topic') != '/model/jarcrac/detailed_cmd_vel':
        raise RuntimeError('Unexpected original command topic')
    if tracked[0].findtext('odom_topic') != ODOM:
        raise RuntimeError('Unexpected odometry topic')
    if any(p.findtext('max_command_age') != '0.3' for p in controllers):
        raise RuntimeError('Required 0.3-second track command timeout missing')
    chassis = model.find("./link[@name='chassis']")
    if chassis is None:
        raise RuntimeError('Missing chassis')
    lidar = chassis.find("./sensor[@name='gpu_lidar_front']")
    if (lidar is None or lidar.findtext('topic') != '/jarcrac/lidar'
            or lidar.findtext('ray/scan/horizontal/samples') != '721'
            or lidar.findtext('ray/scan/vertical/samples') != '32'):
        raise RuntimeError('Unexpected LiDAR')
    lo = float(lidar.findtext('ray/scan/horizontal/min_angle'))
    hi = float(lidar.findtext('ray/scan/horizontal/max_angle'))
    offset = math.radians(0.125)
    if (abs(lo - (-math.pi/2 + offset)) > 1e-6
            or abs(hi - (math.pi/2 + offset)) > 1e-6):
        raise RuntimeError('Half-step LiDAR correction not present')
    if chassis.findtext("./sensor[@type='imu']/topic") != '/jarcrac/imu':
        raise RuntimeError('Missing expected IMU')
    if chassis.findtext("./sensor[@type='contact']/topic") != '/jarcrac/detailed/chassis_contacts':
        raise RuntimeError('Missing contact sensor')
    if world.find("./model[@name='jarcrac_terrain_test_obstacle']") is None:
        raise RuntimeError('Known static obstacle missing')
    tracked[0].find('topic').text = CMD
    world.set('name', WORLD)
    ET.indent(tree, space='  ')
    dest.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dest, encoding='utf-8', xml_declaration=True)
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError('Unexpected source-world modification')
    check = ET.parse(dest).getroot().find('world')
    assert check.get('name') == WORLD
    assert check.find("./model[@name='jarcrac']/plugin[@name='gz::sim::systems::TrackedVehicle']/topic").text == CMD
    print('ISOLATED ZERO-COMMAND WORLD: PASS', flush=True)
    print('Generated:', dest, flush=True)
    print('New command topic:', CMD, flush=True)
    print('Original model and worlds unchanged.', flush=True)
    return dest


def zero_command(policy, at, twist_class):
    decision, reason = policy.tick(at=at)
    if decision != ZERO or policy.enabled or policy.last_request is not None:
        raise RuntimeError('Zero-owner invariant violated: refuse nonzero command')
    msg = twist_class()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    return msg, reason


def selftest():
    class V:
        x = 999.0
        y = 999.0
        z = 999.0
    class M:
        def __init__(self):
            self.linear = V()
            self.angular = V()
    w = Watchdog()
    p = CommandArbiterPolicy(w)
    for step in range(3):
        msg, reason = zero_command(p, 10.0 + step, M)
        assert msg.linear.x == msg.angular.z == 0.0
        assert not p.enabled
    print('PASS: no telemetry -> zero only')
    # Simulate all telemetry healthy, still never automatically arm.
    for k in ('pose', 'imu', 'odom', 'lidar', 'clock'):
        w.observe(k, at=20.1, **({'sim_time': 1.0} if k == 'clock' else {}))
    w.observe('clock', at=20.2, sim_time=1.1)
    msg, reason = zero_command(p, 20.21, M)
    assert reason == 'operator_not_armed' and msg.linear.x == 0
    print('PASS: healthy telemetry -> still zero')
    w.trip('stale_lidar')
    msg, reason = zero_command(p, 20.3, M)
    assert reason == 'LATCHED:watchdog_stale_lidar' and msg.linear.x == 0
    msg, reason = zero_command(p, 21.0, M)
    assert p.latched_fault == 'watchdog_stale_lidar' and msg.linear.x == 0
    print('PASS: latched fault -> continuing zero output')
    # A minimal synthetic scene verifies the isolated world rewrite.
    with tempfile.TemporaryDirectory() as d:
        src = Path(d)/'src.sdf'
        dst = Path(d)/'dst.sdf'
        root = ET.Element('sdf', version='1.10')
        world = ET.SubElement(root, 'world', name=SOURCE_WORLD)
        model = ET.SubElement(world, 'model', name='jarcrac')
        driver = ET.SubElement(model, 'plugin', name='gz::sim::systems::TrackedVehicle')
        ET.SubElement(driver, 'topic').text = '/model/jarcrac/detailed_cmd_vel'
        ET.SubElement(driver, 'odom_topic').text = ODOM
        for _ in range(2):
            controller = ET.SubElement(model, 'plugin', name='gz::sim::systems::TrackController')
            ET.SubElement(controller, 'max_command_age').text = '0.3'
        chassis = ET.SubElement(model, 'link', name='chassis')
        lidar = ET.SubElement(chassis, 'sensor', name='gpu_lidar_front')
        ET.SubElement(lidar, 'topic').text = '/jarcrac/lidar'
        ray = ET.SubElement(lidar, 'ray'); scan = ET.SubElement(ray, 'scan')
        horizontal = ET.SubElement(scan, 'horizontal')
        ET.SubElement(horizontal, 'samples').text = '721'
        ET.SubElement(horizontal, 'min_angle').text = str(-math.pi/2+math.radians(.125))
        ET.SubElement(horizontal, 'max_angle').text = str(math.pi/2+math.radians(.125))
        vertical = ET.SubElement(scan, 'vertical'); ET.SubElement(vertical, 'samples').text = '32'
        imu = ET.SubElement(chassis, 'sensor', type='imu')
        ET.SubElement(imu, 'topic').text = '/jarcrac/imu'
        contact = ET.SubElement(chassis, 'sensor', type='contact')
        ET.SubElement(contact, 'topic').text = '/jarcrac/detailed/chassis_contacts'
        ET.SubElement(world, 'model', name='jarcrac_terrain_test_obstacle')
        ET.ElementTree(root).write(src, encoding='utf-8', xml_declaration=True)
        before = src.read_bytes()
        prepare_world(src, dst)
        assert src.read_bytes() == before
        assert ET.parse(dst).getroot().find('world').get('name') == WORLD
        try:
            prepare_world(src, dst)
        except FileExistsError:
            pass
        else:
            raise AssertionError('Overwriting must be rejected')
    print('PASS: world isolation + no overwrite')
    print('ZERO-OWNER SELFTEST: PASS (offline; no Gazebo publishers)')


def live(args):
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist

    if not WORLD_FILE.is_file():
        raise RuntimeError('Run prepare before live; missing: ' + str(WORLD_FILE))
    tree = ET.parse(WORLD_FILE)
    world = tree.getroot().find('world')
    driver = world.find("./model[@name='jarcrac']/plugin[@name='gz::sim::systems::TrackedVehicle']")
    if world.get('name') != WORLD or driver.findtext('topic') != CMD:
        raise RuntimeError('Wrong isolated world; refusing to publish')

    watchdog = Watchdog()
    arbiter = CommandArbiterPolicy(watchdog)
    node = Node()
    healthy_at = None
    dropped_lidar = 0
    max_speed = 0.0
    received_pose = 0
    publish_calls = 0

    def pose(msg):
        nonlocal received_pose
        try:
            for item in msg.pose:
                if item.name == 'jarcrac' or item.name.endswith('::jarcrac'):
                    coords = (item.position.x, item.position.y, item.position.z)
                    q = (item.orientation.x, item.orientation.y, item.orientation.z, item.orientation.w)
                    if not all(math.isfinite(x) for x in coords + q) or not .8 < sum(x*x for x in q) < 1.2:
                        watchdog.trip('invalid_pose')
                    else:
                        watchdog.observe('pose')
                        received_pose += 1
                    return
        except Exception as exc:
            watchdog.trip('pose_callback_' + type(exc).__name__)

    def imu(msg):
        try:
            q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            if not all(math.isfinite(x) for x in q) or not .8 < sum(x*x for x in q) < 1.2:
                watchdog.trip('invalid_imu')
            else:
                watchdog.observe('imu')
        except Exception as exc:
            watchdog.trip('imu_callback_' + type(exc).__name__)

    def odom(msg):
        nonlocal max_speed
        try:
            v = msg.twist.linear
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            if not math.isfinite(speed):
                watchdog.trip('invalid_odometry')
            elif speed > .06:
                watchdog.trip('unexpected_motion_in_zero_test')
            else:
                max_speed = max(max_speed, speed)
                watchdog.observe('odom')
        except Exception as exc:
            watchdog.trip('odom_callback_' + type(exc).__name__)

    def lidar(msg):
        nonlocal dropped_lidar
        try:
            if (msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                    or msg.count != 721 or msg.vertical_count != 32
                    or len(msg.ranges) != 23072):
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
            watchdog.observe('clock', sim_time=msg.sim.sec + msg.sim.nsec * 1e-9)
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
        (Odometry, ODOM, odom),
        (LaserScan, '/jarcrac/lidar', lidar),
        (Clock, f'/world/{WORLD}/clock', clock),
        (Contacts, CONTACT, contact),
    )
    for typ, topic, fn in topics:
        if not node.subscribe(typ, topic, fn):
            raise RuntimeError('Subscription failed: ' + topic)
        print('SUBSCRIBED:', topic, flush=True)

    pub = node.advertise(CMD, Twist)
    if pub is None:
        raise RuntimeError('Unable to advertise command topic: ' + CMD)
    print('ZERO OWNER: publishing ONLY Twist(0,0) on', CMD, flush=True)
    print('NEVER ARMED; never accepts motion requests; no driving authorized.', flush=True)

    start = time.monotonic()
    last_print = 0.0
    phase, reason = 'WAITING', 'startup'
    passed = False
    try:
        while True:
            now = time.monotonic()
            msg, policy_reason = zero_command(arbiter, now, Twist)
            pub.publish(msg)
            publish_calls += 1
            phase, reason = watchdog.state(at=now)
            if healthy_at is None and phase == 'HEALTHY_MONITOR_ONLY':
                healthy_at = now
                print('HEALTHY: live streams; publishing ZERO only', flush=True)
            if phase == 'TRIPPED':
                print('WATCHDOG TRIPPED:', reason, '| arbiter:', policy_reason, flush=True)
                passed = (args.inject_lidar_drop_after is not None
                          and reason == 'stale_lidar'
                          and arbiter.latched_fault == 'watchdog_stale_lidar'
                          and dropped_lidar > 0)
                break
            if healthy_at is None and now-start >= args.startup_timeout:
                reason = 'startup_timeout_' + reason
                break
            if healthy_at is not None and now-healthy_at >= args.seconds:
                passed = args.inject_lidar_drop_after is None and phase == 'HEALTHY_MONITOR_ONLY'
                if not passed:
                    reason = 'injected_fault_not_detected'
                break
            if now-last_print >= 2:
                print(f'{phase}: {policy_reason} | published_zeros={publish_calls} '
                      f'| counts={watchdog.summary()["counts"]}', flush=True)
                last_print = now
            time.sleep(.1)
    except KeyboardInterrupt:
        reason = 'operator_interrupt'
    except Exception as exc:
        reason = type(exc).__name__ + ': ' + str(exc)
        watchdog.trip('zero_publisher_exception')
    finally:
        # Continue issuing neutral commands briefly even if monitoring fails.
        neutral = Twist()  # protobuf defaults to exactly zero on all axes
        for _ in range(10):
            try:
                pub.publish(neutral)
                publish_calls += 1
            except Exception:
                pass
            time.sleep(.05)

    report = dict(test='camino_arbiter_zero_owner_stationary', world=WORLD,
                  passed=passed, phase=phase, reason=reason,
                  publish_calls=publish_calls, published_nonzero_commands=0,
                  zero_output_invariant_enforced=True,
                  transport_delivery_not_verified=True,
                  no_external_publishers_not_verified=True,
                  max_observed_stationary_speed_mps=max_speed,
                  pose_messages_with_jarcrac=received_pose,
                  watchdog_latched_fault=watchdog.fault,
                  arbiter_latched_fault=arbiter.latched_fault,
                  dropped_lidar_observations=dropped_lidar,
                  injector_after_healthy_wall_s=args.inject_lidar_drop_after,
                  motion_authorized=False, policy_enabled=arbiter.enabled,
                  stale_threshold_wall_s=STALE_WALL_S,
                  contact_silence_proves_sensor_health=False,
                  source_timestamps_not_synchronized=True,
                  safety_rated=False, **watchdog.summary())
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print('ZERO-OWNER GAZEBO:', 'PASS' if passed else 'FAIL', flush=True)
    print('Report:', args.report, flush=True)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if not passed:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('prepare', help='Generate isolated experiment world')
    sub.add_parser('selftest', help='Offline invariants and world rewrite')
    p = sub.add_parser('live', help='Zero-only real Gazebo publisher')
    p.add_argument('--seconds', type=float, default=12.)
    p.add_argument('--startup-timeout', type=float, default=25.)
    p.add_argument('--inject-lidar-drop-after', type=float)
    p.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare_world()
    elif args.action == 'selftest':
        selftest()
    else:
        if not 3 <= args.seconds <= 120 or not 5 <= args.startup_timeout <= 90:
            parser.error('Invalid monitoring duration / startup timeout')
        if args.inject_lidar_drop_after is not None and not (
                1 <= args.inject_lidar_drop_after < args.seconds-STALE_WALL_S-.5):
            parser.error('Injection must leave time for freshness fault')
        live(args)


if __name__ == '__main__':
    main()
