#!/usr/bin/env python3
"""Bounded moving JARCRAC 3D-LiDAR/IMU obstacle-detection SIL diagnostic.

Two-terminal test only. A generated world embeds the checked-in 32-layer model,
adds IMU and chassis-contact sensing, and isolates the velocity topic. Travel
is capped near 0.35 m at 0.10 m/s toward a static obstacle initially ~3.84 m
from the LiDAR. The controller requires validated obstacle perception BEFORE
and THROUGHOUT commanded motion; it stops if data or clearance checks fail.

This is NOT terrain AEB, autonomous avoidance, verified stopping distance,
real-world localization, or a certified safety system. The shared map classifier
uses Gazebo's exact XYZ and wall-reception pairing rather than sensor-time sync.
A SIGKILL / host failure has NO independent actuator-side watchdog. Pause Gazebo
or issue repeated zero velocities if this test process is forcibly terminated.
Do not run other JARCRAC worlds, tests or velocity publishers concurrently.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_imu_stage import MESH, SOURCES, q_to_rpy
from jarcrac_lidar_map_assisted import TerrainMeshIndex, quaternion_tuple
from jarcrac_imu_dynamic import (
    Monitor, bounded_distance, prepare_world as prepare_dynamic_world,
    route_displacement, safe_to_command, MAX_ORIENTATION_ERROR_DEG,
    MAX_UNKNOWN_FRACTION, MAX_DRIVE_DISTANCE,
    HARD_DISTANCE_LIMIT, STALE_S,
)

WORLD_NAME = 'jarcrac_imu_dynamic_obstacle'
WORLD_PATH = Path.home() / 'jarcrac_imu_experiment' / (WORLD_NAME + '.sdf')
POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'
CMD_TOPIC = '/model/jarcrac/obstacle_probe_cmd_vel'
CONTACT_TOPIC = '/jarcrac/obstacle_probe/chassis_contacts'
LIDAR_TOPIC = '/jarcrac/lidar'
KNOWN_OBSTACLE = 'jarcrac_terrain_test_obstacle'
OBSTACLE_XY = (14.633437, 37.316718)
EXPECTED_START_XY = (20.0, 40.0)
MAX_SPEED_MPS = 0.10
STOP_SENSOR_GAP_M = 3.0  # axial scanner-forward gap, NOT verified body clearance
MIN_OBSTACLE_POINTS = 50
MAX_ANALYSIS_AGE_S = 0.9
MAX_DRIVE_WALL_S = 12.0
MIN_PROGRESS_FOR_PASS_M = 0.25
DEFAULT_REPORT = Path('/tmp/jarcrac_dynamic_obstacle_recheck.json')


def prepare_world(output: Path = WORLD_PATH) -> Path:
    """Generate independent obstacle scene without altering the baseline files."""
    result = prepare_dynamic_world(
        output, source=SOURCES['obstacle'], world_name=WORLD_NAME,
        cmd_topic=CMD_TOPIC, contact_topic=CONTACT_TOPIC,
    )
    root = ET.parse(result).getroot()
    world = root.find('world')
    if world is None or world.get('name') != WORLD_NAME:
        raise ValueError('Generated world name mismatch')
    vehicle = world.find("./model[@name='jarcrac']")
    obstacle = world.find(f"./model[@name='{KNOWN_OBSTACLE}']")
    if vehicle is None or obstacle is None:
        raise ValueError('Known vehicle / obstacle missing from generated world')
    lidar = vehicle.find("./link[@name='chassis']/sensor[@name='gpu_lidar_front']")
    if (lidar is None or lidar.findtext('ray/scan/horizontal/samples') != '721'
            or lidar.findtext('ray/scan/vertical/samples') != '32'
            or lidar.findtext('topic') != LIDAR_TOPIC):
        raise ValueError('Generated world does not contain the expected 721x32 LiDAR')
    if obstacle.findtext('static') != 'true':
        raise ValueError('Expected a stationary obstacle')
    obstacle_xy = tuple(map(float, obstacle.findtext('pose').split()[:2]))
    if math.dist(obstacle_xy, OBSTACLE_XY) > 0.02:
        raise ValueError('Known obstacle moved in source world')
    return result


def obstacle_sample_status(row: dict | None) -> str:
    """Expected obstacle is necessary, but uncertainty and bad data fail closed.

    The imported clear-terrain worker labels any obstacle 'unexpected_obstacle';
    this function intentionally re-evaluates its *raw paired summaries* for the
    distinct, known-obstacle experiment. It never treats an invalid row as safe.
    """
    if row is None:
        return 'no_analysis'
    if not row.get('paired'):
        return row.get('reason') or 'unpaired_inputs'
    error = row.get('orientation_error_deg')
    if error is None or not math.isfinite(error) or error < 0:
        return 'invalid_orientation_error'
    if error > MAX_ORIENTATION_ERROR_DEG:
        return 'orientation_mismatch'
    distances = []
    for key in ('exact', 'imu'):
        summary = row.get(key)
        if not isinstance(summary, dict) or not summary.get('valid'):
            return 'invalid_map_classification'
        unknown = summary.get('unknown_fraction')
        if unknown is None or not math.isfinite(unknown) or not 0 <= unknown <= MAX_UNKNOWN_FRACTION:
            return 'unknown_terrain_excess'
        count = summary.get('objects')
        if not isinstance(count, int) or count < MIN_OBSTACLE_POINTS:
            return 'expected_obstacle_missing'
        nearest = summary.get('nearest_m')
        if nearest is None or not math.isfinite(nearest) or nearest <= 0:
            return 'invalid_obstacle_range'
        if nearest < STOP_SENSOR_GAP_M:
            return 'obstacle_too_close'
        distances.append(nearest)
    if abs(distances[0] - distances[1]) > 0.25:
        return 'imu_map_range_disagreement'
    return 'obstacle_confirmed'


def health_gate(*, status, age_analysis, ages, contact, speed,
                roll_deg, pitch_deg, distance, yaw_change_deg):
    """Use existing motion/tilt/contact/staleness gates without calling path clear.

    Here 'clear' passed to the *shared equipment-health gate* means that the
    separately checked known obstacle is present and at a conservative range;
    it does NOT mean the driving corridor is obstacle-free.
    """
    if status != 'obstacle_confirmed':
        return status
    if not math.isfinite(age_analysis) or age_analysis > MAX_ANALYSIS_AGE_S:
        return 'stale_analysis'
    result = safe_to_command(
        age_pose=ages['pose'], age_imu=ages['imu'],
        age_odom=ages['odom'], age_scan=ages['scan'],
        analysis_age=age_analysis, analysis_status='clear',
        contact=contact, speed=speed, roll_deg=roll_deg,
        pitch_deg=pitch_deg, distance=distance,
        yaw_change_deg=yaw_change_deg,
    )
    return 'safe_to_probe' if result == 'clear' else result


class ObstacleMonitor(Monitor):
    """Reuse validated asynchronous monitor; verify precise scene and sensor."""

    def __init__(self, node, msgs, *, every_scans):
        super().__init__(
            node, msgs, every_scans=every_scans,
            pose_topic=POSE_TOPIC, cmd_topic=CMD_TOPIC,
            contact_topic=CONTACT_TOPIC,
        )
        with self.lock:
            self.state.update(obstacle_xyz=None, obstacle_at=0.0,
                              obstacle_pose_count=0)

    def on_pose(self, msg):
        super().on_pose(msg)
        for p in msg.pose:
            if p.name == KNOWN_OBSTACLE or p.name.endswith('::' + KNOWN_OBSTACLE):
                with self.lock:
                    self.state['obstacle_xyz'] = (
                        float(p.position.x), float(p.position.y), float(p.position.z))
                    self.state['obstacle_at'] = time.monotonic()
                    self.state['obstacle_pose_count'] += 1
                break

    def on_scan(self, msg):
        h, v = int(msg.count), int(msg.vertical_count)
        if (msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                or h != 721 or v != 32 or len(msg.ranges) != 23072):
            self._err('lidar_shape', ValueError(
                f'unexpected {msg.frame!r}: {h} x {v}, ranges={len(msg.ranges)}'))
            return
        super().on_scan(msg)


def stop_confirmed(monitor: ObstacleMonitor, *, deadline_wall_s=3.0):
    """Require three distinct low-speed odometry samples while publishing zero."""
    until = time.monotonic() + deadline_wall_s
    last_seq = monitor.snapshot()['odom_count']
    successive = 0
    while time.monotonic() < until:
        monitor.command(0)
        snap = monitor.snapshot()
        if snap['odom_count'] != last_seq:
            last_seq = snap['odom_count']
            if snap['speed'] is not None and math.isfinite(snap['speed']) and snap['speed'] <= 0.04:
                successive += 1
            else:
                successive = 0
            if successive >= 3:
                return True, successive
        time.sleep(.075)
    return False, successive


def run_trial(args):
    try:
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic system-Python bindings missing: ' + str(exc))

    monitor = ObstacleMonitor(Node(), dict(
        Contacts=Contacts, IMU=IMU, LaserScan=LaserScan,
        Odometry=Odometry, Pose_V=Pose_V, Twist=Twist,
    ), every_scans=args.every_scans)
    report = dict(
        test='jarcrac_dynamic_known_obstacle_perception',
        world=WORLD_NAME, case='static_known_obstacle_moving_jarcrac',
        experimental_terrain_motion=True, terrain_aeb_validated=False,
        avoidance_enabled=False, exact_gazebo_xyz_used=True,
        scan_imu_pairing='wall_reception_proximity_not_sim_timestamps',
        commanded_speed_mps=MAX_SPEED_MPS,
        max_distance_command_m=MAX_DRIVE_DISTANCE,
        sensor_gap_abort_m=STOP_SENSOR_GAP_M,
        sensor_gap_is_not_body_clearance=True,
        passed=False, reason='not_started',
    )
    worker = None
    initial_xy = None
    initial_yaw = None
    initial_obstacle_gap = None
    stopped_due_to_distance = False
    confirmed = False
    stop_samples = 0
    max_lateral = 0.0
    signed_forward = 0.0
    moved = 0.0
    try:
        print('JARCRAC MOVING OBSTACLE PERCEPTION | max 0.10 m/s, 0.35 m', flush=True)
        print('World:', WORLD_NAME, '| LiDAR:', LIDAR_TOPIC,
              '| command:', CMD_TOPIC, flush=True)
        monitor.stop(5)
        until = time.monotonic() + 20
        while time.monotonic() < until:
            snap = monitor.snapshot()
            p = snap['pose']
            xyz = snap.get('obstacle_xyz')
            if (p is not None and xyz is not None and snap['imu'] is not None
                    and snap['speed'] is not None):
                break
            monitor.command(0)
            time.sleep(.1)
        else:
            raise RuntimeError('Vehicle, obstacle, IMU or odometry missing: wrong world?')
        if (math.hypot(p.position.x - EXPECTED_START_XY[0],
                       p.position.y - EXPECTED_START_XY[1]) > .6
                or math.dist(xyz[:2], OBSTACLE_XY) > .10
                or snap['speed'] > .04 or snap['contact']):
            raise RuntimeError('Unexpected initial scene, velocity or contact')
        if (snap['callback_errors'] or snap['age_pose'] > STALE_S
                or snap['age_imu'] > STALE_S or snap['age_odom'] > STALE_S):
            raise RuntimeError('Startup sensor/scene stale or callback error')
        initial_xy = (p.position.x, p.position.y)
        _, _, initial_yaw = q_to_rpy(quaternion_tuple(p.orientation))
        expected_yaw = math.degrees(-2.6779450446)
        if abs((initial_yaw - expected_yaw + 180.) % 360. - 180.) > 15.0:
            raise RuntimeError('Unexpected start heading toward obstacle')
        terrain = TerrainMeshIndex(MESH, center_x=p.position.x,
                                   center_y=p.position.y, radius_m=24.)
        worker = threading.Thread(target=monitor.run_worker,
                                  args=(terrain,), daemon=True)
        worker.start()
        good_ids = set()
        until = time.monotonic() + 25
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            row = snap['analysis']
            status = obstacle_sample_status(row)
            if snap['callback_errors'] or snap['worker_errors'] or snap['contact']:
                raise RuntimeError('Sensor processing error or contact at startup')
            if row is not None and status != 'obstacle_confirmed':
                raise RuntimeError('Unsafe initial scan: ' + status)
            if (status == 'obstacle_confirmed'
                    and snap['analysis_age'] <= MAX_ANALYSIS_AGE_S):
                nearest = row['imu']['nearest_m']
                if not 3.2 <= nearest <= 5.0:
                    raise RuntimeError('Unexpected initial obstacle distance')
                good_ids.add(row['scan_number'])
                initial_obstacle_gap = nearest
            if len(good_ids) >= 3:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Three valid paired obstacle scans not observed')
        print('Preflight PASS: 3 distinct paired 3D scans detect known obstacle.', flush=True)

        drive_start = time.monotonic()
        last_print = drive_start
        while time.monotonic() - drive_start < MAX_DRIVE_WALL_S:
            snap = monitor.snapshot()
            p = snap['pose']
            if p is None or snap['speed'] is None:
                report['reason'] = 'missing_pose_or_odometry'
                break
            moved = bounded_distance(initial_xy, (p.position.x, p.position.y))
            signed_forward, lateral = route_displacement(
                initial_xy, (p.position.x, p.position.y), initial_yaw)
            max_lateral = max(max_lateral, abs(lateral))
            roll, pitch, yaw = q_to_rpy(quaternion_tuple(p.orientation))
            delta_yaw = abs((yaw - initial_yaw + 180.) % 360. - 180.)
            if abs(lateral) > .18 or signed_forward < -.05:
                report['reason'] = 'route_deviation'
                break
            if moved >= MAX_DRIVE_DISTANCE:
                stopped_due_to_distance = True
                report['reason'] = 'distance_limit_reached'
                break
            row = snap['analysis']
            status = obstacle_sample_status(row)
            if (snap['callback_errors'] or snap['worker_errors']):
                report['reason'] = 'callback_or_processing_error'
                break
            if (math.dist(snap['obstacle_xyz'][:2], OBSTACLE_XY) > .10
                    ):
                report['reason'] = 'obstacle_pose_missing_or_changed'
                break
            gate = health_gate(
                status=status, age_analysis=snap['analysis_age'],
                ages={name:snap['age_' + name]
                      for name in ('pose', 'imu', 'odom', 'scan')},
                contact=snap['contact'], speed=snap['speed'],
                roll_deg=roll, pitch_deg=pitch, distance=moved,
                yaw_change_deg=delta_yaw,
            )
            if gate != 'safe_to_probe':
                report['reason'] = gate
                break
            monitor.command(MAX_SPEED_MPS)
            if time.monotonic() - last_print > 1.0:
                print('travel=%.3f m measured=%.3f m/s | '
                      'obstacle=%.3f m | IMU objects=%d | %s' % (
                          moved, snap['speed'], row['imu']['nearest_m'],
                          row['imu']['objects'], gate), flush=True)
                last_print = time.monotonic()
            time.sleep(.10)
        else:
            report['reason'] = 'drive_wall_time_limit'
    except KeyboardInterrupt:
        report['reason'] = 'interrupted'
    except Exception as exc:
        report['reason'] = 'exception'
        report['detail'] = f'{type(exc).__name__}: {exc}'
    finally:
        # Zero commands even when perception/odometry checks fail.
        monitor.stop(25)
        confirmed, stop_samples = stop_confirmed(monitor)
        snap = monitor.snapshot()
        monitor.stop_event.set()
        if worker is not None:
            worker.join(timeout=3.0)
        with monitor.lock:
            rows = list(monitor.rows)
        if initial_xy is not None and snap['pose'] is not None:
            moved = bounded_distance(initial_xy, (
                snap['pose'].position.x, snap['pose'].position.y))
            signed_forward, lateral = route_displacement(
                initial_xy, (snap['pose'].position.x, snap['pose'].position.y),
                initial_yaw)
            max_lateral = max(max_lateral, abs(lateral))
        for row in rows:
            row['obstacle_status'] = obstacle_sample_status(row)
        issues = Counter(row['obstacle_status'] for row in rows
                         if row['obstacle_status'] != 'obstacle_confirmed')
        paired = [row for row in rows if row.get('paired')]
        moving = [row for row in paired
                  if initial_xy is not None and row.get('drive_commanded')
                  and row.get('odometry_speed_mps') is not None
                  and row['odometry_speed_mps'] >= .04
                  and bounded_distance(initial_xy, (row['x_m'], row['y_m'])) >= .05]
        gaps = [row['imu']['nearest_m'] for row in paired
                if row.get('imu') and row['imu'].get('nearest_m') is not None]
        moving_gaps = [row['imu']['nearest_m'] for row in moving
                       if row.get('imu') and row['imu'].get('nearest_m') is not None]
        gap_reduction = (initial_obstacle_gap - min(moving_gaps)
                         if initial_obstacle_gap is not None and moving_gaps else None)
        final_speed = snap['speed']
        passed = (
            stopped_due_to_distance
            and report['reason'] == 'distance_limit_reached'
            and MIN_PROGRESS_FOR_PASS_M <= signed_forward <= HARD_DISTANCE_LIMIT
            and max_lateral <= .18
            and confirmed and stop_samples >= 3
            and final_speed is not None and final_speed <= .04
            and len(moving) >= 3 and len(rows) >= 5
            and not issues and gaps and min(gaps) >= STOP_SENSOR_GAP_M
            and gap_reduction is not None and gap_reduction >= .12
            and not snap['contact'] and not snap['callback_errors']
            and not snap['worker_errors'] and snap['queue_dropped'] == 0
        )
        report.update(
            passed=bool(passed), final_displacement_m=round(moved, 4),
            final_forward_progress_m=round(signed_forward, 4),
            max_lateral_deviation_m=round(max_lateral, 4),
            final_speed_mps=None if final_speed is None else round(final_speed, 4),
            stop_confirmation_samples=stop_samples, stop_confirmed=confirmed,
            paired_scan_samples=len(paired), moving_paired_samples=len(moving),
            minimum_scanner_forward_obstacle_m=round(min(gaps), 4) if gaps else None,
            initial_scanner_forward_obstacle_m=initial_obstacle_gap,
            moving_obstacle_range_reduction_m=(round(gap_reduction, 4)
                                               if gap_reduction is not None else None),
            sample_status_issues=dict(issues),
            contact_observed=snap['contact'],
            counts={k: snap[k] for k in ('pose_count', 'imu_count',
                    'odom_count', 'scan_count', 'contact_messages',
                    'obstacle_pose_count', 'queue_dropped')},
            callback_errors=snap['callback_errors'],
            processing_errors=snap['worker_errors'],
            samples=rows,
            note=('Known static obstacle, short slow forward motion only. '
                  'LiDAR obstacle range is not body-to-obstacle clearance. '
                  'No autonomous braking, obstacle avoidance, measured 3D '
                  'separation, local pose estimate, independent command '
                  'watchdog, or real sensor clock synchronization.'),
        )
        target = args.report
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n',
                          encoding='utf-8')
        print('JARCRAC MOVING OBSTACLE PERCEPTION:',
              'PASS' if passed else 'FAIL', flush=True)
        print(json.dumps({k: v for k, v in report.items() if k != 'samples'},
                         indent=2, sort_keys=True), flush=True)
        print('Saved:', target, flush=True)
    if not report['passed']:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    prep = sub.add_parser('prepare', help='Generate isolated known-obstacle world')
    prep.add_argument('--output', type=Path, default=WORLD_PATH)
    drive = sub.add_parser('drive', help='Bounded 0.10 m/s moving perception trial')
    drive.add_argument('--every-scans', type=int, default=4)
    drive.add_argument('--report', type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    if args.action == 'prepare':
        print('Generated:', prepare_world(args.output))
    else:
        if not 2 <= args.every_scans <= 12:
            parser.error('--every-scans must be 2..12')
        run_trial(args)


if __name__ == '__main__':
    main()
