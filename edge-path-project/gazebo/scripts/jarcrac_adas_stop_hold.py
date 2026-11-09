#!/usr/bin/env python3
"""Experimental JARCRAC LiDAR-triggered STOP -> 5-s HOLD on Ruskotunturi.

Only validates a stop/hold transition in a controlled scene with a known STATIC
obstacle. Does NOT implement dynamic-object tracking, route planning, avoidance,
terrain AEB, calibrated braking, or an independent watchdog. Map-LiDAR uses
exact Gazebo XYZ; IMU/scan inputs are paired by wall reception time, not stamps.

Use only the generated isolated world, one Gazebo instance, one command writer.
A forcibly killed controller has no independent actuator-side timeout; pause
Gazebo if that happens. Do not use this experimental controller on real vehicles.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_imu_dynamic import Monitor, bounded_distance, prepare_world as prepare_dynamic_world, route_displacement, STALE_S
from jarcrac_imu_dynamic_obstacle import (
    SOURCES, MESH, KNOWN_OBSTACLE, OBSTACLE_XY, EXPECTED_START_XY,
    STOP_SENSOR_GAP_M, MAX_SPEED_MPS, MAX_DRIVE_DISTANCE,
    MAX_ANALYSIS_AGE_S, obstacle_sample_status, health_gate,
    stop_confirmed,
)
from jarcrac_imu_stage import q_to_rpy
from jarcrac_lidar_map_assisted import TerrainMeshIndex, quaternion_tuple

WORLD_NAME = 'jarcrac_adas_stop_hold'
WORLD_PATH = Path.home() / 'jarcrac_imu_experiment' / (WORLD_NAME + '.sdf')
POSE_TOPIC = '/world/' + WORLD_NAME + '/pose/info'
CLOCK_TOPIC = '/world/' + WORLD_NAME + '/clock'
CMD_TOPIC = '/model/jarcrac/adas_stop_hold_cmd_vel'
CONTACT_TOPIC = '/jarcrac/adas_stop_hold/chassis_contacts'
STOP_TRIGGER_RANGE_M = 3.70  # Early conservative trigger, sensor axial range only
STOP_HOLD_SIM_S = 5.0
MAX_HOLD_WALL_S = 45.0
MAX_CLOCK_AGE_WALL_S = 0.70
MAX_STOPPED_DRIFT_M = 0.08
MAX_HELD_RANGE_CHANGE_M = 0.12
MIN_TRIGGER_PROGRESS_M = 0.06
MIN_HOLD_DISTINCT_SCANS = 6
DEFAULT_REPORT = Path('/tmp/jarcrac_adas_stop_hold.json')


def prepare_world(output: Path = WORLD_PATH) -> Path:
    result = prepare_dynamic_world(
        output, source=SOURCES['obstacle'], world_name=WORLD_NAME,
        cmd_topic=CMD_TOPIC, contact_topic=CONTACT_TOPIC)
    world = ET.parse(result).getroot().find('world')
    if world is None or world.get('name') != WORLD_NAME:
        raise ValueError('Incorrect generated world name')
    vehicle = world.find("./model[@name='jarcrac']")
    obstacle = world.find(f"./model[@name='{KNOWN_OBSTACLE}']")
    if vehicle is None or obstacle is None or obstacle.findtext('static') != 'true':
        raise ValueError('Expected static known obstacle and JARCRAC')
    lidar = vehicle.find("./link[@name='chassis']/sensor[@name='gpu_lidar_front']")
    if (lidar is None or lidar.findtext('topic') != '/jarcrac/lidar'
            or lidar.findtext('ray/scan/horizontal/samples') != '721'
            or lidar.findtext('ray/scan/vertical/samples') != '32'):
        raise ValueError('Expected the actual 721 x 32 JARCRAC LiDAR')
    obstacle_xy = tuple(map(float, obstacle.findtext('pose').split()[:2]))
    if math.dist(obstacle_xy, OBSTACLE_XY) > .02:
        raise ValueError('Known obstacle moved')
    return result


def trigger_reason(*, gap_m: float, progress_m: float) -> str:
    """Return a reason: only a valid near obstacle triggers the planned stop."""
    if not (math.isfinite(gap_m) and math.isfinite(progress_m)):
        return 'invalid_measurement'
    if gap_m < STOP_SENSOR_GAP_M or progress_m < -.05:
        return 'unsafe_measurement'
    if progress_m >= MAX_DRIVE_DISTANCE:
        return 'distance_limit_without_obstacle_trigger'
    if gap_m <= STOP_TRIGGER_RANGE_M and progress_m >= MIN_TRIGGER_PROGRESS_M:
        return 'lidar_triggered_stop'
    return 'continue'


def stationary_consistency(*, current_xy, stop_xy, current_range_m,
                           stop_range_m, obstacle_xy, start_obstacle_xy):
    """Conservative proof for THIS static scene, not generic object tracking."""
    scalars = (*current_xy, *stop_xy, current_range_m, stop_range_m,
               *obstacle_xy, *start_obstacle_xy)
    if not all(math.isfinite(v) for v in scalars):
        return 'invalid_stationary_measurements'
    if bounded_distance(stop_xy, current_xy) > MAX_STOPPED_DRIFT_M:
        return 'vehicle_moved_during_hold'
    if math.dist(obstacle_xy, start_obstacle_xy) > .10:
        return 'known_obstacle_pose_changed'
    if abs(current_range_m - stop_range_m) > MAX_HELD_RANGE_CHANGE_M:
        return 'obstacle_range_changed_during_hold'
    return 'consistent'


class StopHoldMonitor(Monitor):
    def __init__(self, node, msgs, *, every_scans):
        super().__init__(node, msgs, every_scans=every_scans,
                         pose_topic=POSE_TOPIC, cmd_topic=CMD_TOPIC,
                         contact_topic=CONTACT_TOPIC)
        with self.lock:
            self.state.update(obstacle_xyz=None, obstacle_at=0.,
                              obstacle_pose_count=0, sim_time_s=None,
                              clock_at=0., clock_count=0)
        if not node.subscribe(msgs['Clock'], CLOCK_TOPIC, self.on_clock):
            raise RuntimeError('Cannot subscribe to clock: ' + CLOCK_TOPIC)

    def on_clock(self, msg):
        try:
            value = float(msg.sim.sec) + float(msg.sim.nsec) * 1e-9
            if not math.isfinite(value) or value < 0:
                raise ValueError('Invalid simulation clock')
            with self.lock:
                old = self.state['sim_time_s']
                if old is not None and value < old - 1e-4:
                    raise ValueError('Simulation time went backwards')
                self.state['sim_time_s'] = value
                self.state['clock_at'] = time.monotonic()
                self.state['clock_count'] += 1
        except Exception as exc:
            self._err('clock', exc)

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
        if (msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                or msg.count != 721 or msg.vertical_count != 32
                or len(msg.ranges) != 23072):
            self._err('lidar_shape', ValueError('Not the expected JARCRAC 721 x 32 scan'))
            return
        super().on_scan(msg)

    def snapshot(self):
        snap = super().snapshot()
        with self.lock:
            snap['clock_age'] = (time.monotonic() - self.state['clock_at']
                                 if self.state['clock_at'] else math.inf)
            snap['obstacle_age'] = (time.monotonic() - self.state['obstacle_at']
                                    if self.state['obstacle_at'] else math.inf)
        return snap


def sample_health(snap, *, initial_xy, initial_yaw):
    """Fail closed on missing, stale or invalid vehicle and perception inputs."""
    pose = snap['pose']
    if pose is None or snap['speed'] is None or snap['obstacle_xyz'] is None:
        return 'missing_vehicle_or_obstacle_state'
    if snap['sim_time_s'] is None or snap['clock_age'] > MAX_CLOCK_AGE_WALL_S:
        return 'stale_simulation_clock'
    if snap['obstacle_age'] > STALE_S:
        return 'stale_obstacle_pose'
    if snap['callback_errors'] or snap['worker_errors'] or snap['queue_dropped']:
        return 'processing_or_queue_error'
    if math.dist(snap['obstacle_xyz'][:2], OBSTACLE_XY) > .10:
        return 'unexpected_known_obstacle_location'
    progress_xy = (pose.position.x, pose.position.y)
    distance = bounded_distance(initial_xy, progress_xy)
    signed, lateral = route_displacement(initial_xy, progress_xy, initial_yaw)
    if not math.isfinite(signed) or abs(lateral) > .18 or signed < -.05:
        return 'route_deviation'
    roll, pitch, yaw = q_to_rpy(quaternion_tuple(pose.orientation))
    yaw_error = abs((yaw - initial_yaw + 180.) % 360. - 180.)
    return health_gate(
        status=obstacle_sample_status(snap['analysis']),
        age_analysis=snap['analysis_age'],
        ages={k: snap['age_' + k] for k in ('pose', 'imu', 'odom', 'scan')},
        contact=snap['contact'], speed=snap['speed'],
        roll_deg=roll, pitch_deg=pitch, distance=distance,
        yaw_change_deg=yaw_error,
    )


def run_trial(args):
    try:
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic system-Python bindings missing: ' + str(exc))
    monitor = StopHoldMonitor(Node(), dict(
        Clock=Clock, Contacts=Contacts, IMU=IMU, LaserScan=LaserScan,
        Odometry=Odometry, Pose_V=Pose_V, Twist=Twist,
    ), every_scans=args.every_scans)
    report = dict(
        test='jarcrac_lidar_triggered_stop_hold', world=WORLD_NAME,
        triggered_stop_not_calibrated_aeb=True,
        stop_trigger_scanner_gap_m=STOP_TRIGGER_RANGE_M,
        stop_hold_required_sim_s=STOP_HOLD_SIM_S,
        avoidance_enabled=False, independent_watchdog=False,
        terrain_aeb_validated=False, exact_gazebo_xyz_used=True,
        pairing_method='wall_receive_proximity_not_sim_timestamp_sync',
        passed=False, reason='not_started', phase='PREFLIGHT')
    worker = None
    initial_xy = None
    initial_yaw = None
    start_obstacle_xy = None
    initial_gap = None
    trigger_gap = None
    trigger_progress = None
    stop_xy = None
    stop_gap = None
    stop_clock = None
    stop_samples = 0
    observed_scan_ids = set()
    max_lateral = 0.
    actual_hold_sim_s = 0.
    try:
        print('JARCRAC LiDAR STOP -> 5 SIM-SECOND HOLD | 0.10 m/s', flush=True)
        print('World:', WORLD_NAME, '| command:', CMD_TOPIC, flush=True)
        monitor.stop(5)
        until = time.monotonic() + 20
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            if (snap['pose'] is not None and snap['imu'] is not None
                    and snap['speed'] is not None and snap['obstacle_xyz'] is not None
                    and snap['sim_time_s'] is not None):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Missing vehicle, IMU, odometry, obstacle or clock')
        pose = snap['pose']
        if (bounded_distance((pose.position.x, pose.position.y), EXPECTED_START_XY) > .6
                or math.dist(snap['obstacle_xyz'][:2], OBSTACLE_XY) > .10
                or snap['speed'] > .04 or snap['contact']
                or snap['clock_age'] > MAX_CLOCK_AGE_WALL_S):
            raise RuntimeError('Invalid initial scene or vehicle moving')
        initial_xy = (pose.position.x, pose.position.y)
        start_obstacle_xy = snap['obstacle_xyz'][:2]
        initial_yaw = q_to_rpy(quaternion_tuple(pose.orientation))[2]
        if abs((initial_yaw - math.degrees(-2.6779450446) + 180.) % 360. - 180.) > 15.:
            raise RuntimeError('Unexpected initial heading')
        terrain = TerrainMeshIndex(MESH, center_x=pose.position.x,
                                   center_y=pose.position.y, radius_m=24.)
        worker = threading.Thread(target=monitor.run_worker, args=(terrain,), daemon=True)
        worker.start()
        good_ids = set()
        until = time.monotonic() + 25.
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            status = obstacle_sample_status(snap['analysis'])
            if snap['callback_errors'] or snap['worker_errors'] or snap['contact']:
                raise RuntimeError('Unsafe preflight telemetry')
            if snap['analysis'] is not None and status != 'obstacle_confirmed':
                raise RuntimeError('Unsafe preflight LiDAR: ' + status)
            if status == 'obstacle_confirmed' and snap['analysis_age'] <= MAX_ANALYSIS_AGE_S:
                gap = snap['analysis']['imu']['nearest_m']
                if not 3.75 <= gap <= 4.10:
                    raise RuntimeError('Unexpected initial obstacle distance')
                good_ids.add(snap['analysis']['scan_number'])
                initial_gap = gap
            if len(good_ids) >= 3:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Not enough valid distinct 3D obstacle scans')
        print('PREFLIGHT PASS: known obstacle detected in >=3 paired 3D scans.', flush=True)

        report['phase'] = 'DRIVE'
        started = time.monotonic()
        while time.monotonic() - started < 12.:
            snap = monitor.snapshot()
            gate = sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw)
            if gate != 'safe_to_probe':
                report['reason'] = gate
                break
            p = snap['pose']
            xy = (p.position.x, p.position.y)
            progress, lateral = route_displacement(initial_xy, xy, initial_yaw)
            max_lateral = max(max_lateral, abs(lateral))
            gap = snap['analysis']['imu']['nearest_m']
            action = trigger_reason(gap_m=gap, progress_m=progress)
            if action == 'lidar_triggered_stop':
                trigger_gap, trigger_progress = gap, progress
                print('LiDAR STOP trigger | gap=%.3f m | progress=%.3f m'
                      % (gap, progress), flush=True)
                report['reason'] = action
                break
            if action != 'continue':
                report['reason'] = action
                break
            monitor.command(MAX_SPEED_MPS)
            time.sleep(.10)
        else:
            report['reason'] = 'drive_wall_time_limit'
        if report['reason'] != 'lidar_triggered_stop':
            raise RuntimeError('No valid LiDAR-triggered stop: ' + report['reason'])

        report['phase'] = 'BRAKE'
        monitor.stop(12)
        confirmed, stop_samples = stop_confirmed(monitor, deadline_wall_s=4.0)
        if not confirmed:
            raise RuntimeError('Stop not confirmed by distinct odometry updates')
        snap = monitor.snapshot()
        if sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw) != 'safe_to_probe':
            raise RuntimeError('Invalid telemetry after stop')
        stop_xy = (snap['pose'].position.x, snap['pose'].position.y)
        stop_gap = snap['analysis']['imu']['nearest_m']
        stop_clock = snap['sim_time_s']
        if not (stop_clock is not None and stop_gap >= STOP_SENSOR_GAP_M
                and snap['speed'] <= .04):
            raise RuntimeError('Stop state does not meet conservative limits')
        print('STOP confirmed | scanner gap=%.3f m | speed=%.3f m/s'
              % (stop_gap, snap['speed']), flush=True)

        report['phase'] = 'HOLD'
        wall_deadline = time.monotonic() + MAX_HOLD_WALL_S
        while time.monotonic() < wall_deadline:
            monitor.command(0)
            snap = monitor.snapshot()
            gate = sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw)
            if gate != 'safe_to_probe':
                raise RuntimeError('Unsafe HOLD telemetry: ' + gate)
            if snap['speed'] > .04:
                raise RuntimeError('JARCRAC moved during stopped hold')
            xy = (snap['pose'].position.x, snap['pose'].position.y)
            gap = snap['analysis']['imu']['nearest_m']
            consistency = stationary_consistency(
                current_xy=xy, stop_xy=stop_xy,
                current_range_m=gap, stop_range_m=stop_gap,
                obstacle_xy=snap['obstacle_xyz'][:2],
                start_obstacle_xy=start_obstacle_xy)
            if consistency != 'consistent':
                raise RuntimeError('Stationary evidence failed: ' + consistency)
            observed_scan_ids.add(snap['analysis']['scan_number'])
            actual_hold_sim_s = snap['sim_time_s'] - stop_clock
            if actual_hold_sim_s < -1e-4:
                raise RuntimeError('Simulation time regressed during hold')
            if (actual_hold_sim_s >= STOP_HOLD_SIM_S
                    and len(observed_scan_ids) >= MIN_HOLD_DISTINCT_SCANS):
                report['reason'] = 'five_sim_seconds_obstacle_confirmed'
                break
            time.sleep(.10)
        else:
            report['reason'] = 'hold_wall_timeout'
            raise RuntimeError('Stop hold did not finish within wall-time limit')
        print('HOLD PASS | sim time=%.3f s | distinct obstacle scans=%d'
              % (actual_hold_sim_s, len(observed_scan_ids)), flush=True)
        report['phase'] = 'DONE'
    except KeyboardInterrupt:
        report['reason'] = 'interrupted'
    except Exception as exc:
        if report['reason'] in ('not_started', 'lidar_triggered_stop'):
            report['reason'] = 'exception'
        report['detail'] = '%s: %s' % (type(exc).__name__, exc)
    finally:
        monitor.stop(25)
        stop_ok, final_stop_samples = stop_confirmed(monitor, deadline_wall_s=3.0)
        snap = monitor.snapshot()
        monitor.stop_event.set()
        if worker is not None:
            worker.join(timeout=3.)
        with monitor.lock:
            rows = list(monitor.rows)
        last_pose = snap['pose']
        progress = None
        if initial_xy is not None and last_pose is not None:
            progress = route_displacement(initial_xy,
                    (last_pose.position.x, last_pose.position.y), initial_yaw)[0]
        errors = [obstacle_sample_status(r) for r in rows]
        valid_rows = sum(v == 'obstacle_confirmed' for v in errors)
        is_pass = (
            report['phase'] == 'DONE'
            and report['reason'] == 'five_sim_seconds_obstacle_confirmed'
            and trigger_gap is not None and STOP_SENSOR_GAP_M <= trigger_gap <= STOP_TRIGGER_RANGE_M
            and initial_gap is not None and initial_gap - trigger_gap >= .08
            and trigger_progress is not None and trigger_progress >= MIN_TRIGGER_PROGRESS_M
            and progress is not None and MIN_TRIGGER_PROGRESS_M <= progress <= MAX_DRIVE_DISTANCE + .05
            and stop_gap is not None and stop_gap >= STOP_SENSOR_GAP_M
            and actual_hold_sim_s >= STOP_HOLD_SIM_S
            and len(observed_scan_ids) >= MIN_HOLD_DISTINCT_SCANS
            and stop_samples >= 3 and stop_ok and final_stop_samples >= 3
            and snap['speed'] is not None and snap['speed'] <= .04
            and valid_rows >= 6 and len(errors) == valid_rows
            and not snap['contact'] and not snap['callback_errors']
            and not snap['worker_errors'] and snap['queue_dropped'] == 0)
        report.update(passed=bool(is_pass),
            initial_sensor_gap_m=initial_gap,
            trigger_sensor_gap_m=trigger_gap,
            trigger_range_reduction_m=(round(initial_gap - trigger_gap, 4)
                if initial_gap is not None and trigger_gap is not None else None),
            trigger_progress_m=trigger_progress,
            stopped_sensor_gap_m=stop_gap,
            final_forward_progress_m=progress,
            stopped_hold_sim_s=round(actual_hold_sim_s, 4),
            held_distinct_scan_samples=len(observed_scan_ids),
            stop_confirmation_samples=stop_samples,
            final_stop_confirmation_samples=final_stop_samples,
            final_speed_mps=snap['speed'],
            contact_observed=snap['contact'],
            callback_errors=snap['callback_errors'],
            processing_errors=snap['worker_errors'],
            queue_dropped=snap['queue_dropped'],
            sample_counts={k: snap[k] for k in ('pose_count', 'imu_count',
                'odom_count', 'scan_count', 'contact_messages', 'clock_count')},
            samples=rows,
            note=('Controlled known-static-obstacle trial only; 5 s uses Gazebo sim clock. '
                  'No general moving-object tracking, true autonomous AEB, detour, '
                  'independent watchdog or certified body clearance.'))
        target = args.report
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n',
                          encoding='utf-8')
        print('JARCRAC ADAS STOP/HOLD:', 'PASS' if is_pass else 'FAIL', flush=True)
        print(json.dumps({k: v for k, v in report.items() if k != 'samples'},
                         indent=2, sort_keys=True), flush=True)
        print('Report:', target, flush=True)
    if not report['passed']:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('prepare', help='Generate isolated JARCRAC stop-hold world')
    p.add_argument('--output', type=Path, default=WORLD_PATH)
    d = sub.add_parser('drive', help='Conservative LiDAR stop and five sim-second hold')
    d.add_argument('--every-scans', type=int, default=4)
    d.add_argument('--report', type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    if args.action == 'prepare':
        print('Generated:', prepare_world(args.output))
    else:
        if not 2 <= args.every_scans <= 12:
            parser.error('--every-scans must be 2..12')
        run_trial(args)


if __name__ == '__main__':
    main()
