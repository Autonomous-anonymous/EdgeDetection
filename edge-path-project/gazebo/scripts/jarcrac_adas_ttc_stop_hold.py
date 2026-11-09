#!/usr/bin/env python3
"""JARCRAC TTC + stopping-budget BRAKE -> 5-s HOLD, controlled Gazebo trial.

Only the existing KNOWN STATIC Ruskotunturi obstacle is supported. Uses exact
Gazebo XYZ to classify terrain and check the obstacle's world pose, 32-layer
LiDAR, IMU attitude, odometry speed and the simulator clock. Closing speed is
approximated by odometry magnitude for this straight, static-target approach;
it is not general tracked-object relative TTC. The deliberately EARLY TTC
threshold proves the control path at <=0.35 m travel and is NOT AEB tuning.

No avoidance or independent watchdog. No calibrated real brake response or
verified 3-D separation. Keep only one Gazebo world/velocity publisher active.
Pause Gazebo if this experimental controller is forcibly terminated.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from jarcrac_adas_braking import BrakeConfig, braking_decision
from jarcrac_adas_stop_hold import (
    StopHoldMonitor, STOP_HOLD_SIM_S, MAX_HOLD_WALL_S, MAX_CLOCK_AGE_WALL_S,
    MAX_STOPPED_DRIFT_M, MAX_HELD_RANGE_CHANGE_M, MIN_HOLD_DISTINCT_SCANS,
    stationary_consistency, sample_health,
)
from jarcrac_imu_dynamic import (
    Monitor, bounded_distance, prepare_world as prepare_dynamic_world,
    route_displacement, STALE_S,
)
from jarcrac_imu_dynamic_obstacle import (
    SOURCES, MESH, KNOWN_OBSTACLE, OBSTACLE_XY, EXPECTED_START_XY,
    STOP_SENSOR_GAP_M, MAX_SPEED_MPS, MAX_DRIVE_DISTANCE,
    MAX_ANALYSIS_AGE_S, obstacle_sample_status, stop_confirmed,
)
from jarcrac_imu_stage import q_to_rpy
from jarcrac_lidar_map_assisted import TerrainMeshIndex, quaternion_tuple

WORLD_NAME = 'jarcrac_adas_ttc_stop_hold'
WORLD_PATH = Path.home() / 'jarcrac_imu_experiment' / (WORLD_NAME + '.sdf')
POSE_TOPIC = '/world/' + WORLD_NAME + '/pose/info'
CLOCK_TOPIC = '/world/' + WORLD_NAME + '/clock'
CMD_TOPIC = '/model/jarcrac/adas_ttc_cmd_vel'
CONTACT_TOPIC = '/jarcrac/adas_ttc/chassis_contacts'
DEFAULT_REPORT = Path('/tmp/jarcrac_adas_ttc_stop_hold.json')

# Deliberately large TTC for this 0.10 m/s and 0.35 m *instrumentation* trial:
# time-to-impact at preflight is ~35 seconds. Not a production/realistic AEB
# tuning recommendation; reduction requires independently validated dynamics.
TTC_EXPERIMENT_THRESHOLD_S = 32.7
BRAKE_CFG = BrakeConfig(ttc_threshold_s=TTC_EXPERIMENT_THRESHOLD_S)
MIN_TRIGGER_PROGRESS_M = 0.04  # evidence threshold, NEVER a gate delaying braking
MAX_DRIVE_WALL_S = 15.0
MAX_DISTANCE_AFTER_TRIGGER_M = 0.08


def prepare_world(output: Path = WORLD_PATH) -> Path:
    result = prepare_dynamic_world(output, source=SOURCES['obstacle'],
             world_name=WORLD_NAME, cmd_topic=CMD_TOPIC,
             contact_topic=CONTACT_TOPIC)
    world = ET.parse(result).getroot().find('world')
    if world is None or world.get('name') != WORLD_NAME:
        raise ValueError('Unexpected world name')
    model = world.find("./model[@name='jarcrac']")
    obstacle = world.find(f"./model[@name='{KNOWN_OBSTACLE}']")
    if model is None or obstacle is None or obstacle.findtext('static') != 'true':
        raise ValueError('Missing static known obstacle or JARCRAC')
    lidar = model.find("./link[@name='chassis']/sensor[@name='gpu_lidar_front']")
    if (lidar is None or lidar.findtext('topic') != '/jarcrac/lidar'
            or lidar.findtext('ray/scan/horizontal/samples') != '721'
            or lidar.findtext('ray/scan/vertical/samples') != '32'):
        raise ValueError('Expected 721 x 32 front LiDAR')
    if math.dist(tuple(map(float, obstacle.findtext('pose').split()[:2])),
                 OBSTACLE_XY) > .02:
        raise ValueError('Known obstacle pose differs')
    if model.findtext("./plugin[@name='gz::sim::systems::DiffDrive']/topic") != CMD_TOPIC:
        raise ValueError('Wrong command topic')
    return result


class TTCMonitor(StopHoldMonitor):
    """Reuse tested scan shape, static obstacle and simulation-clock callbacks."""

    def __init__(self, node, msgs, *, every_scans):
        # Base StopHoldMonitor has intentionally fixed topics. Reuse its
        # callbacks but register the new isolated topics via Monitor.
        Monitor.__init__(self, node, msgs, every_scans=every_scans,
                         pose_topic=POSE_TOPIC, cmd_topic=CMD_TOPIC,
                         contact_topic=CONTACT_TOPIC)
        with self.lock:
            self.state.update(obstacle_xyz=None, obstacle_at=0.,
                              obstacle_pose_count=0, sim_time_s=None,
                              clock_at=0., clock_count=0)
        if not node.subscribe(msgs['Clock'], CLOCK_TOPIC, self.on_clock):
            raise RuntimeError('Cannot subscribe to ' + CLOCK_TOPIC)


def decide_action(*, snap, progress_m, config=BRAKE_CFG):
    """Never delay a safety brake to meet experiment progress requirements."""
    if not math.isfinite(progress_m) or progress_m < -.05:
        return 'invalid_progress', None
    if progress_m > MAX_DRIVE_DISTANCE:
        return 'distance_limit_without_brake', None
    if snap['analysis'] is None or snap['speed'] is None:
        return 'missing_brake_input', None
    gap = snap['analysis']['imu']['nearest_m']
    # Only valid in this separately checked known-static, forward approach.
    decision = braking_decision(gap, snap['speed'], config)
    if decision.brake:
        return decision.reason, decision
    return 'continue', decision


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
    monitor = TTCMonitor(Node(), dict(
        Clock=Clock, Contacts=Contacts, IMU=IMU, LaserScan=LaserScan,
        Odometry=Odometry, Pose_V=Pose_V, Twist=Twist), every_scans=args.every_scans)
    report = dict(test='jarcrac_ttc_stopping_budget_stop_hold',
        world=WORLD_NAME, case='known_static_obstacle',
        passed=False, reason='not_started', phase='PREFLIGHT',
        ttc_threshold_s=BRAKE_CFG.ttc_threshold_s,
        minimum_clearance_proxy_m=BRAKE_CFG.minimum_clearance_m,
        scanner_uncertainty_m=BRAKE_CFG.scanner_uncertainty_m,
        assumed_deceleration_mps2=BRAKE_CFG.assumed_deceleration_mps2,
        reaction_time_s=BRAKE_CFG.reaction_time_s,
        stop_hold_required_sim_s=STOP_HOLD_SIM_S,
        approximate_static_target_closing_speed_from_odometry=True,
        terrain_aeb_calibrated=False, terrain_aeb_validated=False,
        avoidance_enabled=False, independent_watchdog=False,
        exact_gazebo_xyz_used=True,
        scan_imu_pairing='wall_receive_proximity_not_sim_timestamp_sync')
    worker = None
    initial_xy = initial_yaw = start_obstacle_xy = None
    initial_gap = None
    trigger = None
    trigger_progress = None
    stop_gap = stop_xy = stop_clock = None
    hold_elapsed = 0.0
    stop_samples = 0
    hold_scans = set()
    last_decision = None
    try:
        print('JARCRAC TTC + STOPPING DISTANCE -> 5 SIM-SECOND HOLD', flush=True)
        print('World:', WORLD_NAME, '| topic:', CMD_TOPIC, flush=True)
        print('Early experiment TTC threshold: %.1f s (NOT calibrated AEB tuning)' %
              BRAKE_CFG.ttc_threshold_s, flush=True)
        monitor.stop(5)
        until = time.monotonic() + 20.
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            if (snap['pose'] is not None and snap['imu'] is not None
                    and snap['speed'] is not None and snap['obstacle_xyz'] is not None
                    and snap['sim_time_s'] is not None):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Vehicle, IMU, odometry, obstacle or clock missing')
        p = snap['pose']
        if (bounded_distance((p.position.x,p.position.y), EXPECTED_START_XY) > .6
                or math.dist(snap['obstacle_xyz'][:2], OBSTACLE_XY) > .10
                or snap['speed'] > .04 or snap['contact']
                or snap['clock_age'] > MAX_CLOCK_AGE_WALL_S):
            raise RuntimeError('Unexpected initial scene or vehicle motion')
        initial_xy = (p.position.x, p.position.y)
        start_obstacle_xy = snap['obstacle_xyz'][:2]
        initial_yaw = q_to_rpy(quaternion_tuple(p.orientation))[2]
        if abs((initial_yaw - math.degrees(-2.6779450446)+180)%360-180) > 15.:
            raise RuntimeError('Unexpected start heading')
        terrain = TerrainMeshIndex(MESH, center_x=p.position.x,
                                   center_y=p.position.y, radius_m=24.)
        worker = threading.Thread(target=monitor.run_worker,
                                  args=(terrain,), daemon=True)
        worker.start()
        scan_ids = set()
        until = time.monotonic() + 25.
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            status = obstacle_sample_status(snap['analysis'])
            if snap['callback_errors'] or snap['worker_errors'] or snap['contact']:
                raise RuntimeError('Unsafe preflight telemetry')
            if snap['analysis'] is not None and status != 'obstacle_confirmed':
                raise RuntimeError('Invalid preflight classification: ' + status)
            if status == 'obstacle_confirmed' and snap['analysis_age'] <= MAX_ANALYSIS_AGE_S:
                gap = snap['analysis']['imu']['nearest_m']
                if not 3.75 <= gap <= 4.10:
                    raise RuntimeError('Unexpected initial obstacle range')
                scan_ids.add(snap['analysis']['scan_number'])
                initial_gap = gap
            if len(scan_ids) >= 3:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Insufficient paired obstacle detections')
        print('PREFLIGHT PASS: 3 distinct obstacle scans, stationary target.', flush=True)
        report['phase'] = 'DRIVE'
        begin = time.monotonic()
        while time.monotonic() - begin < MAX_DRIVE_WALL_S:
            snap = monitor.snapshot()
            gate = sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw)
            if gate != 'safe_to_probe':
                report['reason'] = gate
                break
            p = snap['pose']
            progress, lateral = route_displacement(initial_xy,
                (p.position.x, p.position.y), initial_yaw)
            action, decision = decide_action(snap=snap, progress_m=progress)
            if decision is not None:
                last_decision = decision
            if action != 'continue':
                report['reason'] = action
                if decision is not None and decision.brake:
                    trigger = decision
                    trigger_progress = progress
                    print('BRAKE: %s | progress=%.3f m | LiDAR=%.3f m | '
                          'proxy=%.3f m | TTC=%.2f s | required=%.3f m' %
                          (action, progress, decision.scanner_gap_m,
                           decision.conservative_gap_m, decision.ttc_s,
                           decision.required_clearance_m), flush=True)
                break
            monitor.command(MAX_SPEED_MPS)
            time.sleep(.10)
        else:
            report['reason'] = 'drive_wall_timeout'
        if trigger is None:
            raise RuntimeError('No AEB-policy-triggered stop: ' + report['reason'])
        report['phase'] = 'BRAKE'
        monitor.stop(12)
        confirmed, stop_samples = stop_confirmed(monitor, deadline_wall_s=4.)
        if not confirmed:
            raise RuntimeError('Stop not confirmed by distinct odometry samples')
        snap = monitor.snapshot()
        if sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw) != 'safe_to_probe':
            raise RuntimeError('Telemetry invalid after stop')
        stop_xy = (snap['pose'].position.x, snap['pose'].position.y)
        stop_gap = snap['analysis']['imu']['nearest_m']
        stop_clock = snap['sim_time_s']
        if not (stop_gap >= STOP_SENSOR_GAP_M and snap['speed'] <= .04
                and stop_clock is not None):
            raise RuntimeError('Stop clearance / odometry / clock failed')
        print('STOP confirmed | gap=%.3f m | speed=%.3f m/s' %
              (stop_gap,snap['speed']), flush=True)
        report['phase'] = 'HOLD'
        until = time.monotonic() + MAX_HOLD_WALL_S
        while time.monotonic() < until:
            monitor.command(0)
            snap = monitor.snapshot()
            gate = sample_health(snap, initial_xy=initial_xy, initial_yaw=initial_yaw)
            if gate != 'safe_to_probe':
                raise RuntimeError('Unsafe hold telemetry: ' + gate)
            if snap['speed'] > .04:
                raise RuntimeError('Unexpected motion during stopped hold')
            current_xy = (snap['pose'].position.x,snap['pose'].position.y)
            current_gap = snap['analysis']['imu']['nearest_m']
            state = stationary_consistency(
                current_xy=current_xy, stop_xy=stop_xy,
                current_range_m=current_gap, stop_range_m=stop_gap,
                obstacle_xy=snap['obstacle_xyz'][:2],
                start_obstacle_xy=start_obstacle_xy)
            if state != 'consistent':
                raise RuntimeError('Stationary evidence failed: ' + state)
            hold_scans.add(snap['analysis']['scan_number'])
            hold_elapsed = snap['sim_time_s'] - stop_clock
            if hold_elapsed < -1e-4:
                raise RuntimeError('Simulation clock regressed')
            if hold_elapsed >= STOP_HOLD_SIM_S and len(hold_scans) >= MIN_HOLD_DISTINCT_SCANS:
                report['reason'] = 'five_sim_seconds_obstacle_confirmed'
                report['phase'] = 'DONE'
                break
            time.sleep(.10)
        else:
            report['reason'] = 'hold_wall_timeout'
            raise RuntimeError('Hold exceeded bounded wall time')
        print('HOLD PASS: %.3f sim seconds, %d distinct scans' %
              (hold_elapsed,len(hold_scans)), flush=True)
    except KeyboardInterrupt:
        report['reason'] = 'interrupted'
    except Exception as exc:
        if report['reason'] in ('not_started', 'ttc_threshold',
                                'stopping_distance_budget', 'minimum_clearance'):
            report['reason'] = 'exception'
        report['detail'] = '%s: %s' % (type(exc).__name__, exc)
    finally:
        monitor.stop(25)
        final_stop_ok, final_stop_samples = stop_confirmed(monitor, deadline_wall_s=3.)
        snap = monitor.snapshot()
        monitor.stop_event.set()
        if worker is not None:
            worker.join(timeout=3.)
        with monitor.lock:
            rows = list(monitor.rows)
        final_progress = None
        if initial_xy is not None and snap['pose'] is not None:
            final_progress = route_displacement(initial_xy,
                (snap['pose'].position.x,snap['pose'].position.y), initial_yaw)[0]
        errors = [obstacle_sample_status(r) for r in rows]
        pass_trial = (report['phase'] == 'DONE'
            and report['reason'] == 'five_sim_seconds_obstacle_confirmed'
            and trigger is not None
            and trigger.reason in ('ttc_threshold', 'stopping_distance_budget',
                                   'minimum_clearance')
            and trigger_progress is not None
            and MIN_TRIGGER_PROGRESS_M <= trigger_progress <= MAX_DRIVE_DISTANCE
            and final_progress is not None
            and trigger_progress <= final_progress <= MAX_DRIVE_DISTANCE + .05
            and final_progress - trigger_progress <= MAX_DISTANCE_AFTER_TRIGGER_M
            and stop_gap is not None and stop_gap >= STOP_SENSOR_GAP_M
            and stop_samples >= 3 and final_stop_ok and final_stop_samples >= 3
            and hold_elapsed >= STOP_HOLD_SIM_S
            and len(hold_scans) >= MIN_HOLD_DISTINCT_SCANS
            and snap['speed'] is not None and snap['speed'] <= .04
            and not snap['contact'] and not snap['callback_errors']
            and not snap['worker_errors'] and snap['queue_dropped'] == 0
            and len(errors) >= 6 and all(e == 'obstacle_confirmed' for e in errors)
        )
        report.update(passed=bool(pass_trial),
            brake_reason=trigger.reason if trigger else None,
            trigger_progress_m=trigger_progress,
            trigger_sensor_gap_m=trigger.scanner_gap_m if trigger else None,
            trigger_conservative_gap_m=trigger.conservative_gap_m if trigger else None,
            trigger_ttc_s=(trigger.ttc_s if trigger and math.isfinite(trigger.ttc_s)
                           else None),
            trigger_stopping_distance_m=trigger.stopping_distance_m if trigger else None,
            trigger_required_clearance_m=trigger.required_clearance_m if trigger else None,
            initial_scanner_gap_m=initial_gap,
            stopped_scanner_gap_m=stop_gap,
            final_forward_progress_m=final_progress,
            final_speed_mps=snap['speed'],
            stopped_hold_sim_s=round(hold_elapsed,4),
            held_distinct_scan_samples=len(hold_scans),
            stop_confirmation_samples=stop_samples,
            final_stop_confirmation_samples=final_stop_samples,
            contact_observed=snap['contact'],
            processing_errors=snap['worker_errors'],
            callback_errors=snap['callback_errors'],
            queue_dropped=snap['queue_dropped'],
            counts={k:snap[k] for k in ('pose_count','imu_count','odom_count',
                        'scan_count','contact_messages','clock_count')},
            samples=rows,
            note=('EARLY TTC threshold is instrumentation-only for <=0.35 m at '
                  '0.10 m/s; not calibrated AEB tuning. Estimated scanner gap '
                  'uses an uncertainty allowance, not measured 3D body clearance. '
                  'This proves decision integration + stop/hold for known static '
                  'obstacle only; no dynamic obstacle tracking, route avoidance, '
                  'independent watchdog or real sensor-time synchronization.'))
        target = args.report
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',
                          encoding='utf-8')
        print('JARCRAC TTC/STOPPING-BUDGET STOP/HOLD:',
              'PASS' if pass_trial else 'FAIL', flush=True)
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},
                         indent=2, sort_keys=True),flush=True)
        print('Report:',target,flush=True)
    if not report['passed']:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action',required=True)
    p = sub.add_parser('prepare',help='Create isolated TTC+hold world')
    p.add_argument('--output',type=Path,default=WORLD_PATH)
    p = sub.add_parser('drive',help='Run low-speed TTC+stopping-distance hold')
    p.add_argument('--every-scans',type=int,default=4)
    p.add_argument('--report',type=Path,default=DEFAULT_REPORT)
    args = parser.parse_args()
    if args.action == 'prepare':
        print('Generated:',prepare_world(args.output))
    else:
        if not 2 <= args.every_scans <= 12:
            parser.error('--every-scans must be 2..12')
        run_trial(args)


if __name__ == '__main__':
    main()
