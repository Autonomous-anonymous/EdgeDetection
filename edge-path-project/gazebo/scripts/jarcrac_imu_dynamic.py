#!/usr/bin/env python3
"""Bounded dynamic JARCRAC IMU + 32-layer map-LiDAR SIL diagnostic.

prepare: copy the IMU-equipped CLEAR Ruskotunturi world to a new, isolated
         world. Keep the original model, AEB worlds and results untouched.
drive:   command at most 0.10 m/s for at most 0.35 m along the original
         heading, with fail-closed monitoring of perception, pose, odometry,
         IMU and chassis contact. Repeatedly command zero on exit.

This is NOT a terrain AEB or safety system. Gazebo ground-truth XYZ is still
used; wall-time reception proximity is NOT exact simulation-time alignment.
Never run a second JARCRAC world or cmd_vel publisher concurrently. If the
controller terminates unexpectedly, manually publish zero velocity or pause
Gazebo. Restart a fresh world before rerunning.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
from statistics import median
import threading
import time
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

from jarcrac_imu_stage import (
    MODEL, MESH, SOURCES, RESULTS, add_imu_world,
    correct_imu_orientation, frozen_pose, q_to_rpy, quat_error_deg,
)
from jarcrac_lidar_map_assisted import (
    TerrainMeshIndex, quaternion_tuple, scan_to_map_summary,
)

WORLD_NAME = 'jarcrac_imu_dynamic_clear'
WORLD_PATH = Path.home() / 'jarcrac_imu_experiment' / f'{WORLD_NAME}.sdf'
POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'
IMU_TOPIC = '/jarcrac/imu'
LIDAR_TOPIC = '/jarcrac/lidar'
ODOM_TOPIC = '/model/jarcrac/odometry'
CMD_TOPIC = '/model/jarcrac/dynamic_cmd_vel'  # isolated from original controller
CONTACT_TOPIC = '/jarcrac/dynamic/chassis_contacts'
MAX_SPEED = 0.10
MAX_DRIVE_DISTANCE = 0.35
HARD_DISTANCE_LIMIT = 0.65
MAX_DRIVE_WALL_S = 10.0
STALE_S = 0.55
PAIR_RECEPTION_S = 0.35  # NOT exact sensor timestamp synchronization
MAX_ANALYSIS_AGE_S = 2.5
MAX_UNKNOWN_FRACTION = 0.25
MAX_BODY_TILT_DEG = 15.0
MAX_ORIENTATION_ERROR_DEG = 2.0


def bounded_distance(start_xy, xy):
    return math.hypot(xy[0] - start_xy[0], xy[1] - start_xy[1])


def route_displacement(start_xy, xy, yaw_deg):
    """Signed forward progress and lateral deviation from starting heading."""
    dx, dy = xy[0]-start_xy[0], xy[1]-start_xy[1]
    th = math.radians(yaw_deg)
    return (math.cos(th)*dx+math.sin(th)*dy,
            -math.sin(th)*dx+math.cos(th)*dy)


def sample_status(row):
    """A valid but uncertain / obstacle-bearing return is never CLEAR."""
    if not row.get('paired'):
        return row.get('reason', 'unpaired')
    if row['orientation_error_deg'] > MAX_ORIENTATION_ERROR_DEG:
        return 'orientation_mismatch'
    if not row['exact']['valid'] or not row['imu']['valid']:
        return 'invalid_map_classification'
    if row['exact']['objects'] or row['imu']['objects']:
        return 'unexpected_obstacle'
    if (row['exact']['unknown_fraction'] > MAX_UNKNOWN_FRACTION or
            row['imu']['unknown_fraction'] > MAX_UNKNOWN_FRACTION):
        return 'unknown_terrain_excess'
    return 'clear'


def safe_to_command(*, age_pose, age_imu, age_odom, age_scan, analysis_age,
                    analysis_status, contact, speed, roll_deg, pitch_deg,
                    distance, yaw_change_deg):
    if contact:
        return 'chassis_contact'
    if any(not math.isfinite(v) for v in (
            age_pose, age_imu, age_odom, age_scan, analysis_age,
            speed, roll_deg, pitch_deg, distance, yaw_change_deg)):
        return 'invalid_telemetry'
    if any(v < 0 for v in (age_pose, age_imu, age_odom, age_scan,
                            analysis_age, speed, distance, yaw_change_deg)):
        return 'invalid_telemetry'
    if max(age_pose, age_imu, age_odom, age_scan) > STALE_S:
        return 'stale_sensor'
    if analysis_age > MAX_ANALYSIS_AGE_S:
        return 'stale_analysis'
    if analysis_status != 'clear':
        return analysis_status
    if abs(roll_deg) > MAX_BODY_TILT_DEG or abs(pitch_deg) > MAX_BODY_TILT_DEG:
        return 'excessive_tilt'
    if abs(yaw_change_deg) > 12.0:
        return 'unexpected_yaw_change'
    if distance > HARD_DISTANCE_LIMIT:
        return 'distance_hard_limit'
    if speed > 0.25:
        return 'excessive_measured_speed'
    return 'clear'


def prepare_world(output=WORLD_PATH, *, source=SOURCES['clear'],
                  world_name=WORLD_NAME, cmd_topic=CMD_TOPIC,
                  contact_topic=CONTACT_TOPIC):
    """Embed original model + IMU and add a collision sensor and isolated cmd."""
    add_imu_world(source, MODEL, output, world_name)
    tree = ET.parse(output)
    world = tree.getroot().find('world')
    model = world.find("./model[@name='jarcrac']")
    chassis = model.find("./link[@name='chassis']")
    diff = model.find("./plugin[@name='gz::sim::systems::DiffDrive']")
    if chassis is None or diff is None:
        raise ValueError('JARCRAC chassis or DiffDrive plugin missing')
    if diff.findtext('topic') != '/model/jarcrac/cmd_vel':
        raise ValueError('Unexpected original cmd topic; refusing to patch')
    diff.find('topic').text = cmd_topic
    if world.find("./plugin[@name='gz::sim::systems::Contact']") is not None:
        raise ValueError('Unexpected existing Contact plugin')
    ET.SubElement(world, 'plugin', filename='gz-sim-contact-system',
                  name='gz::sim::systems::Contact')
    sensor = ET.SubElement(chassis, 'sensor', name='dynamic_chassis_contact',
                           type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '20'
    ET.SubElement(sensor, 'topic').text = contact_topic
    ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = 'body_collision'
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def short_summary(s):
    return {'valid': bool(s.valid), 'reason': s.reason,
            'ground': int(s.ground_points), 'objects': int(s.obstacle_points),
            'unknown': int(s.unknown_points),
            'unknown_fraction': round(s.unknown_fraction, 5),
            'nearest_m': None if s.nearest_forward_obstacle_m is None else round(
                s.nearest_forward_obstacle_m, 4)}


class Monitor:
    def __init__(self, node, msgs, *, every_scans=4,
                 pose_topic=POSE_TOPIC, cmd_topic=CMD_TOPIC,
                 contact_topic=CONTACT_TOPIC):
        self.node = node
        self.lock = threading.RLock()
        self.q = Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.every_scans = every_scans
        self.state = {
            'pose': None, 'imu': None, 'speed': None, 'contact': False,
            'last_commanded_speed': 0.0,
            'pose_at': 0., 'imu_at': 0., 'odom_at': 0., 'scan_at': 0.,
            'contact_messages': 0, 'contacts_seen': 0,
            'pose_count': 0, 'imu_count': 0, 'odom_count': 0,
            'scan_count': 0, 'queue_dropped': 0,
            'callback_errors': [],
        }
        self.rows = []
        self.last_result = None
        self.last_result_at = 0.0
        self.worker_errors = []
        self.publisher = node.advertise(cmd_topic, msgs['Twist'])
        if self.publisher is None:
            raise RuntimeError('Cannot advertise isolated command topic')
        self.twist_type = msgs['Twist']
        for typ, topic, cb in (
            (msgs['Pose_V'], pose_topic, self.on_pose),
            (msgs['IMU'], IMU_TOPIC, self.on_imu),
            (msgs['Odometry'], ODOM_TOPIC, self.on_odom),
            (msgs['LaserScan'], LIDAR_TOPIC, self.on_scan),
            (msgs['Contacts'], contact_topic, self.on_contact),
        ):
            if not node.subscribe(typ, topic, cb):
                raise RuntimeError(f'Failed to subscribe to {topic}')

    def _err(self, cb, exc):
        with self.lock:
            if len(self.state['callback_errors']) < 5:
                self.state['callback_errors'].append(
                    f'{cb}: {type(exc).__name__}: {exc}')

    def on_pose(self, msg):
        try:
            for p in msg.pose:
                if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
                    with self.lock:
                        self.state['pose'] = frozen_pose(p)
                        self.state['pose_at'] = time.monotonic()
                        self.state['pose_count'] += 1
                    return
        except Exception as exc:
            self._err('pose', exc)

    def on_imu(self, msg):
        try:
            q = quaternion_tuple(msg.orientation)
            with self.lock:
                self.state['imu'] = q
                self.state['imu_at'] = time.monotonic()
                self.state['imu_count'] += 1
        except Exception as exc:
            self._err('imu', exc)

    def on_odom(self, msg):
        try:
            v = msg.twist.linear
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            if not math.isfinite(speed):
                raise ValueError('nonfinite odometry')
            with self.lock:
                self.state['speed'] = speed
                self.state['odom_at'] = time.monotonic()
                self.state['odom_count'] += 1
        except Exception as exc:
            self._err('odometry', exc)

    def on_contact(self, msg):
        try:
            count = len(msg.contact)
            with self.lock:
                self.state['contact_messages'] += 1
                self.state['contacts_seen'] += count
                if count:
                    self.state['contact'] = True
        except Exception as exc:
            self._err('contacts', exc)

    def on_scan(self, msg):
        try:
            now = time.monotonic()
            with self.lock:
                self.state['scan_at'] = now
                self.state['scan_count'] += 1
                n = self.state['scan_count']
                if n % self.every_scans:
                    return
                p, imu = self.state['pose'], self.state['imu']
                stamp = (self.state['pose_at'], self.state['imu_at'], now)
                cmd_speed = self.state['last_commanded_speed']
                odom_speed = self.state['speed']
            try:
                self.q.put_nowait((n, msg, p, imu, stamp, cmd_speed, odom_speed))
            except Full:
                with self.lock:
                    self.state['queue_dropped'] += 1
        except Exception as exc:
            self._err('lidar', exc)

    def command(self, velocity):
        with self.lock:
            self.state['last_commanded_speed'] = float(velocity)
        m = self.twist_type()
        m.linear.x = float(velocity)
        m.angular.z = 0.0
        self.publisher.publish(m)

    def stop(self, iterations=15):
        for _ in range(iterations):
            self.command(0.0)
            time.sleep(0.075)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            d = dict(self.state)
            d['callback_errors'] = list(self.state['callback_errors'])
            d['analysis'] = self.last_result
            d['analysis_at'] = self.last_result_at
            d['worker_errors'] = list(self.worker_errors)
        for name in ('pose','imu','odom','scan'):
            d['age_'+name] = (now - d[name+'_at'] if d[name+'_at'] else math.inf)
        d['analysis_age'] = (now-d['analysis_at'] if d['analysis_at'] else math.inf)
        return d

    def run_worker(self, index):
        while not self.stop_event.is_set():
            try:
                n, scan, p, imu_q, stamps, cmd_speed, odom_speed = self.q.get(timeout=.25)
            except Empty:
                continue
            p_at, imu_at, scan_at = stamps
            row = {'scan_number': n, 'paired': False,
                   'drive_commanded':cmd_speed>0,
                   'odometry_speed_mps':(round(odom_speed,4) if odom_speed is not None else None)}
            try:
                skew = abs(p_at-imu_at)
                age_at_scan = max(scan_at-p_at, scan_at-imu_at)
                row.update(pose_imu_reception_skew_s=round(skew, 4),
                           maximum_reception_age_s=round(age_at_scan, 4))
                if (p is None or imu_q is None or p_at <= 0 or imu_at <= 0
                        or age_at_scan > STALE_S or skew > PAIR_RECEPTION_S):
                    row['reason'] = 'unpaired_or_stale_inputs'
                else:
                    q_ref = quaternion_tuple(p.orientation)
                    q_imu = correct_imu_orientation(imu_q, (0.,0.,0.))
                    imu_pose = NS(position=p.position, orientation=NS(
                        x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3]))
                    exact = scan_to_map_summary(scan, p, index)
                    inferred = scan_to_map_summary(scan, imu_pose, index)
                    roll,pitch,yaw = q_to_rpy(q_ref)
                    row.update(paired=True,
                               x_m=round(p.position.x, 4),
                               y_m=round(p.position.y, 4),
                               z_m=round(p.position.z, 4),
                               roll_deg=round(roll, 4),
                               pitch_deg=round(pitch, 4),
                               yaw_deg=round(yaw, 4),
                               orientation_error_deg=round(
                                   quat_error_deg(q_ref,q_imu), 4),
                               exact=short_summary(exact),
                               imu=short_summary(inferred))
                    row['reason'] = sample_status(row)
            except Exception as exc:
                row['reason'] = 'processing_error'
                with self.lock:
                    if len(self.worker_errors) < 5:
                        self.worker_errors.append(
                            f'scan {n}: {type(exc).__name__}: {exc}')
            with self.lock:
                self.rows.append(row)
                self.last_result = row
                self.last_result_at = time.monotonic()


def run_drive(args):
    try:
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic Python bindings missing: '+str(exc))
    msgs = dict(Contacts=Contacts, IMU=IMU, LaserScan=LaserScan,
                Odometry=Odometry, Pose_V=Pose_V, Twist=Twist)
    monitor = Monitor(Node(), msgs, every_scans=args.every_scans)
    report = {'test':'jarcrac_dynamic_imu_map_lidar',
              'case':'dynamic_clear', 'world':WORLD_NAME,
              'experimental_terrain_motion':True,
              'terrain_aeb_validated':False,
              'exact_gazebo_xyz_used':True,
              'pairing_method':'wall-time receive proximity, NOT simulation timestamp alignment',
              'speed_command_mps':MAX_SPEED,
              'command_distance_limit_m':MAX_DRIVE_DISTANCE,
              'hard_distance_limit_m':HARD_DISTANCE_LIMIT,
              'passed':False, 'reason':'not_started'}
    worker = None
    initial_pose = None
    initial_yaw_deg = None
    moved = 0.0
    max_angular_error = 0.0
    drive_start = None
    stopped_due_to_distance = False
    abort_reason = None
    maximum_lateral_deviation = 0.0
    signed_forward = 0.0
    try:
        print('DYNAMIC JARCRAC IMU + MAP-LIDAR | capped speed %.2f m/s, displacement %.2f m'
              % (MAX_SPEED,MAX_DRIVE_DISTANCE),flush=True)
        print('Topics:',POSE_TOPIC,IMU_TOPIC,LIDAR_TOPIC,ODOM_TOPIC,
              CONTACT_TOPIC,CMD_TOPIC,flush=True)
        monitor.stop(5)
        until = time.monotonic()+20.
        while time.monotonic()<until:
            snap = monitor.snapshot()
            if snap['pose'] is not None and snap['imu'] is not None and snap['speed'] is not None:
                break
            monitor.command(0)
            time.sleep(.10)
        else:
            raise RuntimeError('Sensors/pose/odometry not ready; wrong world?')
        p = snap['pose']
        if (math.hypot(p.position.x-20.,p.position.y-40.) > 0.6
                or snap['speed']>0.04 or snap['contact']):
            raise RuntimeError('Unexpected start position, speed or collision; restart world')
        initial_pose = (p.position.x,p.position.y)
        _,_,initial_yaw_deg = q_to_rpy(quaternion_tuple(p.orientation))
        print('Loading map near',initial_pose,flush=True)
        index = TerrainMeshIndex(MESH,center_x=p.position.x,
                                 center_y=p.position.y,radius_m=24.)
        print('Terrain triangles:',index.triangle_count_in_roi,flush=True)
        worker = threading.Thread(target=monitor.run_worker,args=(index,),daemon=True)
        worker.start()
        # Read-only readiness: 3 distinct recent scan classifications must be clear.
        good_scan_ids = set()
        until = time.monotonic()+20.
        while time.monotonic()<until:
            monitor.command(0)
            snap = monitor.snapshot()
            if snap['callback_errors'] or snap['worker_errors'] or snap['contact']:
                raise RuntimeError('Callback, processing error or initial contact')
            row = snap['analysis']
            if (row and row['reason']=='clear' and
                    snap['analysis_age']<=MAX_ANALYSIS_AGE_S):
                good_scan_ids.add(row['scan_number'])
            elif row and row['reason']!='clear':
                raise RuntimeError('Unsafe readiness scan: '+row['reason'])
            if len(good_scan_ids)>=3:
                break
            time.sleep(.10)
        else:
            raise RuntimeError('Not enough clear paired LiDAR/IMU samples at startup')
        snap = monitor.snapshot()
        if max(snap['age_pose'],snap['age_imu'],snap['age_odom'],snap['age_scan'])>STALE_S:
            raise RuntimeError('Stale telemetry before motion')
        if abs(snap['analysis']['roll_deg'])>MAX_BODY_TILT_DEG or abs(snap['analysis']['pitch_deg'])>MAX_BODY_TILT_DEG:
            raise RuntimeError('Excessive startup tilt')
        print('Preflight OK: 3 clear paired scans. Starting bounded motion.',flush=True)
        drive_start=time.monotonic()
        last_info=0.
        while time.monotonic()-drive_start < MAX_DRIVE_WALL_S:
            snap = monitor.snapshot()
            p=snap['pose']
            if p is None or snap['speed'] is None:
                abort_reason='missing_pose_or_odometry'
                break
            moved=bounded_distance(initial_pose,(p.position.x,p.position.y))
            signed_forward, lateral = route_displacement(
                initial_pose,(p.position.x,p.position.y),initial_yaw_deg)
            maximum_lateral_deviation = max(maximum_lateral_deviation,abs(lateral))
            rpy=q_to_rpy(quaternion_tuple(p.orientation))
            yaw_delta=abs((rpy[2]-initial_yaw_deg+180.)%360.-180.)
            if abs(lateral)>0.18 or signed_forward < -0.05:
                abort_reason='route_deviation'
                break
            if moved>=MAX_DRIVE_DISTANCE:
                stopped_due_to_distance=True
                break
            analysis=snap['analysis']
            state=safe_to_command(age_pose=snap['age_pose'],age_imu=snap['age_imu'],
                   age_odom=snap['age_odom'],age_scan=snap['age_scan'],
                   analysis_age=snap['analysis_age'],
                   analysis_status=analysis['reason'] if analysis else 'no_analysis',
                   contact=snap['contact'],speed=snap['speed'],
                   roll_deg=rpy[0],pitch_deg=rpy[1],distance=moved,
                   yaw_change_deg=yaw_delta)
            if snap['callback_errors'] or snap['worker_errors']:
                state='callback_or_processing_error'
            if state!='clear':
                abort_reason=state
                break
            monitor.command(MAX_SPEED)
            if time.monotonic()-last_info>1.0:
                print('distance=%.3f m speed=%.3f | pitch=%.2f roll=%.2f | '
                      'IMU error=%.3f deg objects=%d unknown=%.1f%%'
                      % (moved,snap['speed'],rpy[1],rpy[0],
                         analysis['orientation_error_deg'],
                         analysis['imu']['objects'],
                         analysis['imu']['unknown_fraction']*100),flush=True)
                last_info=time.monotonic()
            time.sleep(.10)
        else:
            abort_reason='drive_wall_time_limit'
        report['reason'] = ('distance_limit_reached' if stopped_due_to_distance
                            else abort_reason or 'drive_stopped')
    except KeyboardInterrupt:
        report['reason']='interrupted'
    except Exception as exc:
        report['reason']='exception'
        report['detail']=f'{type(exc).__name__}: {exc}'
    finally:
        monitor.stop(25)  # explicit zero commands for ~1.9 seconds
        snap=monitor.snapshot()
        monitor.stop_event.set()
        if worker is not None:
            worker.join(timeout=3.0)
        with monitor.lock:
            worker_rows=list(monitor.rows)
        if snap['pose'] is not None and initial_pose is not None:
            moved=bounded_distance(initial_pose,
                       (snap['pose'].position.x,snap['pose'].position.y))
        else:
            moved=0.
        if snap['pose'] is not None and initial_pose is not None:
            signed_forward, lateral = route_displacement(
                initial_pose,(snap['pose'].position.x,snap['pose'].position.y),
                initial_yaw_deg)
            maximum_lateral_deviation = max(maximum_lateral_deviation,abs(lateral))
        paired=[r for r in worker_rows if r.get('paired')]
        moving=[r for r in paired if initial_pose is not None and
                r['drive_commanded'] and r['odometry_speed_mps'] is not None
                and r['odometry_speed_mps']>=0.04 and
                bounded_distance(initial_pose,(r['x_m'],r['y_m']))>=0.05]
        issues=Counter(r['reason'] for r in worker_rows if r['reason']!='clear')
        max_ori=max((r['orientation_error_deg'] for r in paired),default=None)
        pitch_span=(max(r['pitch_deg'] for r in moving)-min(r['pitch_deg'] for r in moving)
                    if len(moving)>1 else 0.)
        roll_span=(max(r['roll_deg'] for r in moving)-min(r['roll_deg'] for r in moving)
                   if len(moving)>1 else 0.)
        final_speed=snap['speed']
        # Intentional strictness: any sample invalid/obstacle/unpaired prevents PASS.
        passed=(stopped_due_to_distance and report['reason']=='distance_limit_reached'
                and 0.25<=signed_forward<=HARD_DISTANCE_LIMIT
                and maximum_lateral_deviation<=0.18
                and final_speed is not None and final_speed<=0.04
                and len(moving)>=3 and len(worker_rows)>=5
                and not issues and max_ori is not None
                and max_ori<=MAX_ORIENTATION_ERROR_DEG
                and not snap['contact'] and not snap['callback_errors']
                and not snap['worker_errors'] and snap['queue_dropped']==0)
        report.update(passed=bool(passed),
             final_displacement_m=round(moved,4),
             final_signed_forward_m=round(signed_forward,4),
             maximum_lateral_deviation_m=round(maximum_lateral_deviation,4),
             final_speed_mps=None if final_speed is None else round(final_speed,4),
             paired_scan_samples=len(paired),moving_paired_scan_samples=len(moving),
             all_sample_status=dict(issues),
             max_orientation_error_deg=max_ori,
             moving_pitch_span_deg=round(pitch_span,4),
             moving_roll_span_deg=round(roll_span,4),
             contact_observed=snap['contact'],
             contact_reports=snap['contacts_seen'],
             sensor_counts={k:snap[k] for k in ('pose_count','imu_count','odom_count',
                         'scan_count','contact_messages','queue_dropped')},
             callback_errors=snap['callback_errors'],
             processing_errors=snap['worker_errors'],
             samples=worker_rows,
             note=('Limited-speed Gazebo terrain MOTION only; no map-localization estimate, '
                   'no physically calibrated braking, not real LiDAR/IMU clock sync or terrain AEB.'))
        target=args.report or (RESULTS/'imu_dynamic_clear_diagnostic.json')
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
        print('JARCRAC DYNAMIC PERCEPTION:', 'PASS' if passed else 'FAIL',flush=True)
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},
                         indent=2,sort_keys=True),flush=True)
        print('Report:',target,flush=True)
    if not report['passed']:
        raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare',help='Generate isolated dynamic IMU world outside repo')
    p.add_argument('--output',type=Path,default=WORLD_PATH)
    p=sub.add_parser('drive',help='Slow, bounded dynamic experiment; NOT terrain AEB')
    p.add_argument('--every-scans',type=int,default=4)
    p.add_argument('--report',type=Path)
    args=parser.parse_args()
    if args.action=='prepare':
        print('Generated:',prepare_world(args.output))
    elif args.action=='drive':
        if not 2<=args.every_scans<=12:
            parser.error('every-scans must be 2..12')
        run_drive(args)


if __name__=='__main__':
    main()
