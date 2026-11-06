#!/usr/bin/env python3
"""JARCRAC read-only IMU + map-LiDAR calibration experiment, Gazebo Harmonic.

prepare: generate separate worlds outside the repo, by embedding the EXISTING
JARCRAC model and adding an IMU. No original SDFs or controller are modified.
probe: compare IMU attitude with Gazebo model pose, then classify exactly the
same 32-layer scan using (a) ground-truth orientation and (b) IMU orientation
with ground-truth translation. All comparisons are stationary SIL only.

Caveats: this does not estimate position/Z, align clocks by Gazebo sim time,
validate real IMU noise, establish safe terrain AEB, or command any vehicle.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
from statistics import median
import threading
import time
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

from jarcrac_lidar_map_assisted import (
    TerrainMeshIndex, quaternion_tuple, scan_to_map_summary,
)

ROOT = Path(__file__).resolve().parents[3]
GZ = ROOT / 'edge-path-project' / 'gazebo'
MODEL = GZ / 'models/jarcrac/model.sdf'
MESH = GZ / 'models/ruskotunturi_map/meshes/ruskotunturi-map-collision.stl'
EXPERIMENT_DIR = Path.home() / 'jarcrac_imu_experiment'
RESULTS = GZ / 'results/jarcrac'
IMU_TOPIC = '/jarcrac/imu'
LIDAR_TOPIC = '/jarcrac/lidar'
SOURCES = {
    'clear': GZ / 'worlds/jarcrac_terrain_probe_world.sdf',
    'obstacle': GZ / 'worlds/jarcrac_terrain_obstacle_probe_world.sdf',
}
WORLDS = {
    'clear': 'jarcrac_imu_clear',
    'obstacle': 'jarcrac_imu_obstacle',
    'mount_tilt': 'jarcrac_imu_mount_tilt',
    'body_tilt': 'jarcrac_imu_body_tilt',
}
MOUNT_TILT_DEG = (5.0, 2.0, 0.0)  # roll/pitch/yaw in sensor's body frame


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rpy(roll_deg, pitch_deg, yaw_deg):
    r, p, y = (math.radians(t) / 2 for t in (roll_deg, pitch_deg, yaw_deg))
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def q_to_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    s = max(-1.0, min(1.0, 2*(w*y-z*x)))
    pitch = math.asin(s)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def wrap_deg(d):
    return (d+180) % 360 - 180


def quat_error_deg(a, b):
    # q and -q represent identical rotations.
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return math.degrees(2*math.acos(min(1.0, max(0.0, dot))))


def correct_imu_orientation(imu_quat, mount_rpy_deg):
    # IMU frame is chassis frame * fixed sensor extrinsic.
    mount = q_rpy(*mount_rpy_deg)
    corrected = qmul(imu_quat, qconj(mount))
    length = math.sqrt(sum(x*x for x in corrected))
    return tuple(v/length for v in corrected)


def add_imu_world(source: Path, model_file: Path, output: Path,
                  world_name: str, *, mount_rpy_deg=(0., 0., 0.)):
    if not source.is_file() or not model_file.is_file():
        raise FileNotFoundError(f'Missing world or model: {source} / {model_file}')
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('SDF world not found')
    existing = [item for item in world.findall('include')
                if item.findtext('uri') == 'model://jarcrac']
    if len(existing) != 1:
        raise ValueError('Expected precisely one JARCRAC model://jarcrac include')
    vehicle_include = existing[0]
    if vehicle_include.findtext('name') != 'jarcrac':
        raise ValueError('Unexpected vehicle name')
    location = vehicle_include.findtext('pose')
    if location is None or len(location.split()) != 6:
        raise ValueError('Missing six-component spawn pose')
    root = ET.parse(model_file).getroot()
    loaded = root.find('model')
    if loaded is None or loaded.get('name') != 'jarcrac':
        raise ValueError('Original JARCRAC model missing')
    model = deepcopy(loaded)
    chassis = model.find("./link[@name='chassis']")
    if chassis is None or chassis.find("sensor[@type='gpu_lidar']") is None:
        raise ValueError('Expected chassis and original 3D LiDAR')
    if chassis.find("sensor[@type='imu']") is not None:
        raise ValueError('Original model already has an IMU; investigate before adding')
    imu = ET.SubElement(chassis, 'sensor', {'name':'imu_chassis', 'type':'imu'})
    ET.SubElement(imu, 'pose').text = ('0 0 0 ' + ' '.join(
        f'{math.radians(deg):.12f}' for deg in mount_rpy_deg))
    ET.SubElement(imu, 'topic').text = IMU_TOPIC
    ET.SubElement(imu, 'always_on').text = 'true'
    ET.SubElement(imu, 'update_rate').text = '100'
    settings = ET.SubElement(imu, 'imu')
    reference = ET.SubElement(settings, 'orientation_reference_frame')
    ET.SubElement(reference, 'localization').text = 'ENU'
    # Without explicit ENU, Gazebo's default IMU may use a startup reference.
    if world.find("./plugin[@name='gz::sim::systems::Imu']") is not None:
        raise ValueError('Source world already loads Imu system; check configuration')
    ET.SubElement(world, 'plugin', {
        'filename': 'gz-sim-imu-system', 'name': 'gz::sim::systems::Imu',
    })
    ET.SubElement(model, 'pose').text = location
    idx = list(world).index(vehicle_include)
    world.remove(vehicle_include)
    world.insert(idx, model)
    world.set('name', world_name)
    ET.indent(tree, space='  ')
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def prepare(args):
    for case in ('clear', 'obstacle', 'mount_tilt', 'body_tilt'):
        source = SOURCES['obstacle' if case == 'obstacle' else 'clear']
        mount = MOUNT_TILT_DEG if case == 'mount_tilt' else (0., 0., 0.)
        target = args.output_dir / f'{WORLDS[case]}.sdf'
        add_imu_world(source, MODEL, target, WORLDS[case], mount_rpy_deg=mount)
        if case == 'body_tilt':
            # Isolate orientation response, not vehicle terrain dynamics.
            # A gravity-free vehicle 5 m above the map is rotated +3 deg roll
            # and -5 deg pitch, so it cannot collide with the map while tilted.
            tree = ET.parse(target)
            world = tree.getroot().find('world')
            model = world.find("./model[@name='jarcrac']")
            pose = list(map(float,model.findtext('pose').split()))
            pose[2] += 5.0
            pose[3] = math.radians(3.0)
            pose[4] = math.radians(-5.0)
            model.find('pose').text = ' '.join(f'{v:.12g}' for v in pose)
            ET.SubElement(world,'gravity').text = '0 0 0'
            ET.indent(tree,space='  ')
            tree.write(target,encoding='utf-8',xml_declaration=True)
        print(f'Generated {case}: {target}')
    print('Test worlds are outside the Git repository; original SDF unchanged.')
    print('mount_tilt: IMU frame has a known offset, vehicle not tilted.')
    print('body_tilt: vehicle has imposed roll/pitch in zero gravity; not a slope or AEB trial.')


def frozen_pose(p):
    return NS(position=NS(x=float(p.position.x), y=float(p.position.y),
                          z=float(p.position.z)),
              orientation=NS(x=float(p.orientation.x), y=float(p.orientation.y),
                             z=float(p.orientation.z), w=float(p.orientation.w)))


def summarize(s):
    return dict(valid=bool(s.valid), reason=s.reason,
                ground=int(s.ground_points), objects=int(s.obstacle_points),
                unknown=int(s.unknown_points),
                nearest_m=(round(s.nearest_forward_obstacle_m, 4)
                           if s.nearest_forward_obstacle_m is not None else None))


def paired_analysis(samples, *, expectation):
    matched = [s for s in samples if s.get('pair_valid')]
    good = [s for s in matched if s['orientation_error_deg'] <= 2.0]
    equal_detection = [s for s in good if s['exact']['valid'] and s['imu']['valid']
                       and (s['exact']['objects'] > 0) == (s['imu']['objects'] > 0)]
    expected = ((lambda r: r['imu']['objects'] > 0) if expectation == 'obstacle'
                else (lambda r: r['imu']['objects'] == 0))
    expected_good = [s for s in equal_detection if expected(s)]
    result = {
        'total_samples': len(samples), 'paired_fresh_samples': len(matched),
        'orientation_consistent_samples': len(good),
        'map_classification_agreement_samples': len(equal_detection),
        'imu_expected_scene_samples': len(expected_good),
        'median_orientation_error_deg':
            median([r['orientation_error_deg'] for r in matched]) if matched else None,
        'max_orientation_error_deg':
            max((r['orientation_error_deg'] for r in matched), default=None),
        'imu_invalid_samples': sum('imu' in s and not s['imu']['valid'] for s in samples),
        'exact_invalid_samples': sum('exact' in s and not s['exact']['valid'] for s in samples),
        'missing_or_stale_samples': len(samples)-len(matched),
        'unmatched_reasons': dict(Counter(s.get('reason', 'unknown') for s in samples if not s.get('pair_valid'))),
    }
    # Strict experimental expectation: 3+ stationary, fresh, orientation-consistent
    # paired scans where both classifiers agree and match the known scene.
    result['diagnostic_expectation_met'] = (
        len(expected_good) >= 3 and len(expected_good) == len(matched)
        and len(samples) == len(matched))
    return result


def probe(args):
    if args.case == 'body_tilt':
        return probe_body_tilt(args)
    if not MESH.is_file():
        raise SystemExit(f'Ruskotunturi collision STL is missing: {MESH}')
    try:
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic Python bindings missing: ' + str(exc))
    case = args.case
    world_name = WORLDS[case]
    mount = MOUNT_TILT_DEG if case == 'mount_tilt' else (0., 0., 0.)
    pose_topic = f'/world/{world_name}/pose/info'
    node = Node()
    lock = threading.RLock()
    queue = Queue(maxsize=2)
    state = dict(pose=None, imu=None, pose_at=0., imu_at=0.,
                 raw_scans=0, paired_scans=0, dropped=0, imu_count=0,
                 pose_count=0, callback_errors=[])

    def err(kind, exc):
        with lock:
            if len(state['callback_errors']) < 5:
                state['callback_errors'].append(f'{kind}: {type(exc).__name__}: {exc}')

    def on_pose(msg):
        try:
            for entry in msg.pose:
                if entry.name == 'jarcrac' or entry.name.endswith('::jarcrac'):
                    with lock:
                        state['pose'] = frozen_pose(entry)
                        state['pose_at'] = time.monotonic()
                        state['pose_count'] += 1
                    return
        except Exception as exc:
            err('pose', exc)

    def on_imu(msg):
        try:
            q = quaternion_tuple(msg.orientation)
            with lock:
                state['imu'] = q
                state['imu_at'] = time.monotonic()
                state['imu_count'] += 1
        except Exception as exc:
            err('imu', exc)

    def on_scan(msg):
        try:
            with lock:
                state['raw_scans'] += 1
                n = state['raw_scans']
                if n % args.every_scans:
                    return
                p = state['pose']
                q = state['imu']
                age_p = time.monotonic()-state['pose_at']
                age_q = time.monotonic()-state['imu_at']
                delta = abs(state['pose_at']-state['imu_at'])
            item = (n, msg, p, q, age_p, age_q, delta)
            try:
                queue.put_nowait(item)
            except Full:
                with lock:
                    state['dropped'] += 1
        except Exception as exc:
            err('scan', exc)

    for typ, topic, callback in [
        (Pose_V, pose_topic, on_pose), (IMU, IMU_TOPIC, on_imu),
        (LaserScan, LIDAR_TOPIC, on_scan),
    ]:
        if not node.subscribe(typ, topic, callback):
            raise SystemExit('Could not subscribe: ' + topic)
    print('JARCRAC IMU + map-LiDAR | read-only |', case, flush=True)
    print('Topics:', pose_topic, IMU_TOPIC, LIDAR_TOPIC, flush=True)
    deadline = time.monotonic()+15
    start_pose = None
    while time.monotonic() < deadline:
        with lock:
            start_pose = state['pose']
            imu_seen = state['imu'] is not None
        if start_pose is not None and imu_seen:
            break
        time.sleep(.10)
    else:
        raise SystemExit('No fresh IMU and/or Gazebo pose. Check world and IMU topic.')
    print('Loading terrain map near', (start_pose.position.x,start_pose.position.y),flush=True)
    index = TerrainMeshIndex(MESH, center_x=start_pose.position.x,
                             center_y=start_pose.position.y, radius_m=36.)
    print('Terrain triangles:', index.triangle_count_in_roi, flush=True)
    samples = []
    processing_errors = []
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            try:
                n, scan, p, imu_q, p_age, imu_age, age_delta = queue.get(timeout=.5)
            except Empty:
                continue
            row = dict(scan_number=n, pose_age_s=round(p_age,4),
                       imu_age_s=round(imu_age,4),
                       pose_imu_reception_delta_s=round(age_delta,4),
                       pair_valid=False)
            if p is None or imu_q is None or not (
                    0 <= p_age <= .50 and 0 <= imu_age <= .50 and age_delta <= .50):
                row['reason'] = 'missing_or_stale_pose_imu_pair'
            else:
                try:
                    gt_q = quaternion_tuple(p.orientation)
                    corrected_q = correct_imu_orientation(imu_q, mount)
                    gt_angles, imu_angles = q_to_rpy(gt_q), q_to_rpy(corrected_q)
                    imu_pose = NS(position=p.position,
                                  orientation=NS(x=corrected_q[0],y=corrected_q[1],
                                                 z=corrected_q[2],w=corrected_q[3]))
                    reference = scan_to_map_summary(scan, p, index)
                    inferred = scan_to_map_summary(scan, imu_pose, index)
                    row.update(pair_valid=True,
                               gt_rpy_deg=[round(v,3) for v in gt_angles],
                               imu_rpy_deg=[round(v,3) for v in imu_angles],
                               pitch_error_deg=round(wrap_deg(imu_angles[1]-gt_angles[1]),4),
                               roll_error_deg=round(wrap_deg(imu_angles[0]-gt_angles[0]),4),
                               yaw_error_deg=round(wrap_deg(imu_angles[2]-gt_angles[2]),4),
                               orientation_error_deg=round(quat_error_deg(gt_q, corrected_q),4),
                               exact=summarize(reference), imu=summarize(inferred))
                except Exception as exc:
                    row['reason'] = 'processing_error'
                    if len(processing_errors) < 5:
                        processing_errors.append(f'scan={n}: {type(exc).__name__}: {exc}')
            samples.append(row)
            if row['pair_valid']:
                print(f'sample={len(samples)} | imu-vs-pose={row["orientation_error_deg"]:.3f} deg '
                      f'| pitch_delta={row["pitch_error_deg"]:+.3f} '
                      f'| roll_delta={row["roll_error_deg"]:+.3f} '
                      f'| exact objects={row["exact"]["objects"]} '
                      f'| imu objects={row["imu"]["objects"]} '
                      f'| IMU classification valid={row["imu"]["valid"]}',flush=True)
            else:
                print(f'sample={len(samples)} | {row["reason"]}',flush=True)
    except KeyboardInterrupt:
        print('Interrupted; saving partial observations.',flush=True)
    with lock:
        counters = {k:state[k] for k in
                    ('raw_scans','dropped','imu_count','pose_count')}
        callback_errors = list(state['callback_errors'])
    expectation = 'obstacle' if case == 'obstacle' else 'clear'
    analysis = paired_analysis(samples, expectation=expectation)
    report = dict(test='jarcrac_imu_assisted_map_lidar', case=case,
                  world=world_name, read_only=True,
                  imu_reference_frame='ENU',
                  imu_mount_rpy_deg=list(mount),
                  imu_used_for_orientation=True,
                  exact_gazebo_position_used=True,
                  exact_gazebo_yaw_reference_used_for_validation=True,
                  wall_reception_pairing_not_sim_timestamp_synchronized=True,
                  real_localization_or_terrain_aeb_verified=False,
                  counters=counters, callback_errors=callback_errors,
                  processing_errors=processing_errors,
                  analysis=analysis, samples=samples,
                  note=('Stationary SIL calibration only. IMU angle is substituted into '
                        'map projection with *ground-truth XYZ*; no drift, stochastic '
                        'IMU error, real sensor synchronization, or drive test.'))
    target = args.report or (RESULTS/f'imu_{case}_diagnostic.json')
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print('IMU CALIBRATION DIAGNOSTIC:', json.dumps({
        'case': case, **counters, **analysis,
        'callback_errors': callback_errors,
        'processing_errors': processing_errors},indent=2),flush=True)
    print('Saved:',target,flush=True)
    if (not analysis['diagnostic_expectation_met'] or callback_errors or processing_errors):
        raise SystemExit(1)


def probe_body_tilt(args):
    """Isolated orientation test: artificially rotated vehicle in zero gravity.

    This scenario is not a hill test. It checks that world-referenced IMU
    orientation follows the deliberately tilted model pose, not its boot pose.
    """
    try:
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic Python bindings missing: ' + str(exc))
    node = Node()
    lock = threading.RLock()
    state = dict(pose=None, imu=None, pose_at=0., imu_at=0.,
                 imu_count=0, pose_count=0, callback_errors=[])

    def on_pose(msg):
        try:
            for item in msg.pose:
                if item.name == 'jarcrac' or item.name.endswith('::jarcrac'):
                    with lock:
                        state['pose'] = quaternion_tuple(item.orientation)
                        state['pose_at'] = time.monotonic()
                        state['pose_count'] += 1
                    return
        except Exception as exc:
            with lock:
                if len(state['callback_errors']) < 5:
                    state['callback_errors'].append('pose: ' + str(exc))

    def on_imu(msg):
        try:
            q = quaternion_tuple(msg.orientation)
            with lock:
                state['imu'] = q
                state['imu_at'] = time.monotonic()
                state['imu_count'] += 1
        except Exception as exc:
            with lock:
                if len(state['callback_errors']) < 5:
                    state['callback_errors'].append('imu: ' + str(exc))

    topic=f'/world/{WORLDS["body_tilt"]}/pose/info'
    for typ,t,callback in ((Pose_V,topic,on_pose),(IMU,IMU_TOPIC,on_imu)):
        if not node.subscribe(typ,t,callback):
            raise SystemExit('Cannot subscribe '+t)
    print('IMU BODY-TILT CONTROL | zero gravity | read-only',flush=True)
    print('Vehicle starts tilted +3 deg roll, -5 deg pitch; this is NOT a terrain test.',flush=True)
    samples=[]
    started=time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            time.sleep(.5)
            with lock:
                gt,imu=state['pose'],state['imu']
                age_p=time.monotonic()-state['pose_at']
                age_i=time.monotonic()-state['imu_at']
                delta=abs(state['pose_at']-state['imu_at'])
            if gt is None or imu is None or max(age_p,age_i,delta)>.50:
                continue
            roll,pitch,yaw=q_to_rpy(gt)
            a=q_to_rpy(imu)
            error=quat_error_deg(gt,imu)
            samples.append(dict(orientation_error_deg=round(error,4),
                                true_rpy_deg=[round(v,3) for v in (roll,pitch,yaw)],
                                imu_rpy_deg=[round(v,3) for v in a]))
            if len(samples)%5==0:
                print(f'samples={len(samples)} | true pitch={pitch:.2f}deg '
                      f'roll={roll:.2f}deg | imu err={error:.3f}deg',flush=True)
    except KeyboardInterrupt:
        print('Interrupted; saving partial observations.',flush=True)
    with lock:
        errors=list(state['callback_errors'])
        counts={key:state[key] for key in ('pose_count','imu_count')}
    # Require persistent nonzero model tilt and correct absolute attitude;
    # a zero-relative-to-startup IMU should fail here.
    passed=(len(samples)>=5 and not errors
            and all(s['orientation_error_deg']<=2.0 for s in samples)
            and all(abs(s['true_rpy_deg'][0])>1.0
                    and abs(s['true_rpy_deg'][1])>2.0 for s in samples))
    report=dict(test='jarcrac_imu_body_tilt_orientation',case='body_tilt',
                world=WORLDS['body_tilt'],read_only=True,
                gravity_zero_for_orientation_bench=True,
                terrain_driving_verified=False,
                imu_reference_frame='ENU',
                diagnostic_expectation_met=passed,
                orientation_samples=len(samples),
                max_orientation_error_deg=max((s['orientation_error_deg']
                                               for s in samples),default=None),
                counters=counts,callback_errors=errors,samples=samples)
    target=args.report or (RESULTS/'imu_body_tilt_diagnostic.json')
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print('IMU BODY-TILT DIAGNOSTIC:',json.dumps({k:v for k,v in report.items()
        if k!='samples'},indent=2),flush=True)
    print('Saved:',target,flush=True)
    if not passed:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('prepare', help='Generate separate IMU test worlds in home directory')
    p.add_argument('--output-dir',type=Path,default=EXPERIMENT_DIR)
    p.set_defaults(func=prepare)
    p = sub.add_parser('probe', help='Read-only IMU + map-LiDAR comparison')
    p.add_argument('--case',required=True,choices=WORLDS)
    p.add_argument('--duration',type=float,default=20.0)
    p.add_argument('--every-scans',type=int,default=12)
    p.add_argument('--report',type=Path)
    p.set_defaults(func=probe)
    args = parser.parse_args()
    if args.action == 'probe' and not (8. <= args.duration <= 120. and 2 <= args.every_scans <= 50):
        parser.error('duration must be 8..120 s; every-scans 2..50')
    args.func(args)


if __name__ == '__main__':
    main()
