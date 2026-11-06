#!/usr/bin/env python3
"""Read-only pose-error sensitivity experiment for map-assisted JARCRAC LiDAR.

The stationary Ruskotunturi Gazebo world's real scan is reclassified under
13 candidate map-localization poses: exact, +/-0.10 and +/-0.25 m along the
vehicle's forward and left axes, plus +/-2 and +/-5 degrees yaw error.

All pose errors are applied to the *map interpretation only*. They do not move
the Gazebo model, rewrite measurements, or send motor commands. The exact pose
is privileged Gazebo ground truth. This is a diagnostic, not terrain AEB proof.

Requires the existing jarcrac_lidar_map_assisted.py and the Ruskotunturi STL.
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

from jarcrac_lidar_map_assisted import TerrainMeshIndex, quaternion_tuple, scan_to_map_summary

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_STL = ROOT / 'edge-path-project/gazebo/models/ruskotunturi_map/meshes/ruskotunturi-map-collision.stl'
RESULT_DIR = ROOT / 'edge-path-project/gazebo/results/jarcrac'
WORLDS = ('jarcrac_terrain_probe', 'jarcrac_terrain_obstacle_probe')


def cases():
    """Ordered (identifier, forward offset m, left offset m, yaw offset deg)."""
    out = [('exact', 0.0, 0.0, 0.0)]
    for axis in ('forward', 'left'):
        for magnitude in (0.10, 0.25):
            for sign in (+1, -1):
                v = sign * magnitude
                out.append((f'{axis}_{v:+.2f}m', v if axis == 'forward' else 0.0,
                            v if axis == 'left' else 0.0, 0.0))
    for magnitude in (2.0, 5.0):
        for sign in (+1, -1):
            v = sign * magnitude
            out.append((f'yaw_{v:+.0f}deg', 0.0, 0.0, v))
    return out


def frozen_pose(p):
    """A small, detached copy of a Gazebo Pose protobuf."""
    return NS(position=NS(x=float(p.position.x), y=float(p.position.y),
                          z=float(p.position.z)),
              orientation=NS(x=float(p.orientation.x), y=float(p.orientation.y),
                             z=float(p.orientation.z), w=float(p.orientation.w)))


def perturb_pose(original, *, forward_m=0.0, left_m=0.0, yaw_deg=0.0):
    """Apply XY displacements along vehicle heading and a world-up yaw error.

    Quaternion order is delta_z * original, preserving model pitch and roll
    as part of the original orientation (not flattening to yaw-only pose).
    """
    qx, qy, qz, qw = quaternion_tuple(original.orientation)
    heading = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
    px = float(original.position.x) + forward_m*math.cos(heading) - left_m*math.sin(heading)
    py = float(original.position.y) + forward_m*math.sin(heading) + left_m*math.cos(heading)
    pz = float(original.position.z)
    a = math.radians(yaw_deg) / 2.0
    s, c = math.sin(a), math.cos(a)
    # (0,0,s,c) * (qx,qy,qz,qw)
    rx = c*qx - s*qy
    ry = s*qx + c*qy
    rz = c*qz + s*qw
    rw = c*qw - s*qz
    return NS(position=NS(x=px,y=py,z=pz),
              orientation=NS(x=rx,y=ry,z=rz,w=rw))


def compact_result(s):
    return {
        'valid': bool(s.valid),
        'reason': s.reason,
        'ground_points': int(s.ground_points),
        'obstacle_points': int(s.obstacle_points),
        'unknown_points': int(s.unknown_points),
        'unknown_fraction': round(float(s.unknown_fraction), 5),
        'nearest_obstacle_m': (round(float(s.nearest_forward_obstacle_m), 4)
                               if s.nearest_forward_obstacle_m is not None else None),
        'sensor_height_m': (round(float(s.lidar_height_above_mapped_ground_m), 4)
                            if s.lidar_height_above_mapped_ground_m is not None else None),
    }


def analyze_sensitivity(samples, expectation):
    """Produce a paired comparison. Invalid samples are never counted as clear."""
    stats = {}
    for name, *_ in cases():
        values = [sample['cases'][name] for sample in samples]
        valid = [v for v in values if v['valid']]
        positives = [v for v in valid if v['obstacle_points'] > 0]
        baseline_paired = [sample['cases']['exact'] for sample in samples]
        changed_to_object = sum(
            v['valid'] and v['obstacle_points'] > 0 and baseline['valid']
            and baseline['obstacle_points'] == 0
            for v, baseline in zip(values, baseline_paired))
        changed_to_clear = sum(
            v['valid'] and v['obstacle_points'] == 0 and baseline['valid']
            and baseline['obstacle_points'] > 0
            for v, baseline in zip(values, baseline_paired))
        obstacle_count_deltas = [
            v['obstacle_points'] - b['obstacle_points']
            for v,b in zip(values,baseline_paired) if v['valid'] and b['valid']]
        ranges = [v['nearest_obstacle_m'] for v in positives
                  if v['nearest_obstacle_m'] is not None]
        stats[name] = {
            'valid_samples': len(valid),
            'invalid_samples': len(values) - len(valid),
            'invalid_reasons': dict(Counter(v['reason'] for v in values if not v['valid'])),
            'obstacle_detecting_valid_samples': len(positives),
            'valid_scans_with_no_obstacle': len(valid)-len(positives),
            'false_obstacle_vs_exact_samples': changed_to_object,
            'missed_obstacle_vs_exact_samples': changed_to_clear,
            'median_obstacle_points_valid': (median(v['obstacle_points'] for v in valid)
                                             if valid else None),
            'median_obstacle_count_delta_vs_exact': (median(obstacle_count_deltas)
                                                     if obstacle_count_deltas else None),
            'nearest_obstacle_m_min': min(ranges) if ranges else None,
            'nearest_obstacle_m_max': max(ranges) if ranges else None,
            'median_unknown_fraction': (round(median(v['unknown_fraction'] for v in values), 4)
                                         if values else None),
            'max_unknown_fraction': (max(v['unknown_fraction'] for v in values)
                                     if values else None),
        }
    base = stats['exact']
    baseline_ok = (len(samples) >= 3 and base['valid_samples'] >= 3 and
                   (base['obstacle_detecting_valid_samples'] >= 3 if expectation == 'obstacle'
                    else base['obstacle_detecting_valid_samples'] == 0))
    return stats, baseline_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-name', required=True, choices=WORLDS)
    parser.add_argument('--mesh', type=Path, default=DEFAULT_STL)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--every-scans', type=int, default=20)
    parser.add_argument('--report', type=Path)
    expectation = parser.add_mutually_exclusive_group(required=True)
    expectation.add_argument('--expect-clear', action='store_true')
    expectation.add_argument('--expect-obstacle', action='store_true')
    args = parser.parse_args()
    if not (8.0 <= args.duration <= 180.0 and 5 <= args.every_scans <= 100):
        parser.error('--duration must be 8..180 and --every-scans must be 5..100')
    expected = 'obstacle' if args.expect_obstacle else 'clear'
    matching_world = ('jarcrac_terrain_obstacle_probe' if args.expect_obstacle
                      else 'jarcrac_terrain_probe')
    if args.world_name != matching_world:
        parser.error('World name does not match clear/obstacle expectation')
    if not args.mesh.is_file():
        raise SystemExit('Terrain STL not found: '+str(args.mesh))

    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic Python bindings unavailable: '+str(exc))

    topic = f'/world/{args.world_name}/pose/info'
    lidar = '/jarcrac/lidar'
    node = Node()
    lock = threading.Lock()
    incoming = Queue(maxsize=1)
    shared = {'pose': None, 'pose_at': 0.0, 'scans': 0,
              'discarded': 0, 'callback_errors': []}

    def on_pose(message):
        try:
            for obj in message.pose:
                if obj.name == 'jarcrac' or obj.name.endswith('::jarcrac'):
                    p = frozen_pose(obj)
                    with lock:
                        shared['pose'] = p
                        shared['pose_at'] = time.monotonic()
                    return
        except Exception as exc:
            with lock:
                if len(shared['callback_errors']) < 5:
                    shared['callback_errors'].append('pose: '+str(exc))

    def on_scan(message):
        try:
            with lock:
                shared['scans'] += 1
                count = shared['scans']
                pose = shared['pose']
                age = time.monotonic()-shared['pose_at']
            if count % args.every_scans != 0 or pose is None or age > 2.0:
                return
            # Gazebo's protobuf scan is independent of our queued pose snapshot.
            # Never do the 13 classification passes inside a transport callback.
            try:
                incoming.put_nowait((count, message, pose))
            except Full:
                with lock:
                    shared['discarded'] += 1
        except Exception as exc:
            with lock:
                if len(shared['callback_errors']) < 5:
                    shared['callback_errors'].append('scan: '+str(exc))

    if not node.subscribe(Pose_V, topic, on_pose):
        raise SystemExit('Cannot subscribe to vehicle pose topic '+topic)
    if not node.subscribe(LaserScan, lidar, on_scan):
        raise SystemExit('Cannot subscribe to JARCRAC LiDAR topic '+lidar)
    print('MAP POSE SENSITIVITY | read-only | '+expected, flush=True)
    print('Pose topic:',topic,'| LiDAR topic:',lidar,flush=True)

    deadline = time.monotonic()+12.0
    while time.monotonic() < deadline:
        with lock:
            p = shared['pose']
        if p is not None:
            break
        time.sleep(0.1)
    else:
        raise SystemExit('No JARCRAC pose received. Check Gazebo world name and restart Gazebo.')

    print('Loading collision mesh; center:',(p.position.x,p.position.y),flush=True)
    index = TerrainMeshIndex(args.mesh, center_x=p.position.x,
                             center_y=p.position.y, radius_m=36.0)
    print('Indexed triangles:',index.triangle_count_in_roi,
          '| 13 localization cases per scan',flush=True)

    measured = []
    process_errors = []
    begin = time.monotonic()
    try:
        while time.monotonic()-begin < args.duration:
            try:
                scan_no, scan, pose = incoming.get(timeout=0.5)
            except Empty:
                continue
            try:
                variants = {}
                for name, forward, left, yaw in cases():
                    alternative = perturb_pose(pose, forward_m=forward,
                                               left_m=left, yaw_deg=yaw)
                    summary = scan_to_map_summary(scan, alternative, index)
                    variants[name] = compact_result(summary)
                measured.append({'lidar_scan_number':scan_no,
                                 'cases':variants})
                b = variants['exact']
                invalid = sum(not v['valid'] for v in variants.values())
                print(f'sample={len(measured)} | exact valid={b["valid"]} '
                      f'ground={b["ground_points"]} objects={b["obstacle_points"]} '
                      f'unknown={b["unknown_points"]} | '
                      f'error cases invalid={invalid}/13',flush=True)
            except Exception as exc:
                if len(process_errors) < 5:
                    process_errors.append(f'scan {scan_no}: {type(exc).__name__}: {exc}')
    except KeyboardInterrupt:
        print('Interrupted; saving partial diagnostic.',flush=True)

    stats, baseline_ok = analyze_sensitivity(measured, expected)
    with lock:
        snapshots = dict(shared)
    report = {
        'test': 'jarcrac_map_pose_sensitivity',
        'purpose': 'Controlled sensitivity to biased localization, not real pose-estimation error',
        'world': args.world_name,
        'expectation': expected,
        'mesh': str(args.mesh),
        'stationary_vehicle': True,
        'read_only': True,
        'gazebo_exact_pose_used_as_reference': True,
        'expected_real_world_localization_accuracy_verified': False,
        'terrain_aeb_safety_verified': False,
        'scan_topic': lidar,
        'pose_topic': topic,
        'scan_count': snapshots['scans'],
        'sampled_scans': len(measured),
        'discarded_queued_scans': snapshots['discarded'],
        'callback_errors': snapshots['callback_errors'],
        'processing_errors': process_errors,
        'exact_pose_expectation_met': baseline_ok,
        'cases': [{'name':n,'forward_m':f,'left_m':l,'yaw_deg':y}
                  for n,f,l,y in cases()],
        'statistics': stats,
        'samples': measured,
        'note': ('Paired reclassification of identical LiDAR scans with erroneous map poses. '
                 'Invalid/unknown is NOT interpreted as obstacle-free. '
                 'No vehicle motion and no autonomous braking commands.'),
    }
    target = args.report or RESULT_DIR / f'pose_sensitivity_{expected}.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print('Summary (each valid detection/false alarm uses exactly the same scan as baseline):')
    print(f'{"Pose case":<20} {"valid":>6} {"invalid":>7} {"obst":>6} '
          f'{"new FP":>7} {"miss":>6} {"d_obj":>7} {"unknown%":>9}')
    for name, *_ in cases():
        s = stats[name]
        delta = s['median_obstacle_count_delta_vs_exact']
        print(f'{name:<20} {s["valid_samples"]:>6} {s["invalid_samples"]:>7} '
              f'{s["obstacle_detecting_valid_samples"]:>6} '
              f'{s["false_obstacle_vs_exact_samples"]:>7} '
              f'{s["missed_obstacle_vs_exact_samples"]:>6} '
              f'{delta if delta is not None else "n/a":>7} '
              f'{100*s["median_unknown_fraction"] if s["median_unknown_fraction"] is not None else 0:>8.1f}%')
    print('Exact-pose expectation met:',baseline_ok)
    print('Saved:',target,flush=True)
    if not baseline_ok or snapshots['callback_errors'] or process_errors:
        raise SystemExit(1)


if __name__=='__main__':
    main()
