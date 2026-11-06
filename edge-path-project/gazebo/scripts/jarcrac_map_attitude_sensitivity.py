#!/usr/bin/env python3
"""Read-only map-assisted LiDAR sensitivity to Z, pitch, roll, and mixed pose errors.

Reuses the tested Ruskotunturi mesh classifier and the previous XY/yaw probe's
pose-copy helpers. Ground-truth Gazebo pose is deliberately biased *only inside
classification*; the simulated JARCRAC stays still. This is not terrain AEB.

Only level/low-variance or large-box stationary scenes are covered: a small
obstacle, practical localization and moving-platform stability are not tested.
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

from jarcrac_lidar_map_assisted import TerrainMeshIndex, quaternion_tuple, scan_to_map_summary
from jarcrac_map_pose_sensitivity import compact_result, frozen_pose, perturb_pose

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_STL = ROOT / 'edge-path-project/gazebo/models/ruskotunturi_map/meshes/ruskotunturi-map-collision.stl'
RESULT_DIR = ROOT / 'edge-path-project/gazebo/results/jarcrac'
WORLDS = ('jarcrac_terrain_probe', 'jarcrac_terrain_obstacle_probe')


def cases():
    """(name, world_z_m, local_pitch_deg, local_roll_deg, forward_m, world_yaw_deg)."""
    out = [('exact', 0., 0., 0., 0., 0.)]
    for magnitude in (0.05, 0.10, 0.25):
        for sign in (+1, -1):
            z = sign * magnitude
            out.append((f'z_{z:+.2f}m', z, 0., 0., 0., 0.))
    for axis in ('pitch', 'roll'):
        for magnitude in (2., 5.):
            for sign in (+1, -1):
                a = sign * magnitude
                out.append((f'{axis}_{a:+.0f}deg', 0., a if axis == 'pitch' else 0.,
                            a if axis == 'roll' else 0., 0., 0.))
    out.extend([
        ('mixed_positive', +0.10, 0., 0., +0.10, +2.),
        ('mixed_negative', -0.10, 0., 0., -0.10, -2.),
    ])
    return out


def multiply(a, b):
    """Hamilton product (x,y,z,w), a * b; rotations composed right-to-left."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def perturb_attitude(original, *, z_m=0., pitch_deg=0., roll_deg=0.,
                     forward_m=0., yaw_deg=0.):
    """World-frame Z, body-frame pitch/roll; heading/yaw use previous probe.

    A local +pitch rotates the vehicle.s x-forward axis toward -z; +roll
    rotates its +y-left axis toward +z. Baseline simulator pitch and roll are
    retained. Translation does not alter the simulated LiDAR ranges.
    """
    moved = perturb_pose(original, forward_m=forward_m, yaw_deg=yaw_deg)
    moved.position.z += z_m
    q = quaternion_tuple(moved.orientation)
    p = math.radians(pitch_deg)/2
    r = math.radians(roll_deg)/2
    qp = (0., math.sin(p), 0., math.cos(p))
    qr = (math.sin(r), 0., 0., math.cos(r))
    rotated = multiply(multiply(q, qp), qr)
    rotated = tuple(v / math.sqrt(sum(t*t for t in rotated)) for v in rotated)
    moved.orientation.x, moved.orientation.y, moved.orientation.z, moved.orientation.w = rotated
    return moved


def analyze(samples, expected):
    """Paired statistics; invalid scans MUST NOT become false-clear or true-clear."""
    stats = {}
    for name, *_ in cases():
        rows = [sample['cases'][name] for sample in samples]
        paired = [(row, sample['cases']['exact']) for row, sample in zip(rows, samples)]
        valid = [row for row in rows if row['valid']]
        positives = [row for row in valid if row['obstacle_points'] > 0]
        deltas = [r['obstacle_points'] - b['obstacle_points'] for r, b in paired
                  if r['valid'] and b['valid']]
        near_deltas = [r['nearest_obstacle_m'] - b['nearest_obstacle_m']
                       for r, b in paired if r['valid'] and b['valid']
                       and r['nearest_obstacle_m'] is not None
                       and b['nearest_obstacle_m'] is not None]
        stats[name] = {
            'valid_samples': len(valid),
            'invalid_samples': len(rows) - len(valid),
            'invalid_reasons': dict(Counter(r['reason'] for r in rows if not r['valid'])),
            'obstacle_detecting_valid_samples': len(positives),
            'false_obstacle_vs_exact_samples': sum(
                r['valid'] and b['valid'] and r['obstacle_points'] > 0
                and b['obstacle_points'] == 0 for r, b in paired),
            'missed_obstacle_vs_exact_samples': sum(
                r['valid'] and b['valid'] and r['obstacle_points'] == 0
                and b['obstacle_points'] > 0 for r, b in paired),
            'median_delta_obstacle_points_vs_exact': median(deltas) if deltas else None,
            'median_nearest_distance_delta_m_vs_exact': median(near_deltas) if near_deltas else None,
            'median_unknown_fraction': median(r['unknown_fraction'] for r in rows) if rows else None,
            'maximum_unknown_fraction': max((r['unknown_fraction'] for r in rows), default=None),
            'median_sensor_height_m': median(r['sensor_height_m'] for r in rows
                                               if r['sensor_height_m'] is not None)
                                    if any(r['sensor_height_m'] is not None for r in rows)
                                    else None,
        }
    ref = stats['exact']
    baseline_ok = (len(samples) >= 3 and ref['invalid_samples'] == 0 and
                   (ref['obstacle_detecting_valid_samples'] == len(samples)
                    if expected == 'obstacle'
                    else ref['obstacle_detecting_valid_samples'] == 0))
    return stats, baseline_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-name', required=True, choices=WORLDS)
    parser.add_argument('--mesh', type=Path, default=DEFAULT_STL)
    parser.add_argument('--duration', type=float, default=25.)
    parser.add_argument('--every-scans', type=int, default=20)
    parser.add_argument('--report', type=Path)
    expected = parser.add_mutually_exclusive_group(required=True)
    expected.add_argument('--expect-clear', action='store_true')
    expected.add_argument('--expect-obstacle', action='store_true')
    args = parser.parse_args()
    if not (8 <= args.duration <= 180 and 5 <= args.every_scans <= 100):
        parser.error('Duration must be 8..180 s, every-scans must be 5..100')
    expectation = 'obstacle' if args.expect_obstacle else 'clear'
    correct_world = ('jarcrac_terrain_obstacle_probe' if args.expect_obstacle
                     else 'jarcrac_terrain_probe')
    if args.world_name != correct_world:
        parser.error('World name does not match expected clear/obstacle scenario')
    if not args.mesh.is_file():
        raise SystemExit(f'Missing map collision STL: {args.mesh}')

    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Missing Gazebo Harmonic Python bindings: ' + str(exc))

    lidar_topic = '/jarcrac/lidar'
    pose_topic = f'/world/{args.world_name}/pose/info'
    node = Node()
    lock = threading.Lock()
    incoming = Queue(maxsize=1)
    state = {'pose': None, 'pose_at': 0., 'scans': 0,
             'discarded': 0, 'callback_errors': []}

    def on_poses(message):
        try:
            for entry in message.pose:
                if entry.name == 'jarcrac' or entry.name.endswith('::jarcrac'):
                    pose = frozen_pose(entry)
                    with lock:
                        state['pose'] = pose
                        state['pose_at'] = time.monotonic()
                    return
        except Exception as exc:
            with lock:
                if len(state['callback_errors']) < 5:
                    state['callback_errors'].append('pose: ' + str(exc))

    def on_scan(message):
        try:
            with lock:
                state['scans'] += 1
                count = state['scans']
                pose = state['pose']
                age = time.monotonic() - state['pose_at']
            if count % args.every_scans or pose is None or age > 2.:
                return
            try:
                incoming.put_nowait((count, message, pose))
            except Full:
                with lock:
                    state['discarded'] += 1
        except Exception as exc:
            with lock:
                if len(state['callback_errors']) < 5:
                    state['callback_errors'].append('scan: ' + str(exc))

    if not node.subscribe(Pose_V, pose_topic, on_poses):
        raise SystemExit('Cannot subscribe to pose topic ' + pose_topic)
    if not node.subscribe(LaserScan, lidar_topic, on_scan):
        raise SystemExit('Cannot subscribe to lidar topic ' + lidar_topic)

    print('MAP HEIGHT/PITCH/ROLL SENSITIVITY | read-only | '+expectation, flush=True)
    print('Pose:', pose_topic, '| LiDAR:', lidar_topic, flush=True)
    deadline = time.monotonic() + 12.
    while time.monotonic() < deadline:
        with lock:
            p = state['pose']
        if p is not None:
            break
        time.sleep(.1)
    else:
        raise SystemExit('No JARCRAC pose. Check Gazebo world name.')

    print('Loading Ruskotunturi collision mesh near', (p.position.x, p.position.y), flush=True)
    index = TerrainMeshIndex(args.mesh, center_x=p.position.x, center_y=p.position.y,
                             radius_m=36.)
    print('Map triangles in ROI:', index.triangle_count_in_roi,
          '| pose cases per sampled scan:', len(cases()), flush=True)

    observations = []
    processing_errors = []
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            try:
                scan_no, scan, pose = incoming.get(timeout=.5)
            except Empty:
                continue
            try:
                variants = {}
                for name, z, pitch, roll, forward, yaw in cases():
                    altered = perturb_attitude(pose, z_m=z, pitch_deg=pitch,
                                               roll_deg=roll, forward_m=forward,
                                               yaw_deg=yaw)
                    variants[name] = compact_result(scan_to_map_summary(scan, altered, index))
                observations.append({'scan_number': scan_no, 'cases': variants})
                reference = variants['exact']
                invalid = sum(not x['valid'] for x in variants.values())
                print(f'sample={len(observations)} | exact valid={reference["valid"]} '
                      f'ground={reference["ground_points"]} '
                      f'objects={reference["obstacle_points"]} '
                      f'unknown={reference["unknown_points"]} '
                      f'| invalid cases={invalid}/{len(cases())}', flush=True)
            except Exception as exc:
                if len(processing_errors) < 5:
                    processing_errors.append(f'scan {scan_no}: {type(exc).__name__}: {exc}')
    except KeyboardInterrupt:
        print('Interrupted; writing partial evidence.', flush=True)

    stats, baseline_ok = analyze(observations, expectation)
    with lock:
        total = state['scans']
        dropped = state['discarded']
        callback_errors = list(state['callback_errors'])
    report = {
        'test': 'jarcrac_map_height_attitude_sensitivity',
        'world': args.world_name, 'expectation': expectation,
        'mesh': str(args.mesh), 'read_only': True, 'vehicle_stationary': True,
        'gazebo_ground_truth_pose_reference': True,
        'real_world_localization_accuracy_verified': False,
        'terrain_aeb_verified': False,
        'scan_count': total, 'sampled_scans': len(observations),
        'discarded_queued_scans': dropped,
        'callback_errors': callback_errors, 'processing_errors': processing_errors,
        'exact_pose_expectation_met': baseline_ok,
        'cases': [{'name':name, 'z_m':z, 'pitch_deg':pitch, 'roll_deg':roll,
                   'forward_m':forward, 'yaw_deg':yaw}
                  for name,z,pitch,roll,forward,yaw in cases()],
        'statistics': stats, 'samples': observations,
        'interpretation': ('Invalid is not clear. Detection/non-detection is compared '
                           'with exact-pose interpretation of the same scan. '
                           'Pose errors change map interpretation only. '
                           'Not a localization or AEB acceptance test.'),
    }
    path = args.report or RESULT_DIR / f'map_attitude_error_{expectation}.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n', encoding='utf-8')
    print('\nPaired scan comparison (invalid does NOT mean clear):')
    print(f'{"Pose case":<19} {"valid":>5} {"invalid":>7} {"obst":>5} '
          f'{"new FP":>6} {"miss":>5} {"d_obj":>6} {"d_near":>8} {"unknown%":>8}')
    for name, *_ in cases():
        s = stats[name]
        delta = s['median_delta_obstacle_points_vs_exact']
        near = s['median_nearest_distance_delta_m_vs_exact']
        unk = s['median_unknown_fraction']
        print(f'{name:<19} {s["valid_samples"]:>5} {s["invalid_samples"]:>7} '
              f'{s["obstacle_detecting_valid_samples"]:>5} '
              f'{s["false_obstacle_vs_exact_samples"]:>6} '
              f'{s["missed_obstacle_vs_exact_samples"]:>5} '
              f'{str(delta) if delta is not None else "n/a":>6} '
              f'{near if near is not None else float("nan"):>+8.3f} '
              f'{100*unk if unk is not None else 0.:>7.1f}%', flush=True)
    print('Exact-pose expectation met:', baseline_ok, flush=True)
    print('Report:', path, flush=True)
    if len(observations) < 3 or not baseline_ok or callback_errors or processing_errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
