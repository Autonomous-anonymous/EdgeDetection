#!/usr/bin/env python3
"""Read-only post-turn map/LiDAR visibility diagnostic.

Does not publish velocity commands.
Does not authorize detour driving.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan

from jarcrac_imu_stage import MESH, q_to_rpy
from jarcrac_lidar_map_assisted import (
    TerrainMeshIndex,
    quaternion_tuple,
    scan_to_map_summary,
)

WORLD = 'jarcrac_detailed_terrain'
OUTPUT = Path('/tmp/jarcrac_detailed_turn_visibility.json')

lock = threading.RLock()
state = {
    'pose': None,
    'pose_at': 0.,
    'speed': None,
    'odom_at': 0.,
    'scan': None,
    'scan_at': 0.,
    'scan_number': 0,
}


def on_pose(msg):
    for pose in msg.pose:
        if (
            pose.name == 'jarcrac'
            or pose.name.endswith('::jarcrac')
        ):
            with lock:
                state['pose'] = pose
                state['pose_at'] = time.monotonic()
            return


def on_odom(msg):
    v = msg.twist.linear
    speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
    with lock:
        state['speed'] = speed
        state['odom_at'] = time.monotonic()


def on_scan(msg):
    with lock:
        state['scan'] = msg
        state['scan_at'] = time.monotonic()
        state['scan_number'] += 1


node = Node()

for typ, topic, callback in (
    (Pose_V, f'/world/{WORLD}/pose/info', on_pose),
    (Odometry, '/model/jarcrac/detailed_odometry', on_odom),
    (LaserScan, '/jarcrac/lidar', on_scan),
):
    if not node.subscribe(typ, topic, callback):
        raise SystemExit('Subscription failed: ' + topic)

print('READ-ONLY: no velocity publisher', flush=True)
print('Collecting three fresh LiDAR scans...', flush=True)

index = TerrainMeshIndex(
    MESH,
    center_x=20.,
    center_y=40.,
    radius_m=26.
)

results = []
last_number = -1
first = None
deadline = time.monotonic() + 20.

while len(results) < 3 and time.monotonic() < deadline:
    with lock:
        s = dict(state)

    now = time.monotonic()

    if (
        s['pose'] is None
        or s['speed'] is None
        or s['scan'] is None
        or s['scan_number'] == last_number
        or max(
            now-s['pose_at'],
            now-s['odom_at'],
            now-s['scan_at']
        ) > 0.8
    ):
        time.sleep(.05)
        continue

    scan = s['scan']
    pose = s['pose']

    if (
        scan.count != 721
        or scan.vertical_count != 32
        or len(scan.ranges) != 23072
        or 'gpu_lidar_front' not in scan.frame
    ):
        raise SystemExit('FAIL: unexpected LiDAR shape/frame')

    if not math.isfinite(s['speed']) or s['speed'] > .025:
        raise SystemExit('FAIL: vehicle not stationary')

    xy = (float(pose.position.x), float(pose.position.y))
    yaw = q_to_rpy(
        quaternion_tuple(pose.orientation)
    )[2]

    if first is None:
        first = (xy, yaw)
    else:
        drift = math.dist(xy, first[0])
        yaw_change = (yaw-first[1]+180)%360-180

        if drift > .025 or abs(yaw_change) > .5:
            raise SystemExit(
                'FAIL: vehicle moved during observation'
            )

    row = {
        'scan_number': s['scan_number'],
        'world_xy_m': xy,
        'world_heading_deg': yaw,
        'odometry_speed_m_s': s['speed'],
        'sectors': {},
    }

    for half_angle in (12., 30.):
        summary = scan_to_map_summary(
            scan, pose, index,
            sector_half_deg=half_angle
        )
        row['sectors'][f'forward_plus_minus_{int(half_angle)}deg'] = (
            asdict(summary)
        )

        print(
            f'Scan {len(results)+1} | '
            f'heading={yaw:+.2f}deg | '
            f'forward ±{half_angle:.0f}deg | '
            f'valid={summary.valid} | '
            f'objects={summary.obstacle_points} | '
            f'unknown={summary.unknown_points} | '
            f'nearest={summary.nearest_forward_obstacle_m}',
            flush=True
        )

    results.append(row)
    last_number = s['scan_number']
    time.sleep(.05)

if len(results) != 3:
    raise SystemExit('FAIL: fewer than three fresh scans')

report = {
    'test': 'jarcrac_detailed_turn_visibility',
    'world': WORLD,
    'mode': 'READ_ONLY',
    'samples': results,
    'command_published': False,
    'side_corridor_verified': False,
    'detour_drive_authorized': False,
    'note': (
        'Forward-sector terrain-map classification at the '
        'current heading only. Unknown space remains unknown; '
        'no swept-path or full side-corridor verification.'
    ),
}

OUTPUT.write_text(json.dumps(report, indent=2) + '\n')

print('READ-ONLY VISIBILITY CAPTURE: COMPLETE')
print('Report:', OUTPUT)
print('DETOUR DRIVE AUTHORIZED: NO')
