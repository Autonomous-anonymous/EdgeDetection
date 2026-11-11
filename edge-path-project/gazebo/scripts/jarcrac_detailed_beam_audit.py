#!/usr/bin/env python3
"""Read-only +45-degree LiDAR beam and terrain-height audit."""
import math
import statistics
import threading
import time

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.laserscan_pb2 import LaserScan

from jarcrac_imu_stage import MESH
from jarcrac_lidar_map_assisted import (
    TerrainMeshIndex, quaternion_tuple, rotate
)

lock = threading.Lock()
state = {'pose': None, 'scan': None}

def on_pose(msg):
    for p in msg.pose:
        if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
            with lock:
                state['pose'] = p
            return

def on_scan(msg):
    with lock:
        state['scan'] = msg

node = Node()

if not node.subscribe(
    Pose_V, '/world/jarcrac_detailed_terrain/pose/info',
    on_pose
):
    raise SystemExit('Pose subscription failed')

if not node.subscribe(
    LaserScan, '/jarcrac/lidar', on_scan
):
    raise SystemExit('LiDAR subscription failed')

deadline = time.monotonic() + 15

while time.monotonic() < deadline:
    with lock:
        pose = state['pose']
        scan = state['scan']
    if pose is not None and scan is not None:
        break
    time.sleep(.05)
else:
    raise SystemExit('No pose or LiDAR data')

if (scan.count, scan.vertical_count, len(scan.ranges)) != (
    721, 32, 23072
):
    raise SystemExit('Unexpected LiDAR dimensions')

center = round(
    (math.radians(45)-scan.angle_min)/scan.angle_step
)

print('NEIGHBORING BEAMS')
for offset in (-2, -1, 0, 1, 2):
    col = center + offset
    az = math.degrees(
        scan.angle_min + col*scan.angle_step
    )

    values = [
        float(scan.ranges[ring*scan.count+col])
        for ring in range(scan.vertical_count)
    ]
    finite = [r for r in values if math.isfinite(r)]
    close = [r for r in finite if r < 1.0]

    print(
        f'azimuth={az:+7.2f}deg '
        f'finite={len(finite):2d} '
        f'under_1m={len(close):2d} '
        f'min={min(finite) if finite else None}'
    )

q = quaternion_tuple(pose.orientation)
model_xyz = (
    pose.position.x,
    pose.position.y,
    pose.position.z
)
sensor_xyz = rotate(q, (1.68, 0., .600))
sensor_xyz = tuple(
    model_xyz[i]+sensor_xyz[i] for i in range(3)
)

index = TerrainMeshIndex(
    MESH,
    center_x=model_xyz[0],
    center_y=model_xyz[1],
    radius_m=26.
)

print('\nPROJECTED +45-DEGREE RETURNS')
print('Columns: vertical angle, range, terrain residual')

for ring in (0, 8, 16, 24, 31):
    r = float(scan.ranges[ring*scan.count+center])
    elev = scan.vertical_angle_min + (
        ring*scan.vertical_angle_step
    )
    az = scan.angle_min + center*scan.angle_step

    if not math.isfinite(r):
        print(f'ring={ring}: nonfinite return')
        continue

    local_ray = (
        r*math.cos(elev)*math.cos(az),
        r*math.cos(elev)*math.sin(az),
        r*math.sin(elev)
    )

    delta = rotate(q, local_ray)
    hit = tuple(
        sensor_xyz[i]+delta[i] for i in range(3)
    )
    ground = index.height_at(hit[0], hit[1])
    residual = (
        hit[2]-ground if ground is not None else None
    )

    print(
        f'ring={ring:2d} '
        f'elev={math.degrees(elev):+6.2f}deg '
        f'range={r:.3f}m '
        f'world_xyz={tuple(round(v,3) for v in hit)} '
        f'above_map={None if residual is None else round(residual,3)}'
    )

print('\nREAD-ONLY AUDIT COMPLETE')
print('DETOUR DRIVE AUTHORIZED: NO')
