#!/usr/bin/env python3
"""Read-only full-front LiDAR angular audit; never authorizes motion."""
from pathlib import Path
import json
import math
import threading
import time

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.odometry_pb2 import Odometry

from jarcrac_imu_stage import MESH, q_to_rpy
from jarcrac_lidar_map_assisted import (
    TerrainMeshIndex, quaternion_tuple, rotate
)

WORLD = 'jarcrac_detailed_terrain'
OBSTACLE = 'jarcrac_terrain_test_obstacle'
OUTPUT = Path('/tmp/jarcrac_detailed_wide_visibility.json')
SENSOR_OFFSET = (1.68, 0., .600)

lock = threading.RLock()
state = dict(
    vehicle=None, obstacle=None, scan=None, speed=None,
    pose_at=0., scan_at=0., odom_at=0., scan_seq=0
)


def poses(msg):
    now = time.monotonic()
    with lock:
        for p in msg.pose:
            name = p.name.split('::')[-1]
            if name == 'jarcrac':
                state['vehicle'], state['pose_at'] = p, now
            elif name == OBSTACLE:
                state['obstacle'] = p


def lidar(msg):
    with lock:
        state['scan'], state['scan_at'] = msg, time.monotonic()
        state['scan_seq'] += 1


def odom(msg):
    v = msg.twist.linear
    with lock:
        state['speed'] = math.sqrt(
            v.x*v.x + v.y*v.y + v.z*v.z
        )
        state['odom_at'] = time.monotonic()


node = Node()

for typ, topic, fn in (
    (Pose_V, f'/world/{WORLD}/pose/info', poses),
    (LaserScan, '/jarcrac/lidar', lidar),
    (Odometry, '/model/jarcrac/detailed_odometry', odom),
):
    if not node.subscribe(typ, topic, fn):
        raise SystemExit('Cannot subscribe: ' + topic)


def angular_delta(deg):
    return (deg + 180.) % 360. - 180.


def classify_sector(scan, q, origin, index, center_deg, half_width_deg):
    counts = dict(
        finite_returns=0,
        terrain_returns=0,
        elevated_returns=0,
        unknown_returns=0,
        nearest_elevated_range_m=None
    )

    indices = [
        c for c in range(scan.count)
        if abs(angular_delta(
            math.degrees(scan.angle_min + c*scan.angle_step)
            - center_deg
        )) <= half_width_deg
    ]

    for row in range(scan.vertical_count):
        elev = (
            scan.vertical_angle_min
            + row*scan.vertical_angle_step
        )
        ce, se = math.cos(elev), math.sin(elev)

        for col in indices:
            r = float(scan.ranges[row*scan.count + col])
            if not (
                math.isfinite(r)
                and scan.range_min <= r < scan.range_max
            ):
                continue

            az = scan.angle_min + col*scan.angle_step
            local = (
                r*ce*math.cos(az),
                r*ce*math.sin(az),
                r*se
            )
            xyz = rotate(q, local)
            hit = tuple(origin[i] + xyz[i] for i in range(3))

            counts['finite_returns'] += 1
            ground = index.height_at(hit[0], hit[1])

            if ground is None:
                counts['unknown_returns'] += 1
                continue

            residual = hit[2] - ground

            if abs(residual) <= .15:
                counts['terrain_returns'] += 1
            elif .30 <= residual <= 3.0:
                counts['elevated_returns'] += 1
                old = counts['nearest_elevated_range_m']
                if old is None or r < old:
                    counts['nearest_elevated_range_m'] = round(r, 3)
            else:
                counts['unknown_returns'] += 1

    return counts


print(
    'READ-ONLY: no command publisher; waiting for three scans',
    flush=True
)

results = []
last = -1
first_xy = None
first_yaw = None
index = None
deadline = time.monotonic() + 25

while len(results) < 3 and time.monotonic() < deadline:
    with lock:
        s = dict(state)

    now = time.monotonic()

    if (
        any(s[k] is None for k in (
            'vehicle', 'obstacle', 'scan', 'speed'
        ))
        or s['scan_seq'] == last
        or any(
            now - s[k] > .75
            for k in ('pose_at', 'scan_at', 'odom_at')
        )
    ):
        time.sleep(.05)
        continue

    p, o, scan = s['vehicle'], s['obstacle'], s['scan']

    if (
        scan.count, scan.vertical_count, len(scan.ranges)
    ) != (721, 32, 23072):
        raise SystemExit('Wrong LiDAR shape')

    if 'gpu_lidar_front' not in scan.frame:
        raise SystemExit('Unexpected sensor frame')

    if not math.isfinite(s['speed']) or s['speed'] > .025:
        raise SystemExit('Vehicle moving; stop before audit')

    if math.dist(
        (o.position.x, o.position.y),
        (14.633437, 37.316718)
    ) > .1:
        raise SystemExit('Known obstacle location mismatch')

    q = quaternion_tuple(p.orientation)
    yaw = q_to_rpy(q)[2]
    xy = (p.position.x, p.position.y)
    cumulative = angular_delta(yaw - (-153.434949))

    if not -61 <= cumulative <= -51:
        raise SystemExit(
            f'Expected post-turn ~-56 deg; got {cumulative:+.2f}'
        )

    if first_xy is None:
        first_xy, first_yaw = xy, yaw
        index = TerrainMeshIndex(
            MESH,
            center_x=xy[0],
            center_y=xy[1],
            radius_m=26.
        )
    elif (
        math.dist(xy, first_xy) > .025
        or abs(angular_delta(yaw - first_yaw)) > .5
    ):
        raise SystemExit('Vehicle pose changed during audit')

    offset = rotate(q, SENSOR_OFFSET)
    origin = tuple(
        getattr(p.position, axis) + offset[i]
        for i, axis in enumerate('xyz')
    )

    bearing_world = math.degrees(math.atan2(
        o.position.y - origin[1],
        o.position.x - origin[0]
    ))
    bearing_sensor = angular_delta(bearing_world - yaw)

    sectors = {
        'front_plus_minus_30':
            classify_sector(scan, q, origin, index, 0, 30),
        'right_minus_75_to_minus_45':
            classify_sector(scan, q, origin, index, -60, 15),
        'left_plus_45_to_plus_75':
            classify_sector(scan, q, origin, index, 60, 15),
    }

    if abs(bearing_sensor) <= 90:
        sectors['known_obstacle_bearing_plus_minus_8'] = (
            classify_sector(
                scan, q, origin, index, bearing_sensor, 8
            )
        )

    row = dict(
        scan_seq=s['scan_seq'],
        yaw_deg=yaw,
        cumulative_yaw_deg=cumulative,
        obstacle_bearing_from_sensor_deg=bearing_sensor,
        sectors=sectors
    )
    results.append(row)
    last = s['scan_seq']

    print(
        f'Scan {len(results)}: heading={yaw:+.2f} deg | '
        f'obstacle at {bearing_sensor:+.1f} deg sensor-relative'
    )
    for name, vals in sectors.items():
        print(f'  {name}: {vals}')

    time.sleep(.05)

if len(results) != 3:
    raise SystemExit('Did not obtain three fresh scans')

report = dict(
    test='jarcrac_detailed_wide_visibility',
    mode='READ_ONLY',
    world=WORLD,
    scans=results,
    command_published=False,
    corridor_verified=False,
    drive_authorized=False,
    note=(
        'Map-height residuals are provisional. Elevated returns '
        'are not obstacle identities; occluded rays and unseen '
        'corridors remain unknown.'
    )
)

OUTPUT.write_text(json.dumps(report, indent=2) + '\n')

print('READ-ONLY ANGULAR AUDIT: COMPLETE')
print('Report:', OUTPUT)
print('DETOUR DRIVE AUTHORIZED: NO')
