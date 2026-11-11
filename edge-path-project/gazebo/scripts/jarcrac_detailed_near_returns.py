#!/usr/bin/env python3
"""Read-only audit of close LiDAR returns, no movement commands."""
import math
import threading
from collections import Counter

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

event = threading.Event()
result = {}

def callback(msg):
    if not event.is_set():
        result['scan'] = msg
        event.set()

node = Node()

if not node.subscribe(
    LaserScan, '/jarcrac/lidar', callback
):
    raise SystemExit('Cannot subscribe to LiDAR')

print('Waiting for one 3D scan...', flush=True)

if not event.wait(15):
    raise SystemExit('No LiDAR scan received')

scan = result['scan']

if (
    scan.count != 721
    or scan.vertical_count != 32
    or len(scan.ranges) != 23072
):
    raise SystemExit('Unexpected LiDAR dimensions')

close = []

for index, value in enumerate(scan.ranges):
    r = float(value)

    if not (
        math.isfinite(r)
        and scan.range_min <= r < 1.0
    ):
        continue

    col = index % scan.count
    ring = index // scan.count

    azimuth = math.degrees(
        scan.angle_min + col * scan.angle_step
    )
    elevation = math.degrees(
        scan.vertical_angle_min
        + ring * scan.vertical_angle_step
    )

    close.append((r, azimuth, elevation))

print('Close returns (<1 m):', len(close))

bins = Counter(
    int(math.floor(az / 10.0)) * 10
    for _, az, _ in close
)

print('\nANGULAR DISTRIBUTION')
for low, count in sorted(bins.items()):
    print(f'{low:+4d} to {low+10:+4d} deg: {count}')

print('\n20 NEAREST RETURNS')
for r, az, el in sorted(close)[:20]:
    print(
        f'range={r:.3f} m | '
        f'azimuth={az:+.2f} deg | '
        f'elevation={el:+.2f} deg'
    )

near_obstacle_bearing = [
    row for row in close
    if abs(row[1] - 71.6) <= 8
]

print(
    '\nClose returns in known-obstacle bearing ±8°:',
    len(near_obstacle_bearing)
)
print('READ-ONLY AUDIT COMPLETE')
print('DETOUR DRIVE AUTHORIZED: NO')
