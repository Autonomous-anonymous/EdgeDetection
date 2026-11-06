#!/usr/bin/env python3
"""Inspect real Gazebo multi-layer JARCRAC LiDAR messages in the flat test world.

Run while `gz sim -r worlds/jarcrac_flat_test_world.sdf` is active.
The front-centre sensor is ~0.600 m above flat terrain in the unperturbed model.
On forestry terrain the flat-ground height filter must be replaced with
local ground estimation.
"""
import argparse
import sys
import time
from jarcrac_lidar_3d import summarize_forward


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--topic', default='/jarcrac/lidar')
    ap.add_argument('--sensor-height', type=float, default=0.600)
    ap.add_argument('--duration', type=float, default=15.0)
    args = ap.parse_args()
    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node
    except ImportError as exc:
        sys.exit(f'Gazebo Harmonic Python bindings required: {exc}')
    node = Node()
    received = [0]
    last_print = [0.0]

    def callback(scan):
        received[0] += 1
        now = time.monotonic()
        if now - last_print[0] < 1.0:
            return
        last_print[0] = now
        summary = summarize_forward(scan, sensor_height_m=args.sensor_height)
        nearest = summary.nearest_forward_obstacle_m
        suspect_self = summary.near_points > 0 and (nearest is not None and nearest < 1.6)
        print(f'3D LiDAR: valid={summary.valid} layers={summary.vertical_layers} '
              f'horizontal={summary.horizontal_samples} '
              f'forward_obstacle_pts={summary.obstacle_points} '
              f'ground_pts={summary.ground_points} possible_self_pts={summary.near_points} '
              f'nearest_forward={nearest if nearest is not None else "none"} '
              f'suspect_self_occlusion={suspect_self}', flush=True)
    if not node.subscribe(LaserScan, args.topic, callback):
        sys.exit(f'failed to subscribe to {args.topic}')
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    if received[0] == 0:
        sys.exit(f'FAIL: no LaserScan messages on {args.topic}')
    print(f'Received {received[0]} multi-layer scan messages.', flush=True)


if __name__ == '__main__':
    main()
