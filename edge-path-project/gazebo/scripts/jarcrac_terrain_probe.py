#!/usr/bin/env python3
"""Read-only JARCRAC terrain LiDAR probe; DOES NOT move or brake the vehicle."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import threading
import time
from jarcrac_lidar_terrain import summarize_terrain_forward


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/jarcrac/lidar')
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--sensor-height', type=float, default=0.6)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 1 <= args.duration <= 120:
        parser.error('duration must be 1..120 seconds')
    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node
    except ImportError as exc:
        raise SystemExit('Gazebo Harmonic Python packages missing: ' + str(exc))

    node = Node()
    lock = threading.Lock()
    received = 0
    samples = []
    errors = []

    def on_scan(msg):
        nonlocal received
        try:
            summary = summarize_terrain_forward(msg, sensor_height_m=args.sensor_height)
            with lock:
                received += 1
                # Sample every 10 received scans (about once per sim second).
                if received == 1 or received % 10 == 0:
                    samples.append(dict(
                        valid=summary.valid, reason=summary.reason,
                        slope_forward=summary.slope_forward,
                        slope_lateral=summary.slope_lateral,
                        ground_fit_rms_m=summary.ground_fit_rms_m,
                        support_span_m=summary.ground_support_span_m,
                        ground_points=summary.ground_points,
                        obstacle_points=summary.obstacle_points,
                        unknown_points=summary.unknown_points,
                        near_points=summary.near_points,
                        nearest_forward_obstacle_m=summary.nearest_forward_obstacle_m,
                    ))
        except Exception as exc:
            # Avoid flooding the transport thread with repeated tracebacks.
            with lock:
                if len(errors) < 3:
                    errors.append(repr(exc))

    if not node.subscribe(LaserScan, args.topic, on_scan):
        raise SystemExit('FAILED to subscribe: ' + args.topic)
    print('Read-only terrain probe on', args.topic, '| duration', args.duration, 's', flush=True)
    before = time.monotonic()
    last_sample_count = 0
    while time.monotonic() - before < args.duration:
        time.sleep(0.5)
        with lock:
            count = received
            sample_count = len(samples)
            latest = dict(samples[-1]) if samples else None
        if sample_count != last_sample_count and latest:
            print('scans=%d | valid=%s | slope_x=%s | slope_y=%s | ground=%s '
                  '| objects=%s | unknown=%s | nearest=%s | %s' % (
                     count, latest['valid'],
                     latest['slope_forward'], latest['slope_lateral'],
                     latest['ground_points'],latest['obstacle_points'],
                     latest['unknown_points'], latest['nearest_forward_obstacle_m'],
                     latest['reason']), flush=True)
            last_sample_count = sample_count
    with lock:
        results = list(samples)
        total = received
        callback_errors = list(errors)
    report = dict(test='jarcrac_terrain_lidar_diagnostic',
                  passed=None,  # No SIL safety acceptance is asserted.
                  gazebo_scans_received=total,
                  sampled_scans=len(results),
                  valid_samples=sum(bool(s['valid']) for s in results),
                  invalid_samples=sum(not s['valid'] for s in results),
                  samples_with_unknown_returns=sum(s['unknown_points'] > 0 for s in results),
                  callback_errors=callback_errors,
                  readings=results,
                  note='Read-only experimental local-ground classifier. Does not drive JARCRAC or validate terrain AEB.')
    print('TERRAIN LIDAR DIAGNOSTIC:', json.dumps({
        k:v for k,v in report.items() if k != 'readings'}, indent=2), flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
        print('Report:',args.report,flush=True)
    if total == 0 or callback_errors:
        raise SystemExit(1)


if __name__=='__main__':
    main()
