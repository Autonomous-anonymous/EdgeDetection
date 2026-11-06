#!/usr/bin/env python3
"""Read-only comparison of JARCRAC LiDAR against mapped Ruskotunturi STL.

Requires stationary JARCRAC in jarcrac_terrain_probe_world.sdf OR
jarcrac_terrain_obstacle_probe_world.sdf. Uses Gazebo ground-truth model
pose and the known map: this is experimental SIL, not real localization/AEB.
Does not publish vehicle motion commands. Does not modify any existing model.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import threading
import time
from jarcrac_lidar_map_assisted import TerrainMeshIndex, scan_to_map_summary
from jarcrac_lidar_terrain import summarize_terrain_forward

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_STL=ROOT/'edge-path-project/gazebo/models/ruskotunturi_map/meshes/ruskotunturi-map-collision.stl'
RESULTS=ROOT/'edge-path-project/gazebo/results/jarcrac'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world-name', default='jarcrac_terrain_probe',
                   choices=('jarcrac_terrain_probe','jarcrac_terrain_obstacle_probe'))
    p.add_argument('--mesh',type=Path,default=DEFAULT_STL)
    p.add_argument('--duration',type=float,default=15)
    p.add_argument('--every-scans',type=int,default=10)
    p.add_argument('--report',type=Path)
    expect=p.add_mutually_exclusive_group()
    expect.add_argument('--expect-obstacle',action='store_true')
    expect.add_argument('--expect-clear',action='store_true')
    args=p.parse_args()
    if not(5<=args.duration<=120 and 1<=args.every_scans<=30):
        p.error('duration must be 5..120 and every-scans must be 1..30')
    if not args.mesh.exists():
        raise SystemExit('Terrain STL missing: '+str(args.mesh))
    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as e:
        raise SystemExit('Gazebo Harmonic Python bindings unavailable: '+str(e))
    lidar_topic='/jarcrac/lidar'
    pose_topic=f'/world/{args.world_name}/pose/info'
    node=Node()
    lock=threading.RLock()
    pose=None
    pose_received_at=0.0
    index=None
    scan_count=0
    sampled=0
    results=[]
    errors=[]
    first_valid_pose=None

    def on_poses(msg):
        nonlocal pose, pose_received_at, first_valid_pose
        for obj in msg.pose:
            if obj.name=='jarcrac' or obj.name.endswith('::jarcrac'):
                with lock:
                    pose=obj
                    pose_received_at=time.monotonic()
                    if first_valid_pose is None:
                        first_valid_pose=(obj.position.x,obj.position.y,obj.position.z)
                return

    def on_scan(msg):
        nonlocal scan_count,sampled
        with lock:
            scan_count+=1
            if scan_count!=1 and scan_count % args.every_scans:
                return
            scan_pose=pose
            pose_age=time.monotonic()-pose_received_at
            current_index=index
        if scan_pose is None or pose_age>2.0 or current_index is None:
            return
        try:
            s=scan_to_map_summary(msg,scan_pose,current_index)
            local=summarize_terrain_forward(msg,sensor_height_m=0.600)
            item=dict(valid=s.valid,reason=s.reason,
                      lidar_only_valid=local.valid,
                      lidar_only_obstacle_points=local.obstacle_points,
                      lidar_only_unknown_points=local.unknown_points,
                      lidar_only_nearest_obstacle_m=local.nearest_forward_obstacle_m,
                      scan_returns=s.scan_returns,mapped_returns=s.mapped_returns,
                      ground_points=s.ground_points,obstacle_points=s.obstacle_points,
                      unknown_points=s.unknown_points,
                      unknown_fraction=round(s.unknown_fraction,4),
                      nearest_forward_obstacle_m=s.nearest_forward_obstacle_m,
                      lidar_height_above_mapped_ground_m=s.lidar_height_above_mapped_ground_m)
            with lock:
                results.append(item)
                sampled+=1
        except Exception as e:
            with lock:
                if len(errors)<4:
                    errors.append(str(e))

    if not node.subscribe(Pose_V,pose_topic,on_poses):
        raise SystemExit('Cannot subscribe to Gazebo pose topic '+pose_topic)
    if not node.subscribe(LaserScan,lidar_topic,on_scan):
        raise SystemExit('Cannot subscribe to lidar '+lidar_topic)
    print('Read-only map-assisted JARCRAC LiDAR probe')
    print('Pose:',pose_topic,'| LiDAR:',lidar_topic,flush=True)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with lock:
            start_pose=first_valid_pose
        if start_pose is not None:
            break
        time.sleep(0.10)
    else:
        raise SystemExit('FAIL: no JARCRAC model pose from '+pose_topic+'; use the correct world')
    print('Building XY height lookup for collision mesh near',start_pose[:2],flush=True)
    index_obj=TerrainMeshIndex(args.mesh,center_x=start_pose[0],center_y=start_pose[1],radius_m=36)
    with lock:
        index=index_obj
    print('Map triangles in ROI:',index_obj.triangle_count_in_roi,
          '| buckets:',len(index_obj.buckets),flush=True)
    print('Important: exact Gazebo pose and source collision mesh are privileged SIL inputs.',flush=True)
    begin=time.monotonic()
    last=0
    while time.monotonic()-begin<args.duration:
        time.sleep(0.5)
        with lock:
            n=len(results)
            item=results[-1].copy() if results else None
        if n>last and item is not None:
            print(f'samples={n} | valid={item["valid"]} | '
                  f'ground={item["ground_points"]} | '
                  f'objects={item["obstacle_points"]} | '
                  f'unknown={item["unknown_points"]} | '
                  f'nearest={item["nearest_forward_obstacle_m"]} | '
                  f'sensor_h={item["lidar_height_above_mapped_ground_m"]} | '
                  f'LiDAR-only objects={item["lidar_only_obstacle_points"]} | '
                  f'{item["reason"]}',flush=True)
            last=n
    with lock:
        data=list(results)
        total_scans=scan_count
        callback_errors=list(errors)
    valid=[s for s in data if s['valid']]
    detected=sum(s['obstacle_points']>0 for s in valid)
    # These are DIAGNOSTIC expectations only, not AEB or clearance acceptance.
    condition=None
    if args.expect_obstacle:
        condition=len(valid)>=3 and detected>=3
    elif args.expect_clear:
        condition=len(valid)>=3 and detected==0
    report=dict(test='jarcrac_map_assisted_terrain_probe',passed=None,
                world=args.world_name,mesh=str(args.mesh),
                exact_gazebo_pose_used=True,read_only=True,
                lidar_scans_received=total_scans,sampled_scans=len(data),
                valid_samples=len(valid),invalid_samples=len(data)-len(valid),
                obstacle_detecting_samples=detected,callback_errors=callback_errors,
                diagnostic_expectation_met=condition,
                diagnostics=data,
                note='Ground-truth pose + mesh diagnostic, not localized perception or terrain AEB safety evidence.')
    print('MAP-ASSISTED TERRAIN DIAGNOSTIC:',json.dumps({k:v for k,v in report.items() if k!='diagnostics'},indent=2),flush=True)
    target=args.report or (RESULTS / f'map_assisted_{args.world_name}.json')
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print('Report:',target,flush=True)
    if not data or callback_errors or (condition is False):
        raise SystemExit(1)

if __name__=='__main__':
    main()
