#!/usr/bin/env python3
"""READ-ONLY known-obstacle detour preview. NEVER publishes motion commands.

Uses privileged Gazebo pose and terrain STL; the side corridors have NOT been
verified free by LiDAR. A geometry PASS is NOT authorization to drive.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
GZ = ROOT / 'edge-path-project/gazebo'
WORLD = 'jarcrac_detailed_ttc_stop_hold'
OBSTACLE = 'jarcrac_terrain_test_obstacle'
SOURCE = GZ / 'worlds/jarcrac_terrain_obstacle_probe_world.sdf'
MODEL = GZ / 'models/jarcrac_detailed_tracks/model.sdf'
MESH = GZ / 'models/ruskotunturi_map/meshes/ruskotunturi-map-collision.stl'
MARGIN = 1.0
MAX_GRADE = 0.30


def obstacle_geometry(source=SOURCE):
    world = ET.parse(source).getroot().find('world')
    m = world.find(f"./model[@name='{OBSTACLE}']")
    if m is None or m.findtext('static') != 'true':
        raise ValueError('Known static obstacle not found')
    pose = list(map(float, m.findtext('pose').split()))
    size = list(map(float, m.findtext('link/collision/geometry/box/size').split()))
    if len(pose) != 6 or len(size) != 3 or min(size) <= 0:
        raise ValueError('Invalid static obstacle dimensions')
    return pose, size


def vehicle_radius(model=MODEL):
    m = ET.parse(model).getroot().find(
        "./model/link[@name='chassis']/collision[@name='body_collision']"
    )
    if m is None:
        raise ValueError('JARCRAC body collision missing')
    size = list(map(float, m.findtext('geometry/box/size').split()))
    off = list(map(float, m.findtext('pose').split()))
    if len(size) != 3 or len(off) != 6 or min(size) <= 0:
        raise ValueError('Invalid collision geometry')
    return math.hypot(abs(off[0])+size[0]/2, abs(off[1])+size[1]/2)


def transform(p, origin, yaw):
    x, y = p
    return (
        origin[0] + x*math.cos(yaw)-y*math.sin(yaw),
        origin[1] + x*math.sin(yaw)+y*math.cos(yaw)
    )


def to_local(p, origin, yaw):
    x, y = p[0]-origin[0], p[1]-origin[1]
    return (
        x*math.cos(yaw)+y*math.sin(yaw),
        -x*math.sin(yaw)+y*math.cos(yaw)
    )


def rect_distance(p, center, yaw, size):
    x, y = to_local(p, center, yaw)
    return math.hypot(
        max(0, abs(x)-size[0]/2),
        max(0, abs(y)-size[1]/2)
    )


def candidate_waypoints(side):
    if side not in (-1, 1):
        raise ValueError('Expected left=1 or right=-1')
    return [(0.,0.), (.8,4.5*side), (8.,4.5*side), (11.5,0.)]


def sample_polyline(pts, step=.25):
    if not (math.isfinite(step) and step > 0):
        raise ValueError('Bad sample step')
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, math.ceil(math.dist(a,b)/step))
        out.extend(
            (a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)
            for i in range(1,n+1)
        )
    return out


def screen_candidate(pts, start, yaw, obstacle, size, radius, height_at):
    if not (math.isfinite(radius) and radius > 0):
        raise ValueError('Bad radius')
    issues = set()
    min_gap, max_grade = math.inf, 0.
    sampled = sample_polyline(pts)

    for p in sampled:
        x, y = transform(p, start, yaw)
        clearance = (
            rect_distance((x,y), obstacle[:2], obstacle[5], size)
            - radius
        )
        min_gap = min(min_gap, clearance)

        if clearance < MARGIN:
            issues.add('known_obstacle_clearance')

        z = height_at(x,y)
        if z is None or not math.isfinite(z):
            issues.add('missing_terrain')
            continue

        for dx,dy in ((.75,0),(-.75,0),(0,.75),(0,-.75)):
            near = height_at(x+dx,y+dy)
            if near is None or not math.isfinite(near):
                issues.add('missing_terrain')
                continue

            grade = abs(near-z)/.75
            max_grade = max(max_grade,grade)
            if grade > MAX_GRADE:
                issues.add('terrain_grade_screen')

    return dict(
        geometry_screen_pass=not issues,
        issues=sorted(issues),
        sampled_positions=len(sampled),
        min_known_obstacle_clearance_proxy_m=round(min_gap,3),
        max_local_terrain_grade=round(max_grade,3),
        local_waypoints_m=pts,
        world_waypoints_m=[
            tuple(round(t,3) for t in transform(p,start,yaw))
            for p in pts
        ],
        drive_authorized=False
    )


def write_svg(path, results, obstacle, size, start, yaw, radius):
    def coords(p):
        return f'{100+36*p[0]:.1f},{315-36*p[1]:.1f}'

    corners = []
    for u,v in (
        (-size[0]/2,-size[1]/2),
        (size[0]/2,-size[1]/2),
        (size[0]/2,size[1]/2),
        (-size[0]/2,size[1]/2)
    ):
        corners.append(
            to_local(
                transform((u,v),obstacle[:2],obstacle[5]),
                start,yaw
            )
        )

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 630">',
        '<rect width="640" height="630" fill="#f8fafc"/>',
        '<text x="20" y="30" font-size="19">JARCRAC — NO DRIVE AUTHORIZATION</text>',
        '<text x="20" y="54" font-size="13">x forward →, y left ↑; known static obstacle only</text>',
        '<line x1="100" y1="315" x2="590" y2="315" stroke="#9aa" stroke-dasharray="5 5"/>',
        f'<polygon points="{" ".join(map(coords,corners))}" fill="#e88948" stroke="#a53"/>',
        f'<circle cx="100" cy="315" r="{radius*36:.1f}" fill="none" stroke="#35a"/>'
    ]

    for name, side in (('left',1),('right',-1)):
        result = results[name]
        color = '#168450' if result['geometry_screen_pass'] else '#c33'
        lines.append(
            f'<polyline points="{" ".join(map(coords,candidate_waypoints(side)))}" '
            f'fill="none" stroke="{color}" stroke-width="3"/>'
        )
        label = (
            f'{name.upper()} — '
            f'{"geometry pass" if result["geometry_screen_pass"] else "rejected"}; '
            'NOT verified for driving'
        )
        lines.append(
            f'<text x="20" y="{90 if side==1 else 564}" '
            f'font-size="14" fill="{color}">{label}</text>'
        )

    lines += [
        '<text x="20" y="604" font-size="12">Unknown side obstacles and vehicle turn dynamics NOT assessed.</text>',
        '</svg>'
    ]
    path.write_text('\n'.join(lines)+'\n')


def run(args):
    from jarcrac_lidar_map_assisted import (
        TerrainMeshIndex, scan_to_map_summary, quaternion_tuple
    )
    from jarcrac_imu_stage import q_to_rpy
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.transport13 import Node

    obstacle, size = obstacle_geometry()
    radius = vehicle_radius()
    node, lock = Node(), threading.RLock()
    data = dict(
        vehicle=None, obstacle=None, speed=None, scan=None,
        pose_at=0., odom_at=0., scan_at=0., seq=0
    )

    def poses(msg):
        with lock:
            for p in msg.pose:
                name = p.name.split('::')[-1]
                if name == 'jarcrac':
                    data['vehicle'], data['pose_at'] = p, time.monotonic()
                elif name == OBSTACLE:
                    data['obstacle'] = p

    def odom(msg):
        v = msg.twist.linear
        with lock:
            data['speed'], data['odom_at'] = (
                math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                time.monotonic()
            )

    def scan(msg):
        with lock:
            data['scan'], data['scan_at'] = msg, time.monotonic()
            data['seq'] += 1

    for typ, topic, cb in (
        (Pose_V,f'/world/{WORLD}/pose/info',poses),
        (Odometry,'/model/jarcrac/detailed_odometry',odom),
        (LaserScan,'/jarcrac/lidar',scan)
    ):
        if not node.subscribe(typ,topic,cb):
            raise RuntimeError('Cannot subscribe to '+topic)

    print('READ-ONLY detour preview: NO cmd_vel publisher',flush=True)

    deadline = time.monotonic()+15
    while time.monotonic()<deadline:
        with lock:
            d = data.copy()
        if all(d[k] is not None for k in (
            'vehicle','obstacle','speed','scan'
        )):
            break
        time.sleep(.1)
    else:
        raise RuntimeError('Missing world pose, obstacle, odometry or LiDAR')

    p = d['vehicle']
    start0 = (p.position.x,p.position.y)
    index = TerrainMeshIndex(
        MESH, center_x=p.position.x,
        center_y=p.position.y,radius_m=26.
    )

    seen = set()
    summary = None
    deadline = time.monotonic()+15

    while len(seen)<3 and time.monotonic()<deadline:
        with lock:
            d = data.copy()
        now = time.monotonic()

        if not all(
            t>0 and now-t<.7
            for t in (d['pose_at'],d['odom_at'],d['scan_at'])
        ):
            time.sleep(.1)
            continue

        if d['seq'] in seen:
            time.sleep(.05)
            continue

        seen.add(d['seq'])
        p = d['vehicle']
        o = d['obstacle']
        s = d['scan']

        if d['speed']>.04 or math.dist(
            start0,(p.position.x,p.position.y)
        )>.03:
            raise RuntimeError(
                'Vehicle moving: run AFTER the TTC/hold trial'
            )

        if math.dist(
            (o.position.x,o.position.y),obstacle[:2]
        )>.10:
            raise RuntimeError('Wrong world / known obstacle moved')

        if not (
            s.count==721
            and s.vertical_count==32
            and len(s.ranges)==23072
            and 'gpu_lidar_front' in s.frame
        ):
            raise RuntimeError(
                'Wrong LiDAR: expected 721 x 32 gpu_lidar_front'
            )

        summary = scan_to_map_summary(s,p,index)
        if (
            not summary.valid
            or summary.obstacle_points<100
            or summary.nearest_forward_obstacle_m is None
        ):
            raise RuntimeError('Invalid/missing known obstacle returns')

    if len(seen)<3:
        raise RuntimeError('Too few fresh independent 3D obstacle scans')

    start = (p.position.x,p.position.y)
    yaw = math.radians(
        q_to_rpy(quaternion_tuple(p.orientation))[2]
    )

    results = {
        name: screen_candidate(
            candidate_waypoints(side),start,yaw,
            obstacle,size,radius,index.height_at
        )
        for name,side in (('left',1),('right',-1))
    }

    report = dict(
        test='jarcrac_detour_preview',
        mode='READ_ONLY',
        world=WORLD,
        command_published=False,
        planning_authorized=False,
        valid_distinct_scans=len(seen),
        lidar_obstacle_returns=summary.obstacle_points,
        lidar_nearest_m=summary.nearest_forward_obstacle_m,
        body_radius_m=round(radius,3),
        minimum_static_margin_m=MARGIN,
        exact_gazebo_pose_used=True,
        candidates=results,
        note=(
            'Only known static obstacle + terrain mesh screened. '
            'No lateral LiDAR coverage, dynamic feasibility, '
            'independent localization or drive authorization.'
        )
    )

    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.svg.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(
        json.dumps(report,indent=2,sort_keys=True)+'\n'
    )
    write_svg(args.svg,results,obstacle,size,start,yaw,radius)

    print(json.dumps(report,indent=2),flush=True)
    print(
        'Report:',args.report,
        ' | Preview:',args.svg,flush=True
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--report',type=Path,
        default=Path('/tmp/jarcrac_detailed_detour_preview.json')
    )
    ap.add_argument(
        '--svg',type=Path,
        default=Path('/tmp/jarcrac_detailed_detour_preview.svg')
    )
    run(ap.parse_args())


if __name__=='__main__':
    main()
