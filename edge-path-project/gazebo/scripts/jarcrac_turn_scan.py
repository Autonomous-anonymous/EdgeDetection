#!/usr/bin/env python3
"""Isolated 10-degree JARCRAC turning/3D-LiDAR test; NOT detour control.

Uses Gazebo ground-truth pose and terrain mesh; known static obstacle only.
Never run another JARCRAC command publisher concurrently. No independent
watchdog: if force-killed, pause Gazebo immediately.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

WORLD = 'jarcrac_turn_scan'
PATH = Path.home() / 'jarcrac_imu_experiment' / f'{WORLD}.sdf'
CMD_TOPIC = '/model/jarcrac/turn_scan_cmd_vel'
CONTACT_TOPIC = '/jarcrac/turn_scan/chassis_contacts'
POSE_TOPIC = f'/world/{WORLD}/pose/info'
CLOCK_TOPIC = f'/world/{WORLD}/clock'
TARGET_DEG = 10.0
YAW_CUTOFF_DEG = 9.4
ANGULAR_CMD_RAD_S = 0.12
MAX_TRANSLATION_M = 0.14
MAX_TILT_DEG = 15.0
STALE_WALL_S = 0.75


def yaw_delta_deg(start, current):
    return (current - start + 180.) % 360. - 180.


def terrain_sweep(height_at, x, y, radius=2.1, max_grade=.30):
    """Approximate swept-disk local terrain check, not 3D collision proof."""
    maximum = 0.
    for r in (0., radius/3, radius*2/3, radius):
        for i in range(24 if r else 1):
            a = i * 2 * math.pi / 24
            px, py = x+r*math.cos(a), y+r*math.sin(a)
            z = height_at(px, py)
            if z is None or not math.isfinite(z):
                return 'unmapped_turning_footprint', maximum
            for dx, dy in ((.4,0),(-.4,0),(0,.4),(0,-.4)):
                q = height_at(px+dx, py+dy)
                if q is None or not math.isfinite(q):
                    return 'unmapped_turning_footprint', maximum
                maximum = max(maximum, abs(q-z)/.4)
                if maximum > max_grade:
                    return 'turning_terrain_grade_exceeded', maximum
    return None, maximum


def turn_gate(*, ages, analysis_status, analysis_age, contact,
              speed, angular_speed, roll, pitch, travel, yaw):
    vals = (speed, angular_speed, roll, pitch, travel, yaw, analysis_age, *ages)
    if contact:
        return 'chassis_contact'
    if any(not math.isfinite(v) for v in vals):
        return 'invalid_telemetry'
    if any(v < 0 for v in (*ages,analysis_age,speed,angular_speed,travel)):
        return 'invalid_telemetry'
    if max(*ages) > STALE_WALL_S or analysis_age > .9:
        return 'stale_sensor_or_classification'
    if analysis_status != 'obstacle_confirmed':
        return analysis_status
    if speed > .08 or angular_speed > .30:
        return 'unexpected_velocity'
    if abs(roll) > MAX_TILT_DEG or abs(pitch) > MAX_TILT_DEG:
        return 'excessive_tilt'
    if travel > MAX_TRANSLATION_M:
        return 'unexpected_translation'
    if abs(yaw) > 13.:
        return 'excessive_yaw'
    return None


def prepare(output=PATH):
    from jarcrac_imu_dynamic import prepare_world
    from jarcrac_imu_stage import SOURCES

    prepare_world(
        output,
        source=SOURCES['obstacle'],
        world_name=WORLD,
        cmd_topic=CMD_TOPIC,
        contact_topic=CONTACT_TOPIC,
    )

    world = ET.parse(output).getroot().find('world')
    model = world.find("./model[@name='jarcrac']")
    obstacle = world.find(
        "./model[@name='jarcrac_terrain_test_obstacle']"
    )
    lidar = model.find(
        "./link[@name='chassis']/sensor[@name='gpu_lidar_front']"
    )

    if (
        obstacle is None
        or obstacle.findtext('static') != 'true'
        or lidar is None
        or lidar.findtext('ray/scan/horizontal/samples') != '721'
        or lidar.findtext('ray/scan/vertical/samples') != '32'
        or model.findtext(
            "./plugin[@name='gz::sim::systems::DiffDrive']/topic"
        ) != CMD_TOPIC
    ):
        raise ValueError('Generated world has incorrect vehicle or sensor')

    return output


def run(report_path):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    from jarcrac_imu_dynamic import Monitor
    from jarcrac_imu_dynamic_obstacle import obstacle_sample_status
    from jarcrac_imu_stage import MESH, q_to_rpy
    from jarcrac_lidar_map_assisted import (
        TerrainMeshIndex, quaternion_tuple
    )
    from jarcrac_detour_preview import (
        obstacle_geometry, rect_distance, vehicle_radius
    )

    class TurnMonitor(Monitor):
        def __init__(self):
            super().__init__(
                Node(),
                dict(
                    Contacts=Contacts,
                    IMU=IMU,
                    LaserScan=LaserScan,
                    Odometry=Odometry,
                    Pose_V=Pose_V,
                    Twist=Twist,
                ),
                every_scans=2,
                pose_topic=POSE_TOPIC,
                cmd_topic=CMD_TOPIC,
                contact_topic=CONTACT_TOPIC,
            )
            with self.lock:
                self.state.update(
                    obstacle_xy=None,
                    obstacle_at=0.,
                    clock_at=0.,
                    sim_time=None,
                    angular_speed=None,
                )
            if not self.node.subscribe(Clock, CLOCK_TOPIC, self.on_clock):
                raise RuntimeError('Cannot subscribe to simulation clock')

        def on_clock(self, msg):
            with self.lock:
                self.state['sim_time'] = msg.sim.sec+msg.sim.nsec*1e-9
                self.state['clock_at'] = time.monotonic()

        def on_pose(self, msg):
            super().on_pose(msg)
            for p in msg.pose:
                if p.name.split('::')[-1] == 'jarcrac_terrain_test_obstacle':
                    with self.lock:
                        self.state['obstacle_xy'] = (
                            p.position.x, p.position.y
                        )
                        self.state['obstacle_at'] = time.monotonic()
                    break

        def on_odom(self, msg):
            super().on_odom(msg)
            with self.lock:
                self.state['angular_speed'] = abs(msg.twist.angular.z)

        def on_scan(self, msg):
            if (
                msg.frame != 'jarcrac::chassis::gpu_lidar_front'
                or msg.count != 721
                or msg.vertical_count != 32
                or len(msg.ranges) != 23072
            ):
                self._err(
                    'scan_shape',
                    ValueError('Expected 721 x 32 gpu_lidar_front')
                )
                return
            super().on_scan(msg)

        def snapshot(self):
            d = super().snapshot()
            with self.lock:
                d['age_clock'] = (
                    time.monotonic()-self.state['clock_at']
                    if self.state['clock_at'] else math.inf
                )
                d['age_obstacle'] = (
                    time.monotonic()-self.state['obstacle_at']
                    if self.state['obstacle_at'] else math.inf
                )
            return d

        def turn(self, omega):
            msg = Twist()
            msg.linear.x = 0.
            msg.angular.z = omega
            self.publisher.publish(msg)

    m = TurnMonitor()
    worker = None

    report = dict(
        test='jarcrac_bounded_turn_scan',
        world=WORLD,
        passed=False,
        target_turn_deg=TARGET_DEG,
        angular_command_rad_s=ANGULAR_CMD_RAD_S,
        cmd_topic=CMD_TOPIC,
        exact_gazebo_pose_and_map_used=True,
        avoidance_enabled=False,
        independent_watchdog=False,
    )

    started = False
    initial_xy = None
    initial_yaw = None
    confirmed_scans = set()
    turning_scans = set()
    reason = 'not_started'
    peak_translation = 0.
    initial_points = None
    final_points = None

    try:
        print('JARCRAC BOUNDED 10-DEG TURN | no detour driving', flush=True)
        m.stop(3)

        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            d = m.snapshot()
            if (
                d['pose'] is not None
                and d['imu'] is not None
                and d['speed'] is not None
                and d['angular_speed'] is not None
                and d['obstacle_xy'] is not None
                and d['sim_time'] is not None
            ):
                break
            m.command(0)
            time.sleep(.1)
        else:
            raise RuntimeError('Missing world pose / IMU / clock / odometry')

        p = d['pose']
        initial_xy = (p.position.x, p.position.y)
        obstacle, size = obstacle_geometry()

        if (
            math.dist(initial_xy, (20.,40.)) > .6
            or d['speed'] > .04
            or d['angular_speed'] > .035
            or d['contact']
            or math.dist(d['obstacle_xy'], obstacle[:2]) > .1
        ):
            raise RuntimeError('Unexpected initial scene or vehicle motion')

        initial_yaw = q_to_rpy(
            quaternion_tuple(p.orientation)
        )[2]

        if abs(
            yaw_delta_deg(math.degrees(-2.6779450446),initial_yaw)
        ) > 12:
            raise RuntimeError('Unexpected initial heading; restart Gazebo')

        static_gap = (
            rect_distance(initial_xy, obstacle[:2], obstacle[5], size)
            - vehicle_radius()
        )
        if static_gap < 1.0:
            raise RuntimeError('Insufficient static-obstacle clearance')

        index = TerrainMeshIndex(
            MESH,
            center_x=p.position.x,
            center_y=p.position.y,
            radius_m=18,
        )

        issue, grade = terrain_sweep(index.height_at, *initial_xy)
        if issue:
            raise RuntimeError(issue)

        report.update(
            known_obstacle_clearance_proxy_m=round(static_gap,3),
            swept_terrain_max_grade=round(grade,3),
        )

        worker = threading.Thread(
            target=m.run_worker,
            args=(index,),
            daemon=True,
        )
        worker.start()

        deadline = time.monotonic()+22
        while len(confirmed_scans) < 3 and time.monotonic() < deadline:
            m.command(0)
            d = m.snapshot()
            r = d['analysis']

            if d['callback_errors'] or d['worker_errors'] or d['contact']:
                raise RuntimeError('Preflight perception/contact failure')

            if r is not None:
                status = obstacle_sample_status(r)
                if status != 'obstacle_confirmed':
                    raise RuntimeError('Invalid preflight scan: '+status)
                if d['analysis_age'] <= .9:
                    confirmed_scans.add(r['scan_number'])
                    initial_points = r['imu']['objects']

            time.sleep(.1)

        if len(confirmed_scans) < 3:
            raise RuntimeError('Insufficient fresh 3D obstacle scans')

        print(
            'PREFLIGHT PASS: known obstacle, 3D LiDAR, terrain sweep',
            flush=True,
        )

        last_scan = max(confirmed_scans)
        deadline = time.monotonic()+18

        while time.monotonic() < deadline:
            d = m.snapshot()
            p = d['pose']
            r = d['analysis']

            if p is None:
                raise RuntimeError('Missing vehicle pose')

            roll, pitch, yaw = q_to_rpy(
                quaternion_tuple(p.orientation)
            )
            delta = yaw_delta_deg(initial_yaw, yaw)
            distance = math.dist(
                initial_xy, (p.position.x, p.position.y)
            )
            peak_translation = max(peak_translation,distance)

            status = obstacle_sample_status(r)
            issue = turn_gate(
                ages=[
                    d['age_'+k]
                    for k in ('pose','imu','odom','scan')
                ] + [d['age_clock'],d['age_obstacle']],
                analysis_status=status,
                analysis_age=d['analysis_age'],
                contact=d['contact'],
                speed=d['speed'],
                angular_speed=(
                    d['angular_speed']
                    if d['angular_speed'] is not None
                    else math.nan
                ),
                roll=roll,
                pitch=pitch,
                travel=distance,
                yaw=delta,
            )

            if (
                d['callback_errors']
                or d['worker_errors']
                or d['queue_dropped']
            ):
                issue = 'perception_processing_or_queue_error'

            if (
                d['obstacle_xy'] is None
                or math.dist(d['obstacle_xy'],obstacle[:2]) > .1
            ):
                issue = 'known_obstacle_moved'

            if issue:
                raise RuntimeError(issue)

            if r['scan_number'] != last_scan:
                turning_scans.add(r['scan_number'])
                last_scan = r['scan_number']
                final_points = r['imu']['objects']

            if delta >= YAW_CUTOFF_DEG:
                started = True
                print(
                    'TURN REACHED: yaw=%.2f deg, drift=%.3f m, 3D scans=%d'
                    % (delta,distance,len(turning_scans)),
                    flush=True,
                )
                break

            m.turn(ANGULAR_CMD_RAD_S)
            time.sleep(.075)

        if not started:
            raise RuntimeError('Turn timeout: target not reached')

        reason = 'turn_target_reached'

    except KeyboardInterrupt:
        reason = 'interrupted'

    except Exception as exc:
        reason = f'{type(exc).__name__}: {exc}'

    finally:
        m.stop(25)

        stopped = set()
        until = time.monotonic()+3

        while time.monotonic() < until and len(stopped) < 3:
            d = m.snapshot()
            if (
                d['odom_count'] not in stopped
                and d['speed'] is not None
                and d['angular_speed'] is not None
                and d['speed'] <= .04
                and d['angular_speed'] <= .035
                and d['age_odom'] < STALE_WALL_S
            ):
                stopped.add(d['odom_count'])
            m.command(0)
            time.sleep(.1)

        d = m.snapshot()
        final_yaw = None
        final_translation = None

        if (
            d['pose'] is not None
            and initial_xy is not None
            and initial_yaw is not None
        ):
            p = d['pose']
            final_yaw = yaw_delta_deg(
                initial_yaw,
                q_to_rpy(quaternion_tuple(p.orientation))[2],
            )
            final_translation = math.dist(
                initial_xy, (p.position.x,p.position.y)
            )

        if worker:
            m.stop_event.set()
            worker.join(timeout=3)

        passed = (
            started
            and reason == 'turn_target_reached'
            and final_yaw is not None
            and 8. <= final_yaw <= 12.
            and final_translation is not None
            and final_translation <= MAX_TRANSLATION_M
            and len(turning_scans) >= 3
            and len(stopped) >= 3
            and not d['contact']
            and not d['callback_errors']
            and not d['worker_errors']
            and d['age_pose'] < STALE_WALL_S
        )

        report.update(
            passed=bool(passed),
            reason=reason,
            final_yaw_delta_deg=final_yaw,
            final_translation_m=final_translation,
            max_translation_m=round(peak_translation,4),
            valid_turning_3d_scans=len(turning_scans),
            first_obstacle_returns=initial_points,
            last_obstacle_returns=final_points,
            zero_linear_and_angular_odom_samples=len(stopped),
            final_linear_speed_mps=d['speed'],
            final_angular_speed_rad_s=d['angular_speed'],
            contact_observed=d['contact'],
            callback_errors=d['callback_errors'],
            processing_errors=d['worker_errors'],
            note=(
                'Controlled simulation turn only, NOT safe detour '
                'authorization. Exact Gazebo pose, terrain mesh, '
                'and approximate collision geometry.'
            ),
        )

        report_path.parent.mkdir(parents=True,exist_ok=True)
        report_path.write_text(
            json.dumps(report,indent=2,sort_keys=True)+'\n'
        )

        print(
            'JARCRAC 10-DEG TURN/3D SCAN:',
            'PASS' if passed else 'FAIL',
            flush=True,
        )
        print(json.dumps(report,indent=2,sort_keys=True),flush=True)
        print('Report:',report_path,flush=True)

        if not passed:
            raise SystemExit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='action',required=True)

    p = sub.add_parser('prepare')
    p.add_argument('--output',type=Path,default=PATH)

    p = sub.add_parser('turn')
    p.add_argument(
        '--report',
        type=Path,
        default=Path('/tmp/jarcrac_turn_scan.json'),
    )

    args = ap.parse_args()

    if args.action == 'prepare':
        print('Generated:',prepare(args.output))
    else:
        run(args.report)


if __name__ == '__main__':
    main()
