#!/usr/bin/env python3
"""Bounded tracked JARCRAC yaw calibration, Gazebo simulation only."""
import argparse
import json
import math
import threading
import time
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

CMD_W = 0.12
MAX_DRIFT = 0.14
MAX_SIM_S = 3.0
MAX_WALL_S = 15.0
MAX_AGE = 0.75

def yaw_deg(q):
    return math.degrees(math.atan2(
        2*(q.w*q.z+q.x*q.y),
        1-2*(q.y*q.y+q.z*q.z)
    ))

def wrap(d):
    return (d+180) % 360-180

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--case', choices=('flat','terrain'), required=True
    )
    args = ap.parse_args()
    world = f'jarcrac_tracks_{args.case}'

    node = Node()
    lock = threading.RLock()
    state = dict(
        pose=None, odom=None, sim=None,
        pose_at=0., odom_at=0., clock_at=0.,
        contact=False, odom_seq=0, contact_messages=0
    )

    def pose_cb(msg):
        for p in msg.pose:
            if p.name == 'jarcrac' or p.name.endswith('::jarcrac'):
                with lock:
                    state['pose'] = (
                        float(p.position.x),
                        float(p.position.y),
                        yaw_deg(p.orientation)
                    )
                    state['pose_at'] = time.monotonic()
                break

    def odom_cb(msg):
        v = msg.twist.linear
        with lock:
            state['odom'] = (
                math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                float(msg.twist.angular.z)
            )
            state['odom_at'] = time.monotonic()
            state['odom_seq'] += 1

    def clock_cb(msg):
        with lock:
            state['sim'] = msg.sim.sec+msg.sim.nsec*1e-9
            state['clock_at'] = time.monotonic()

    def contact_cb(msg):
        with lock:
            state['contact_messages'] += 1
            state['contact'] |= bool(len(msg.contact))

    for typ,topic,cb in (
        (Pose_V,f'/world/{world}/pose/info',pose_cb),
        (Odometry,'/model/jarcrac/tracks_odometry',odom_cb),
        (Clock,f'/world/{world}/clock',clock_cb),
        (Contacts,'/jarcrac/tracks/chassis_contacts',contact_cb)
    ):
        if not node.subscribe(typ,topic,cb):
            raise RuntimeError('Cannot subscribe: '+topic)

    pub = node.advertise(
        '/model/jarcrac/tracks_cmd_vel',Twist
    )

    def command(w):
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = w
        pub.publish(msg)

    def snapshot():
        with lock:
            return dict(state)

    def fresh(s):
        now = time.monotonic()
        return (
            s['pose'] is not None and
            s['odom'] is not None and
            s['sim'] is not None and
            all(
                0 <= now-s[k] <= MAX_AGE
                for k in ('pose_at','odom_at','clock_at')
            )
        )

    report = dict(
        test='jarcrac_tracks_yaw_calibration',
        world=world,
        requested_yaw_rate_rad_s=CMD_W,
        steering_efficiency=0.5,
        passed=False,
        reason='not_started',
        independent_watchdog=False
    )

    first_moving = None
    last_moving = None
    max_drift = 0.
    start = None

    try:
        deadline = time.monotonic()+20
        while time.monotonic()<deadline:
            s = snapshot()
            if fresh(s):
                break
            command(0.)
            time.sleep(.1)
        else:
            raise RuntimeError('Missing telemetry')

        x0,y0,yaw0 = s['pose']
        expected = (
            (0.,0.) if args.case=='flat' else (20.,40.)
        )

        if (
            math.dist((x0,y0),expected)>.6 or
            s['odom'][0]>.04 or
            abs(s['odom'][1])>.035 or
            s['contact']
        ):
            raise RuntimeError('Unexpected initial vehicle state')

        start = (x0,y0,yaw0,s['sim'])
        print('PREFLIGHT PASS:',world,flush=True)

        deadline = time.monotonic()+MAX_WALL_S
        last_print = -math.inf

        while time.monotonic()<deadline:
            s = snapshot()

            if not fresh(s):
                raise RuntimeError('Stale telemetry')
            if s['contact']:
                raise RuntimeError('Chassis contact')

            x,y,yaw = s['pose']
            delta = wrap(yaw-yaw0)
            drift = math.dist((x,y),(x0,y0))
            elapsed = s['sim']-start[3]

            if (
                not all(map(
                    math.isfinite,
                    (delta,drift,elapsed,*s['odom'])
                ))
                or elapsed < 0
            ):
                raise RuntimeError('Invalid telemetry')

            max_drift = max(max_drift,drift)

            if drift>.14 or delta < -2 or delta > 13:
                raise RuntimeError('Translation/yaw guard')

            if elapsed>MAX_SIM_S:
                raise RuntimeError('Turn timeout')

            if delta>=1.5 and first_moving is None:
                first_moving = (s['sim'],delta)

            if delta>=1.5:
                last_moving = (s['sim'],delta)

            if elapsed-last_print>=.2:
                print(
                    f'sim={elapsed:.3f}s '
                    f'yaw={delta:+.3f}deg '
                    f'drift={drift:.4f}m '
                    f'odom_w={s["odom"][1]:+.3f}rad/s',
                    flush=True
                )
                last_print = elapsed

            if delta>=9:
                report['reason']='reached_9_degrees'
                break

            command(CMD_W)
            time.sleep(.05)
        else:
            raise RuntimeError('Wall timeout')

    except KeyboardInterrupt:
        report['reason']='interrupted'
    except Exception as exc:
        report['reason']=f'{type(exc).__name__}: {exc}'

    finally:
        for _ in range(25):
            command(0.)
            time.sleep(.075)

        stopped = set()
        deadline = time.monotonic()+3

        while time.monotonic()<deadline and len(stopped)<3:
            s = snapshot()
            if (
                fresh(s) and
                s['odom'][0]<=.04 and
                abs(s['odom'][1])<=.04
            ):
                stopped.add(s['odom_seq'])
            command(0.)
            time.sleep(.1)

        s = snapshot()
        final_yaw = (
            wrap(s['pose'][2]-start[2])
            if start and s['pose'] else None
        )

        physical_rate = None
        if (
            first_moving and last_moving and
            last_moving[0]>first_moving[0]+.05
        ):
            physical_rate = (
                math.radians(
                    last_moving[1]-first_moving[1]
                ) /
                (last_moving[0]-first_moving[0])
            )

        passed = (
            report['reason']=='reached_9_degrees'
            and final_yaw is not None
            and 8<=final_yaw<=12
            and max_drift<=MAX_DRIFT
            and len(stopped)>=3
            and fresh(s)
            and not s['contact']
        )

        report.update(
            passed=bool(passed),
            final_yaw_deg=final_yaw,
            measured_physical_yaw_rate_rad_s=physical_rate,
            physical_vs_command_ratio=(
                physical_rate/CMD_W
                if physical_rate is not None else None
            ),
            max_translation_m=max_drift,
            stop_confirmation_samples=len(stopped),
            chassis_contact_observed=s['contact'],
            contact_messages=s['contact_messages'],
            note='Yaw rate from Gazebo pose and simulation time'
        )

        path = Path(
            f'/tmp/jarcrac_tracks_{args.case}_calibration.json'
        )
        path.write_text(json.dumps(report,indent=2)+'\n')

        print(
            'TRACK CALIBRATION:',
            'PASS' if passed else 'FAIL',
            flush=True
        )
        print(json.dumps(report,indent=2),flush=True)
        print('Report:',path,flush=True)

        if not passed:
            raise SystemExit(1)

if __name__=='__main__':
    main()
