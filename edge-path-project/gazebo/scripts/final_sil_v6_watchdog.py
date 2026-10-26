#!/usr/bin/env python3
"""Independent command arbiter: sole publisher to vehicle cmd_vel topics.

A controller crash, stale command, stale/invalid scan, excessive tilt, missing
scene poses, clock stall, or ground-truth envelope violation causes zero commands.
This is simulation infrastructure, not a certified safety controller.
"""
import math
import signal
import threading
import time
from gz.msgs10.clock_pb2 import Clock
from ruskotunturi_final_sil_demo_v6 import Node, Pose_V, LaserScan, Twist, WORLD_NAME, POSE_TOPIC, quaternion_to_rpy
from final_sil_v6_checks import scan_valid, point_clearance, scene_clearance


def main():
    node=Node(); lock=threading.Lock()
    state={'clock':None,'clock_wall':0,'poses':{},'pose_wall':{},'cmd':{},'lidar':{}}
    models=('aeb_vehicle_360','aeb_vehicle_360_b')
    pubs={m:node.advertise(f'/model/{m}/cmd_vel',Twist) for m in models}
    def clock(msg):
        with lock:
            value=msg.sim.sec+msg.sim.nsec*1e-9
            if value!=state['clock']: state['clock_wall']=time.monotonic()
            state['clock']=value
    def poses(msg):
        with lock:
            for p in msg.pose:
                r,pt,y=quaternion_to_rpy(p.orientation)
                state['poses'][p.name]=(p.position.x,p.position.y,y,r,pt)
                state['pose_wall'][p.name]=time.monotonic()
    def command(model):
        def cb(msg):
            with lock: state['cmd'][model]=(msg,time.monotonic())
        return cb
    def lidar(model):
        def cb(msg):
            valid=scan_valid(msg)
            clear=min((point_clearance(v,msg.angle_min+i*msg.angle_step)
                       for i,v in enumerate(msg.ranges) if math.isfinite(v) and msg.range_min<=v<=msg.range_max),default=math.inf)
            with lock: state['lidar'][model]=(valid and clear>=1,time.monotonic())
        return cb
    callbacks=[]  # Keep Python callback references alive.
    node.subscribe(Clock,f'/world/{WORLD_NAME}/clock',clock)
    node.subscribe(Pose_V,POSE_TOPIC,poses)
    for m,t in zip(models,('/aeb360/lidar','/aeb360b/lidar')):
        c=command(m); l=lidar(m); callbacks.extend((c,l))
        node.subscribe(Twist,f'/model/{m}/requested_cmd_vel_v6',c)
        node.subscribe(LaserScan,t,l)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        while True:
            now=time.monotonic()
            with lock:
                clear=scene_clearance(state['poses'])
                safe=(clear is not None and clear>=1 and now-state['clock_wall']<.5)
                for m in models:
                    p=next((v for k,v in state['poses'].items() if k==m or k.endswith('::'+m)),None)
                    pose_stamp=next((v for k,v in state['pose_wall'].items() if k==m or k.endswith('::'+m)),0)
                    safe=safe and now-pose_stamp<.5 and p is not None and abs(p[3])<=math.radians(25) and abs(p[4])<=math.radians(30)
                    good,stamp=state['lidar'].get(m,(False,0))
                    safe=safe and good and now-stamp<.5
                for m in models:
                    cmd,stamp=state['cmd'].get(m,(Twist(),0))
                    pubs[m].publish(cmd if safe and now-stamp<.3 else Twist())
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            for pub in pubs.values(): pub.publish(Twist())
            time.sleep(.02)

if __name__=='__main__': main()
