#!/usr/bin/env python3
"""Separate scenario driver. Status is evaluation-only, never controller input.

Uses simulation time, verifies the previous commanded pose against subsequent
Gazebo observations, and fails closed if bounded retries cannot establish the commanded pose.
"""
import json
import math
import threading
import time
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.stringmsg_pb2 import StringMsg
from ruskotunturi_final_sil_demo_v6 import (
    Node, Pose_V, Boolean, WORLD_NAME, POSE_TOPIC, SET_POSE_VECTOR_SERVICE, MOVING_POSE_TIMEOUT_MS, MOVING_SPEED_MPS, MOVING_CENTER_DWELL_S,
    MOVING_OBSTACLE_MODEL, MOVING_PATH, MOVING_POSE_UPDATE_PERIOD_S,
)


REQUEST_TIMEOUT_MS = 100
MAX_ATTEMPTS = 3
RETRY_INTERVAL_S = 0.55
VERIFY_DEADLINE_S = 2.0
MAX_TRACKING_LAG_S = 2.0
MAX_POSE_STEP_M = 0.35


class ScenarioDriver:
    def set_moving_pose(self, x, y, z, yaw):
        request = Pose_V()
        pose = request.pose.add()
        pose.name = MOVING_OBSTACLE_MODEL
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        half = yaw * 0.5
        pose.orientation.z = math.sin(half)
        pose.orientation.w = math.cos(half)
        try:
            result, response = self.node.request(SET_POSE_VECTOR_SERVICE, request, Pose_V, Boolean, REQUEST_TIMEOUT_MS)
        except Exception as error:
            self.last_request_detail = f'{type(error).__name__}: {error}'
            return False
        if not result:
            self.last_request_detail = 'service request unsuccessful (timeout or transport/service failure)'
            return False
        if response is None or not response.data:
            self.last_request_detail = 'service returned a negative/empty response'
            return False
        self.last_request_detail = 'service acknowledged'
        return True

    def moving_pose_at_time(self, elapsed):
        centre_index = 2
        first_length = 0.0
        for i in range(centre_index):
            p0 = MOVING_PATH[i]
            p1 = MOVING_PATH[i + 1]
            first_length += math.dist(p0, p1)
        second_length = 0.0
        for i in range(centre_index, len(MOVING_PATH) - 1):
            second_length += math.dist(MOVING_PATH[i], MOVING_PATH[i + 1])
        first_time = first_length / MOVING_SPEED_MPS
        second_time = second_length / MOVING_SPEED_MPS
        if elapsed <= first_time:
            distance = elapsed * MOVING_SPEED_MPS
            path_start = 0
            path_end = centre_index
        elif elapsed <= first_time + MOVING_CENTER_DWELL_S:
            return (MOVING_PATH[centre_index], False)
        else:
            second_elapsed = elapsed - first_time - MOVING_CENTER_DWELL_S
            if second_elapsed >= second_time:
                return (MOVING_PATH[-1], True)
            distance = second_elapsed * MOVING_SPEED_MPS
            path_start = centre_index
            path_end = len(MOVING_PATH) - 1
        remaining = distance
        for i in range(path_start, path_end):
            p0 = MOVING_PATH[i]
            p1 = MOVING_PATH[i + 1]
            segment = math.dist(p0, p1)
            if remaining <= segment:
                f = remaining / segment if segment > 1e-09 else 0.0
                point = (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]), p0[2] + f * (p1[2] - p0[2]))
                return (point, False)
            remaining -= segment
        return (MOVING_PATH[path_end], False)

def target_for_time(driver, elapsed, confirmed_elapsed, confirmed_point):
    """Use absolute scenario time; reject excessive lag or spatial catch-up.

    Never increment scenario time by a nominal period: real update periods
    include scheduling and transport overhead and are not exactly 0.4 seconds.
    """
    if elapsed < confirmed_elapsed:
        raise ValueError('simulation time moved backwards')
    if elapsed-confirmed_elapsed > MAX_TRACKING_LAG_S:
        raise ValueError(f'obstacle trajectory lag exceeded {MAX_TRACKING_LAG_S}s')
    target,final=driver.moving_pose_at_time(elapsed)
    step=math.dist(target,confirmed_point)
    if step > MAX_POSE_STEP_M:
        raise ValueError(f'pose step {step:.3f} m exceeds {MAX_POSE_STEP_M:.3f} m')
    return target,final


def main():
    node=Node(); lock=threading.Lock()
    state={'sim':None,'poses':{},'pose_seq':0,'pose_sim':None,'obstacle_seq':0}
    def clock(msg):
        with lock: state['sim']=msg.sim.sec+msg.sim.nsec*1e-9
    def poses(msg):
        with lock:
            if any(p.name=='aeb_vehicle_360_b' or p.name.endswith('::aeb_vehicle_360_b') for p in msg.pose):
                state['pose_seq']+=1
                state['pose_sim']=state['sim']
            for p in msg.pose:
                if p.name==MOVING_OBSTACLE_MODEL or p.name.endswith('::'+MOVING_OBSTACLE_MODEL):
                    state['obstacle_seq']+=1
                state['poses'][p.name]=(p.position.x,p.position.y,p.position.z)
    node.subscribe(Clock,f'/world/{WORLD_NAME}/clock',clock)
    node.subscribe(Pose_V,POSE_TOPIC,poses)
    pub=node.advertise('/final_sil_v6/harness',StringMsg)
    driver=ScenarioDriver()
    driver.node=node
    started=False; completed=False; failures=0; start=None; last=-math.inf
    previous_b=None; previous_sim=None; previous_seq=-1; stopped_since=None; stop_verified=False; resume_verified=False
    pending=None; pending_final=False; pending_seq=0; request_wall=0
    attempts=0; last_attempt=-math.inf; request_errors=0; confirmed_elapsed=0; pending_elapsed=0; pending_errors=0
    confirmed_point=MOVING_PATH[0]
    yaw=math.atan2(MOVING_PATH[-1][1]-MOVING_PATH[0][1],MOVING_PATH[-1][0]-MOVING_PATH[0][0])
    while True:
        with lock:
            sim=state['sim']; ps=dict(state['poses']); seq=state['pose_seq']; pose_sim=state['pose_sim']; obstacle_seq=state['obstacle_seq']
        def find(name):
            return next((p for k,p in ps.items() if k==name or k.endswith('::'+name)),None)
        b=find('aeb_vehicle_360_b'); observed=find(MOVING_OBSTACLE_MODEL)
        if sim is not None and b is not None and not started and b[0]<=16.5:
            started=True; start=sim
        if seq!=previous_seq and b is not None and pose_sim is not None:
            if previous_sim is not None and pose_sim>previous_sim and previous_b is not None:
                speed=math.dist(b,previous_b)/(pose_sim-previous_sim)
                near=observed is not None and math.dist(b,observed)<8
                if started and not completed and pose_sim-start>1 and near and speed<.1:
                    if stopped_since is None: stopped_since=pose_sim
                    if pose_sim-stopped_since>=.3: stop_verified=True
                else:
                    stopped_since=None
                if completed and stop_verified and speed>.15:
                    resume_verified=True
            previous_b=b; previous_sim=pose_sim; previous_seq=seq
        wall_now=time.monotonic()
        if pending is not None:
            # Verify this obstacle's fresh pose, not an unrelated vehicle update.
            if obstacle_seq>pending_seq and observed is not None and math.dist(observed,pending)<=.12:
                completed=pending_final
                confirmed_elapsed=pending_elapsed
                confirmed_point=observed
                if attempts>1 or pending_errors:
                    print(f'[HARNESS] Pose observed after {attempts} attempt(s); total request errors={request_errors}', flush=True)
                pending=None
            elif wall_now-request_wall>=VERIFY_DEADLINE_S:
                failures+=1
                print(f'[HARNESS] INVALID: pose not verified within {VERIFY_DEADLINE_S}s; target={pending}, observed={observed}, attempts={attempts}', flush=True)
                pending=None
        if started and not completed and not failures and pending is None and sim-last>=MOVING_POSE_UPDATE_PERIOD_S:
            elapsed=sim-start
            try:
                target,final=target_for_time(driver,elapsed,confirmed_elapsed,confirmed_point)
            except ValueError as error:
                failures+=1
                print(f'[HARNESS] INVALID: {error}', flush=True)
            else:
                pending_elapsed=elapsed
                pending,pending_final=target,final
                pending_seq=obstacle_seq; request_wall=wall_now; last_attempt=-math.inf; attempts=0; pending_errors=0
                last=sim
        if pending is not None and not failures and attempts<MAX_ATTEMPTS and wall_now-last_attempt>=RETRY_INTERVAL_S:
            attempts+=1; last_attempt=wall_now
            ok=driver.set_moving_pose(*pending,yaw)
            if not ok:
                request_errors+=1; pending_errors+=1
                print(f'[HARNESS] Pose request {attempts}/{MAX_ATTEMPTS}: {getattr(driver,"last_request_detail","unsuccessful")}; checking observed pose before retry', flush=True)
        message=StringMsg(); message.data=json.dumps({'started':started,'completed':completed,'failures':failures,'request_errors':request_errors,'stop_verified':stop_verified,'resume_verified':resume_verified})
        pub.publish(message)
        time.sleep(.02)

if __name__=='__main__':
    main()
