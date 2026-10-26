"""Run without Gazebo: python3 -m unittest discover -s tests -v"""
import ast
import contextlib
import io
import math
from pathlib import Path
import sys
import threading
import types
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'edge-path-project/gazebo/scripts'
sys.path.insert(0,str(SCRIPTS))
from final_sil_v6_checks import scene_clearance_detail, margins_ok, starting_scene_issue, loading_approach_clearance, scan_valid, point_clearance, footprint, polygon_distance, scene_clearance

# Execute actual controller source with only Gazebo imports removed. No surrogate
# implementation of the controller is used; tests don't instantiate transport.
tree=ast.parse((SCRIPTS/'ruskotunturi_final_sil_demo_v6.py').read_text())
tree.body=[n for n in tree.body if not (isinstance(n,ast.ImportFrom) and n.module.startswith('gz.'))]
ns={'__name__':'controller_under_test','__file__':str(SCRIPTS/'ruskotunturi_final_sil_demo_v6.py')}
exec(compile(tree,'controller_under_test','exec'),ns)
Controller=ns['FinalSilDemoLargeArea']

def scan(values):
    return types.SimpleNamespace(ranges=values,range_min=.1,range_max=30.,angle_min=-math.pi,angle_step=math.pi/360)

def controller():
    c=Controller.__new__(Controller); c.lock=threading.Lock(); c.reset_state()
    c.invalid_scan_seen=False; c.ground_min=math.inf;c.ground_min_pair="test"; c.observed={}
    c.trace_path="";c.encounter_stop_verified=False; c.encounter_resume_verified=False
    return c

class SensorTests(unittest.TestCase):
    def test_nan_scan_rejected(self): self.assertFalse(scan_valid(scan([math.nan]*721)))
    def test_positive_infinity_is_no_return(self): self.assertTrue(scan_valid(scan([math.inf]*721)))
    def test_negative_infinity_rejected(self): self.assertFalse(scan_valid(scan([-math.inf]*721)))
    def test_partial_scan_rejected(self): self.assertFalse(scan_valid(scan([10.]*100)))
    def test_invalid_range_rejected(self): self.assertFalse(scan_valid(scan([-.1]*721)))
    def test_mixed_valid_scan(self): self.assertTrue(scan_valid(scan([10.]*700+[math.nan]*21)))
    def test_invalid_data_latches(self):
        c=controller(); c.update_lidar(c.a,scan([math.nan]*721)); c.update_lidar(c.a,scan([10.]*721))
        self.assertTrue(c.a['scan_valid']); self.assertTrue(c.invalid_scan_seen)

class GeometryTests(unittest.TestCase):
    def test_side_distance_includes_wheels(self): self.assertAlmostEqual(point_clearance(3,math.pi/2),1.95)
    def test_diagonal_uses_shortest_distance(self):
        self.assertAlmostEqual(point_clearance(3,math.pi/4),math.hypot(3/math.sqrt(2)-1.35,3/math.sqrt(2)-1.05))
    def test_overlap(self):
        a=footprint((0,0,0,0,0),1,1);b=footprint((.5,0,.5,0,0),1,1)
        self.assertEqual(polygon_distance(a,b),0)
    def test_known_gap(self):
        self.assertAlmostEqual(polygon_distance(footprint((0,0,0,0,0),1,1),footprint((4,0,0,0,0),1,1)),2)
    def test_rotated_box(self):
        a=footprint((0,0,0,0,0),1,1); b=footprint((5,0,math.pi/4,0,0),1,1)
        self.assertAlmostEqual(polygon_distance(a,b),4-math.sqrt(2))
    def test_pitch_projection(self):
        flat=footprint((0,0,0,0,0),1.35,1.05,0,1.5)
        tilt=footprint((0,0,0,0,.4),1.35,1.05,0,1.5)
        self.assertGreater(max(x for x,y in tilt),max(x for x,y in flat))
    def test_missing_ground_truth_is_not_safe(self): self.assertIsNone(scene_clearance({}))
    def test_scene_vehicle_collision(self):
        poses={'aeb_vehicle_360':(0,0,0,0,0),'aeb_vehicle_360_b':(1,0,0,0,0),'final_static_obstacle':(20,0,0,0,0),'final_moving_obstacle':(30,0,0,0,0)}
        self.assertEqual(scene_clearance(poses),0)

class ControlTests(unittest.TestCase):
    def test_unannounced_obstacle_stops_b_anywhere(self):
        c=controller(); c.b['target_index']=12;c.b['front_range']=4
        self.assertFalse(c.moving_started)
        with contextlib.redirect_stdout(io.StringIO()): result=c.b_control(0)
        self.assertEqual(result,(0,0));self.assertTrue(c.b_dynamic_hold)
    def test_resume_counts_new_scans_not_control_ticks(self):
        c=controller(); c.b_dynamic_hold=True;c.b['front_range']=10;c.b['last_lidar_time']=1
        for _ in range(20): self.assertEqual(c.b_control(0),(0,0))
        self.assertEqual(c.b_dynamic_clear_count,1)
    def test_stopped_goal_stays_stopped(self):
        c=controller();c.b['goal_reached']=True
        self.assertEqual(c.b_control(0),(0,0))
    def test_failure_prevents_pass(self):
        c=controller(); c.a['goal_reached']=c.b['goal_reached']=True
        c.a['minimum_clearance_seen']=c.b['minimum_clearance_seen']=2
        for attr in ('a_loading_arrived','static_avoidance_completed','camera_used_for_control','moving_started','moving_completed','encounter_stop_verified','encounter_resume_verified'):
            setattr(c,attr,True)
        c.b_dynamic_hold_events=c.b_dynamic_resume_events=c.coordinator_profile_events=c.coordinator_release_events=1
        c.ground_min=2;c.end_reason='final_demo_complete';c.write_result=lambda *args:None
        with contextlib.redirect_stdout(io.StringIO()): self.assertTrue(c.print_result(100))
        c.moving_pose_failures=1
        with contextlib.redirect_stdout(io.StringIO()): self.assertFalse(c.print_result(100))
    def test_controller_contains_no_obstacle_schedule_gate(self):
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        method=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name=='b_control')
        attrs={n.attr for n in ast.walk(method) if isinstance(n,ast.Attribute)}
        self.assertFalse({'moving_started','moving_completed'} & attrs)


class HarnessTests(unittest.TestCase):
    def exercise(self, accepted, observed_final):
        source=ast.parse((SCRIPTS/'final_sil_v6_harness.py').read_text())
        source.body=[n for n in source.body if not (isinstance(n,ast.ImportFrom) and (n.module.startswith('gz.') or n.module=='ruskotunturi_final_sil_demo_v6'))]
        env=dict(ns);env['__name__']='harness_under_test'
        exec(compile(source,'harness_under_test','exec'),env)
        messages=[];callbacks={};ticks=[0]
        class End(Exception): pass
        class Message:
            def __init__(self): self.data=''
        class FakeNode:
            def subscribe(self, kind, topic, callback):
                callbacks[topic]=callback
                if topic.endswith('/clock'): callback(types.SimpleNamespace(sim=types.SimpleNamespace(sec=5,nsec=0)))
                if topic.endswith('/pose/info'): send_pose(False)
                return True
            def advertise(self,*args):
                return types.SimpleNamespace(publish=lambda m:messages.append(__import__('json').loads(m.data)))
        def send_pose(final):
            point=ns['MOVING_PATH'][-1 if final else 0]
            ps=[types.SimpleNamespace(name='aeb_vehicle_360_b',position=types.SimpleNamespace(x=15,y=24,z=-9.4)),types.SimpleNamespace(name=ns['MOVING_OBSTACLE_MODEL'],position=types.SimpleNamespace(x=point[0],y=point[1],z=point[2]))]
            callbacks[ns['POSE_TOPIC']](types.SimpleNamespace(pose=ps))
        def sleep(t):
            ticks[0]+=1
            if ticks[0]>120: raise End()
            send_pose(observed_final(ticks[0]) if callable(observed_final) else observed_final)
        env.update(MAX_POSE_STEP_M=100,Node=FakeNode,Clock=object,Pose_V=object,StringMsg=Message,
                   time=types.SimpleNamespace(monotonic=lambda:ticks[0]*.02,sleep=sleep))
        env['ScenarioDriver'].set_moving_pose=lambda *args:accepted
        env['ScenarioDriver'].moving_pose_at_time=lambda *args:(ns['MOVING_PATH'][-1],True)
        with self.assertRaises(End): env['main']()
        return messages
    def test_rejected_final_pose_cannot_complete(self):
        status=self.exercise(False,False)[-1]
        self.assertEqual(status['failures'],1); self.assertFalse(status['completed'])
    def test_ack_without_observation_cannot_complete(self):
        self.assertFalse(self.exercise(True,False)[-1]['completed'])
    def test_timeout_response_but_observed_endpoint_complete(self):
        status=self.exercise(False,True)[-1]
        self.assertTrue(status['completed']); self.assertEqual(status['failures'],0)
    def test_delayed_observation_recovers_within_deadline(self):
        status=self.exercise(False,lambda tick:tick>=35)[-1]
        self.assertTrue(status['completed']); self.assertEqual(status['failures'],0)
    def test_ack_and_observed_endpoint_complete(self):
        self.assertTrue(self.exercise(True,True)[-1]['completed'])

class WatchdogTests(unittest.TestCase):
    def exercise(self, valid=True):
        source=ast.parse((SCRIPTS/'final_sil_v6_watchdog.py').read_text())
        source.body=[n for n in source.body if not (isinstance(n,ast.ImportFrom) and (n.module.startswith('gz.') or n.module=='ruskotunturi_final_sil_demo_v6'))]
        env=dict(ns);env['__name__']='watchdog_under_test'
        exec(compile(source,'watchdog_under_test','exec'),env)
        sent={};now=[10.];sleeps=[0]
        class Twist:
            def __init__(self): self.linear=types.SimpleNamespace(x=0.); self.angular=types.SimpleNamespace(z=0.)
        class FakeNode:
            def advertise(self,topic,*args):
                sent[topic]=[]
                return types.SimpleNamespace(publish=lambda m:sent[topic].append(m.linear.x))
            def subscribe(self,kind,topic,cb):
                if topic.endswith('/clock'): cb(types.SimpleNamespace(sim=types.SimpleNamespace(sec=1,nsec=0)))
                elif topic.endswith('/pose/info'):
                    ps=[]
                    for name,x in [('aeb_vehicle_360',0),('aeb_vehicle_360_b',10),('final_static_obstacle',20),('final_moving_obstacle',30)]:
                        ps.append(types.SimpleNamespace(name=name,position=types.SimpleNamespace(x=x,y=0,z=0),orientation=types.SimpleNamespace(x=0,y=0,z=0,w=1)))
                    cb(types.SimpleNamespace(pose=ps))
                elif topic.endswith('/requested_cmd_vel_v6'):
                    m=Twist();m.linear.x=.9;cb(m)
                elif topic.endswith('/lidar'):
                    cb(scan([math.inf if valid else math.nan]*721))
                return True
        def sleep(t):
            sleeps[0]+=1;now[0]+=t
            if sleeps[0]==25: raise KeyboardInterrupt()
        env.update(Node=FakeNode,Clock=object,Pose_V=object,LaserScan=object,Twist=Twist,
                   signal=types.SimpleNamespace(SIGTERM=15,signal=lambda *args:None),
                   time=types.SimpleNamespace(monotonic=lambda:now[0],sleep=sleep))
        env['main']()
        return sent
    def test_healthy_commands_forwarded(self):
        for values in self.exercise().values(): self.assertEqual(values[0],.9)
    def test_stale_commands_stop(self):
        for values in self.exercise().values():
            self.assertEqual(values[16],0);self.assertEqual(values[-1],0)
    def test_invalid_scans_block_commands(self):
        for values in self.exercise(False).values(): self.assertTrue(all(v==0 for v in values))




class ClearanceReportingTests(unittest.TestCase):
    def test_ground_truth_violation_overrides_good_lidar(self):
        self.assertFalse(margins_ok(1.051,2.939,.99))
    def test_ground_truth_missing_cannot_report_safe(self):
        self.assertFalse(margins_ok(2,2,math.inf))
    def test_detail_identifies_colliding_pair(self):
        poses={'aeb_vehicle_360':(0,0,0,0,0),'aeb_vehicle_360_b':(10,0,0,0,0),'final_static_obstacle':(0,0,0,0,0),'final_moving_obstacle':(30,0,0,0,0)}
        gap,pair=scene_clearance_detail(poses)
        self.assertEqual(gap,0)
        self.assertEqual(pair,'aeb_vehicle_360 / final_static_obstacle')
    def test_outward_target_has_larger_margin(self):
        c=controller();c.a.update(x=8.9,y=37,yaw=-2.68)
        c.static_line_ux,c.static_line_uy,c.static_line_nx,c.static_line_ny=c.static_line_geometry()
        distance,offset=c.static_line_project(c.a['x'],c.a['y'])
        target=c.static_line_target(distance,-ns['STATIC_OFFSET_M'])
        self.assertLess(c.static_line_project(*target)[1],offset-2)
        self.assertEqual(ns['HARD_BODY_CLEARANCE_M'],1.0)




class HarnessTimingTests(unittest.TestCase):
    def setUp(self):
        source=ast.parse((SCRIPTS/'final_sil_v6_harness.py').read_text())
        source.body=[n for n in source.body if not (isinstance(n,ast.ImportFrom) and (n.module.startswith('gz.') or n.module=='ruskotunturi_final_sil_demo_v6'))]
        self.env=dict(ns); self.env['__name__']='timing_under_test'
        exec(compile(source,'timing_under_test','exec'),self.env)
        self.driver=self.env['ScenarioDriver']()
    def follow(self, dt):
        point=ns['MOVING_PATH'][0];confirmed=0
        for i in range(1,200):
            now=i*dt
            point,done=self.env['target_for_time'](self.driver,now,confirmed,point)
            confirmed=now
            if done:return
        self.fail('Crossing did not complete')
    def test_small_scheduler_overhead_does_not_accumulate_lag(self): self.follow(.42)
    def test_half_second_updates_do_not_accumulate_lag(self): self.follow(.5)
    def test_variable_update_periods_complete(self):
        point=ns['MOVING_PATH'][0];confirmed=0;now=0
        for i in range(200):
            now+=(.4,.42,.5,.48)[i%4]
            point,done=self.env['target_for_time'](self.driver,now,confirmed,point)
            confirmed=now
            if done:return
        self.fail('Crossing did not complete')
    def test_excessive_spatial_step_still_fails(self):
        with self.assertRaisesRegex(ValueError,'pose step'):
            self.env['target_for_time'](self.driver,1,0,ns['MOVING_PATH'][0])
    def test_excessive_time_lag_still_fails(self):
        with self.assertRaisesRegex(ValueError,'trajectory lag'):
            self.env['target_for_time'](self.driver,2.1,0,ns['MOVING_PATH'][0])
    def test_backward_clock_still_fails(self):
        with self.assertRaisesRegex(ValueError,'backwards'):
            self.env['target_for_time'](self.driver,0,1,ns['MOVING_PATH'][0])




class FreshWorldTests(unittest.TestCase):
    def setUp(self):
        self.poses={'aeb_vehicle_360':(20,40,math.atan2(-2,-4),0,0),'aeb_vehicle_360_b':(20,24,math.pi,0,0),'final_static_obstacle':(8,34,0,0,0),'final_moving_obstacle':(8,20,math.pi/2,0,0)}
    def test_initial_scene_accepted(self): self.assertIsNone(starting_scene_issue(self.poses))
    def test_displaced_vehicle_refused(self):
        self.poses['aeb_vehicle_360']=(13,37,0,0,0)
        self.assertIn('away',starting_scene_issue(self.poses))
    def test_completed_crossing_refused(self):
        self.poses['final_moving_obstacle']=(8,28,math.pi/2,0,0)
        self.assertIn('away',starting_scene_issue(self.poses))
    def test_wrong_heading_refused(self):
        self.poses['aeb_vehicle_360_b']=(20,24,0,0,0)
        self.assertIn('heading',starting_scene_issue(self.poses))
    def test_missing_model_refused(self):
        del self.poses['final_static_obstacle']
        self.assertIn('missing',starting_scene_issue(self.poses))
    def test_controller_has_no_world_control_request(self):
        text=(SCRIPTS/'ruskotunturi_final_sil_demo_v6.py').read_text()
        self.assertNotIn('reset: {all: true}',text)
        self.assertNotIn('pause: false',text)
        self.assertNotIn('self.world_control(',text)




class StartupExecutionTests(unittest.TestCase):
    setUp = FreshWorldTests.setUp
    # Only execute the preparation-specific methods here; inherited validation
    # methods are already covered by FreshWorldTests.
    def exercise_start(self, advancing=True, displaced=False):
        c=controller();c.observed=dict(self.poses)
        if displaced:c.observed['aeb_vehicle_360']=(13,37,0,0,0)
        c.sim_time=1.;c.clock_wall=10.;c.camera_frames=1
        for vehicle,name in ((c.a,'aeb_vehicle_360'),(c.b,'aeb_vehicle_360_b')):
            vehicle.update(x=self.poses[name][0],y=self.poses[name][1],pose_received=True,lidar_received=True,scan_valid=True,last_pose_time=10.,last_lidar_time=10.,min_clearance=5.)
        c.stop_repeatedly=lambda:None;c.reset_state=lambda:None
        now=[10.]
        def sleep(dt):
            now[0]+=dt
            if advancing:
                c.sim_time+=dt;c.clock_wall=now[0]
                for v in (c.a,c.b):v['last_pose_time']=v['last_lidar_time']=now[0]
        old=ns['time'];ns['time']=types.SimpleNamespace(monotonic=lambda:now[0],sleep=sleep)
        try:
            with contextlib.redirect_stdout(io.StringIO()):result=c.prepare_fresh_world()
        finally:ns['time']=old
        return result,c.end_reason
    def test_running_fresh_scene_prepares(self):self.assertTrue(self.exercise_start()[0])
    def test_stalled_world_times_out(self):self.assertEqual(self.exercise_start(False),(False,'fresh_world_not_ready'))
    def test_displaced_scene_refuses_before_start(self):self.assertEqual(self.exercise_start(displaced=True),(False,'initial_scene_mismatch'))




class MergeApproachTests(unittest.TestCase):
    def setup_pair(self):
        c=controller();c.a.update(x=0,y=30,target_index=4)
        c.b.update(x=2,y=25,target_index=4)
        return c
    def test_b_stops_before_entering_return_conflict(self):
        c=self.setup_pair()
        with contextlib.redirect_stdout(io.StringIO()):speed=c.coordinator_b_speed(.85)
        self.assertEqual(speed,0);self.assertEqual(c.coordinator_stop_events,1)
        self.assertFalse(c.b_released)
    def test_repeated_hold_is_one_stop_event(self):
        c=self.setup_pair()
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(10):self.assertEqual(c.coordinator_b_speed(.85),0)
        self.assertEqual(c.coordinator_stop_events,1)
    def test_b_slows_before_hold(self):
        c=self.setup_pair();c.b.update(x=4,y=24)
        with contextlib.redirect_stdout(io.StringIO()):speed=c.coordinator_b_speed(.85)
        self.assertGreater(speed,0);self.assertLess(speed,.85)
    def test_b_released_only_after_a_clears_merge(self):
        c=self.setup_pair();c.a.update(x=-10,y=22,target_index=7)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertGreater(c.coordinator_b_speed(.85),0)
            c.coordinator_b_speed(.85)
        self.assertTrue(c.b_released);self.assertEqual(c.coordinator_release_events,1)
    def test_a_near_merge_does_not_release_b(self):
        c=self.setup_pair();c.a.update(x=-5,y=27,target_index=6)
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(c.coordinator_b_speed(.85),0)
    def test_rejoin_skips_already_reached_waypoint(self):
        c=self.setup_pair();c.a.update(x=-.2,y=30.5,target_index=2)
        c.static_line_ux,c.static_line_uy,c.static_line_nx,c.static_line_ny=c.static_line_geometry()
        c.a_detour_phase='RETURN';c.a_mode='STATIC_DETOUR';c.static_obstacle_s_m=10
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(c.static_detour_control(),(0,0))
        self.assertTrue(c.static_avoidance_completed)
        self.assertEqual(c.a['target_index'],ns['A_MERGE_INDEX'])
    def test_nominal_hold_envelope_clear_of_return_and_merge(self):
        # Numerical geometry check only, not a vehicle-dynamics simulation.
        # B follows its last approach segment toward (-4,28); place it .30 m
        # inside the 7 m stop boundary to include a modest braking displacement.
        radius=ns['MERGE_STOP_DISTANCE_M']-.30
        bx=-4+radius*2/math.sqrt(5);by=28-radius/math.sqrt(5)
        b=footprint((bx,by,math.atan2(2,-4),math.radians(6),math.radians(6)),1.35,1.05,0,1.5)
        points=[(3.6,36.7),(1.5,34.1),(-.2,30.5),(0,30),(-4,28),(-8,24),(-12,20)]
        worst=math.inf
        for start,end in zip(points,points[1:]):
            for i in range(21):
                f=i/20;x=start[0]+f*(end[0]-start[0]);y=start[1]+f*(end[1]-start[1])
                for angle in range(0,360,15):
                    a=footprint((x,y,math.radians(angle),math.radians(6),math.radians(6)),1.35,1.05,0,1.5)
                    worst=min(worst,polygon_distance(a,b))
        self.assertGreater(worst,ns['HARD_BODY_CLEARANCE_M'])




class LoadingApproachTests(unittest.TestCase):
    def setup_pair(self, ax=-42.4, ay=-36.7, finished=True):
        c=controller();c.b_released=True
        c.a.update(x=ax,y=ay,yaw=-2.8,roll_deg=3,pitch_deg=8,goal_reached=finished)
        c.b.update(x=-39.6,y=-31,target_index=20)
        return c
    def test_old_parked_pose_blocks_loading_approach(self):
        c=self.setup_pair()
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(c.coordinator_b_speed(.85),0)
        self.assertTrue(c.loading_gate_hold)
    def test_exit_flag_alone_cannot_release_b(self):
        c=self.setup_pair(finished=True)
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(4):self.assertEqual(c.coordinator_b_speed(.85),0)
        self.assertEqual(c.loading_gate_hold_events,1)
    def test_distant_a_still_must_finish_exiting(self):
        c=self.setup_pair(ax=-48,ay=-36,finished=False)
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(c.coordinator_b_speed(.85),0)
    def test_clear_parked_a_releases_loading_approach(self):
        c=self.setup_pair(ax=-47,ay=-36)
        with contextlib.redirect_stdout(io.StringIO()):self.assertGreater(c.coordinator_b_speed(.85),0)
        self.assertFalse(c.loading_gate_hold)
    def test_gate_rechecks_if_parked_a_moves(self):
        c=self.setup_pair(ax=-48,ay=-36)
        self.assertGreater(c.coordinator_b_speed(.85),0)
        c.a.update(x=-42.4,y=-36.7)
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(c.coordinator_b_speed(.85),0)
    def test_new_exit_acceptance_circle_has_geometric_room(self):
        target=ns['A_ROUTE'][ns['A_EXIT_INDEX']]
        approach=ns['B_ROUTE'][ns['B_LOADING_INDEX']-2]
        goal=ns['LOADING_ZONE'];radius=ns['GOAL_REACHED_M']
        worst=math.inf
        for angle in range(0,360,15):
            x=target[0]+radius*math.cos(math.radians(angle))
            y=target[1]+radius*math.sin(math.radians(angle))
            for yaw in range(0,360,15):
                for roll,pitch in ((25,30),(-25,30),(25,-30),(-25,-30),(0,0)):
                    gap=loading_approach_clearance((x,y,math.radians(yaw),math.radians(roll),math.radians(pitch)),approach,goal)
                    worst=min(worst,gap)
        self.assertGreater(worst,ns['LOADING_APPROACH_RESERVED_MARGIN_M'])




class ResultDiagnosticTests(unittest.TestCase):
    def test_incomplete_mission_explains_failure(self):
        c=controller();c.write_result=lambda *args:None
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(c.print_result(0))
        self.assertIn('A_exit_reached',c.failed_checks)
        self.assertIn('B_loading_reached',c.failed_checks)
        self.assertIn('observed_B_resume',c.failed_checks)

if __name__=='__main__': unittest.main()
