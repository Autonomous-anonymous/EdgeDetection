"""Acceptance for the integrated Ruskotunturi showcase and its loop entry point."""
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
import xml.etree.ElementTree as ET

from test_jarcrac_convoy_v2 import Box, DT, FrontFixture, SHAPES, b_ranges, surface, terrain
from test_jarcrac_cooperative_sil import integrate, projected_scan

from jarcrac_convoy_score import Score
from jarcrac_convoy_trial import DEFAULT_OBSTACLES, SHOWCASE_WALL_LIMIT, play_loop, prepare
from jarcrac_convoy_v2 import planar_scan
from jarcrac_cooperative_control import CLEARANCE, HOLD_SECONDS, STOP, State
from jarcrac_cooperative_geometry import rotate
from jarcrac_cooperative_sil import NAMES
from jarcrac_cooperative_terrain import FRONT_SENSOR
from jarcrac_showcase_mission import SHOWCASE, TURN_IN_ROOM, ShowcaseMission


def front_fixture(vehicle, frame, obstacle):
    local = frame.state(vehicle)
    ox, oy, _ = rotate(FRONT_SENSOR, vehicle.roll, vehicle.pitch, vehicle.yaw)
    scan = projected_scan(local, (obstacle.x, obstacle.y) if local.x < obstacle.x else None)
    fixture = FrontFixture(vehicle.stamp, frame, (obstacle,))
    fixture.current = replace(scan, x=vehicle.x+ox, y=vehicle.y+oy, yaw=vehicle.yaw)
    return fixture


class ShowcaseAcceptance(unittest.TestCase):
    def test_both_merge_bypass_the_shared_obstacle_and_park(self):
        trial = SHOWCASE
        frame = trial.frame
        obstacle = Box(*DEFAULT_OBSTACLES["showcase"])
        actual_obstacle = obstacle.pose(frame)
        a, b = (surface(State(1., *frame.world(*trial.start(i)), frame.yaw+trial.start_yaw[i]))
                for i in range(2))
        mission, score = ShowcaseMission(SHAPES, trial), Score(SHAPES, trial)
        while b.stamp-1. < trial.sim_timeout and not mission.done:
            a_scan = front_fixture(a, frame, obstacle)
            b_scan = planar_scan(b_ranges(b, frame, (obstacle,), a), b.stamp, b, terrain())
            request = mission.step(b.stamp, a, b, a_scan, b_scan)
            score.observe(a, b, actual_obstacle)
            a = surface(integrate(a, request.a, mission.leader.brakes, DT))
            b = surface(integrate(b, request.b, mission.follower.brakes, DT))
        report = score.result()
        self.assertTrue(mission.done and report["geometry_passed"], report)
        self.assertEqual(report["merge_order"], ["A", "B"])
        self.assertTrue(report["bypass_after_merge"])
        self.assertTrue(report["b_followed_into_queue"])
        self.assertGreaterEqual(report["b_follow_distance_m"], 3.)
        self.assertGreaterEqual(report["b_advance_during_a_bypass_m"], 1.)
        self.assertLess(report["b_stationary_pose_hold_s"], 60.)
        self.assertEqual(report["dual_merge_samples"], 0)
        self.assertGreaterEqual(report["minimum_obstacle_footprint_gap_m"], CLEARANCE)
        self.assertGreaterEqual(report["minimum_vehicle_footprint_gap_m"], CLEARANCE)
        self.assertTrue(report["a_rejoin_verified"] and report["b_rejoin_verified"])
        self.assertGreaterEqual(min(mission.leader.completed_hold_seconds,
                                    mission.follower.completed_hold_seconds), HOLD_SECONDS)
        self.assertLessEqual(report["duration_sim_s"], 165.)

    def test_waiting_point_requires_measured_head_start_and_clear_sweep(self):
        mission = ShowcaseMission(SHAPES)
        mission.merged = True
        mission.observed_obstruction_front = 8.
        mission.leader.phase = "wait"
        self.assertIsNone(mission.waiting_point(State(1., 2.5, 0.), 1.))

        mission.leader.phase = "detour"
        leader = State(1., 5., -.9)
        point = mission.waiting_point(leader, 3.)
        self.assertLessEqual(point, mission.observed_obstruction_front-TURN_IN_ROOM)
        self.assertLess(point+mission.follower.padded_bounds[0][1]+CLEARANCE, mission.reserved_entry)
        self.assertGreater(point, -5.)

        frame = SHOWCASE.frame
        a = State(1., *frame.world(leader.x, leader.y), frame.yaw, *frame.world_vector(.2, 0.))
        b = State(1., *frame.world(-5., 0.), frame.yaw)
        blocked = SimpleNamespace(stamp=1., contacts=(), covers=lambda *_: False)
        for scan in (None, blocked):
            decision = mission.step(1., a, b, None, scan)
            self.assertEqual(decision.b, STOP)
            self.assertFalse(mission.follower.detour_committed)

    def test_visual_elements_leave_physics_unchanged(self):
        with TemporaryDirectory() as directory:
            output = Path(directory) / "world.sdf"
            prepare(output, SHOWCASE, DEFAULT_OBSTACLES["showcase"])
            world = ET.parse(output).getroot().find("world")
            for name in ("loading_zone_a", "loading_zone_b", "merge_zone_marker"):
                marker = world.find(f"model[@name='{name}']")
                self.assertEqual(marker.findtext("static"), "true")
                self.assertTrue(marker.findall(".//visual"))
                self.assertFalse(marker.findall(".//collision"))
            self.assertIsNotNone(world.find("gui/plugin[@filename='VisualizeLidar']"))
            stats = world.find("gui/plugin[@filename='WorldStats']")
            self.assertIsNotNone(stats)
            self.assertEqual(stats.findtext("real_time_factor"), "true")
            self.assertEqual(stats.findtext("sim_time"), "true")
            self.assertEqual(stats.findtext("real_time"), "true")
            self.assertEqual(stats.findtext("topic"), "/world/camino_ruskotunturi_convoy_v2/stats")
            for name in NAMES:
                model = world.find(f"model[@name='{name}']")
                self.assertEqual(model.findtext(".//sensor[@type='gpu_lidar']/visualize"), "true")
            view = world.findtext("gui/plugin[@filename='MinimalScene']/camera_pose")
            self.assertEqual(len(view.split()), 6)

    def test_performance_parameters_are_explicit_in_generated_world(self):
        with TemporaryDirectory() as directory:
            baseline = Path(directory) / "baseline.sdf"
            trial = Path(directory) / "trial.sdf"
            prepare(baseline, SHOWCASE, DEFAULT_OBSTACLES["showcase"])
            prepare(trial, SHOWCASE, DEFAULT_OBSTACLES["showcase"],
                    physics_step=.002, target_rtf=1.0)
            original = ET.parse(baseline).getroot().find("world/physics")
            modified = ET.parse(trial).getroot().find("world/physics")
            self.assertEqual(original.findtext("max_step_size"), "0.001")
            self.assertEqual(modified.findtext("max_step_size"), "0.002")
            self.assertEqual(modified.findtext("real_time_factor"), "1.0")
            # Faster stepping must not remove the safety sensors.
            for name in NAMES:
                model = ET.parse(trial).getroot().find(f"world/model[@name='{name}']")
                self.assertIsNotNone(model.find(".//sensor[@type='gpu_lidar']"))

    def test_loop_uses_fresh_lap_outputs_and_stops_on_failed_run(self):
        def exit_with(status):
            child = MagicMock()
            child.wait.return_value = status
            return child

        with TemporaryDirectory() as parent:
            out = Path(parent) / "laps"
            args = SimpleNamespace(out=out, laps=4, wall_timeout=SHOWCASE_WALL_LIMIT,
                                   headless=False, no_lidar_view=False)
            with patch("jarcrac_convoy_trial.subprocess.Popen",
                       side_effect=[exit_with(0), exit_with(1)]) as launch:
                self.assertEqual(play_loop(args), 1)
            self.assertEqual(launch.call_count, 2)
            for lap, call in enumerate(launch.call_args_list, 1):
                command = call.args[0]
                self.assertIn("showcase", command)
                self.assertEqual(command[command.index("--out")+1], str(out / f"lap-{lap:03d}"))
                self.assertEqual(float(command[command.index("--wall-timeout")+1]), 300.)
                self.assertTrue(call.kwargs["start_new_session"])
