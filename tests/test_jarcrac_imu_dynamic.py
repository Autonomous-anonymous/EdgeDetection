"""Offline tests of isolated JARCRAC dynamic IMU/LiDAR world and safety gates."""
from pathlib import Path
import math
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'edge-path-project/gazebo/scripts'))
from jarcrac_imu_dynamic import (
    prepare_world, sample_status, safe_to_command, bounded_distance, route_displacement,
    MAX_DRIVE_DISTANCE, CMD_TOPIC, CONTACT_TOPIC,
)


def row(*,objs=0,unknown=0.0,valid=True,orientation_error=0.0,paired=True):
    return dict(paired=paired,orientation_error_deg=orientation_error,
       exact=dict(valid=valid,objects=objs,unknown_fraction=unknown),
       imu=dict(valid=valid,objects=objs,unknown_fraction=unknown))


def gate(**kwargs):
    data=dict(age_pose=.01,age_imu=.01,age_odom=.01,age_scan=.01,
       analysis_age=.2,analysis_status='clear',contact=False,speed=.1,
       roll_deg=1,pitch_deg=2,distance=.1,yaw_change_deg=1.)
    data.update(kwargs)
    return safe_to_command(**data)


class TestDecision(unittest.TestCase):
    def test_normal_clear(self): self.assertEqual(sample_status(row()),'clear')
    def test_invalid_unpaired(self): self.assertNotEqual(sample_status(dict(paired=False,reason='stale')),'clear')
    def test_invalid_map_not_clear(self): self.assertEqual(sample_status(row(valid=False)),'invalid_map_classification')
    def test_obstacle_not_clear(self): self.assertEqual(sample_status(row(objs=1)),'unexpected_obstacle')
    def test_unknown_not_clear(self): self.assertEqual(sample_status(row(unknown=.30)),'unknown_terrain_excess')
    def test_orientation_error_not_clear(self): self.assertEqual(sample_status(row(orientation_error=3.)),'orientation_mismatch')
    def test_normal_drive(self): self.assertEqual(gate(),'clear')
    def test_negative_roll_pitch_are_valid(self): self.assertEqual(gate(roll_deg=-3.,pitch_deg=-2.),'clear')
    def test_stale_pose(self): self.assertEqual(gate(age_pose=1.),'stale_sensor')
    def test_stale_imu(self): self.assertEqual(gate(age_imu=1.),'stale_sensor')
    def test_stale_lidar(self): self.assertEqual(gate(age_scan=1.),'stale_sensor')
    def test_stale_processing(self): self.assertEqual(gate(analysis_age=4.),'stale_analysis')
    def test_obstacle_gate(self): self.assertEqual(gate(analysis_status='unexpected_obstacle'),'unexpected_obstacle')
    def test_contact_gate(self): self.assertEqual(gate(contact=True),'chassis_contact')
    def test_pitch_gate(self): self.assertEqual(gate(pitch_deg=19.),'excessive_tilt')
    def test_roll_gate(self): self.assertEqual(gate(roll_deg=19.),'excessive_tilt')
    def test_yaw_gate(self): self.assertEqual(gate(yaw_change_deg=13.),'unexpected_yaw_change')
    def test_distance_gate(self): self.assertEqual(gate(distance=.66),'distance_hard_limit')
    def test_speed_gate(self): self.assertEqual(gate(speed=.3),'excessive_measured_speed')
    def test_nan_fails_closed(self): self.assertEqual(gate(speed=math.nan),'invalid_telemetry')
    def test_distance(self): self.assertAlmostEqual(bounded_distance((1.,2.),(1.3,2.4)),.5)
    def test_projected_route(self):
        forward,left=route_displacement((20.,40.),(19.7,39.85),-153.43494882)
        self.assertAlmostEqual(forward,math.hypot(.3,.15),places=6)
        self.assertAlmostEqual(left,0,places=6)
    def test_distance_setting(self): self.assertLess(MAX_DRIVE_DISTANCE,.65)


class TestWorld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=ROOT/'edge-path-project/gazebo/worlds/jarcrac_terrain_probe_world.sdf'
        cls.model=ROOT/'edge-path-project/gazebo/models/jarcrac/model.sdf'

    def test_isolated_world_and_topics(self):
        a=self.original.read_bytes()
        b=self.model.read_bytes()
        with tempfile.TemporaryDirectory() as td:
            f=prepare_world(Path(td)/'world.sdf')
            w=ET.parse(f).getroot().find('world')
            self.assertEqual(w.get('name'),'jarcrac_imu_dynamic_clear')
            self.assertIsNotNone(w.find("./plugin[@name='gz::sim::systems::Imu']"))
            self.assertIsNotNone(w.find("./plugin[@name='gz::sim::systems::Contact']"))
            model=w.find("./model[@name='jarcrac']")
            self.assertEqual(model.findtext("./plugin[@name='gz::sim::systems::DiffDrive']/topic"),CMD_TOPIC)
            sensor=model.find("./link[@name='chassis']/sensor[@type='contact']")
            self.assertEqual(sensor.findtext('topic'),CONTACT_TOPIC)
            self.assertEqual(sensor.findtext('contact/collision'),'body_collision')
            self.assertEqual(model.findtext("./link[@name='chassis']/sensor[@type='imu']/topic"),'/jarcrac/imu')
            self.assertEqual(model.findtext("./link[@name='chassis']/sensor[@type='gpu_lidar']/topic"),'/jarcrac/lidar')
            self.assertIsNotNone(w.find("./include[uri='model://ruskotunturi_map']"))
        self.assertEqual(self.original.read_bytes(),a)
        self.assertEqual(self.model.read_bytes(),b)


if __name__=='__main__': unittest.main()
