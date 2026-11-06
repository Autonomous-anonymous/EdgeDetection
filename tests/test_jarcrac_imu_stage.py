"""Offline regressions: experimental IMU-world generation and frame correction."""
from __future__ import annotations
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'edge-path-project/gazebo/scripts'))
from jarcrac_imu_stage import (
    IMU_TOPIC, MOUNT_TILT_DEG, add_imu_world, correct_imu_orientation,
    paired_analysis, qconj, qmul, q_rpy, q_to_rpy, quat_error_deg, wrap_deg,
)


class QuaternionTests(unittest.TestCase):
    def assert_quat_near(self, a, b, tol=1e-5):
        self.assertLess(quat_error_deg(a, b), tol)

    def test_identity(self):
        self.assert_quat_near(q_rpy(0,0,0), (0.,0.,0.,1.))

    def test_roll_pitch_roundtrip(self):
        r,p,y = q_to_rpy(q_rpy(5., -2., 30.))
        self.assertAlmostEqual(r,5.,places=7)
        self.assertAlmostEqual(p,-2.,places=7)
        self.assertAlmostEqual(y,30.,places=7)

    def test_mount_correction_at_zero_vehicle_orientation(self):
        self.assert_quat_near(
            correct_imu_orientation(q_rpy(*MOUNT_TILT_DEG), MOUNT_TILT_DEG),
            q_rpy(0,0,0))

    def test_mount_correction_with_vehicle_tilt(self):
        body = q_rpy(3.,-4.,20.)
        actual = qmul(body,q_rpy(*MOUNT_TILT_DEG))
        self.assert_quat_near(correct_imu_orientation(actual,MOUNT_TILT_DEG),body)

    def test_zero_mount_does_not_change_attitude(self):
        body = q_rpy(3.,1.,-140.)
        self.assert_quat_near(correct_imu_orientation(body,(0.,0.,0.)),body)

    def test_quat_error_sign_invariance(self):
        q = q_rpy(8,3,11)
        self.assertAlmostEqual(quat_error_deg(q,tuple(-v for v in q)),0.,places=4)

    def test_quat_difference(self):
        self.assertAlmostEqual(quat_error_deg(q_rpy(0,0,0),q_rpy(0,5,0)),5.,places=4)

    def test_wrap(self):
        self.assertAlmostEqual(wrap_deg(359),-1)
        self.assertAlmostEqual(wrap_deg(-359),1)

    def test_multiplication_inverse(self):
        q = q_rpy(4,5,6)
        self.assert_quat_near(qmul(q,qconj(q)),q_rpy(0,0,0))


class WorldPatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        p = Path(self.tmp.name)
        self.world = p/'source_world.sdf'
        self.model = p/'model.sdf'
        self.output = p/'output.sdf'
        self.world.write_text('''<sdf version="1.10"><world name="sample">
          <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
          <include><uri>model://ruskotunturi_map</uri><name>ruskotunturi_map</name></include>
          <include><uri>model://jarcrac</uri><name>jarcrac</name>
          <pose>20 40 -10 0 0 -2.67</pose></include>
          </world></sdf>''')
        self.model.write_text('''<sdf version="1.10"><model name="jarcrac">
          <link name="chassis"><pose>0 0 0.711 0 0 0</pose>
          <sensor name="gpu_lidar_front" type="gpu_lidar"><topic>/jarcrac/lidar</topic></sensor>
          </link><plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system"/>
          </model></sdf>''')

    def tearDown(self):
        self.tmp.cleanup()

    def test_creates_isolated_imu_world_with_same_lidar(self):
        old_world = self.world.read_bytes()
        old_model = self.model.read_bytes()
        add_imu_world(self.world,self.model,self.output,'jarcrac_imu_clear')
        self.assertEqual(self.world.read_bytes(),old_world)
        self.assertEqual(self.model.read_bytes(),old_model)
        world = ET.parse(self.output).getroot().find('world')
        self.assertEqual(world.get('name'),'jarcrac_imu_clear')
        self.assertIsNotNone(world.find("./plugin[@name='gz::sim::systems::Imu']"))
        model = world.find("./model[@name='jarcrac']")
        self.assertIsNotNone(model)
        self.assertEqual(model.findtext('pose'),'20 40 -10 0 0 -2.67')
        self.assertEqual(model.findtext("./link/sensor[@type='gpu_lidar']/topic"),'/jarcrac/lidar')
        self.assertIsNotNone(model.find("./plugin[@name='gz::sim::systems::DiffDrive']"))
        imu = model.find("./link/sensor[@type='imu']")
        self.assertEqual(imu.findtext('topic'),IMU_TOPIC)
        self.assertEqual(imu.findtext('update_rate'),'100')
        self.assertEqual(imu.findtext('imu/orientation_reference_frame/localization'),'ENU')
        self.assertEqual(len(world.findall("./include[uri='model://jarcrac']")),0)
        self.assertIsNotNone(world.find("./include[uri='model://ruskotunturi_map']"))

    def test_mount_tilt_is_configured_in_radians(self):
        add_imu_world(self.world,self.model,self.output,'test_tilt',mount_rpy_deg=MOUNT_TILT_DEG)
        world=ET.parse(self.output).getroot().find('world')
        imu=world.find("./model/link/sensor[@type='imu']")
        rx, ry, rz = list(map(float,imu.findtext('pose').split()))[3:]
        self.assertAlmostEqual(rx,math.radians(MOUNT_TILT_DEG[0]))
        self.assertAlmostEqual(ry,math.radians(MOUNT_TILT_DEG[1]))
        self.assertAlmostEqual(rz,0.)

    def test_missing_vehicle_include_is_rejected(self):
        self.world.write_text('<sdf version="1.10"><world name="test"/></sdf>')
        with self.assertRaises(ValueError):
            add_imu_world(self.world,self.model,self.output,'test')

    def test_missing_pose_rejected(self):
        text = self.world.read_text()
        self.world.write_text(text.replace('<pose>20 40 -10 0 0 -2.67</pose>',''))
        with self.assertRaises(ValueError):
            add_imu_world(self.world,self.model,self.output,'test')

    def test_existing_imu_rejected(self):
        text=self.model.read_text()
        self.model.write_text(text.replace('</link>','<sensor name="original_imu" type="imu"/></link>'))
        with self.assertRaises(ValueError):
            add_imu_world(self.world,self.model,self.output,'test')


class AnalysisTests(unittest.TestCase):
    def sample(self, *, valid=True, objects=0, angle=0.1, detected=True):
        reference={'valid':True,'objects':objects}
        inferred={'valid':valid,'objects':objects if detected else 0}
        return {'pair_valid':True,'orientation_error_deg':angle,
                'exact':reference,'imu':inferred}

    def test_clear_accepts_three_paired_samples(self):
        s=[self.sample() for _ in range(3)]
        self.assertTrue(paired_analysis(s,expectation='clear')['diagnostic_expectation_met'])

    def test_obstacle_accepts_three_paired_samples(self):
        s=[self.sample(objects=100) for _ in range(3)]
        self.assertTrue(paired_analysis(s,expectation='obstacle')['diagnostic_expectation_met'])

    def test_missed_obstacle_not_accepted(self):
        s=[self.sample(objects=100) for _ in range(3)]
        s[1]=self.sample(objects=100,detected=False)
        self.assertFalse(paired_analysis(s,expectation='obstacle')['diagnostic_expectation_met'])

    def test_invalid_map_is_not_clear(self):
        s=[self.sample() for _ in range(3)]
        s[2]=self.sample(valid=False)
        self.assertFalse(paired_analysis(s,expectation='clear')['diagnostic_expectation_met'])

    def test_invalid_imu_pose_is_not_accepted(self):
        s=[self.sample(angle=4) for _ in range(3)]
        self.assertFalse(paired_analysis(s,expectation='clear')['diagnostic_expectation_met'])

    def test_unpaired_scan_is_not_accepted(self):
        s=[self.sample() for _ in range(3)]
        s.append({'pair_valid':False,'reason':'missing_or_stale_pose_imu_pair'})
        self.assertFalse(paired_analysis(s,expectation='clear')['diagnostic_expectation_met'])

    def test_insufficient_samples_rejected(self):
        self.assertFalse(paired_analysis([self.sample(),self.sample()],expectation='clear')['diagnostic_expectation_met'])


if __name__ == '__main__':
    unittest.main()
