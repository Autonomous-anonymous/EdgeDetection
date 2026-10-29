"""Offline regressions. Run: python3 -m unittest discover -s tests -p 'test_hil10*.py' -v"""

import contextlib
import io
import math
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "edge-path-project" / "gazebo" / "scripts"))
sys.path.insert(0, str(ROOT / "edge-path-project" / "hil"))

from hil10_safety import HILLink, OdometryMonitor, BRAKE_PACKET, SENSOR_PACKET
from r2100_aeb_controller import R2100AEBController, pack_can_frame
from can_protocol import (CAN_ID_VEHICLE_SPEED, CAN_ID_OBSTACLE_DISTANCE,
                          encode_vehicle_speed, encode_obstacle_distance)


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.incoming = []
        self.closed = False

    def send(self, packet):
        self.sent.append(packet)
        return len(packet)

    def recv(self, size):
        if self.incoming:
            return self.incoming.pop(0)
        raise BlockingIOError

    def close(self):
        self.closed = True


def brake(counter=1, active=0, reason=0):
    return BRAKE_PACKET.pack(b"HILB", 0, active, reason, counter)


def bare_aeb():
    obj = R2100AEBController.__new__(R2100AEBController)
    obj.speed_mps = obj.front_distance_m = None
    obj.last_speed_time = obj.last_distance_time = None
    obj.aeb_latched = False
    obj.latched_reason = 0
    return obj


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sock = FakeSocket()
        self.link = HILLink(sock=self.sock, clock=self.clock)

    def drive_ready(self):
        self.link.odom.update(0.85)
        self.link.accept_brake(brake())
        self.assertEqual(self.link.apply(0.85, 0.2), (0.85, 0.2, "none"))

    def test_missing_odometry_blocks_start(self):
        self.link.accept_brake(brake())
        self.assertEqual(self.link.apply(1, 0.4), (0.0, 0.0, "odometry_timeout"))
        self.assertFalse(self.link.send_speed())

    def test_startup_timeout_can_clear_before_motion(self):
        self.link.apply(1, 0.2)
        self.drive_ready()
        self.assertIsNone(self.link.fault)

    def test_ten_second_old_sample_cannot_drive_or_be_transmitted(self):
        self.link.odom.update(0.0)
        self.clock.advance(10)
        self.link.accept_brake(brake())
        self.assertFalse(self.link.send_speed())
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "odometry_timeout"))
        self.assertEqual(self.sock.sent, [])

    def test_same_measurement_is_transmitted_only_once(self):
        self.link.odom.update(0.85)
        self.assertTrue(self.link.send_speed())
        for _ in range(9):
            self.clock.advance(0.05)
            self.assertFalse(self.link.send_speed())
        self.assertEqual(len(self.sock.sent), 1)
        self.link.odom.update(0.85)
        self.assertTrue(self.link.send_speed())

    def test_odometry_loss_latches_even_after_recovery(self):
        self.drive_ready()
        self.clock.advance(0.501)
        self.link.accept_brake(brake(2))
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "odometry_timeout"))
        self.link.odom.update(0)
        self.link.accept_brake(brake(3))
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "odometry_timeout"))

    def test_brake_loss_stops_with_fresh_odometry(self):
        self.drive_ready()
        self.clock.advance(0.501)
        self.link.odom.update(0.85)
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "brake_timeout"))

    def test_stale_zero_speed_cannot_verify_stop(self):
        monitor = self.link.odom
        monitor.update(0)
        monitor.begin_stop()
        self.clock.advance(10)
        for _ in range(50):
            self.assertFalse(monitor.stopped())

    def test_three_distinct_post_stop_samples_are_required(self):
        monitor = self.link.odom
        for _ in range(3):
            monitor.update(0)
        monitor.begin_stop()
        for index in range(3):
            self.clock.advance(0.05)
            monitor.update(0.02)
            for _ in range(10):
                self.assertEqual(monitor.stopped(), index == 2)

    def test_motion_between_samples_resets_confirmation(self):
        monitor = self.link.odom
        monitor.begin_stop()
        for speed in (0, 0, 0.2, 0, 0):
            self.clock.advance(0.05)
            monitor.update(speed)
        self.assertFalse(monitor.stopped())
        self.clock.advance(0.05)
        monitor.update(0)
        self.assertTrue(monitor.stopped())

    def test_gap_and_invalid_sample_reset_confirmation(self):
        monitor = self.link.odom
        monitor.begin_stop()
        for _ in range(3):
            self.clock.advance(0.05)
            monitor.update(0)
        self.assertTrue(monitor.stopped())
        self.clock.advance(0.501)
        self.assertFalse(monitor.stopped())
        monitor.update(0)
        self.assertEqual(monitor.stop_samples, 1)
        monitor.update(math.nan)
        self.assertFalse(monitor.fresh())
        self.assertEqual(monitor.stop_samples, 0)

    def test_repeated_or_backward_source_stamp_is_not_new_odometry(self):
        monitor = self.link.odom
        monitor.begin_stop()
        self.clock.advance(0.05)
        self.assertTrue(monitor.update(0, (1, 0)))
        for stamp in ((1, 0), (0, 900), (1, 0)):
            self.clock.advance(0.2)
            self.assertFalse(monitor.update(0, stamp))
        self.assertFalse(monitor.fresh())
        self.assertFalse(monitor.stopped())

    def test_nonfinite_speeds_do_not_refresh_or_transmit(self):
        for speed in (math.nan, math.inf, -math.inf, -1, 656):
            self.link.odom.update(speed)
            self.assertFalse(self.link.send_speed())
            self.assertTrue(self.link.decision()[0])

    def test_duplicate_and_reordered_brake_packets_do_not_refresh(self):
        self.link.odom.update(1)
        self.assertTrue(self.link.accept_brake(brake(10)))
        self.clock.advance(0.6)
        self.assertFalse(self.link.accept_brake(brake(10)))
        self.assertFalse(self.link.accept_brake(brake(9)))
        self.assertFalse(self.link.brake_fresh())

    def test_brake_counter_wrap_is_accepted(self):
        for counter in (254, 255, 0, 1):
            self.assertTrue(self.link.accept_brake(brake(counter)))

    def test_malformed_or_inconsistent_brake_does_not_release(self):
        packets = [b"", b"bad packet", brake(active=0, reason=3), brake(active=1, reason=0),
                   BRAKE_PACKET.pack(b"BAD!", 0, 0, 0, 1), brake(reason=9), brake(active=2)]
        for packet in packets:
            self.assertFalse(self.link.accept_brake(packet))
        self.assertFalse(self.link.brake_fresh())

    def test_ttc_is_intervention_not_communication_fault(self):
        self.drive_ready()
        self.link.accept_brake(brake(2, 1, 2))
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "ttc_threshold"))
        self.assertIsNone(self.link.fault)

    def test_sensor_timeout_from_r2100_is_latched_after_motion(self):
        self.drive_ready()
        self.link.accept_brake(brake(2, 1, 3))
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "sensor_timeout"))
        self.assertEqual(self.link.fault, "sensor_timeout")

    def test_udp_error_fails_closed(self):
        self.link.odom.update(1)
        def broken(_packet):
            raise OSError("connection refused")
        self.sock.send = broken
        self.link.pump()
        self.assertEqual(self.link.apply(1, 0.2), (0.0, 0.0, "udp_error"))

    def test_late_reply_does_not_erase_expired_brake_lease(self):
        self.drive_ready()
        self.clock.advance(0.501)
        self.link.odom.update(0.85)
        self.sock.incoming.append(brake(2))
        self.link.pump()
        self.assertEqual(self.link.apply(1, 0), (0.0, 0.0, "brake_timeout"))

    def test_sensor_timeout_cannot_be_hidden_by_later_drive_in_same_batch(self):
        self.drive_ready()
        self.sock.incoming.extend([brake(2, 1, 3), brake(3)])
        self.link.pump()
        self.assertEqual(self.link.apply(1, 0), (0.0, 0.0, "sensor_timeout"))

    def test_invalid_motion_commands_stop(self):
        for speed, steering in ((math.nan, 0), (1, math.inf), (-1, 0)):
            self.link.fault = None
            self.drive_ready()
            self.assertEqual(self.link.apply(speed, steering),
                             (0.0, 0.0, "invalid_motion_command"))

    def test_disappearing_source_stamp_does_not_refresh(self):
        self.link.odom.update(0.85, (1, 0))
        self.clock.advance(0.501)
        self.assertFalse(self.link.odom.update(0.0))
        self.assertFalse(self.link.odom.fresh())

    def test_r2100_no_longer_receives_cached_speed_heartbeats(self):
        controller = bare_aeb()
        self.link.odom.update(0.85)
        self.link.send_speed()
        _, _, speed, _ = SENSOR_PACKET.unpack(self.sock.sent[-1])
        controller.handle_frame(pack_can_frame(CAN_ID_VEHICLE_SPEED, encode_vehicle_speed(speed)), self.clock())
        for _ in range(12):
            self.clock.advance(0.05)
            self.link.send_speed()
            controller.handle_frame(pack_can_frame(CAN_ID_OBSTACLE_DISTANCE, encode_obstacle_distance(None)), self.clock())
        self.assertEqual(len(self.sock.sent), 1)
        self.assertEqual(controller.evaluate(self.clock())[:2], (True, 3))

    def test_r2100_lidar_loss_brakes_while_speed_keeps_updating(self):
        controller = bare_aeb()
        controller.handle_frame(pack_can_frame(CAN_ID_OBSTACLE_DISTANCE, encode_obstacle_distance(None)), self.clock())
        for _ in range(12):
            self.clock.advance(0.05)
            controller.handle_frame(pack_can_frame(CAN_ID_VEHICLE_SPEED, encode_vehicle_speed(0.85)), self.clock())
        self.assertEqual(controller.evaluate(self.clock())[:2], (True, 3))


if __name__ == "__main__":
    unittest.main()
