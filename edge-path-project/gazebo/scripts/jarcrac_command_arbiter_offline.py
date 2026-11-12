#!/usr/bin/env python3
"""Offline policy prototype for the CAMINO JARCRAC single-owner command arbiter.

No Gazebo imports, transport Node, publishers, motors or vehicle motion.
Uses the existing independent watchdog's pure-Python health state.

This is NOT a safety-rated controller. It does not enforce unique Gazebo topic
ownership, classify obstacles, validate a route, or authorize a driving trial.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from jarcrac_detailed_watchdog_monitor import Watchdog, CHANNELS

MAX_FORWARD_MPS = 0.10
REQUEST_TTL_WALL_S = 0.25       # provisional software policy, not a stop-time bound
EXPECTED_SOURCE = 'jarcrac_ttc_controller'


@dataclass(frozen=True)
class Twist2:
    linear_x: float = 0.0
    angular_z: float = 0.0


ZERO = Twist2()


class CommandArbiterPolicy:
    """Fail-closed policy: call tick on every intended command publication cycle.

    arm() is an explicit operator action, not inferred from fresh telemetry.
    receive_request() must receive LOCAL wall-clock receipt times, never a
    controller-supplied timestamp. Disallowed requests trip a permanent latch.
    tick() returns ZERO unless every gate is satisfied at that instant.
    """

    def __init__(self, watchdog: Watchdog):
        self.watchdog = watchdog
        self.enabled = False
        self.latched_fault: str | None = None
        self.last_seq: int | None = None
        self.last_request: tuple[Twist2, float] | None = None
        self.ever_received_request = False

    def trip(self, reason: str) -> None:
        if self.latched_fault is None:
            self.latched_fault = reason
        self.enabled = False

    def arm(self, *, at: float) -> bool:
        if self.latched_fault is not None or self.enabled:
            return False
        if not math.isfinite(at):
            self.trip('invalid_wall_time')
            return False
        phase, _ = self.watchdog.state(at=at)
        if phase != 'HEALTHY_MONITOR_ONLY':
            return False
        self.enabled = True
        self.last_request = None  # never execute a request from before arming
        self.last_seq = None
        self.ever_received_request = False
        return True

    def disarm(self) -> None:
        if self.enabled:
            self.trip('operator_disarmed')
        else:
            self.enabled = False

    def receive_request(self, *, source: str, seq: int, speed: float,
                        turn: float, at: float) -> bool:
        if self.latched_fault is not None or not self.enabled:
            return False
        if source != EXPECTED_SOURCE:
            self.trip('unexpected_request_source')
            return False
        if type(seq) is not int or seq < 0:
            self.trip('invalid_request_sequence')
            return False
        if self.last_seq is not None and seq <= self.last_seq:
            self.trip('repeated_or_out_of_order_request')
            return False
        if not all(math.isfinite(v) for v in (speed, turn, at)):
            self.trip('nonfinite_request')
            return False
        if speed < 0 or speed > MAX_FORWARD_MPS + 1e-12:
            self.trip('request_outside_forward_limit')
            return False
        if turn != 0.0:
            self.trip('turning_not_enabled')
            return False
        # This time is supplied by the arbiter's subscriber, not by the client.
        if self.last_request is not None and at < self.last_request[1]:
            self.trip('receipt_clock_regression')
            return False
        self.last_seq = seq
        self.last_request = (Twist2(speed, 0.0), at)
        self.ever_received_request = True
        return True

    def tick(self, *, at: float) -> tuple[Twist2, str]:
        if self.latched_fault is not None:
            return ZERO, 'LATCHED:' + self.latched_fault
        if not math.isfinite(at):
            self.trip('invalid_wall_time')
            return ZERO, 'LATCHED:' + self.latched_fault

        phase, reason = self.watchdog.state(at=at)
        if phase == 'TRIPPED':
            self.trip('watchdog_' + reason)
            return ZERO, 'LATCHED:' + self.latched_fault
        if phase != 'HEALTHY_MONITOR_ONLY':
            if self.ever_received_request:
                self.trip('watchdog_lost_health')
                return ZERO, 'LATCHED:' + self.latched_fault
            return ZERO, 'watchdog_not_ready:' + reason
        if not self.enabled:
            return ZERO, 'operator_not_armed'
        if self.last_request is None:
            return ZERO, 'no_controller_request'

        command, received_at = self.last_request
        age = at - received_at
        if not math.isfinite(age) or age < 0:
            self.trip('invalid_request_age')
            return ZERO, 'LATCHED:' + self.latched_fault
        if age > REQUEST_TTL_WALL_S:
            self.trip('stale_controller_request')
            return ZERO, 'LATCHED:' + self.latched_fault
        return command, 'fresh_request_all_gates_healthy'


def healthy_watchdog():
    w = Watchdog()
    for channel in CHANNELS:
        if channel == 'clock':
            w.observe('clock', at=10.0, sim_time=1.0)
            w.observe('clock', at=10.10, sim_time=1.10)
        else:
            w.observe(channel, at=10.10)
    assert w.state(at=10.11)[0] == 'HEALTHY_MONITOR_ONLY'
    return w


def allowed_arbiter():
    p = CommandArbiterPolicy(healthy_watchdog())
    assert p.arm(at=10.12)
    assert p.receive_request(source=EXPECTED_SOURCE, seq=1,
                             speed=.06, turn=0.0, at=10.13)
    assert p.tick(at=10.14) == (Twist2(.06, 0.0),
                               'fresh_request_all_gates_healthy')
    return p


def selftest():
    checks = 0
    def check(title, run):
        nonlocal checks
        run()
        checks += 1
        print('PASS:', title, flush=True)

    def startup():
        p = CommandArbiterPolicy(Watchdog())
        assert p.tick(at=10.0)[0] == ZERO
        assert not p.arm(at=10.0)
        assert p.tick(at=10.0)[0] == ZERO
    check('missing sensors / startup -> ZERO, no arm', startup)

    def disabled():
        p = CommandArbiterPolicy(healthy_watchdog())
        assert p.tick(at=10.11) == (ZERO, 'operator_not_armed')
        assert not p.receive_request(source=EXPECTED_SOURCE, seq=1,
                                     speed=.06, turn=0.0, at=10.12)
        assert p.tick(at=10.13)[0] == ZERO
    check('healthy telemetry never automatically arms motion', disabled)

    def no_request():
        p = CommandArbiterPolicy(healthy_watchdog())
        assert p.arm(at=10.12)
        assert p.tick(at=10.13) == (ZERO, 'no_controller_request')
    check('armed but no controller request -> ZERO', no_request)

    def bounded_request():
        p = allowed_arbiter()
        assert p.tick(at=10.15)[0].linear_x == .06
        assert p.tick(at=10.15)[0].angular_z == 0.0
    check('fresh explicit forward request passes offline policy', bounded_request)

    def watchdog_fault():
        p = allowed_arbiter()
        p.watchdog.trip('invalid_lidar_shape')
        assert p.tick(at=10.16)[0] == ZERO
        assert p.latched_fault == 'watchdog_invalid_lidar_shape'
        assert not p.arm(at=10.17)
        assert p.tick(at=10.17)[0] == ZERO
    check('watchdog fault -> permanent inhibit', watchdog_fault)

    def stale_lidar():
        p = allowed_arbiter()
        # Refresh other streams but intentionally withhold LiDAR.
        for channel in CHANNELS:
            if channel == 'lidar':
                continue
            if channel == 'clock':
                p.watchdog.observe(channel, at=12.5, sim_time=2.5)
            else:
                p.watchdog.observe(channel, at=12.5)
        assert p.tick(at=12.5)[0] == ZERO
        assert p.latched_fault == 'watchdog_stale_lidar'
    check('stale LiDAR -> ZERO + latched watchdog fault', stale_lidar)

    def stalled_clock():
        p = allowed_arbiter()
        for channel in CHANNELS:
            p.watchdog.observe(channel, at=12.5,
                               **({'sim_time': 1.10} if channel == 'clock' else {}))
        assert p.tick(at=12.5)[0] == ZERO
        assert p.latched_fault == 'watchdog_clock_not_advancing'
    check('stalled simulation clock -> ZERO + latched fault', stalled_clock)

    def expired_request():
        p = allowed_arbiter()
        assert p.tick(at=10.13 + REQUEST_TTL_WALL_S + .01)[0] == ZERO
        assert p.latched_fault == 'stale_controller_request'
        assert not p.receive_request(source=EXPECTED_SOURCE, seq=2,
                                     speed=.06, turn=0.0, at=10.5)
    check('stale TTC request -> ZERO and latch', expired_request)

    def invalid_requests():
        cases = [
            ('nan', float('nan'), 0.0, 'nonfinite_request'),
            ('infinity', float('inf'), 0.0, 'nonfinite_request'),
            ('reverse', -.01, 0.0, 'request_outside_forward_limit'),
            ('too_fast', .101, 0.0, 'request_outside_forward_limit'),
            ('turn', .06, .01, 'turning_not_enabled'),
        ]
        for label, speed, turn, reason in cases:
            p = allowed_arbiter()
            assert not p.receive_request(source=EXPECTED_SOURCE, seq=2,
                                         speed=speed, turn=turn, at=10.15), label
            assert p.latched_fault == reason, label
            assert p.tick(at=10.16)[0] == ZERO, label
    check('NaN/Inf/reverse/excess-speed/turn request rejected', invalid_requests)

    def replay():
        p = allowed_arbiter()
        assert not p.receive_request(source=EXPECTED_SOURCE, seq=1,
                                     speed=.06, turn=0.0, at=10.15)
        assert p.latched_fault == 'repeated_or_out_of_order_request'
        assert p.tick(at=10.16)[0] == ZERO
    check('duplicate request sequence -> latched ZERO', replay)

    def wrong_source():
        p = allowed_arbiter()
        assert not p.receive_request(source='other_controller', seq=2,
                                     speed=.06, turn=0.0, at=10.15)
        assert p.latched_fault == 'unexpected_request_source'
        assert p.tick(at=10.16)[0] == ZERO
    check('unrecognized source -> latched ZERO', wrong_source)

    def disarm():
        p = allowed_arbiter()
        p.disarm()
        assert p.tick(at=10.16)[0] == ZERO
        assert p.latched_fault == 'operator_disarmed'
        assert not p.arm(at=10.17)
    check('operator disarm -> permanent inhibit until new instance', disarm)

    def bad_wall_time():
        p = allowed_arbiter()
        assert p.tick(at=float('nan'))[0] == ZERO
        assert p.latched_fault == 'invalid_wall_time'
    check('invalid local time -> latched ZERO', bad_wall_time)

    print(f'ARBITER OFFLINE SELFTEST: PASS ({checks} scenarios; no Gazebo, no publisher)')


if __name__ == '__main__':
    selftest()
