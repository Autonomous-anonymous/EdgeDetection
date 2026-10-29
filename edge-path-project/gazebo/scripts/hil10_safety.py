"""HIL-10 freshness/stop checks. Standard library only; no Gazebo dependency."""

from dataclasses import dataclass
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time
import uuid

ODOMETRY_TIMEOUT_S = 0.50
BRAKE_TIMEOUT_S = 0.50
STOP_SPEED_MPS = 0.05
STOP_SAMPLES = 3
MAX_SPEED_MPS = 655.35  # Existing unsigned 16-bit CAN signal, 0.01 m/s per bit.
SENSOR_PACKET = struct.Struct("!4sIff")
BRAKE_PACKET = struct.Struct("!4sIBBBx")
REASONS = {0: "none", 1: "emergency_distance", 2: "ttc_threshold", 3: "sensor_timeout"}
BASE_COMMIT = "52331fde756991988cbb94f01e6d33d615c8d7df"


@dataclass(frozen=True)
class Sample:
    serial: int
    speed: float
    received: float


class OdometryMonitor:
    """Count new callback samples, never repeated polls of a cached value.

    If Gazebo provides a source timestamp, require it to advance too. Restart
    the test after resetting simulation time. All age checks use local monotonic
    time; timestamps from separate machines are never compared.
    """

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.sample = None
        self.serial = 0
        self.source_stamp = None
        self.stop_since = None
        self.stop_samples = 0

    def update(self, speed, source_stamp=None):
        now = self.clock()
        with self.lock:
            if not math.isfinite(speed) or not 0.0 <= speed <= MAX_SPEED_MPS:
                self.sample = None
                self.stop_samples = 0
                return False
            if source_stamp is None and self.source_stamp is not None:
                return False
            if source_stamp is not None:
                if self.source_stamp is not None and source_stamp <= self.source_stamp:
                    return False
                self.source_stamp = source_stamp
            previous = self.sample
            self.serial += 1
            self.sample = Sample(self.serial, speed, now)
            if previous is None or now - previous.received > ODOMETRY_TIMEOUT_S:
                self.stop_samples = 0
            if self.stop_since is not None and now > self.stop_since:
                self.stop_samples = self.stop_samples + 1 if speed <= STOP_SPEED_MPS else 0
            return True

    def snapshot(self):
        with self.lock:
            return self.sample

    def fresh(self, now=None):
        now = self.clock() if now is None else now
        sample = self.snapshot()
        return sample is not None and 0 <= now - sample.received <= ODOMETRY_TIMEOUT_S

    def begin_stop(self):
        with self.lock:
            self.stop_since = self.clock()
            self.stop_samples = 0

    def stopped(self):
        with self.lock:
            return self.fresh() and self.stop_samples >= STOP_SAMPLES


class HILLink:
    """Fresh speed -> UDP; fresh progressing brake replies -> command gate.

    An armed communication/sensor fault is latched for this test run. A fresh
    DRIVE reply during startup is allowed to release the initial stop. Normal
    TTC/emergency-distance braking is distinct from a communication fault.
    """

    def __init__(self, host=None, port=15000, *, sock=None, clock=time.monotonic):
        self.clock = clock
        self.odom = OdometryMonitor(clock)
        self.sock = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sock is None:
            self.sock.connect((host, port))
            self.sock.setblocking(False)
        self.sequence = 0
        self.last_sent_sample = 0
        self.sent_count = 0
        self.last_brake_time = None
        self.brake_active = True
        self.brake_reason = 3
        self.brake_counter = None
        self.armed = False
        self.fault = None
        self.io_error = None
        self.drop_brakes = False
        self.rejected_replies = 0

    def send_speed(self):
        now = self.clock()
        sample = self.odom.snapshot()
        if sample is None or not 0 <= now - sample.received <= ODOMETRY_TIMEOUT_S:
            return False
        if sample.serial == self.last_sent_sample:
            return False
        packet = SENSOR_PACKET.pack(b"HILS", self.sequence, sample.speed, math.nan)
        self.sock.send(packet)
        self.last_sent_sample = sample.serial
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.sent_count += 1
        return True

    def accept_brake(self, packet):
        if self.drop_brakes:
            return False
        if len(packet) != BRAKE_PACKET.size:
            self.rejected_replies += 1
            return False
        magic, _ack, active, reason, counter = BRAKE_PACKET.unpack(packet)
        valid = magic == b"HILB" and active in (0, 1) and reason in REASONS
        valid = valid and bool(active) == (reason != 0)
        if self.brake_counter is not None:
            delta = (counter - self.brake_counter) & 0xFF
            valid = valid and 0 < delta < 128
        if not valid:
            self.rejected_replies += 1
            return False
        self.brake_active = bool(active)
        self.brake_reason = reason
        self.brake_counter = counter
        self.last_brake_time = self.clock()
        # Preserve a fault even if a later packet in the same receive batch
        # reports DRIVE again. Only a new test run may clear a fault.
        if self.armed and reason == 3:
            self.fault = self.fault or "sensor_timeout"
        return True

    def pump(self):
        # Check age before draining the socket; a late reply must not erase an
        # already expired command lease after the vehicle has started moving.
        self.decision()
        try:
            self.send_speed()
            # Bound receive work so a packet backlog cannot starve the stop gate.
            for _ in range(256):
                try:
                    packet = self.sock.recv(1024)
                except BlockingIOError:
                    break
                self.accept_brake(packet)
        except OSError as exc:
            self.io_error = str(exc)
            self.fault = "udp_error"

    def brake_fresh(self):
        return self.last_brake_time is not None and 0 <= self.clock() - self.last_brake_time <= BRAKE_TIMEOUT_S

    def decision(self):
        if self.fault:
            return True, self.fault
        if not self.odom.fresh():
            reason = "odometry_timeout"
        elif not self.brake_fresh():
            reason = "brake_timeout"
        else:
            reason = REASONS[self.brake_reason]
        if self.armed and reason in {"odometry_timeout", "brake_timeout", "sensor_timeout"}:
            self.fault = reason
        return reason != "none", reason

    def apply(self, speed, steering):
        if not (math.isfinite(speed) and math.isfinite(steering) and speed >= 0):
            self.fault = self.fault or "invalid_motion_command"
        brake, reason = self.decision()
        if brake:
            return 0.0, 0.0, reason
        if speed > 0:
            self.armed = True
        return speed, steering, reason

    def close(self):
        self.sock.close()


class RunLog:
    def __init__(self, directory, mode):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:8]
        self.stem = directory / ("hil10_" + mode + "_" + run_id)
        self.handle = self.stem.with_suffix(".csv").open("x", newline="", encoding="utf-8")
        self.fields = ["monotonic_s", "sample", "speed_mps", "odom_age_s", "brake_age_s",
                       "brake_active", "brake_reason", "brake_counter", "requested_mps",
                       "applied_mps", "gate_reason", "fault", "sent_speed_packets", "stop_samples",
                       "observer_sample", "observer_speed_mps", "observer_age_s", "observer_stop_samples",
                       "route_s_m", "lateral_m", "fault_injected_s"]
        self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
        self.writer.writeheader()
        self.mode = mode

    def record(self, link, requested, applied, reason, observer=None, **extra):
        now = link.clock()
        sample = link.odom.snapshot()
        observed = observer.snapshot() if observer is not None else None
        row = dict(zip(self.fields, [now, sample.serial if sample else None,
            sample.speed if sample else None, now - sample.received if sample else None,
            now - link.last_brake_time if link.last_brake_time is not None else None,
            link.brake_active, REASONS[link.brake_reason], link.brake_counter, requested,
            applied, reason, link.fault, link.sent_count, link.odom.stop_samples,
            observed.serial if observed else None, observed.speed if observed else None,
            now - observed.received if observed else None, observer.stop_samples if observer else None]))
        row.update(extra)
        self.writer.writerow(row)
        self.handle.flush()

    def finish(self, passed, reason, **details):
        self.handle.close()
        result = dict(test="HIL-10", mode=self.mode, passed=bool(passed), end_reason=reason,
                      baseline_commit=BASE_COMMIT, odometry_timeout_s=ODOMETRY_TIMEOUT_S,
                      brake_timeout_s=BRAKE_TIMEOUT_S, stop_speed_mps=STOP_SPEED_MPS,
                      stop_samples_required=STOP_SAMPLES, **details)
        self.stem.with_suffix(".json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print("\nHIL-10 RESULT: " + ("PASS" if passed else "FAIL"))
        print("End reason: " + reason)
        print("Results: " + str(self.stem.with_suffix(".json")))
        return bool(passed)
