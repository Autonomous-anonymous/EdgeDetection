#!/usr/bin/env python3
"""Automated HIL runner process-termination fail-safe test."""

import argparse
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

import ruskotunturi_hil_b_route_test as legacy


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parent

WATCHDOG = SCRIPTS / "hil_command_watchdog.py"
HIL_RUNNER = SCRIPTS / "ruskotunturi_hil10_b_test.py"

MOVE_SPEED_MPS = 0.20
STOP_SPEED_MPS = 0.05
STOP_CONFIRM_SAMPLES = 3

STARTUP_TIMEOUT_S = 20.0
WATCHDOG_ZERO_LIMIT_S = 0.60
STOP_TIMEOUT_S = 3.0
STOP_HOLD_S = 1.0


class TestObserver:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()

        self.speed = None
        self.last_odom_time = None

        self.cmd_count = 0
        self.last_cmd_speed = None
        self.last_cmd_time = None

        self.moved = False
        self.drive_command_seen = False

        self.kill_time = None
        self.kill_cmd_count = None

        self.watchdog_zero_time = None

        self.stop_samples = 0
        self.stop_hold_start = None
        self.stop_verified = False

        if not self.node.subscribe(
            Odometry,
            legacy.ODOM_B_TOPIC,
            self.odom_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {legacy.ODOM_B_TOPIC}"
            )

        if not self.node.subscribe(
            Twist,
            legacy.CMD_B_TOPIC,
            self.command_callback,
        ):
            raise RuntimeError(
                f"failed to subscribe to {legacy.CMD_B_TOPIC}"
            )

    def odom_callback(self, msg):
        now = time.monotonic()

        speed = math.sqrt(
            msg.twist.linear.x ** 2
            + msg.twist.linear.y ** 2
            + msg.twist.linear.z ** 2
        )

        with self.lock:
            self.speed = speed
            self.last_odom_time = now

            if speed >= MOVE_SPEED_MPS:
                self.moved = True

            if self.watchdog_zero_time is None:
                return

            if now < self.watchdog_zero_time:
                return

            if speed <= STOP_SPEED_MPS:
                self.stop_samples += 1

                if self.stop_hold_start is None:
                    self.stop_hold_start = now

                if (
                    self.stop_samples >= STOP_CONFIRM_SAMPLES
                    and now - self.stop_hold_start >= STOP_HOLD_S
                ):
                    self.stop_verified = True
            else:
                self.stop_samples = 0
                self.stop_hold_start = None

    def command_callback(self, msg):
        now = time.monotonic()

        speed = msg.linear.x
        steering = msg.angular.z

        with self.lock:
            self.cmd_count += 1
            self.last_cmd_speed = speed
            self.last_cmd_time = now

            if speed >= MOVE_SPEED_MPS:
                self.drive_command_seen = True

            if self.kill_time is None:
                return

            if self.cmd_count <= self.kill_cmd_count:
                return

            if (
                abs(speed) <= 1e-6
                and abs(steering) <= 1e-6
                and self.watchdog_zero_time is None
            ):
                self.watchdog_zero_time = now

    def mark_kill(self):
        with self.lock:
            self.kill_time = time.monotonic()
            self.kill_cmd_count = self.cmd_count
            return self.kill_time

    def snapshot(self):
        with self.lock:
            return {
                "speed": self.speed,
                "moved": self.moved,
                "drive_command_seen": self.drive_command_seen,
                "watchdog_zero_time": self.watchdog_zero_time,
                "kill_time": self.kill_time,
                "stop_samples": self.stop_samples,
                "stop_verified": self.stop_verified,
                "last_odom_time": self.last_odom_time,
            }


def stop_process(process):
    if process is None:
        return

    if process.poll() is not None:
        return

    process.terminate()

    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Kill the running HIL route process and verify "
            "independent watchdog fail-safe stopping."
        )
    )

    parser.add_argument(
        "--r2100-host",
        required=True,
    )

    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.30,
    )

    args = parser.parse_args()

    observer = TestObserver()

    watchdog = None
    runner = None

    print()
    print("============================================================")
    print(" HIL PROCESS-TERMINATION FAIL-SAFE TEST")
    print("============================================================")
    print(f"R2100 host       : {args.r2100_host}")
    print(f"Command topic    : {legacy.CMD_B_TOPIC}")
    print(f"Odometry topic   : {legacy.ODOM_B_TOPIC}")
    print(
        f"Watchdog timeout : "
        f"{args.watchdog_timeout:.3f} s"
    )
    print()

    try:
        print("Starting independent command watchdog...")

        watchdog = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(WATCHDOG),
                "--timeout",
                str(args.watchdog_timeout),
            ],
            cwd=ROOT,
        )

        time.sleep(0.25)

        if watchdog.poll() is not None:
            print()
            print("PROCESS TERMINATION FAIL-SAFE: FAIL")
            print("Reason: watchdog_start_failed")
            return 1

        print("Starting HIL route runner...")

        runner = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(HIL_RUNNER),
                "--mode",
                "route",
                "--r2100-host",
                args.r2100_host,
            ],
            cwd=ROOT,
        )

        print()
        print("Waiting for Vehicle B to move...")

        deadline = time.monotonic() + STARTUP_TIMEOUT_S

        ready_time = None

        while time.monotonic() < deadline:
            if runner.poll() is not None:
                print()
                print("PROCESS TERMINATION FAIL-SAFE: FAIL")
                print(
                    "Reason: HIL runner exited before "
                    "Vehicle B reached moving state"
                )
                return 1

            if watchdog.poll() is not None:
                print()
                print("PROCESS TERMINATION FAIL-SAFE: FAIL")
                print(
                    "Reason: watchdog exited before injection"
                )
                return 1

            state = observer.snapshot()

            if (
                state["moved"]
                and state["drive_command_seen"]
            ):
                if ready_time is None:
                    ready_time = time.monotonic()

                # Let the normal command stream remain healthy briefly
                # before injecting the process failure.
                if time.monotonic() - ready_time >= 0.50:
                    break

            time.sleep(0.02)
        else:
            print()
            print("PROCESS TERMINATION FAIL-SAFE: FAIL")
            print("Reason: vehicle_did_not_move")
            return 1

        state = observer.snapshot()

        print()
        print(
            "Vehicle B moving: PASS "
            f"(speed={state['speed']:.3f} m/s)"
        )

        kill_time = observer.mark_kill()

        print(
            f"Injecting SIGKILL into HIL runner "
            f"PID={runner.pid}"
        )

        runner.kill()

        try:
            runner.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            print()
            print("PROCESS TERMINATION FAIL-SAFE: FAIL")
            print("Reason: runner_sigkill_failed")
            return 1

        print("Runner SIGKILL: PASS")

        print("Waiting for watchdog STOP command...")

        zero_deadline = (
            kill_time
            + WATCHDOG_ZERO_LIMIT_S
        )

        zero_time = None

        while time.monotonic() < zero_deadline:
            if watchdog.poll() is not None:
                print()
                print("PROCESS TERMINATION FAIL-SAFE: FAIL")
                print(
                    "Reason: watchdog exited after runner kill"
                )
                return 1

            state = observer.snapshot()
            zero_time = state["watchdog_zero_time"]

            if zero_time is not None:
                break

            time.sleep(0.01)

        if zero_time is None:
            print()
            print("Watchdog STOP command: FAIL")
            print("PROCESS TERMINATION FAIL-SAFE: FAIL")
            print("Reason: watchdog_stop_command_missing")
            return 1

        watchdog_latency = zero_time - kill_time

        print(
            "Watchdog STOP command: PASS "
            f"({watchdog_latency:.3f} s after SIGKILL)"
        )

        print("Waiting for verified Vehicle B stop...")

        stop_deadline = (
            kill_time
            + STOP_TIMEOUT_S
            + STOP_HOLD_S
        )

        while time.monotonic() < stop_deadline:
            state = observer.snapshot()

            if state["stop_verified"]:
                stop_latency = (
                    time.monotonic()
                    - kill_time
                )

                print(
                    "Vehicle B stopped: PASS "
                    f"(speed={state['speed']:.3f} m/s, "
                    f"samples={state['stop_samples']})"
                )
                print(
                    "Stop and hold verified: PASS "
                    f"({stop_latency:.3f} s after SIGKILL)"
                )

                print()
                print(
                    "PROCESS TERMINATION FAIL-SAFE: PASS"
                )
                return 0

            time.sleep(0.02)

        state = observer.snapshot()

        print()
        print("Vehicle B stopped: FAIL")
        print(
            "Final observed speed: "
            + (
                "unknown"
                if state["speed"] is None
                else f"{state['speed']:.3f} m/s"
            )
        )
        print(
            f"Stop confirmation samples: "
            f"{state['stop_samples']}"
        )
        print("PROCESS TERMINATION FAIL-SAFE: FAIL")
        print("Reason: stop_not_verified")
        return 1

    except KeyboardInterrupt:
        print()
        print("Test interrupted.")
        return 1

    finally:
        if runner is not None and runner.poll() is None:
            runner.kill()
            runner.wait()

        stop_process(watchdog)


if __name__ == "__main__":
    raise SystemExit(main())
