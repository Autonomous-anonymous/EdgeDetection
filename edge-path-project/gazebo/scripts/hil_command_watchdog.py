#!/usr/bin/env python3
"""Independent fail-safe watchdog for the HIL Vehicle B command stream.

The watchdog does not publish while the command stream is healthy.
After at least one command has been observed, loss of command updates
for the configured timeout latches the watchdog into STOP mode.

The latch is intentionally not cleared during the same process run.
"""

import argparse
import signal
import threading
import time


DEFAULT_COMMAND_TIMEOUT_S = 0.30
DEFAULT_PUBLISH_PERIOD_S = 0.02


class CommandWatchdogState:
    def __init__(self, timeout_s=DEFAULT_COMMAND_TIMEOUT_S):
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")

        self.timeout_s = float(timeout_s)
        self.armed = False
        self.latched = False
        self.last_command_time = None
        self.latch_time = None

    def observe_command(self, now):
        if self.latched:
            return

        self.armed = True
        self.last_command_time = float(now)

    def command_age(self, now):
        if self.last_command_time is None:
            return None

        return float(now) - self.last_command_time

    def evaluate(self, now):
        if self.latched:
            return True

        if not self.armed:
            return False

        age = self.command_age(now)

        if age is not None and age >= self.timeout_s:
            self.latched = True
            self.latch_time = float(now)

        return self.latched


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Independent fail-safe watchdog for "
            "Gazebo Vehicle B HIL commands."
        )
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_COMMAND_TIMEOUT_S,
        help="maximum command age before fail-safe STOP",
    )

    parser.add_argument(
        "--period",
        type=float,
        default=DEFAULT_PUBLISH_PERIOD_S,
        help="STOP publish period after watchdog latch",
    )

    args = parser.parse_args()

    if args.period <= 0.0:
        parser.error("--period must be positive")

    # Keep Gazebo dependencies out of module import so the watchdog state
    # can be unit-tested without Gazebo installed.
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    from ruskotunturi_phase6_two_vehicle_test import (
        CMD_B_TOPIC,
    )

    state = CommandWatchdogState(
        timeout_s=args.timeout
    )

    node = Node()
    lock = threading.Lock()

    stop_publisher = node.advertise(
        CMD_B_TOPIC,
        Twist,
    )

    def command_callback(_msg):
        with lock:
            state.observe_command(
                time.monotonic()
            )

    if not node.subscribe(
        Twist,
        CMD_B_TOPIC,
        command_callback,
    ):
        raise RuntimeError(
            f"failed to subscribe to {CMD_B_TOPIC}"
        )

    print("HIL command watchdog")
    print()
    print(f"Vehicle B command topic : {CMD_B_TOPIC}")
    print(f"Command timeout         : {args.timeout:.3f} s")
    print(f"STOP publish period     : {args.period:.3f} s")
    print()
    print("Waiting for the first Vehicle B command...")

    reported_latch = False

    def terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(
        signal.SIGTERM,
        terminate,
    )

    try:
        while True:
            now = time.monotonic()

            with lock:
                latched = state.evaluate(now)
                armed = state.armed
                age = state.command_age(now)

            if armed and latched:
                if not reported_latch:
                    print()
                    print("*** HIL COMMAND WATCHDOG TRIGGERED ***")
                    print(
                        f"Command age: {age:.3f} s"
                    )
                    print("Fail-safe state: STOP LATCHED")
                    print()

                    reported_latch = True

                # Once latched, continuously publish zero commands.
                stop_publisher.publish(
                    Twist()
                )

            time.sleep(args.period)

    except KeyboardInterrupt:
        print()
        print("Watchdog stopping.")

    finally:
        # Always leave Vehicle B with repeated zero commands when this
        # independent safety process exits.
        for _ in range(10):
            stop_publisher.publish(
                Twist()
            )
            time.sleep(0.02)


if __name__ == "__main__":
    main()
