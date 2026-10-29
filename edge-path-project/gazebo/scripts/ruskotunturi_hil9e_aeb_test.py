#!/usr/bin/env python3

import argparse
import math
import os
import re
import signal
import subprocess
import sys
from pathlib import Path


# Vehicle B route
B_START = (-8.0, 36.0)
B_GOAL = (8.0, 24.0)

# HIL-9E obstacle
OBSTACLE_CENTER = (-3.0, 32.25)
OBSTACLE_HALF_X = 0.75
OBSTACLE_HALF_Y = 0.75

# Vehicle geometry
VEHICLE_HALF_LENGTH_M = 1.30

# PASS criteria
REQUIRED_CLEARANCE_M = 1.00
STOP_SPEED_MPS = 0.05
STOP_CONFIRM_SAMPLES = 3

VALID_AEB_REASONS = {
    "ttc_threshold",
    "emergency_distance",
}


def route_geometry():
    dx = B_GOAL[0] - B_START[0]
    dy = B_GOAL[1] - B_START[1]

    length = math.hypot(dx, dy)

    return (
        length,
        dx / length,
        dy / length,
    )


ROUTE_LENGTH, UX, UY = route_geometry()

OBSTACLE_S = (
    (OBSTACLE_CENTER[0] - B_START[0]) * UX
    + (OBSTACLE_CENTER[1] - B_START[1]) * UY
)

# Projection of the axis-aligned obstacle half-size onto B's route.
OBSTACLE_HALF_ROUTE_M = (
    OBSTACLE_HALF_X * abs(UX)
    + OBSTACLE_HALF_Y * abs(UY)
)


def physical_clearance(route_s):
    """
    Conservative longitudinal separation between
    Vehicle B front body and obstacle body.
    """
    return (
        OBSTACLE_S
        - route_s
        - VEHICLE_HALF_LENGTH_M
        - OBSTACLE_HALF_ROUTE_M
    )


def stop_child(child):
    if child.poll() is not None:
        return

    try:
        os.killpg(
            os.getpgid(child.pid),
            signal.SIGINT,
        )
        child.wait(timeout=2.0)

    except subprocess.TimeoutExpired:
        child.terminate()

        try:
            child.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            child.kill()

    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--r2100-host",
        required=True,
    )

    args = parser.parse_args()

    route_script = (
        Path(__file__).resolve().parent
        / "ruskotunturi_hil_b_route_test.py"
    )

    print()
    print("============================================================")
    print(" RUSKOTUNTURI HIL-9E - AUTOMATED AEB STOP TEST")
    print("============================================================")
    print(f"Obstacle route position : {OBSTACLE_S:.3f} m")
    print(f"Required clearance      : {REQUIRED_CLEARANCE_M:.3f} m")
    print(f"Stop speed threshold    : {STOP_SPEED_MPS:.3f} m/s")
    print()

    child = subprocess.Popen(
        [
            sys.executable,
            "-u",
            str(route_script),
            "--r2100-host",
            args.r2100_host,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )

    saw_drive = False
    saw_valid_aeb = False

    trigger_reason = None
    trigger_s = None
    stop_s = None
    stop_speed = None

    consecutive_stopped = 0
    passed = False

    try:
        assert child.stdout is not None

        for line in child.stdout:
            print(line, end="")

            if "| DRIVE | reason=none" in line:
                saw_drive = True

            s_match = re.search(
                r"\|\s*s=\s*([+-]?\d+(?:\.\d+)?)/",
                line,
            )

            v_match = re.search(
                r"\|\s*v=([+-]?\d+(?:\.\d+)?)",
                line,
            )

            reason_match = re.search(
                r"\|\s*(BRAKE|DRIVE)\s*\|\s*reason=([a-zA-Z0-9_]+)",
                line,
            )

            if not (
                s_match
                and v_match
                and reason_match
            ):
                continue

            route_s = float(s_match.group(1))
            speed = float(v_match.group(1))

            mode = reason_match.group(1)
            reason = reason_match.group(2)

            valid_aeb_brake = (
                saw_drive
                and mode == "BRAKE"
                and reason in VALID_AEB_REASONS
            )

            if valid_aeb_brake and not saw_valid_aeb:
                saw_valid_aeb = True
                trigger_reason = reason
                trigger_s = route_s

                print()
                print(
                    "*** AUTOMATED TEST: R2100 AEB TRIGGER DETECTED ***"
                )
                print(
                    f"Trigger reason           : {reason}"
                )
                print(
                    f"Trigger route position   : {route_s:.3f} m"
                )
                print(
                    f"Trigger body clearance   : "
                    f"{physical_clearance(route_s):.3f} m"
                )
                print()

            if (
                saw_valid_aeb
                and valid_aeb_brake
                and speed <= STOP_SPEED_MPS
            ):
                consecutive_stopped += 1
                stop_s = route_s
                stop_speed = speed

            else:
                consecutive_stopped = 0

            if (
                consecutive_stopped
                >= STOP_CONFIRM_SAMPLES
            ):
                clearance = physical_clearance(
                    stop_s
                )

                passed = (
                    saw_drive
                    and saw_valid_aeb
                    and clearance
                    >= REQUIRED_CLEARANCE_M
                )

                break

    finally:
        stop_child(child)

    print()
    print("============================================================")
    print(" HIL-9E AUTOMATED RESULT")
    print("============================================================")
    print(
        f"Vehicle moved            : "
        f"{'YES' if saw_drive else 'NO'}"
    )
    print(
        f"Valid R2100 AEB trigger  : "
        f"{'YES' if saw_valid_aeb else 'NO'}"
    )
    print(
        f"AEB reason               : "
        f"{trigger_reason or 'none'}"
    )

    if trigger_s is not None:
        print(
            f"AEB trigger s            : "
            f"{trigger_s:.3f} m"
        )

    if stop_s is not None:
        clearance = physical_clearance(stop_s)

        print(
            f"Confirmed stop s         : "
            f"{stop_s:.3f} m"
        )
        print(
            f"Final measured speed     : "
            f"{stop_speed:.3f} m/s"
        )
        print(
            f"Body-to-obstacle clearance: "
            f"{clearance:.3f} m"
        )

    else:
        print(
            "Confirmed stop           : NO"
        )

    print(
        f"Required clearance       : "
        f"{REQUIRED_CLEARANCE_M:.3f} m"
    )

    print()
    print(
        "HIL-9E RESULT            : "
        + ("PASS" if passed else "FAIL")
    )
    print("============================================================")

    raise SystemExit(
        0 if passed else 1
    )


if __name__ == "__main__":
    main()
