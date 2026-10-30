#!/usr/bin/env python3
"""Deployment preflight checks for the R2100 HIL runtime."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import re
import socket
import subprocess

from signal_mapping import DEFAULT_CONFIG_PATH, load_signal_mapping


GATEWAY_UDP_PORT = 15000
LIDAR_UDP_PORT = 15010


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str


def check_can_interface(
    interface: str,
    expected_bitrate: int,
) -> CheckResult:
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", interface],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return CheckResult(
            f"CAN {interface}",
            False,
            "'ip' command not found",
        )

    if result.returncode != 0:
        detail = result.stderr.strip() or "interface not found"
        return CheckResult(
            f"CAN {interface}",
            False,
            detail,
        )

    output = result.stdout

    first_line = output.splitlines()[0] if output else ""

    flags_match = re.search(r"<([^>]+)>", first_line)
    flags = set()

    if flags_match:
        flags = set(flags_match.group(1).split(","))

    if "UP" not in flags:
        return CheckResult(
            f"CAN {interface}",
            False,
            "interface is not UP",
        )

    state_match = re.search(
        r"\bcan state ([A-Z-]+)",
        output,
    )

    if not state_match:
        return CheckResult(
            f"CAN {interface}",
            False,
            "CAN state not reported",
        )

    can_state = state_match.group(1)

    if can_state != "ERROR-ACTIVE":
        return CheckResult(
            f"CAN {interface}",
            False,
            f"CAN state is {can_state}",
        )

    bitrate_match = re.search(
        r"\bbitrate\s+(\d+)",
        output,
    )

    if not bitrate_match:
        return CheckResult(
            f"CAN {interface}",
            False,
            "bitrate not reported",
        )

    actual_bitrate = int(bitrate_match.group(1))

    if actual_bitrate != expected_bitrate:
        return CheckResult(
            f"CAN {interface}",
            False,
            (
                f"bitrate {actual_bitrate}, "
                f"expected {expected_bitrate}"
            ),
        )

    return CheckResult(
        f"CAN {interface}",
        True,
        (
            f"UP, ERROR-ACTIVE, "
            f"bitrate={actual_bitrate}"
        ),
    )


def check_udp_port(port: int, purpose: str) -> CheckResult:
    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        return CheckResult(
            f"UDP {port}",
            False,
            f"{purpose}: {exc}",
        )
    finally:
        sock.close()

    return CheckResult(
        f"UDP {port}",
        True,
        f"available for {purpose}",
    )


def run_preflight(config: str) -> bool:
    try:
        mapping = load_signal_mapping(config)
    except ValueError as exc:
        print(f"[FAIL] config: {exc}")
        print()
        print("R2100 HIL preflight: FAIL")
        return False

    print(f"[PASS] config: {mapping.profile}")

    checks = [
        check_can_interface(
            mapping.can.gateway_interface,
            mapping.can.bitrate,
        ),
        check_can_interface(
            mapping.can.controller_interface,
            mapping.can.bitrate,
        ),
        check_udp_port(
            GATEWAY_UDP_PORT,
            "gateway",
        ),
        check_udp_port(
            LIDAR_UDP_PORT,
            "LiDAR perception",
        ),
    ]

    for check in checks:
        status = "PASS" if check.ok else "FAIL"
        print(
            f"[{status}] {check.name}: "
            f"{check.detail}"
        )

    passed = all(check.ok for check in checks)

    print()
    print(
        "R2100 HIL preflight: "
        + ("PASS" if passed else "FAIL")
    )

    return passed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate R2100 HIL runtime readiness",
    )

    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="path to HIL signal mapping JSON",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not run_preflight(args.config):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
