#!/usr/bin/env python3
"""Single executable entry point for the R2100 HIL runtime."""

from __future__ import annotations

import argparse
import importlib
import signal
from pathlib import Path
import sys

from signal_mapping import load_signal_mapping, print_summary


COMPONENTS = {
    "perception": "r2100_lidar_perception",
    "gateway": "r2100_can_gateway",
    "aeb": "r2100_aeb_controller",
}


def deployment_root() -> Path:
    return Path(sys.argv[0]).resolve().parent


def default_config_path() -> Path:
    return deployment_root() / "config" / "ruskotunturi_hil.json"


def has_config_argument(args: list[str]) -> bool:
    return any(
        arg == "--config" or arg.startswith("--config=")
        for arg in args
    )


def print_help() -> None:
    print("R2100 HIL deployment executable")
    print()
    print("Usage:")
    print("  r2100_hil.pyz check [--config FILE]")
    print("  r2100_hil.pyz perception [OPTIONS]")
    print("  r2100_hil.pyz gateway [OPTIONS]")
    print("  r2100_hil.pyz aeb [OPTIONS]")
    print()
    print(f"Default config: {default_config_path()}")


def run_check(args: list[str]) -> None:
    parser = argparse.ArgumentParser(
        prog="r2100_hil.pyz check",
        description="Validate the deployed HIL configuration",
    )
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
    )

    parsed = parser.parse_args(args)

    mapping = load_signal_mapping(parsed.config)

    print()
    print("Deployment configuration: PASS")
    print()
    print_summary(mapping)


def run_component(command: str, args: list[str]) -> None:
    module_name = COMPONENTS[command]

    if not has_config_argument(args):
        args = [
            "--config",
            str(default_config_path()),
            *args,
        ]

    module = importlib.import_module(module_name)

    sys.argv = [
        f"r2100_hil.pyz {command}",
        *args,
    ]

    module.main()


def _handle_sigterm(_signum, _frame) -> None:
    raise KeyboardInterrupt


def main() -> None:
    signal.signal(signal.SIGTERM, _handle_sigterm)

    if len(sys.argv) < 2:
        print_help()
        return

    command = sys.argv[1]

    if command in {"-h", "--help", "help"}:
        print_help()
        return

    args = sys.argv[2:]

    if command == "check":
        run_check(args)
        return

    if command not in COMPONENTS:
        raise SystemExit(
            f"unknown component {command!r}; "
            f"expected check, perception, gateway or aeb"
        )

    run_component(command, args)


if __name__ == "__main__":
    main()
