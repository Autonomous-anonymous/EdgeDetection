#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]

SCENARIOS = {
    "emergency_braking": (
        ROOT
        / "edge-path-project"
        / "gazebo"
        / "scenarios"
        / "emergency_braking.json"
    )
}


def load_scenario(name):
    path = SCENARIOS[name]
    data = json.loads(path.read_text())

    if data.get("schema_version") != 1:
        raise SystemExit("unsupported scenario schema version")

    if data.get("name") != name:
        raise SystemExit("scenario name does not match file")

    return path, data


def build_command(args, scenario):
    target = scenario["targets"][args.target]

    runner = ROOT / target["runner"]

    if not runner.is_file():
        raise SystemExit(
            f"target runner not found: {runner}"
        )

    command = [
        sys.executable,
        "-u",
        str(runner),
    ]

    command.extend(
        target.get("arguments", [])
    )

    if target.get("requires_r2100_host"):
        if not args.r2100_host:
            raise SystemExit(
                "--r2100-host is required for --target hil"
            )

        command.extend([
            "--r2100-host",
            args.r2100_host,
        ])

    return command


def main():
    parser = argparse.ArgumentParser(
        description="Run one staged SIL/HIL validation scenario."
    )

    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default="emergency_braking",
    )

    parser.add_argument(
        "--target",
        choices=("sil", "hil"),
        required=True,
    )

    parser.add_argument(
        "--r2100-host",
        help="R2100 address for HIL target",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the selected scenario without running it",
    )

    args = parser.parse_args()

    _scenario_path, scenario = load_scenario(
        args.scenario
    )

    command = build_command(
        args,
        scenario,
    )

    print("Staged scenario")
    print()
    print(f"Scenario : {scenario['name']}")
    print(f"Target   : {args.target}")
    print(f"World    : {scenario['world']}")
    print()

    print("Acceptance:")
    for key, value in scenario["acceptance"].items():
        print(f"  {key}: {value}")

    print()
    print("Command:")
    print("  " + " ".join(command))

    if args.dry_run:
        print()
        print("Scenario configuration: PASS")
        return

    print()
    print("Starting scenario backend...")
    print()

    completed = subprocess.run(
        command,
        cwd=ROOT,
    )

    raise SystemExit(
        completed.returncode
    )


if __name__ == "__main__":
    main()
