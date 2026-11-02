#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[3]
HIL_DIR = ROOT / "edge-path-project" / "hil"
sys.path.insert(0, str(HIL_DIR))
from stage_signal_config import write_stage_config  # noqa: E402

SCENARIOS = {
    "emergency_braking": (
        ROOT / "edge-path-project" / "gazebo" / "scenarios" / "emergency_braking.json"
    )
}


def load_scenario(name):
    path = SCENARIOS[name]
    data = json.loads(path.read_text())
    if data.get("schema_version") != 1:
        raise SystemExit("unsupported scenario schema version")
    if data.get("name") != name:
        raise SystemExit("scenario name does not match file")
    interface = ROOT / data.get("signal_interface", "")
    if not interface.is_file():
        raise SystemExit(f"signal interface not found: {interface}")
    return path, data


def generate_target_config(scenario, target, output_dir: Path) -> Path:
    interface_path = ROOT / scenario["signal_interface"]
    output_path = output_dir / f"{scenario['name']}_{target}_signals.json"
    write_stage_config(interface_path, target, output_path)
    return output_path


def build_command(args, scenario, stage_config_path: Path):
    target = scenario["targets"][args.target]
    runner = ROOT / target["runner"]
    if not runner.is_file():
        raise SystemExit(f"target runner not found: {runner}")

    command = [sys.executable, "-u", str(runner)]
    command.extend(target.get("arguments", []))

    if target.get("uses_generated_signal_config") and args.target == "sil":
        command.extend(["--stage-config", str(stage_config_path)])

    if target.get("requires_r2100_host"):
        if not args.r2100_host:
            raise SystemExit("--r2100-host is required for --target hil")
        command.extend(["--r2100-host", args.r2100_host])
    return command


def main():
    parser = argparse.ArgumentParser(description="Run one staged SIL/HIL validation scenario.")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="emergency_braking")
    parser.add_argument("--target", choices=("sil", "hil"), required=True)
    parser.add_argument("--r2100-host", help="R2100 address for HIL target")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    _scenario_path, scenario = load_scenario(args.scenario)
    with tempfile.TemporaryDirectory(prefix="edgepath-stage-") as directory:
        stage_config = generate_target_config(scenario, args.target, Path(directory))
        command = build_command(args, scenario, stage_config)

        print("Staged scenario")
        print()
        print(f"Scenario      : {scenario['name']}")
        print(f"Target        : {args.target}")
        print(f"World         : {scenario['world']}")
        print(f"Signal config : {stage_config}")
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
        completed = subprocess.run(command, cwd=ROOT)
        raise SystemExit(completed.returncode)


if __name__ == "__main__":
    main()
