#!/usr/bin/env python3
"""Stage-configured adapter for the existing SIL emergency-braking backend."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_stage_config(path: str | Path) -> dict:
    config = json.loads(Path(path).read_text())
    if config.get("schema_version") != 1 or config.get("target") != "sil":
        raise ValueError("expected generated SIL stage configuration")
    required = {"vehicle_speed", "obstacle_distance", "brake_command"}
    missing = required - set(config.get("signals", {}))
    if missing:
        raise ValueError("missing SIL stage signals: " + ", ".join(sorted(missing)))
    return config


def apply_stage_config(backend, config: dict) -> None:
    signals = config["signals"]
    backend.ODOM_TOPIC = signals["vehicle_speed"]["source"]
    backend.LIDAR_TOPIC = signals["obstacle_distance"]["source"]
    backend.CMD_B_TOPIC = signals["brake_command"]["source"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-config", type=Path, required=True)
    args = parser.parse_args()

    # Delayed import keeps config-only tests independent of Gazebo bindings.
    import ruskotunturi_emergency_braking_sil as backend

    config = load_stage_config(args.stage_config)
    apply_stage_config(backend, config)
    test = backend.EmergencyBrakingSIL()
    raise SystemExit(0 if test.run() else 1)


if __name__ == "__main__":
    main()
