#!/usr/bin/env python3
"""Generate stage-specific SIL/HIL signal configuration from one interface.

The canonical interface contains the semantic signals once and target bindings for
SIL and HIL.  Selecting a stage transforms that interface into the concrete
configuration consumed by that phase.  This intentionally does not claim generic
simulator introspection; it makes phase transformation automatic and fail-closed
for the project's validated AEB signal set.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REQUIRED_SIGNALS = {
    "vehicle_speed",
    "obstacle_distance",
    "brake_command",
}
VALID_TARGETS = {"sil", "hil"}
VALID_DIRECTIONS = {
    "simulation_to_controller",
    "perception_to_controller",
    "controller_to_simulation",
}
VALID_ENCODINGS = {"uint16_le", "brake_command_v1"}


class StageConfigError(ValueError):
    """Canonical interface or generated stage configuration is invalid."""


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise StageConfigError(f"{name} must be an object")
    return value


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise StageConfigError(f"{name} must be a non-empty string")
    return value


def _can_id(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise StageConfigError(f"{name} must be an integer or numeric string")
    try:
        result = int(value, 0) if isinstance(value, str) else int(value)
    except (TypeError, ValueError) as exc:
        raise StageConfigError(f"{name} is not a valid CAN identifier") from exc
    if not 0 <= result <= 0x7FF:
        raise StageConfigError(f"{name} must be a classic 11-bit CAN ID")
    return result


def load_interface(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    try:
        raw = json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise StageConfigError(f"interface file does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise StageConfigError(f"invalid JSON in interface: {exc}") from exc

    root = _object(raw, "root")
    if root.get("schema_version") != 1:
        raise StageConfigError(
            f"unsupported schema_version: {root.get('schema_version')!r}"
        )
    _string(root.get("profile"), "profile")
    profiles = _object(root.get("profiles"), "profiles")
    for target in VALID_TARGETS:
        _string(profiles.get(target), f"profiles.{target}")

    can = _object(root.get("can"), "can")
    bitrate = can.get("bitrate")
    if not isinstance(bitrate, int) or isinstance(bitrate, bool) or bitrate <= 0:
        raise StageConfigError("can.bitrate must be a positive integer")
    gateway = _string(can.get("gateway_interface"), "can.gateway_interface")
    controller = _string(
        can.get("controller_interface"), "can.controller_interface"
    )
    if gateway == controller:
        raise StageConfigError("CAN gateway and controller interfaces must differ")

    signals = _object(root.get("signals"), "signals")
    missing = REQUIRED_SIGNALS - set(signals)
    if missing:
        raise StageConfigError(
            "missing required signals: " + ", ".join(sorted(missing))
        )

    used_ids: dict[int, str] = {}
    for name, raw_signal in signals.items():
        signal = _object(raw_signal, f"signals.{name}")
        direction = _string(signal.get("direction"), f"signals.{name}.direction")
        if direction not in VALID_DIRECTIONS:
            raise StageConfigError(f"invalid direction for {name}: {direction!r}")
        encoding = _string(signal.get("encoding"), f"signals.{name}.encoding")
        if encoding not in VALID_ENCODINGS:
            raise StageConfigError(f"unsupported encoding for {name}: {encoding!r}")

        scale = signal.get("scale")
        if encoding == "uint16_le":
            if not isinstance(scale, (int, float)) or isinstance(scale, bool) or scale <= 0:
                raise StageConfigError(
                    f"signals.{name}.scale must be positive for uint16_le"
                )
        elif scale is not None:
            raise StageConfigError(
                f"signals.{name}.scale is not valid for brake_command_v1"
            )

        bindings = _object(signal.get("bindings"), f"signals.{name}.bindings")
        for target in VALID_TARGETS:
            binding = _object(
                bindings.get(target), f"signals.{name}.bindings.{target}"
            )
            _string(binding.get("source"), f"signals.{name}.bindings.{target}.source")

        hil_id = _can_id(
            bindings["hil"].get("can_id"),
            f"signals.{name}.bindings.hil.can_id",
        )
        if hil_id in used_ids:
            raise StageConfigError(
                f"duplicate HIL CAN ID 0x{hil_id:03X}: "
                f"{used_ids[hil_id]} and {name}"
            )
        used_ids[hil_id] = name

    return root


def generate_stage_config(interface: dict[str, Any], target: str) -> dict[str, Any]:
    if target not in VALID_TARGETS:
        raise StageConfigError(
            f"unsupported target {target!r}; expected one of {sorted(VALID_TARGETS)}"
        )

    generated: dict[str, Any] = {
        "schema_version": 1,
        "profile": interface["profiles"][target],
        "target": target,
        "generated_from": "ruskotunturi_aeb_interface.json",
        "signals": {},
    }

    if target == "hil":
        generated["can"] = dict(interface["can"])

    for name, signal in interface["signals"].items():
        binding = signal["bindings"][target]
        output: dict[str, Any] = {
            "direction": signal["direction"],
            "source": binding["source"],
            "encoding": signal["encoding"],
        }
        for field in ("scale", "unit", "no_value_raw"):
            if field in signal:
                output[field] = signal[field]
        if target == "hil":
            output["can_id"] = binding["can_id"]
        generated["signals"][name] = output

    return generated


def generate_from_file(interface_path: str | Path, target: str) -> dict[str, Any]:
    return generate_stage_config(load_interface(interface_path), target)


def write_stage_config(
    interface_path: str | Path,
    target: str,
    output_path: str | Path,
) -> dict[str, Any]:
    generated = generate_from_file(interface_path, target)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(generated, indent=2) + "\n")
    return generated


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate SIL/HIL signal bindings from the canonical interface"
    )
    parser.add_argument("--interface", type=Path, required=True)
    parser.add_argument("--target", choices=sorted(VALID_TARGETS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = write_stage_config(args.interface, args.target, args.output)
    print("Stage signal configuration: PASS")
    print(f"Target : {config['target']}")
    print(f"Output : {args.output}")


if __name__ == "__main__":
    main()
