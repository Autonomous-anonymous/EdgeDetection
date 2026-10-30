#!/usr/bin/env python3
"""Configuration loader and validator for HIL signal mapping."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent
    / "config"
    / "ruskotunturi_hil.json"
)

REQUIRED_SIGNALS = {
    "vehicle_speed",
    "obstacle_distance",
    "brake_command",
}

VALID_DIRECTIONS = {
    "simulation_to_controller",
    "perception_to_controller",
    "controller_to_simulation",
}

VALID_ENCODINGS = {
    "uint16_le",
    "brake_command_v1",
}


@dataclass(frozen=True)
class SignalDefinition:
    name: str
    can_id: int
    direction: str
    source: str
    encoding: str
    scale: float | None = None
    unit: str | None = None
    no_value_raw: int | None = None


@dataclass(frozen=True)
class CANDefinition:
    bitrate: int
    gateway_interface: str
    controller_interface: str


@dataclass(frozen=True)
class SignalMapping:
    schema_version: int
    profile: str
    can: CANDefinition
    signals: dict[str, SignalDefinition]

    def signal(self, name: str) -> SignalDefinition:
        try:
            return self.signals[name]
        except KeyError as exc:
            raise KeyError(f"unknown signal: {name}") from exc


def _require_dict(
    value: Any,
    field_name: str,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be an object")
    return value


def _require_nonempty_string(
    value: Any,
    field_name: str,
) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")
    return value


def _parse_can_id(
    value: Any,
    field_name: str,
) -> int:
    if isinstance(value, int):
        can_id = value

    elif isinstance(value, str):
        try:
            can_id = int(value, 0)
        except ValueError as exc:
            raise ValueError(
                f"{field_name} is not a valid CAN identifier: {value!r}"
            ) from exc

    else:
        raise ValueError(
            f"{field_name} must be an integer or numeric string"
        )

    if not 0 <= can_id <= 0x7FF:
        raise ValueError(
            f"{field_name} must be a classic 11-bit CAN ID "
            f"(0x000..0x7FF)"
        )

    return can_id


def _optional_number(
    value: Any,
    field_name: str,
) -> float | None:
    if value is None:
        return None

    if not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be numeric")

    return float(value)


def load_signal_mapping(
    path: str | Path = DEFAULT_CONFIG_PATH,
) -> SignalMapping:
    config_path = Path(path)

    try:
        raw = json.loads(config_path.read_text())
    except FileNotFoundError as exc:
        raise ValueError(
            f"signal mapping file does not exist: {config_path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"invalid JSON in signal mapping: {exc}"
        ) from exc

    root = _require_dict(raw, "root")

    schema_version = root.get("schema_version")
    if schema_version != 1:
        raise ValueError(
            f"unsupported schema_version: {schema_version!r}"
        )

    profile = _require_nonempty_string(
        root.get("profile"),
        "profile",
    )

    can_raw = _require_dict(
        root.get("can"),
        "can",
    )

    bitrate = can_raw.get("bitrate")
    if not isinstance(bitrate, int) or bitrate <= 0:
        raise ValueError("can.bitrate must be a positive integer")

    gateway_interface = _require_nonempty_string(
        can_raw.get("gateway_interface"),
        "can.gateway_interface",
    )

    controller_interface = _require_nonempty_string(
        can_raw.get("controller_interface"),
        "can.controller_interface",
    )

    if gateway_interface == controller_interface:
        raise ValueError(
            "gateway_interface and controller_interface must be different"
        )

    can = CANDefinition(
        bitrate=bitrate,
        gateway_interface=gateway_interface,
        controller_interface=controller_interface,
    )

    signals_raw = _require_dict(
        root.get("signals"),
        "signals",
    )

    missing = REQUIRED_SIGNALS - set(signals_raw)
    if missing:
        raise ValueError(
            "missing required signals: "
            + ", ".join(sorted(missing))
        )

    signals: dict[str, SignalDefinition] = {}
    used_can_ids: dict[int, str] = {}

    for name, definition_raw in signals_raw.items():
        definition = _require_dict(
            definition_raw,
            f"signals.{name}",
        )

        can_id = _parse_can_id(
            definition.get("can_id"),
            f"signals.{name}.can_id",
        )

        if can_id in used_can_ids:
            raise ValueError(
                f"duplicate CAN ID 0x{can_id:03X}: "
                f"{used_can_ids[can_id]} and {name}"
            )

        used_can_ids[can_id] = name

        direction = _require_nonempty_string(
            definition.get("direction"),
            f"signals.{name}.direction",
        )

        if direction not in VALID_DIRECTIONS:
            raise ValueError(
                f"invalid direction for {name}: {direction!r}"
            )

        source = _require_nonempty_string(
            definition.get("source"),
            f"signals.{name}.source",
        )

        encoding = _require_nonempty_string(
            definition.get("encoding"),
            f"signals.{name}.encoding",
        )

        if encoding not in VALID_ENCODINGS:
            raise ValueError(
                f"unsupported encoding for {name}: {encoding!r}"
            )

        scale = _optional_number(
            definition.get("scale"),
            f"signals.{name}.scale",
        )

        if scale is not None and scale <= 0.0:
            raise ValueError(
                f"signals.{name}.scale must be positive"
            )

        unit = definition.get("unit")
        if unit is not None:
            unit = _require_nonempty_string(
                unit,
                f"signals.{name}.unit",
            )

        no_value_raw = definition.get("no_value_raw")
        if no_value_raw is not None:
            if not isinstance(no_value_raw, int):
                raise ValueError(
                    f"signals.{name}.no_value_raw must be an integer"
                )

            if not 0 <= no_value_raw <= 0xFFFF:
                raise ValueError(
                    f"signals.{name}.no_value_raw must fit uint16"
                )

        if encoding == "uint16_le":
            if scale is None:
                raise ValueError(
                    f"signals.{name}.scale is required "
                    f"for uint16_le encoding"
                )

        elif encoding == "brake_command_v1":
            if scale is not None:
                raise ValueError(
                    f"signals.{name}.scale is not valid "
                    f"for brake_command_v1"
                )

            if no_value_raw is not None:
                raise ValueError(
                    f"signals.{name}.no_value_raw is not valid "
                    f"for brake_command_v1"
                )

        signals[name] = SignalDefinition(
            name=name,
            can_id=can_id,
            direction=direction,
            source=source,
            encoding=encoding,
            scale=scale,
            unit=unit,
            no_value_raw=no_value_raw,
        )

    return SignalMapping(
        schema_version=schema_version,
        profile=profile,
        can=can,
        signals=signals,
    )


def print_summary(mapping: SignalMapping) -> None:
    print(f"Profile             : {mapping.profile}")
    print(f"Schema version      : {mapping.schema_version}")
    print(f"CAN bitrate         : {mapping.can.bitrate}")
    print(f"Gateway interface   : {mapping.can.gateway_interface}")
    print(f"Controller interface: {mapping.can.controller_interface}")
    print()
    print("Signals:")

    for name, signal in mapping.signals.items():
        scale_text = (
            "-"
            if signal.scale is None
            else str(signal.scale)
        )

        unit_text = signal.unit or "-"

        print(
            f"  {name:20s} "
            f"CAN=0x{signal.can_id:03X} "
            f"direction={signal.direction:24s} "
            f"encoding={signal.encoding:16s} "
            f"scale={scale_text:8s} "
            f"unit={unit_text}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate and display a HIL signal mapping"
    )

    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="path to signal mapping JSON",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mapping = load_signal_mapping(args.config)

    print()
    print("Signal mapping validation: PASS")
    print()
    print_summary(mapping)


if __name__ == "__main__":
    main()
