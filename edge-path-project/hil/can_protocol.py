"""CAN signal definitions and codec helpers for the first R2100 HIL milestone.

Classic CAN, 11-bit identifiers, 500 kbit/s.

Signal map
----------
0x100 VEHICLE_SPEED
    Bytes 0..1: unsigned speed in centimetres/second (0.01 m/s per bit)

0x101 OBSTACLE_DISTANCE
    Bytes 0..1: unsigned distance in millimetres
    0xFFFF means "no obstacle"

0x200 BRAKE_COMMAND
    Byte 0: brake active (0 or 1)
    Byte 1: reason code
        0 = none
        1 = emergency distance
        2 = TTC threshold
        3 = sensor timeout
    Byte 2: rolling counter (0..255)

The remaining bytes are reserved and currently transmitted as zero.
"""

from __future__ import annotations

import struct

CAN_ID_VEHICLE_SPEED = 0x100
CAN_ID_OBSTACLE_DISTANCE = 0x101
CAN_ID_BRAKE_COMMAND = 0x200

NO_OBSTACLE_RAW = 0xFFFF

BRAKE_REASON_NONE = 0
BRAKE_REASON_EMERGENCY_DISTANCE = 1
BRAKE_REASON_TTC = 2
BRAKE_REASON_SENSOR_TIMEOUT = 3

BRAKE_REASON_TEXT = {
    BRAKE_REASON_NONE: "none",
    BRAKE_REASON_EMERGENCY_DISTANCE: "emergency_distance",
    BRAKE_REASON_TTC: "ttc_threshold",
    BRAKE_REASON_SENSOR_TIMEOUT: "sensor_timeout",
}


def encode_vehicle_speed(speed_mps: float) -> bytes:
    """Encode speed as uint16 with 0.01 m/s resolution."""
    if speed_mps < 0:
        raise ValueError("speed_mps must be >= 0")

    raw = round(speed_mps * 100.0)
    if raw > 0xFFFF:
        raise ValueError("speed_mps is too large for the CAN encoding")

    return struct.pack("<H6x", raw)


def decode_vehicle_speed(data: bytes) -> float:
    """Decode vehicle speed in m/s."""
    _require_8_bytes(data)
    (raw,) = struct.unpack("<H6x", data)
    return raw / 100.0


def encode_obstacle_distance(distance_m: float | None) -> bytes:
    """Encode obstacle distance in millimetres.

    None is encoded as 0xFFFF and means that no obstacle is detected.
    """
    if distance_m is None:
        raw = NO_OBSTACLE_RAW
    else:
        if distance_m < 0:
            raise ValueError("distance_m must be >= 0 or None")

        raw = round(distance_m * 1000.0)
        if raw >= NO_OBSTACLE_RAW:
            raise ValueError(
                "distance_m must be below 65.535 m; use None for no obstacle"
            )

    return struct.pack("<H6x", raw)


def decode_obstacle_distance(data: bytes) -> float | None:
    """Decode obstacle distance in metres."""
    _require_8_bytes(data)
    (raw,) = struct.unpack("<H6x", data)

    if raw == NO_OBSTACLE_RAW:
        return None

    return raw / 1000.0


def encode_brake_command(
    active: bool,
    reason: int = BRAKE_REASON_NONE,
    counter: int = 0,
) -> bytes:
    """Encode the brake command."""
    if reason not in BRAKE_REASON_TEXT:
        raise ValueError(f"unknown brake reason: {reason}")

    if not 0 <= counter <= 0xFF:
        raise ValueError("counter must be in range 0..255")

    return struct.pack("<BBB5x", int(active), reason, counter)


def decode_brake_command(data: bytes) -> tuple[bool, int, int]:
    """Decode brake active flag, reason code and rolling counter."""
    _require_8_bytes(data)
    active, reason, counter = struct.unpack("<BBB5x", data)

    if active not in (0, 1):
        raise ValueError(f"invalid brake active value: {active}")

    return bool(active), reason, counter


def brake_reason_text(reason: int) -> str:
    """Return a human-readable brake reason."""
    return BRAKE_REASON_TEXT.get(reason, f"unknown_{reason}")


def _require_8_bytes(data: bytes) -> None:
    if len(data) != 8:
        raise ValueError(f"expected exactly 8 CAN data bytes, got {len(data)}")
