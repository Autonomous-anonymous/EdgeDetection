"""Configuration-driven CAN signal encoding and decoding."""

from __future__ import annotations

import struct

from signal_mapping import SignalDefinition


CAN_PAYLOAD_SIZE = 8
UINT16_MAX = 0xFFFF

SUPPORTED_SCALAR_ENCODINGS = {
    "uint16_le",
}


def encode_signal_value(
    signal: SignalDefinition,
    value: float | None,
) -> bytes:
    """Encode one scalar signal according to its mapping definition."""
    if signal.encoding not in SUPPORTED_SCALAR_ENCODINGS:
        raise ValueError(
            f"unsupported scalar encoding for {signal.name}: "
            f"{signal.encoding!r}"
        )

    if signal.encoding == "uint16_le":
        return _encode_uint16_le(signal, value)

    raise AssertionError("unreachable encoding branch")


def decode_signal_value(
    signal: SignalDefinition,
    data: bytes,
) -> float | None:
    """Decode one scalar signal according to its mapping definition."""
    _require_8_bytes(data)

    if signal.encoding not in SUPPORTED_SCALAR_ENCODINGS:
        raise ValueError(
            f"unsupported scalar encoding for {signal.name}: "
            f"{signal.encoding!r}"
        )

    if signal.encoding == "uint16_le":
        return _decode_uint16_le(signal, data)

    raise AssertionError("unreachable encoding branch")


def _encode_uint16_le(
    signal: SignalDefinition,
    value: float | None,
) -> bytes:
    scale = _require_positive_scale(signal)

    if value is None:
        if signal.no_value_raw is None:
            raise ValueError(
                f"{signal.name} does not define no_value_raw"
            )

        raw = signal.no_value_raw

    else:
        if value < 0.0:
            raise ValueError(
                f"{signal.name} value must be >= 0 or None"
            )

        raw = round(value / scale)

        if not 0 <= raw <= UINT16_MAX:
            raise ValueError(
                f"{signal.name} value is outside uint16 range"
            )

        if (
            signal.no_value_raw is not None
            and raw == signal.no_value_raw
        ):
            raise ValueError(
                f"{signal.name} value encodes to reserved "
                f"no_value_raw={signal.no_value_raw}"
            )

    return struct.pack("<H6x", raw)


def _decode_uint16_le(
    signal: SignalDefinition,
    data: bytes,
) -> float | None:
    scale = _require_positive_scale(signal)

    (raw,) = struct.unpack("<H6x", data)

    if (
        signal.no_value_raw is not None
        and raw == signal.no_value_raw
    ):
        return None

    return raw * scale


def _require_positive_scale(
    signal: SignalDefinition,
) -> float:
    if signal.scale is None or signal.scale <= 0.0:
        raise ValueError(
            f"{signal.name} requires a positive scale "
            f"for encoding {signal.encoding}"
        )

    return signal.scale


def _require_8_bytes(data: bytes) -> None:
    if len(data) != CAN_PAYLOAD_SIZE:
        raise ValueError(
            f"expected exactly {CAN_PAYLOAD_SIZE} CAN data bytes, "
            f"got {len(data)}"
        )
