from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"

if str(HIL_DIR) not in sys.path:
    sys.path.insert(0, str(HIL_DIR))


from signal_codec import decode_signal_value, encode_signal_value
from signal_mapping import SignalDefinition, load_signal_mapping


CONFIG = HIL_DIR / "config" / "ruskotunturi_hil.json"


class SignalCodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mapping = load_signal_mapping(CONFIG)

    def test_vehicle_speed_uses_configured_scale(self):
        signal = self.mapping.signal("vehicle_speed")

        data = encode_signal_value(signal, 0.85)

        self.assertEqual(data[:2], bytes([85, 0]))
        self.assertAlmostEqual(
            decode_signal_value(signal, data),
            0.85,
        )

    def test_obstacle_distance_uses_configured_scale(self):
        signal = self.mapping.signal("obstacle_distance")

        data = encode_signal_value(signal, 1.234)

        self.assertEqual(data[:2], bytes([0xD2, 0x04]))
        self.assertAlmostEqual(
            decode_signal_value(signal, data),
            1.234,
        )

    def test_no_value_raw_round_trip(self):
        signal = self.mapping.signal("obstacle_distance")

        data = encode_signal_value(signal, None)

        self.assertEqual(data[:2], bytes([0xFF, 0xFF]))
        self.assertIsNone(
            decode_signal_value(signal, data)
        )

    def test_custom_scale_changes_encoding_without_code_change(self):
        signal = SignalDefinition(
            name="test_signal",
            can_id=0x123,
            direction="simulation_to_controller",
            source="test.source",
            encoding="uint16_le",
            scale=0.25,
            unit="test",
        )

        data = encode_signal_value(signal, 3.0)

        self.assertEqual(data[:2], bytes([12, 0]))
        self.assertAlmostEqual(
            decode_signal_value(signal, data),
            3.0,
        )

    def test_missing_scale_is_rejected(self):
        signal = replace(
            self.mapping.signal("vehicle_speed"),
            scale=None,
        )

        with self.assertRaises(ValueError):
            encode_signal_value(signal, 1.0)

    def test_negative_value_is_rejected(self):
        signal = self.mapping.signal("vehicle_speed")

        with self.assertRaises(ValueError):
            encode_signal_value(signal, -0.1)

    def test_reserved_no_value_raw_cannot_encode_real_value(self):
        signal = SignalDefinition(
            name="test_signal",
            can_id=0x123,
            direction="perception_to_controller",
            source="test.source",
            encoding="uint16_le",
            scale=1.0,
            no_value_raw=100,
        )

        with self.assertRaises(ValueError):
            encode_signal_value(signal, 100.0)

    def test_brake_command_is_not_treated_as_scalar(self):
        signal = self.mapping.signal("brake_command")

        with self.assertRaises(ValueError):
            encode_signal_value(signal, 1.0)


if __name__ == "__main__":
    unittest.main()
