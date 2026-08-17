from __future__ import annotations

import math
import unittest

from relay_control.config import ConfigError, parse_config
from relay_control.model import SensorFrame
from tests.helpers import synthetic_config_dict


class ConfigContractTests(unittest.TestCase):
    def test_frame_allows_absent_signal(self) -> None:
        frame = SensorFrame(sequence=1, captured_at_ns=10, samples={})

        self.assertIsNone(frame.sample("camera.posture_margin"))

    def test_config_rejects_withdrawal_faster_than_support_increase(self) -> None:
        raw = synthetic_config_dict()
        raw["adaptive"]["k_down"] = raw["adaptive"]["k_up"]

        with self.assertRaisesRegex(ConfigError, "k_up"):
            parse_config(raw)

    def test_config_requires_synthetic_warning(self) -> None:
        raw = synthetic_config_dict()
        raw["metadata"]["human_use"] = True

        with self.assertRaisesRegex(ConfigError, "human use"):
            parse_config(raw)

    def test_config_rejects_nonfinite_control_parameter(self) -> None:
        raw = synthetic_config_dict()
        raw["wheel"]["virtual_mass"] = math.inf

        with self.assertRaisesRegex(ConfigError, "finite"):
            parse_config(raw)


if __name__ == "__main__":
    unittest.main()
