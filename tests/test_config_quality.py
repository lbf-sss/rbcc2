from __future__ import annotations

import math
import unittest

from relay_control.config import ConfigError, parse_config
from relay_control.estimation import evaluate_margins, fuse_evidence
from relay_control.model import RuntimeMode, SensorFrame, SignalFlag, Task
from relay_control.quality import evaluate_task_quality, resolve_signals
from tests.helpers import NOW_NS, frame_without, sample, synthetic_config_dict, valid_frame


class ConfigContractTests(unittest.TestCase):
    def test_frame_allows_absent_signal(self) -> None:
        frame = SensorFrame(sequence=1, captured_at_ns=10, samples={})

        self.assertIsNone(frame.sample("camera.posture_margin"))

    def test_config_rejects_withdrawal_faster_than_support_increase(self) -> None:
        raw = synthetic_config_dict()
        raw["adaptive"]["k_down"] = raw["adaptive"]["k_up"]

        with self.assertRaisesRegex(ConfigError, "k_up"):
            parse_config(raw)


class SignalQualityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict())

    def test_missing_optional_camera_selects_degraded_hold(self) -> None:
        frame = frame_without("camera.posture_margin")

        result = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        self.assertEqual(result.mode, RuntimeMode.DEGRADED_HOLD)
        self.assertIn("optional_capability_unavailable:posture_quality", result.reasons)

    def test_missing_participation_evidence_selects_degraded_hold(self) -> None:
        frame = frame_without("sim.foot.affected_load")

        result = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        self.assertEqual(result.mode, RuntimeMode.DEGRADED_HOLD)
        self.assertIn("optional_capability_unavailable:participation_quality", result.reasons)

    def test_missing_hand_compensation_evidence_selects_degraded_hold(self) -> None:
        frame = frame_without("sim.handle.left.vertical")

        result = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        self.assertEqual(result.mode, RuntimeMode.DEGRADED_HOLD)
        self.assertIn("optional_capability_unavailable:hand_quality", result.reasons)

    def test_alternative_phase_evidence_keeps_capability_available(self) -> None:
        frame = frame_without("sim.imu.trunk_pitch")

        result = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        self.assertIn("stand_phase", result.available_capabilities)
        self.assertNotEqual(result.mode, RuntimeMode.SAFE_HOLD)

    def test_missing_seat_feedback_selects_safe_hold(self) -> None:
        frame = frame_without("sim.seat.angle")

        result = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        self.assertEqual(result.mode, RuntimeMode.SAFE_HOLD)
        self.assertIn("required_capability_unavailable:seat_feedback", result.reasons)

    def test_wrong_unit_is_rejected_without_replacing_value_with_zero(self) -> None:
        frame = valid_frame()
        samples = dict(frame.samples)
        samples["sim.seat.angle"] = sample("sim.seat.angle", 0.4, "deg")
        invalid = SensorFrame(frame.sequence, frame.captured_at_ns, samples)

        resolved = resolve_signals(invalid, self.config, NOW_NS)

        self.assertFalse(resolved["seat_angle"].usable)
        self.assertEqual(resolved["seat_angle"].sample.value, 0.4)
        self.assertEqual(resolved["seat_angle"].reason, "unit_mismatch")

    def test_stale_or_flagged_signal_is_unusable(self) -> None:
        frame = valid_frame()
        samples = dict(frame.samples)
        samples["sim.seat.angle"] = sample(
            "sim.seat.angle",
            0.4,
            "rad",
            timestamp_ns=NOW_NS - 200_000_000,
            flags=frozenset({SignalFlag.DRIFT}),
        )
        invalid = SensorFrame(frame.sequence, frame.captured_at_ns, samples)

        resolved = resolve_signals(invalid, self.config, NOW_NS)

        self.assertFalse(resolved["seat_angle"].usable)
        self.assertIn(resolved["seat_angle"].reason, {"stale", "quality_flag:DRIFT"})

    def test_fusion_renormalizes_available_sources(self) -> None:
        frame = frame_without("sim.seat.load")
        resolved = resolve_signals(frame, self.config, NOW_NS)

        estimate = fuse_evidence(self.config.phase_evidence, resolved)

        self.assertGreaterEqual(estimate.value, 0.0)
        self.assertLessEqual(estimate.value, 1.0)
        self.assertEqual(estimate.contributors, ("sim.seat.angle",))

    def test_degraded_data_forces_quality_gate_to_zero(self) -> None:
        frame = frame_without("camera.posture_margin")
        quality = evaluate_task_quality(Task.STAND_UP, frame, self.config, NOW_NS)

        margins = evaluate_margins(self.config.quality_margins, quality.resolved, quality.mode)

        self.assertEqual(margins.gate, 0.0)
        self.assertIn("posture", margins.unavailable)

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
