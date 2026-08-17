from __future__ import annotations

from dataclasses import replace
import unittest

from relay_control.config import parse_config
from relay_control.engine import ControlEngine
from relay_control.model import (
    ControlInput,
    OperatorInput,
    RuntimeMode,
    SafetyContext,
    SeatCommandMode,
    SensorFrame,
    Task,
)
from relay_control.safety import SafetyGuard
from tests.helpers import NOW_NS, frame_without, sample, synthetic_config_dict, valid_frame


def active_operator(**changes) -> OperatorInput:
    values = {
        "local_confirmation": True,
        "active_intent": True,
        "stop_requested": False,
        "transport_linear_m_s": 0.2,
        "transport_yaw_rad_s": 0.0,
    }
    values.update(changes)
    return OperatorInput(**values)


def control_input(
    task: Task,
    frame: SensorFrame | None = None,
    *,
    now_ns: int = NOW_NS,
    dt_s: float = 0.02,
    operator: OperatorInput | None = None,
) -> ControlInput:
    return ControlInput(
        now_ns=now_ns,
        dt_s=dt_s,
        task=task,
        frame=frame or valid_frame(now_ns=now_ns),
        operator=operator or active_operator(),
    )


def healthy_safety_context(**changes) -> SafetyContext:
    values = {
        "emergency_stop": False,
        "actuator_healthy": True,
        "brakes_healthy": True,
        "seat_locked": True,
        "allowed_tasks": frozenset(Task),
    }
    values.update(changes)
    return SafetyContext(**values)


class ControlEngineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict())
        self.engine = ControlEngine(self.config)

    def test_optional_loss_still_produces_executable_seat_candidate(self) -> None:
        frame = frame_without("camera.posture_margin")

        decision = self.engine.step(control_input(Task.STAND_UP, frame))

        self.assertEqual(decision.mode, RuntimeMode.DEGRADED_HOLD)
        self.assertEqual(decision.candidate.seat_mode, SeatCommandMode.TORQUE)
        self.assertGreaterEqual(
            decision.assistance_alpha, self.config.adaptive.initial_alpha
        )
        self.assertGreater(decision.candidate.seat_torque_nm, 0.0)

    def test_missing_handle_intent_produces_controlled_stop(self) -> None:
        frame = frame_without(
            "sim.handle.left.forward", "sim.handle.right.forward"
        )

        decision = self.engine.step(control_input(Task.GAIT, frame))

        self.assertEqual(decision.mode, RuntimeMode.CONTROLLED_STOP)
        self.assertIn(
            "required_capability_unavailable:gait_intent", decision.reasons
        )

    def test_transport_does_not_update_rehabilitation_assistance(self) -> None:
        self.engine.step(control_input(Task.STAND_UP))
        before = self.engine.state.assistance_alpha

        self.engine.step(
            control_input(
                Task.SEATED_TRANSPORT,
                now_ns=NOW_NS + 20_000_000,
            )
        )

        self.assertEqual(self.engine.state.assistance_alpha, before)

    def test_vertical_handle_force_never_produces_forward_motion(self) -> None:
        frame = valid_frame()
        samples = dict(frame.samples)
        samples["sim.handle.left.forward"] = sample(
            "sim.handle.left.forward", 0.0, "N"
        )
        samples["sim.handle.right.forward"] = sample(
            "sim.handle.right.forward", 0.0, "N"
        )
        samples["sim.handle.left.vertical"] = sample(
            "sim.handle.left.vertical", 500.0, "N"
        )
        samples["sim.handle.right.vertical"] = sample(
            "sim.handle.right.vertical", 500.0, "N"
        )
        vertical_only = SensorFrame(frame.sequence, frame.captured_at_ns, samples)

        decision = self.engine.step(control_input(Task.GAIT, vertical_only))

        self.assertEqual(decision.candidate.left_wheel_rad_s, 0.0)
        self.assertEqual(decision.candidate.right_wheel_rad_s, 0.0)

    def test_invalid_cycle_time_returns_safe_hold_instead_of_raising(self) -> None:
        decision = self.engine.step(control_input(Task.STAND_UP, dt_s=0.0))

        self.assertEqual(decision.mode, RuntimeMode.SAFE_HOLD)
        self.assertEqual(decision.candidate.seat_mode, SeatCommandMode.HOLD)
        self.assertIn("invalid_cycle_time", decision.reasons)


class SafetyGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict())
        self.engine = ControlEngine(self.config)
        self.guard = SafetyGuard(self.config)
        self.candidate = self.engine.step(control_input(Task.STAND_UP)).candidate

    def test_emergency_stop_overrides_finite_candidate(self) -> None:
        safe = self.guard.review(
            self.candidate,
            healthy_safety_context(emergency_stop=True),
            NOW_NS,
        )

        self.assertTrue(safe.brake_request)
        self.assertEqual(safe.seat_mode, SeatCommandMode.HOLD)
        self.assertEqual(safe.left_wheel_rad_s, 0.0)
        self.assertEqual(safe.right_wheel_rad_s, 0.0)
        self.assertIn("emergency_stop", safe.reasons)

    def test_nonfinite_candidate_is_rejected(self) -> None:
        unsafe = replace(self.candidate, seat_torque_nm=float("nan"))

        result = self.guard.review(unsafe, healthy_safety_context(), NOW_NS)

        self.assertEqual(result.mode, RuntimeMode.SAFE_HOLD)
        self.assertEqual(result.seat_mode, SeatCommandMode.HOLD)
        self.assertIn("nonfinite_candidate", result.reasons)

    def test_candidate_is_clamped_to_configured_limits(self) -> None:
        unsafe = replace(
            self.candidate,
            seat_torque_nm=100.0,
            left_wheel_rad_s=-10.0,
            right_wheel_rad_s=10.0,
        )

        result = self.guard.review(unsafe, healthy_safety_context(), NOW_NS)

        self.assertEqual(result.seat_torque_nm, self.config.limits.seat_torque_max_nm)
        self.assertEqual(result.left_wheel_rad_s, -self.config.limits.wheel_speed_max_rad_s)
        self.assertEqual(result.right_wheel_rad_s, self.config.limits.wheel_speed_max_rad_s)
        self.assertIn("seat_torque_clamped", result.reasons)
        self.assertIn("wheel_speed_clamped", result.reasons)

    def test_expired_candidate_is_rejected(self) -> None:
        result = self.guard.review(
            self.candidate,
            healthy_safety_context(),
            self.candidate.expires_at_ns + 1,
        )

        self.assertEqual(result.mode, RuntimeMode.SAFE_HOLD)
        self.assertIn("candidate_expired", result.reasons)


if __name__ == "__main__":
    unittest.main()
