from __future__ import annotations

import math

from relay_control.config import ControlConfig
from relay_control.model import (
    ControlCandidate,
    RuntimeMode,
    SafeCommand,
    SafetyContext,
    SeatCommandMode,
    Task,
)


class SafetyGuard:
    def __init__(self, config: ControlConfig) -> None:
        self._config = config
        self._last_sequence = -1

    def review(
        self,
        candidate: ControlCandidate,
        context: SafetyContext,
        now_ns: int,
    ) -> SafeCommand:
        failures = self._failures(candidate, context, now_ns)
        self._last_sequence = max(self._last_sequence, candidate.sequence)
        if failures or candidate.mode is RuntimeMode.SAFE_HOLD:
            return self._safe_hold(candidate, failures)

        limits = self._config.limits
        torque = _clip(
            candidate.seat_torque_nm,
            limits.seat_torque_min_nm,
            limits.seat_torque_max_nm,
        )
        left = _clip(
            candidate.left_wheel_rad_s,
            -limits.wheel_speed_max_rad_s,
            limits.wheel_speed_max_rad_s,
        )
        right = _clip(
            candidate.right_wheel_rad_s,
            -limits.wheel_speed_max_rad_s,
            limits.wheel_speed_max_rad_s,
        )
        reasons = list(candidate.reasons)
        if torque != candidate.seat_torque_nm:
            reasons.append("seat_torque_clamped")
        if (
            left != candidate.left_wheel_rad_s
            or right != candidate.right_wheel_rad_s
        ):
            reasons.append("wheel_speed_clamped")
        return SafeCommand(
            sequence=candidate.sequence,
            task=candidate.task,
            mode=candidate.mode,
            seat_mode=candidate.seat_mode,
            seat_torque_nm=torque,
            left_wheel_rad_s=left,
            right_wheel_rad_s=right,
            brake_request=candidate.brake_request,
            reasons=tuple(reasons),
        )

    def _failures(
        self,
        candidate: ControlCandidate,
        context: SafetyContext,
        now_ns: int,
    ) -> tuple[str, ...]:
        reasons: list[str] = []
        values = (
            candidate.seat_torque_nm,
            candidate.left_wheel_rad_s,
            candidate.right_wheel_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            reasons.append("nonfinite_candidate")
        if candidate.sequence <= self._last_sequence:
            reasons.append("candidate_sequence_not_increasing")
        if now_ns > candidate.expires_at_ns:
            reasons.append("candidate_expired")
        if now_ns < candidate.created_at_ns:
            reasons.append("candidate_from_future")
        if candidate.expires_at_ns <= candidate.created_at_ns:
            reasons.append("candidate_expiry_invalid")
        if context.emergency_stop:
            reasons.append("emergency_stop")
        if not context.actuator_healthy:
            reasons.append("actuator_unhealthy")
        if not context.brakes_healthy:
            reasons.append("brakes_unhealthy")
        if candidate.task not in context.allowed_tasks and candidate.task is not Task.IDLE:
            reasons.append("task_not_allowed")
        if candidate.task in {Task.STAND_UP, Task.SIT_DOWN, Task.SEATED_TRANSPORT} and not context.seat_locked:
            reasons.append("seat_not_locked")
        wheel_motion = (
            candidate.left_wheel_rad_s != 0.0
            or candidate.right_wheel_rad_s != 0.0
        )
        seat_command = (
            candidate.seat_mode is SeatCommandMode.TORQUE
            or candidate.seat_torque_nm != 0.0
        )
        if candidate.task in {Task.STAND_UP, Task.SIT_DOWN} and wheel_motion:
            reasons.append("wheel_command_not_allowed_for_task")
        if candidate.task in {Task.GAIT, Task.SEATED_TRANSPORT} and seat_command:
            reasons.append("seat_command_not_allowed_for_task")
        if candidate.task is Task.IDLE and (wheel_motion or seat_command):
            reasons.append("idle_motion_not_allowed")
        if (
            candidate.seat_mode is not SeatCommandMode.TORQUE
            and candidate.seat_torque_nm != 0.0
        ):
            reasons.append("seat_torque_without_torque_mode")
        if candidate.brake_request and wheel_motion:
            reasons.append("brake_motion_conflict")
        return tuple(reasons)

    def _safe_hold(
        self,
        candidate: ControlCandidate,
        failures: tuple[str, ...],
    ) -> SafeCommand:
        return SafeCommand(
            sequence=candidate.sequence,
            task=candidate.task,
            mode=RuntimeMode.SAFE_HOLD,
            seat_mode=SeatCommandMode.HOLD,
            seat_torque_nm=0.0,
            left_wheel_rad_s=0.0,
            right_wheel_rad_s=0.0,
            brake_request=True,
            reasons=tuple(dict.fromkeys(candidate.reasons + failures)),
        )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
