from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType

from relay_control.config import ControlConfig
from relay_control.controllers import (
    WheelController,
    interpolate_curve,
    seat_control,
    update_assistance,
)
from relay_control.estimation import (
    EstimationUnavailable,
    MarginEvaluation,
    evaluate_margins,
    fuse_evidence,
)
from relay_control.model import (
    ControlCandidate,
    ControlDecision,
    ControlInput,
    DecisionTrace,
    Estimate,
    RuntimeMode,
    SeatCommandMode,
    Task,
)
from relay_control.quality import evaluate_task_quality


@dataclass(frozen=True)
class EngineState:
    assistance_alpha: float
    previous_phase: float | None
    last_timestamp_ns: int | None
    last_task: Task


_SEVERITY = {
    RuntimeMode.NORMAL: 0,
    RuntimeMode.DEGRADED_HOLD: 1,
    RuntimeMode.CONTROLLED_STOP: 2,
    RuntimeMode.SAFE_HOLD: 3,
}


class ControlEngine:
    def __init__(self, config: ControlConfig) -> None:
        self._config = config
        self._assistance_alpha = config.adaptive.initial_alpha
        self._previous_phase: float | None = None
        self._last_timestamp_ns: int | None = None
        self._last_task = Task.IDLE
        self._sequence = 0
        self._wheel = WheelController(config.wheel)

    @property
    def state(self) -> EngineState:
        return EngineState(
            assistance_alpha=self._assistance_alpha,
            previous_phase=self._previous_phase,
            last_timestamp_ns=self._last_timestamp_ns,
            last_task=self._last_task,
        )

    def step(self, cycle: ControlInput) -> ControlDecision:
        self._sequence += 1
        cycle_error = self._cycle_error(cycle)
        if cycle_error is not None:
            return self._safe_hold_decision(cycle, (cycle_error,))

        quality = evaluate_task_quality(
            cycle.task,
            cycle.frame,
            self._config,
            cycle.now_ns,
        )
        mode = quality.mode
        reasons = list(quality.reasons)

        if cycle.operator.stop_requested:
            mode = _more_severe(mode, RuntimeMode.CONTROLLED_STOP)
            reasons.append("operator_stop_requested")
        if cycle.task is not Task.IDLE and not cycle.operator.local_confirmation:
            mode = RuntimeMode.SAFE_HOLD
            reasons.append("local_confirmation_required")
        if cycle.task in {Task.STAND_UP, Task.SIT_DOWN, Task.GAIT} and not cycle.operator.active_intent:
            mode = RuntimeMode.SAFE_HOLD
            reasons.append("active_intent_required")

        phase: Estimate | None = None
        intent: Estimate | None = None
        phase_velocity = 0.0
        progress_margin = 0.0
        margin_evaluation = MarginEvaluation(MappingProxyType({}), 0.0, ())
        assistance_before = self._assistance_alpha
        device_compensation = 0.0
        rehabilitation_assistance = 0.0
        seat_mode = SeatCommandMode.DISABLED
        seat_torque = 0.0
        left_wheel = 0.0
        right_wheel = 0.0
        brake_request = cycle.task is Task.IDLE

        if cycle.task in {Task.STAND_UP, Task.SIT_DOWN}:
            try:
                phase = fuse_evidence(
                    self._config.phase_evidence, quality.resolved
                )
                intent = fuse_evidence(
                    self._config.intent_evidence, quality.resolved
                )
            except EstimationUnavailable:
                mode = _more_severe(mode, RuntimeMode.CONTROLLED_STOP)
                reasons.append("state_estimation_unavailable")

            if phase is not None and intent is not None:
                if (
                    self._previous_phase is not None
                    and self._last_timestamp_ns is not None
                ):
                    elapsed_s = (
                        cycle.now_ns - self._last_timestamp_ns
                    ) / 1_000_000_000
                    if elapsed_s > 0.0:
                        phase_velocity = (
                            phase.value - self._previous_phase
                        ) / elapsed_s
                safe_progress = interpolate_curve(
                    self._config.adaptive.safe_progress_curve,
                    phase.value,
                )
                progress_margin = (
                    phase_velocity - safe_progress
                ) / self._config.adaptive.progress_sigma
                margin_evaluation = evaluate_margins(
                    self._config.quality_margins,
                    quality.resolved,
                    mode,
                )
                if cycle.task is Task.STAND_UP and mode in {
                    RuntimeMode.NORMAL,
                    RuntimeMode.DEGRADED_HOLD,
                }:
                    self._assistance_alpha = update_assistance(
                        self._assistance_alpha,
                        progress_margin,
                        margin_evaluation.gate,
                        cycle.dt_s,
                        self._config.adaptive,
                    )
                if mode in {RuntimeMode.NORMAL, RuntimeMode.DEGRADED_HOLD}:
                    seat = seat_control(
                        cycle.task,
                        _value(quality.resolved, "seat_angle"),
                        _value(quality.resolved, "seat_velocity"),
                        phase,
                        intent,
                        self._assistance_alpha,
                        self._config.seat,
                    )
                    seat_mode = SeatCommandMode.TORQUE
                    seat_torque = seat.torque_nm
                    device_compensation = seat.device_compensation_nm
                    rehabilitation_assistance = (
                        seat.rehabilitation_assistance_nm
                    )
                else:
                    seat_mode = SeatCommandMode.HOLD
                    brake_request = True
                self._previous_phase = phase.value

        elif cycle.task is Task.GAIT:
            margin_evaluation = evaluate_margins(
                self._config.quality_margins,
                quality.resolved,
                mode,
            )
            if mode in {RuntimeMode.NORMAL, RuntimeMode.DEGRADED_HOLD}:
                wheel = self._wheel.step(
                    _value(quality.resolved, "left_handle_forward"),
                    _value(quality.resolved, "right_handle_forward"),
                    cycle.dt_s,
                    mode,
                )
            else:
                wheel = self._wheel.step(0.0, 0.0, cycle.dt_s, mode)
            left_wheel = wheel.left_wheel_rad_s
            right_wheel = wheel.right_wheel_rad_s
            brake_request = wheel.brake_request

        elif cycle.task is Task.SEATED_TRANSPORT:
            wheel = self._wheel.transport(
                cycle.operator.transport_linear_m_s,
                cycle.operator.transport_yaw_rad_s,
                cycle.dt_s,
                mode,
            )
            left_wheel = wheel.left_wheel_rad_s
            right_wheel = wheel.right_wheel_rad_s
            brake_request = wheel.brake_request

        if mode is RuntimeMode.SAFE_HOLD:
            seat_mode = SeatCommandMode.HOLD
            seat_torque = 0.0
            left_wheel = 0.0
            right_wheel = 0.0
            brake_request = True

        candidate = ControlCandidate(
            sequence=self._sequence,
            created_at_ns=cycle.now_ns,
            expires_at_ns=cycle.now_ns
            + int(self._config.limits.candidate_ttl_s * 1_000_000_000),
            task=cycle.task,
            mode=mode,
            seat_mode=seat_mode,
            seat_torque_nm=seat_torque,
            left_wheel_rad_s=left_wheel,
            right_wheel_rad_s=right_wheel,
            brake_request=brake_request,
            reasons=tuple(reasons),
        )
        used, rejected = _signal_trace(quality.resolved, self._config)
        trace = DecisionTrace(
            phase=phase,
            intent=intent,
            phase_velocity=phase_velocity,
            progress_margin=progress_margin,
            quality_margins=margin_evaluation.margins,
            quality_gate=margin_evaluation.gate,
            assistance_before=assistance_before,
            assistance_after=self._assistance_alpha,
            device_compensation_nm=device_compensation,
            rehabilitation_assistance_nm=rehabilitation_assistance,
            used_signals=used,
            rejected_signals=rejected,
            clamps=(),
        )
        self._last_timestamp_ns = cycle.now_ns
        self._last_task = cycle.task
        return ControlDecision(
            mode=mode,
            assistance_alpha=self._assistance_alpha,
            candidate=candidate,
            trace=trace,
            reasons=tuple(reasons),
        )

    def _cycle_error(self, cycle: ControlInput) -> str | None:
        if (
            not math.isfinite(cycle.dt_s)
            or cycle.dt_s <= 0.0
            or cycle.dt_s > self._config.limits.max_dt_s
        ):
            return "invalid_cycle_time"
        if (
            self._last_timestamp_ns is not None
            and cycle.now_ns <= self._last_timestamp_ns
        ):
            return "nonmonotonic_cycle_time"
        return None

    def _safe_hold_decision(
        self, cycle: ControlInput, reasons: tuple[str, ...]
    ) -> ControlDecision:
        candidate = ControlCandidate(
            sequence=self._sequence,
            created_at_ns=cycle.now_ns,
            expires_at_ns=cycle.now_ns
            + int(self._config.limits.candidate_ttl_s * 1_000_000_000),
            task=cycle.task,
            mode=RuntimeMode.SAFE_HOLD,
            seat_mode=SeatCommandMode.HOLD,
            seat_torque_nm=0.0,
            left_wheel_rad_s=0.0,
            right_wheel_rad_s=0.0,
            brake_request=True,
            reasons=reasons,
        )
        trace = DecisionTrace(
            phase=None,
            intent=None,
            phase_velocity=0.0,
            progress_margin=0.0,
            quality_margins=MappingProxyType({}),
            quality_gate=0.0,
            assistance_before=self._assistance_alpha,
            assistance_after=self._assistance_alpha,
            device_compensation_nm=0.0,
            rehabilitation_assistance_nm=0.0,
            used_signals=(),
            rejected_signals=(),
            clamps=(),
        )
        return ControlDecision(
            mode=RuntimeMode.SAFE_HOLD,
            assistance_alpha=self._assistance_alpha,
            candidate=candidate,
            trace=trace,
            reasons=reasons,
        )


def _value(resolved, role: str) -> float:
    signal = resolved[role]
    if not signal.usable or signal.sample is None:
        raise EstimationUnavailable(f"required role unavailable: {role}")
    return signal.sample.value


def _more_severe(current: RuntimeMode, requested: RuntimeMode) -> RuntimeMode:
    return requested if _SEVERITY[requested] > _SEVERITY[current] else current


def _signal_trace(resolved, config: ControlConfig) -> tuple[tuple[str, ...], tuple[str, ...]]:
    used: list[str] = []
    rejected: list[str] = []
    for role, signal in resolved.items():
        signal_id = config.signals[role].signal_id
        if signal.usable:
            used.append(signal_id)
        else:
            rejected.append(f"{signal_id}:{signal.reason}")
    return tuple(used), tuple(rejected)
