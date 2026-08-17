from __future__ import annotations

from dataclasses import dataclass, replace

from relay_control.config import ControlConfig
from relay_control.engine import ControlEngine
from relay_control.model import (
    ActuatorReceipt,
    AuditEvent,
    ControlDecision,
    ControlInput,
    OperatorInput,
    RuntimeMode,
    SafeCommand,
    SafetyContext,
    SeatCommandMode,
    SensorFrame,
    Task,
)
from relay_control.ports import ActuatorPort, AuditPort, SensorPort
from relay_control.safety import SafetyGuard


@dataclass(frozen=True)
class RuntimeCycleResult:
    decision: ControlDecision
    safe_command: SafeCommand
    receipt: ActuatorReceipt


class DeviceRuntime:
    def __init__(
        self,
        config: ControlConfig,
        sensor: SensorPort,
        actuator: ActuatorPort,
        audit: AuditPort,
    ) -> None:
        self._sensor = sensor
        self._actuator = actuator
        self._audit = audit
        self._engine = ControlEngine(config)
        self._safety = SafetyGuard(config)

    def cycle(
        self,
        task: Task,
        operator: OperatorInput,
        safety_context: SafetyContext,
        now_ns: int,
        dt_s: float,
    ) -> RuntimeCycleResult:
        deadline_ns = now_ns + max(int(dt_s * 1_000_000_000), 0)
        sensor_error: str | None = None
        try:
            frame = self._sensor.read(deadline_ns)
        except Exception as exc:
            sensor_error = f"sensor_port_error:{type(exc).__name__}"
            frame = SensorFrame(
                sequence=0,
                captured_at_ns=now_ns,
                samples={},
            )

        decision = self._engine.step(
            ControlInput(
                now_ns=now_ns,
                dt_s=dt_s,
                task=task,
                frame=frame,
                operator=operator,
            )
        )
        if sensor_error is not None:
            reasons = tuple(dict.fromkeys(decision.reasons + (sensor_error,)))
            candidate = replace(
                decision.candidate,
                mode=RuntimeMode.SAFE_HOLD,
                seat_mode=SeatCommandMode.HOLD,
                seat_torque_nm=0.0,
                left_wheel_rad_s=0.0,
                right_wheel_rad_s=0.0,
                brake_request=True,
                reasons=reasons,
            )
            decision = replace(
                decision,
                mode=RuntimeMode.SAFE_HOLD,
                candidate=candidate,
                reasons=reasons,
            )

        safe_command = self._safety.review(
            decision.candidate,
            safety_context,
            now_ns,
        )
        try:
            receipt = self._actuator.apply(safe_command, deadline_ns)
        except Exception as exc:
            reason = f"actuator_port_error:{type(exc).__name__}"
            safe_command = _failed_actuator_command(safe_command, reason)
            receipt = ActuatorReceipt(
                sequence=safe_command.sequence,
                accepted=False,
                status="ACTUATOR_ERROR",
                reasons=(reason,),
            )

        event = AuditEvent(
            schema_version=1,
            timestamp_ns=now_ns,
            event_type="control_cycle",
            payload={
                "sequence": safe_command.sequence,
                "task": task.value,
                "decision_mode": decision.mode.value,
                "safe_mode": safe_command.mode.value,
                "accepted": receipt.accepted,
                "reasons": list(safe_command.reasons),
            },
        )
        try:
            self._audit.append(event)
        except Exception:
            pass
        return RuntimeCycleResult(decision, safe_command, receipt)


def _failed_actuator_command(
    command: SafeCommand, reason: str
) -> SafeCommand:
    return SafeCommand(
        sequence=command.sequence,
        task=command.task,
        mode=RuntimeMode.SAFE_HOLD,
        seat_mode=SeatCommandMode.HOLD,
        seat_torque_nm=0.0,
        left_wheel_rad_s=0.0,
        right_wheel_rad_s=0.0,
        brake_request=True,
        reasons=tuple(dict.fromkeys(command.reasons + (reason,))),
    )
