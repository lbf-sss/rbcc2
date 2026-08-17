from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from types import MappingProxyType
from typing import Any, Mapping


class Task(StrEnum):
    IDLE = "IDLE"
    STAND_UP = "STAND_UP"
    SIT_DOWN = "SIT_DOWN"
    GAIT = "GAIT"
    SEATED_TRANSPORT = "SEATED_TRANSPORT"


class RuntimeMode(StrEnum):
    NORMAL = "NORMAL"
    DEGRADED_HOLD = "DEGRADED_HOLD"
    CONTROLLED_STOP = "CONTROLLED_STOP"
    SAFE_HOLD = "SAFE_HOLD"


class SignalFlag(StrEnum):
    MISSING = "MISSING"
    STALE = "STALE"
    DRIFT = "DRIFT"
    SATURATED = "SATURATED"
    CONFLICT = "CONFLICT"
    OUT_OF_RANGE = "OUT_OF_RANGE"


class SeatCommandMode(StrEnum):
    DISABLED = "DISABLED"
    TORQUE = "TORQUE"
    HOLD = "HOLD"


@dataclass(frozen=True)
class SignalSample:
    signal_id: str
    value: float
    unit: str
    timestamp_ns: int
    confidence: float
    calibration_version: str
    source: str
    flags: frozenset[SignalFlag] = frozenset()

    def __post_init__(self) -> None:
        if not self.signal_id:
            raise ValueError("signal_id is required")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be nonnegative")


@dataclass(frozen=True)
class SensorFrame:
    sequence: int
    captured_at_ns: int
    samples: Mapping[str, SignalSample]

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("sequence must be nonnegative")
        if self.captured_at_ns < 0:
            raise ValueError("captured_at_ns must be nonnegative")
        object.__setattr__(self, "samples", MappingProxyType(dict(self.samples)))

    def sample(self, signal_id: str) -> SignalSample | None:
        return self.samples.get(signal_id)


@dataclass(frozen=True)
class Estimate:
    value: float
    confidence: float
    contributors: tuple[str, ...]


@dataclass(frozen=True)
class QualityDecision:
    mode: RuntimeMode
    resolved: Mapping[str, Any]
    available_capabilities: frozenset[str]
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class OperatorInput:
    local_confirmation: bool = False
    active_intent: bool = False
    stop_requested: bool = False
    transport_linear_m_s: float = 0.0
    transport_yaw_rad_s: float = 0.0


@dataclass(frozen=True)
class SafetyContext:
    emergency_stop: bool
    actuator_healthy: bool
    brakes_healthy: bool
    seat_locked: bool
    allowed_tasks: frozenset[Task]


@dataclass(frozen=True)
class ControlInput:
    now_ns: int
    dt_s: float
    task: Task
    frame: SensorFrame
    operator: OperatorInput


@dataclass(frozen=True)
class ControlCandidate:
    sequence: int
    created_at_ns: int
    expires_at_ns: int
    task: Task
    mode: RuntimeMode
    seat_mode: SeatCommandMode
    seat_torque_nm: float
    left_wheel_rad_s: float
    right_wheel_rad_s: float
    brake_request: bool
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class SafeCommand:
    sequence: int
    task: Task
    mode: RuntimeMode
    seat_mode: SeatCommandMode
    seat_torque_nm: float
    left_wheel_rad_s: float
    right_wheel_rad_s: float
    brake_request: bool
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class DecisionTrace:
    phase: Estimate | None
    intent: Estimate | None
    phase_velocity: float
    progress_margin: float
    quality_margins: Mapping[str, float]
    quality_gate: float
    assistance_before: float
    assistance_after: float
    device_compensation_nm: float
    rehabilitation_assistance_nm: float
    used_signals: tuple[str, ...]
    rejected_signals: tuple[str, ...]
    clamps: tuple[str, ...]


@dataclass(frozen=True)
class ControlDecision:
    mode: RuntimeMode
    assistance_alpha: float
    candidate: ControlCandidate
    trace: DecisionTrace
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ActuatorReceipt:
    sequence: int
    accepted: bool
    status: str
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class AuditEvent:
    schema_version: int
    timestamp_ns: int
    event_type: str
    payload: Mapping[str, Any] = field(default_factory=dict)
