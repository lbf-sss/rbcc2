from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

from relay_control.config import ControlConfig
from relay_control.model import QualityDecision, RuntimeMode, SignalSample, Task


@dataclass(frozen=True)
class ResolvedSignal:
    role: str
    sample: SignalSample | None
    usable: bool
    reason: str | None


_SEVERITY = {
    RuntimeMode.NORMAL: 0,
    RuntimeMode.DEGRADED_HOLD: 1,
    RuntimeMode.CONTROLLED_STOP: 2,
    RuntimeMode.SAFE_HOLD: 3,
}


def resolve_signals(
    frame,
    config: ControlConfig,
    now_ns: int,
) -> Mapping[str, ResolvedSignal]:
    resolved: dict[str, ResolvedSignal] = {}
    for role, binding in config.signals.items():
        sample = frame.sample(binding.signal_id)
        reason = _unusable_reason(sample, binding, now_ns)
        resolved[role] = ResolvedSignal(
            role=role,
            sample=sample,
            usable=reason is None,
            reason=reason,
        )
    return MappingProxyType(resolved)


def evaluate_task_quality(
    task: Task,
    frame,
    config: ControlConfig,
    now_ns: int,
) -> QualityDecision:
    resolved = resolve_signals(frame, config, now_ns)
    available = frozenset(
        name
        for name, capability in config.capabilities.items()
        if any(all(resolved[role].usable for role in group) for group in capability.alternatives)
    )
    requirements = config.task_requirements[task]
    mode = RuntimeMode.NORMAL
    reasons: list[str] = []

    for capability_name in requirements.required:
        if capability_name in available:
            continue
        requested_mode = config.capabilities[capability_name].loss_mode
        mode = _more_severe(mode, requested_mode)
        reasons.append(f"required_capability_unavailable:{capability_name}")

    for capability_name in requirements.optional:
        if capability_name in available:
            continue
        mode = _more_severe(mode, RuntimeMode.DEGRADED_HOLD)
        reasons.append(f"optional_capability_unavailable:{capability_name}")

    return QualityDecision(
        mode=mode,
        resolved=resolved,
        available_capabilities=available,
        reasons=tuple(reasons),
    )


def _unusable_reason(sample, binding, now_ns: int) -> str | None:
    if sample is None:
        return "missing"
    if sample.signal_id != binding.signal_id:
        return "signal_id_mismatch"
    if sample.unit != binding.unit:
        return "unit_mismatch"
    if not math.isfinite(sample.value):
        return "nonfinite"
    if sample.timestamp_ns > now_ns:
        return "future_timestamp"
    age_ns = now_ns - sample.timestamp_ns
    if age_ns > int(binding.max_age_s * 1_000_000_000):
        return "stale"
    if sample.confidence < binding.min_confidence:
        return "low_confidence"
    if sample.flags:
        flag = sorted(sample.flags, key=lambda item: item.value)[0]
        return f"quality_flag:{flag.value}"
    return None


def _more_severe(current: RuntimeMode, requested: RuntimeMode) -> RuntimeMode:
    if _SEVERITY[requested] > _SEVERITY[current]:
        return requested
    return current
