from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

from relay_control.config import (
    EvidenceSource,
    MarginAggregation,
    MarginDirection,
    MarginPolicy,
)
from relay_control.model import Estimate, RuntimeMode
from relay_control.quality import ResolvedSignal


class EstimationUnavailable(ValueError):
    pass


@dataclass(frozen=True)
class MarginEvaluation:
    margins: Mapping[str, float]
    gate: float
    unavailable: tuple[str, ...]


def fuse_evidence(
    sources: tuple[EvidenceSource, ...],
    resolved: Mapping[str, ResolvedSignal],
) -> Estimate:
    weighted_sum = 0.0
    available_weight = 0.0
    configured_weight = sum(source.weight for source in sources)
    confidence_sum = 0.0
    contributors: list[str] = []

    for source in sources:
        signal = resolved[source.role]
        if not signal.usable or signal.sample is None:
            continue
        position = (signal.sample.value - source.input_min) / (
            source.input_max - source.input_min
        )
        position = _clip(position, 0.0, 1.0)
        value = source.output_min + position * (
            source.output_max - source.output_min
        )
        effective_weight = source.weight * signal.sample.confidence
        weighted_sum += value * effective_weight
        available_weight += effective_weight
        confidence_sum += source.weight * signal.sample.confidence
        contributors.append(signal.sample.signal_id)

    if available_weight == 0.0:
        raise EstimationUnavailable("no usable evidence")

    return Estimate(
        value=weighted_sum / available_weight,
        confidence=_clip(confidence_sum / configured_weight, 0.0, 1.0),
        contributors=tuple(contributors),
    )


def evaluate_margins(
    policies: Mapping[str, MarginPolicy],
    resolved: Mapping[str, ResolvedSignal],
    mode: RuntimeMode,
) -> MarginEvaluation:
    margins: dict[str, float] = {}
    unavailable: list[str] = []

    for name, policy in policies.items():
        signals = [resolved[role] for role in policy.roles]
        if any(not signal.usable or signal.sample is None for signal in signals):
            unavailable.append(name)
            continue
        values = [
            signal.sample.value
            for signal in signals
            if signal.sample is not None
        ]
        if policy.aggregation is MarginAggregation.SUM:
            value = sum(values)
        elif policy.aggregation is MarginAggregation.MEAN:
            value = sum(values) / len(values)
        else:
            value = max(values)
        if policy.direction is MarginDirection.ABOVE:
            margin = (value - policy.boundary) / policy.sigma
        else:
            margin = (policy.boundary - value) / policy.sigma
        margins[name] = margin

    if mode is not RuntimeMode.NORMAL or unavailable or not margins:
        gate = 0.0
    else:
        gate = _clip(min(margins.values()), 0.0, 1.0)
    return MarginEvaluation(
        margins=MappingProxyType(margins),
        gate=gate,
        unavailable=tuple(unavailable),
    )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
