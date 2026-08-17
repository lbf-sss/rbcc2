from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
import math
from pathlib import Path
import tomllib
from types import MappingProxyType
from typing import Any, Mapping, Sequence

from relay_control.model import RuntimeMode, Task


class ConfigError(ValueError):
    pass


class MarginDirection(StrEnum):
    ABOVE = "ABOVE"
    BELOW = "BELOW"


class MarginAggregation(StrEnum):
    SUM = "SUM"
    MEAN = "MEAN"
    MAX = "MAX"


@dataclass(frozen=True)
class MetadataConfig:
    configuration_id: str
    version: int
    human_use: bool


@dataclass(frozen=True)
class SignalBinding:
    signal_id: str
    unit: str
    max_age_s: float
    min_confidence: float
    required: bool


@dataclass(frozen=True)
class CapabilityConfig:
    alternatives: tuple[frozenset[str], ...]
    loss_mode: RuntimeMode


@dataclass(frozen=True)
class TaskRequirement:
    required: tuple[str, ...]
    optional: tuple[str, ...]


@dataclass(frozen=True)
class EvidenceSource:
    role: str
    input_min: float
    input_max: float
    output_min: float
    output_max: float
    weight: float


@dataclass(frozen=True)
class MarginPolicy:
    roles: tuple[str, ...]
    aggregation: MarginAggregation
    boundary: float
    sigma: float
    direction: MarginDirection


@dataclass(frozen=True)
class PiecewiseCurve:
    points: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class AdaptiveConfig:
    k_up: float
    k_down: float
    alpha_min: float
    alpha_max: float
    initial_alpha: float
    max_slew_per_s: float
    progress_sigma: float
    safe_progress_curve: PiecewiseCurve


@dataclass(frozen=True)
class SeatConfig:
    device_torque_curve: PiecewiseCurve
    support_torque_curve: PiecewiseCurve
    descent_safe_speed_curve: PiecewiseCurve
    descent_damping: float


@dataclass(frozen=True)
class WheelConfig:
    virtual_mass: float
    linear_damping: float
    virtual_inertia: float
    yaw_damping: float
    wheel_radius_m: float
    track_width_m: float
    max_linear_m_s: float
    max_yaw_rad_s: float
    max_wheel_accel_rad_s2: float
    controlled_stop_decel_rad_s2: float


@dataclass(frozen=True)
class LimitsConfig:
    seat_torque_min_nm: float
    seat_torque_max_nm: float
    wheel_speed_max_rad_s: float
    candidate_ttl_s: float
    max_dt_s: float


@dataclass(frozen=True)
class ControlConfig:
    metadata: MetadataConfig
    signals: Mapping[str, SignalBinding]
    capabilities: Mapping[str, CapabilityConfig]
    task_requirements: Mapping[Task, TaskRequirement]
    phase_evidence: tuple[EvidenceSource, ...]
    intent_evidence: tuple[EvidenceSource, ...]
    quality_margins: Mapping[str, MarginPolicy]
    adaptive: AdaptiveConfig
    seat: SeatConfig
    wheel: WheelConfig
    limits: LimitsConfig


def load_config(path: str | Path) -> ControlConfig:
    with Path(path).open("rb") as stream:
        return parse_config(tomllib.load(stream))


def parse_config(raw: Mapping[str, Any]) -> ControlConfig:
    metadata_raw = _section(raw, "metadata")
    metadata = MetadataConfig(
        configuration_id=_text(metadata_raw, "configuration_id"),
        version=_integer(metadata_raw, "version", minimum=1),
        human_use=_boolean(metadata_raw, "human_use"),
    )
    if metadata.human_use:
        raise ConfigError("repository configurations are not approved for human use")

    signals = _parse_signals(_section(raw, "signals"))
    capabilities = _parse_capabilities(_section(raw, "capabilities"), signals)
    requirements = _parse_requirements(
        _section(raw, "task_requirements"), capabilities
    )
    estimators = _section(raw, "estimators")
    phase_evidence = _parse_evidence(estimators, "phase", signals)
    intent_evidence = _parse_evidence(estimators, "intent", signals)
    margins = _parse_margins(_section(raw, "quality_margins"), signals)
    adaptive = _parse_adaptive(_section(raw, "adaptive"))
    seat = _parse_seat(_section(raw, "seat"))
    wheel = _parse_wheel(_section(raw, "wheel"))
    limits = _parse_limits(_section(raw, "limits"))

    return ControlConfig(
        metadata=metadata,
        signals=MappingProxyType(signals),
        capabilities=MappingProxyType(capabilities),
        task_requirements=MappingProxyType(requirements),
        phase_evidence=phase_evidence,
        intent_evidence=intent_evidence,
        quality_margins=MappingProxyType(margins),
        adaptive=adaptive,
        seat=seat,
        wheel=wheel,
        limits=limits,
    )


def _parse_signals(raw: Mapping[str, Any]) -> dict[str, SignalBinding]:
    result: dict[str, SignalBinding] = {}
    if not raw:
        raise ConfigError("signals must not be empty")
    for role, item_value in raw.items():
        item = _mapping(item_value, f"signals.{role}")
        result[role] = SignalBinding(
            signal_id=_text(item, "signal_id"),
            unit=_text(item, "unit"),
            max_age_s=_positive(item, "max_age_s"),
            min_confidence=_bounded(item, "min_confidence", 0.0, 1.0),
            required=_boolean(item, "required"),
        )
    ids = [binding.signal_id for binding in result.values()]
    if len(ids) != len(set(ids)):
        raise ConfigError("signal_id values must be unique")
    return result


def _parse_capabilities(
    raw: Mapping[str, Any], signals: Mapping[str, SignalBinding]
) -> dict[str, CapabilityConfig]:
    result: dict[str, CapabilityConfig] = {}
    for name, item_value in raw.items():
        item = _mapping(item_value, f"capabilities.{name}")
        alternatives_value = item.get("alternatives")
        if not isinstance(alternatives_value, Sequence) or isinstance(alternatives_value, str):
            raise ConfigError(f"capabilities.{name}.alternatives must be a list")
        alternatives: list[frozenset[str]] = []
        for index, group_value in enumerate(alternatives_value):
            if not isinstance(group_value, Sequence) or isinstance(group_value, str):
                raise ConfigError(f"capabilities.{name}.alternatives[{index}] must be a list")
            group = frozenset(str(role) for role in group_value)
            if not group:
                raise ConfigError(f"capabilities.{name} has an empty alternative")
            unknown = group.difference(signals)
            if unknown:
                raise ConfigError(f"capabilities.{name} references unknown signals: {sorted(unknown)}")
            alternatives.append(group)
        if not alternatives:
            raise ConfigError(f"capabilities.{name} has no alternatives")
        try:
            loss_mode = RuntimeMode(_text(item, "loss_mode"))
        except ValueError as exc:
            raise ConfigError(f"capabilities.{name}.loss_mode is invalid") from exc
        if loss_mode is RuntimeMode.NORMAL:
            raise ConfigError(f"capabilities.{name}.loss_mode cannot be NORMAL")
        result[name] = CapabilityConfig(tuple(alternatives), loss_mode)
    return result


def _parse_requirements(
    raw: Mapping[str, Any], capabilities: Mapping[str, CapabilityConfig]
) -> dict[Task, TaskRequirement]:
    result: dict[Task, TaskRequirement] = {}
    for task in Task:
        item = _mapping(raw.get(task.value), f"task_requirements.{task.value}")
        required = _text_list(item, "required")
        optional = _text_list(item, "optional")
        unknown = set(required + optional).difference(capabilities)
        if unknown:
            raise ConfigError(f"{task.value} references unknown capabilities: {sorted(unknown)}")
        if set(required).intersection(optional):
            raise ConfigError(f"{task.value} repeats required capability as optional")
        result[task] = TaskRequirement(required, optional)
    return result


def _parse_evidence(
    raw: Mapping[str, Any], key: str, signals: Mapping[str, SignalBinding]
) -> tuple[EvidenceSource, ...]:
    value = raw.get(key)
    if not isinstance(value, Sequence) or isinstance(value, str) or not value:
        raise ConfigError(f"estimators.{key} must be a nonempty list")
    result: list[EvidenceSource] = []
    for index, item_value in enumerate(value):
        item = _mapping(item_value, f"estimators.{key}[{index}]")
        role = _text(item, "role")
        if role not in signals:
            raise ConfigError(f"estimators.{key} references unknown role {role}")
        input_min = _number(item, "input_min")
        input_max = _number(item, "input_max")
        if input_min == input_max:
            raise ConfigError(f"estimators.{key} input range must not be zero")
        output_min = _number(item, "output_min")
        output_max = _number(item, "output_max")
        if not (
            0.0 <= output_min <= 1.0 and 0.0 <= output_max <= 1.0
        ):
            raise ConfigError(
                f"estimators.{key} output values must be in [0, 1]"
            )
        result.append(
            EvidenceSource(
                role=role,
                input_min=input_min,
                input_max=input_max,
                output_min=output_min,
                output_max=output_max,
                weight=_positive(item, "weight"),
            )
        )
    return tuple(result)


def _parse_margins(
    raw: Mapping[str, Any], signals: Mapping[str, SignalBinding]
) -> dict[str, MarginPolicy]:
    required_names = {"participation", "hand", "posture", "fatigue"}
    if set(raw) != required_names:
        raise ConfigError("quality_margins must define participation, hand, posture, and fatigue")
    result: dict[str, MarginPolicy] = {}
    for name, item_value in raw.items():
        item = _mapping(item_value, f"quality_margins.{name}")
        roles = _text_list(item, "roles")
        if not roles:
            raise ConfigError(f"quality_margins.{name}.roles must not be empty")
        unknown = set(roles).difference(signals)
        if unknown:
            raise ConfigError(
                f"quality_margins.{name} references unknown roles: {sorted(unknown)}"
            )
        try:
            direction = MarginDirection(_text(item, "direction"))
        except ValueError as exc:
            raise ConfigError(f"quality_margins.{name}.direction is invalid") from exc
        try:
            aggregation = MarginAggregation(_text(item, "aggregation"))
        except ValueError as exc:
            raise ConfigError(f"quality_margins.{name}.aggregation is invalid") from exc
        result[name] = MarginPolicy(
            roles=roles,
            aggregation=aggregation,
            boundary=_number(item, "boundary"),
            sigma=_positive(item, "sigma"),
            direction=direction,
        )
    return result


def _parse_adaptive(raw: Mapping[str, Any]) -> AdaptiveConfig:
    k_up = _positive(raw, "k_up")
    k_down = _positive(raw, "k_down")
    if k_up <= k_down:
        raise ConfigError("adaptive k_up must be greater than k_down")
    alpha_min = _number(raw, "alpha_min")
    alpha_max = _number(raw, "alpha_max")
    initial_alpha = _number(raw, "initial_alpha")
    if not 0.0 <= alpha_min < alpha_max <= 1.0:
        raise ConfigError("adaptive alpha bounds must satisfy 0 <= min < max <= 1")
    if not alpha_min <= initial_alpha <= alpha_max:
        raise ConfigError("initial_alpha must be within alpha bounds")
    return AdaptiveConfig(
        k_up=k_up,
        k_down=k_down,
        alpha_min=alpha_min,
        alpha_max=alpha_max,
        initial_alpha=initial_alpha,
        max_slew_per_s=_positive(raw, "max_slew_per_s"),
        progress_sigma=_positive(raw, "progress_sigma"),
        safe_progress_curve=_curve(raw, "safe_progress_curve"),
    )


def _parse_seat(raw: Mapping[str, Any]) -> SeatConfig:
    return SeatConfig(
        device_torque_curve=_curve(raw, "device_torque_curve"),
        support_torque_curve=_curve(raw, "support_torque_curve"),
        descent_safe_speed_curve=_curve(raw, "descent_safe_speed_curve"),
        descent_damping=_positive(raw, "descent_damping"),
    )


def _parse_wheel(raw: Mapping[str, Any]) -> WheelConfig:
    return WheelConfig(
        virtual_mass=_positive(raw, "virtual_mass"),
        linear_damping=_positive(raw, "linear_damping"),
        virtual_inertia=_positive(raw, "virtual_inertia"),
        yaw_damping=_positive(raw, "yaw_damping"),
        wheel_radius_m=_positive(raw, "wheel_radius_m"),
        track_width_m=_positive(raw, "track_width_m"),
        max_linear_m_s=_positive(raw, "max_linear_m_s"),
        max_yaw_rad_s=_positive(raw, "max_yaw_rad_s"),
        max_wheel_accel_rad_s2=_positive(raw, "max_wheel_accel_rad_s2"),
        controlled_stop_decel_rad_s2=_positive(raw, "controlled_stop_decel_rad_s2"),
    )


def _parse_limits(raw: Mapping[str, Any]) -> LimitsConfig:
    torque_min = _number(raw, "seat_torque_min_nm")
    torque_max = _number(raw, "seat_torque_max_nm")
    if torque_min >= torque_max:
        raise ConfigError("seat torque bounds are reversed")
    return LimitsConfig(
        seat_torque_min_nm=torque_min,
        seat_torque_max_nm=torque_max,
        wheel_speed_max_rad_s=_positive(raw, "wheel_speed_max_rad_s"),
        candidate_ttl_s=_positive(raw, "candidate_ttl_s"),
        max_dt_s=_positive(raw, "max_dt_s"),
    )


def _curve(raw: Mapping[str, Any], key: str) -> PiecewiseCurve:
    value = raw.get(key)
    if not isinstance(value, Sequence) or isinstance(value, str) or len(value) < 2:
        raise ConfigError(f"{key} must contain at least two points")
    points: list[tuple[float, float]] = []
    for index, point in enumerate(value):
        if not isinstance(point, Sequence) or isinstance(point, str) or len(point) != 2:
            raise ConfigError(f"{key}[{index}] must contain x and y")
        x = _finite_value(point[0], f"{key}[{index}].x")
        y = _finite_value(point[1], f"{key}[{index}].y")
        if points and x <= points[-1][0]:
            raise ConfigError(f"{key} x values must be strictly increasing")
        points.append((x, y))
    return PiecewiseCurve(tuple(points))


def _section(raw: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    return _mapping(raw.get(key), key)


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{path} must be a table")
    return value


def _number(raw: Mapping[str, Any], key: str) -> float:
    if key not in raw:
        raise ConfigError(f"missing required parameter {key}")
    return _finite_value(raw[key], key)


def _finite_value(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{path} must be finite")
    return result


def _positive(raw: Mapping[str, Any], key: str) -> float:
    result = _number(raw, key)
    if result <= 0.0:
        raise ConfigError(f"{key} must be positive")
    return result


def _bounded(raw: Mapping[str, Any], key: str, minimum: float, maximum: float) -> float:
    result = _number(raw, key)
    if not minimum <= result <= maximum:
        raise ConfigError(f"{key} must be in [{minimum}, {maximum}]")
    return result


def _text(raw: Mapping[str, Any], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise ConfigError(f"{key} must be nonempty text")
    return value


def _text_list(raw: Mapping[str, Any], key: str) -> tuple[str, ...]:
    value = raw.get(key)
    if not isinstance(value, Sequence) or isinstance(value, str):
        raise ConfigError(f"{key} must be a list")
    result = tuple(str(item) for item in value)
    if len(result) != len(set(result)):
        raise ConfigError(f"{key} contains duplicate values")
    return result


def _boolean(raw: Mapping[str, Any], key: str) -> bool:
    value = raw.get(key)
    if not isinstance(value, bool):
        raise ConfigError(f"{key} must be boolean")
    return value


def _integer(raw: Mapping[str, Any], key: str, minimum: int) -> int:
    value = raw.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ConfigError(f"{key} must be an integer >= {minimum}")
    return value
