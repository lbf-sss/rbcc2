from __future__ import annotations

from dataclasses import dataclass

from relay_control.config import AdaptiveConfig, PiecewiseCurve, SeatConfig, WheelConfig
from relay_control.model import Estimate, RuntimeMode, Task


@dataclass(frozen=True)
class SeatControlResult:
    torque_nm: float
    device_compensation_nm: float
    rehabilitation_assistance_nm: float
    descent_damping_nm: float


@dataclass(frozen=True)
class WheelControlResult:
    left_wheel_rad_s: float
    right_wheel_rad_s: float
    brake_request: bool


def interpolate_curve(curve: PiecewiseCurve, x: float) -> float:
    points = curve.points
    if x <= points[0][0]:
        return points[0][1]
    if x >= points[-1][0]:
        return points[-1][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            ratio = (x - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    raise RuntimeError("validated curve has no interpolation interval")


def update_assistance(
    alpha: float,
    progress_margin: float,
    quality_gate: float,
    dt_s: float,
    config: AdaptiveConfig,
) -> float:
    if dt_s < 0.0:
        raise ValueError("dt_s must be nonnegative")
    raw_rate = config.k_up * max(-progress_margin, 0.0)
    raw_rate -= config.k_down * max(progress_margin, 0.0) * _clip(
        quality_gate, 0.0, 1.0
    )
    rate = _clip(raw_rate, -config.max_slew_per_s, config.max_slew_per_s)
    return _clip(
        alpha + rate * dt_s,
        config.alpha_min,
        config.alpha_max,
    )


def seat_control(
    task: Task,
    seat_angle: float,
    seat_angular_velocity: float,
    phase: Estimate,
    intent: Estimate,
    alpha: float,
    config: SeatConfig,
) -> SeatControlResult:
    device = interpolate_curve(config.device_torque_curve, seat_angle)
    support = interpolate_curve(config.support_torque_curve, phase.value)
    descent_damping = 0.0

    if task is Task.STAND_UP:
        rehabilitation = intent.value * alpha * support
    elif task is Task.SIT_DOWN:
        rehabilitation = alpha * support
        safe_speed = interpolate_curve(
            config.descent_safe_speed_curve, phase.value
        )
        excess_speed = max(abs(seat_angular_velocity) - safe_speed, 0.0)
        descent_damping = config.descent_damping * excess_speed
    else:
        rehabilitation = 0.0

    return SeatControlResult(
        torque_nm=device + rehabilitation + descent_damping,
        device_compensation_nm=device,
        rehabilitation_assistance_nm=rehabilitation,
        descent_damping_nm=descent_damping,
    )


class WheelController:
    def __init__(self, config: WheelConfig) -> None:
        self._config = config
        self._linear_velocity = 0.0
        self._yaw_rate = 0.0
        self._left_wheel = 0.0
        self._right_wheel = 0.0

    def step(
        self,
        left_forward_force: float,
        right_forward_force: float,
        dt_s: float,
        mode: RuntimeMode,
    ) -> WheelControlResult:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if mode is RuntimeMode.SAFE_HOLD:
            self.reset()
            return WheelControlResult(0.0, 0.0, True)
        if mode is RuntimeMode.CONTROLLED_STOP:
            maximum_delta = self._config.controlled_stop_decel_rad_s2 * dt_s
            left = _approach_zero(self._left_wheel, maximum_delta)
            right = _approach_zero(self._right_wheel, maximum_delta)
            self._store_wheel_state(left, right)
            return WheelControlResult(left, right, left == 0.0 and right == 0.0)

        linear_acceleration = (
            left_forward_force
            + right_forward_force
            - self._config.linear_damping * self._linear_velocity
        ) / self._config.virtual_mass
        yaw_acceleration = (
            right_forward_force
            - left_forward_force
            - self._config.yaw_damping * self._yaw_rate
        ) / self._config.virtual_inertia
        linear = _clip(
            self._linear_velocity + linear_acceleration * dt_s,
            -self._config.max_linear_m_s,
            self._config.max_linear_m_s,
        )
        yaw = _clip(
            self._yaw_rate + yaw_acceleration * dt_s,
            -self._config.max_yaw_rad_s,
            self._config.max_yaw_rad_s,
        )
        target_right = (
            linear + 0.5 * self._config.track_width_m * yaw
        ) / self._config.wheel_radius_m
        target_left = (
            linear - 0.5 * self._config.track_width_m * yaw
        ) / self._config.wheel_radius_m
        maximum_delta = self._config.max_wheel_accel_rad_s2 * dt_s
        left = _slew(self._left_wheel, target_left, maximum_delta)
        right = _slew(self._right_wheel, target_right, maximum_delta)
        self._store_wheel_state(left, right)
        return WheelControlResult(left, right, False)

    def transport(
        self,
        linear_m_s: float,
        yaw_rad_s: float,
        dt_s: float,
        mode: RuntimeMode,
    ) -> WheelControlResult:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if mode is RuntimeMode.SAFE_HOLD:
            self.reset()
            return WheelControlResult(0.0, 0.0, True)
        if mode is RuntimeMode.CONTROLLED_STOP:
            return self.step(0.0, 0.0, dt_s, mode)

        linear = _clip(
            linear_m_s,
            -self._config.max_linear_m_s,
            self._config.max_linear_m_s,
        )
        yaw = _clip(
            yaw_rad_s,
            -self._config.max_yaw_rad_s,
            self._config.max_yaw_rad_s,
        )
        target_right = (
            linear + 0.5 * self._config.track_width_m * yaw
        ) / self._config.wheel_radius_m
        target_left = (
            linear - 0.5 * self._config.track_width_m * yaw
        ) / self._config.wheel_radius_m
        maximum_delta = self._config.max_wheel_accel_rad_s2 * dt_s
        left = _slew(self._left_wheel, target_left, maximum_delta)
        right = _slew(self._right_wheel, target_right, maximum_delta)
        self._store_wheel_state(left, right)
        return WheelControlResult(left, right, False)

    def reset(self) -> None:
        self._linear_velocity = 0.0
        self._yaw_rate = 0.0
        self._left_wheel = 0.0
        self._right_wheel = 0.0

    def _store_wheel_state(self, left: float, right: float) -> None:
        self._left_wheel = left
        self._right_wheel = right
        radius = self._config.wheel_radius_m
        self._linear_velocity = radius * (left + right) / 2.0
        self._yaw_rate = radius * (right - left) / self._config.track_width_m


def _slew(current: float, target: float, maximum_delta: float) -> float:
    return current + _clip(target - current, -maximum_delta, maximum_delta)


def _approach_zero(value: float, maximum_delta: float) -> float:
    if abs(value) <= maximum_delta:
        return 0.0
    return value - maximum_delta if value > 0.0 else value + maximum_delta


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
