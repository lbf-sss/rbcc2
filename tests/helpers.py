from __future__ import annotations

from copy import deepcopy
from typing import Any

from relay_control.model import SensorFrame, SignalFlag, SignalSample


NOW_NS = 1_000_000_000


def synthetic_config_dict() -> dict[str, Any]:
    config: dict[str, Any] = {
        "metadata": {
            "configuration_id": "synthetic-test-v1",
            "version": 1,
            "human_use": False,
        },
        "signals": {
            "seat_angle": _binding("sim.seat.angle", "rad"),
            "seat_velocity": _binding("sim.seat.velocity", "rad/s"),
            "seat_torque": _binding("sim.seat.torque", "N*m"),
            "seat_load": _binding("sim.seat.load", "N"),
            "affected_load": _binding("sim.foot.affected_load", "N"),
            "unaffected_load": _binding("sim.foot.unaffected_load", "N"),
            "trunk_pitch": _binding("sim.imu.trunk_pitch", "rad"),
            "left_handle_forward": _binding("sim.handle.left.forward", "N"),
            "right_handle_forward": _binding("sim.handle.right.forward", "N"),
            "left_handle_vertical": _binding("sim.handle.left.vertical", "N"),
            "right_handle_vertical": _binding("sim.handle.right.vertical", "N"),
            "left_wheel_speed": _binding("sim.wheel.left.speed", "rad/s"),
            "right_wheel_speed": _binding("sim.wheel.right.speed", "rad/s"),
            "posture_margin": _binding("camera.posture_margin", "1", required=False),
            "fatigue_margin": _binding("wearable.fatigue_margin", "1", required=False),
            "actuator_health": _binding("sim.actuator.health", "bool"),
            "brake_health": _binding("sim.brake.health", "bool"),
        },
        "capabilities": {
            "seat_feedback": {
                "alternatives": [["seat_angle", "seat_velocity", "seat_torque"]],
                "loss_mode": "SAFE_HOLD",
            },
            "stand_phase": {
                "alternatives": [
                    ["seat_load", "trunk_pitch"],
                    ["seat_load", "affected_load", "unaffected_load"],
                ],
                "loss_mode": "CONTROLLED_STOP",
            },
            "gait_intent": {
                "alternatives": [["left_handle_forward", "right_handle_forward"]],
                "loss_mode": "CONTROLLED_STOP",
            },
            "wheel_feedback": {
                "alternatives": [["left_wheel_speed", "right_wheel_speed", "brake_health"]],
                "loss_mode": "SAFE_HOLD",
            },
            "posture_quality": {
                "alternatives": [["posture_margin"]],
                "loss_mode": "DEGRADED_HOLD",
            },
            "fatigue_quality": {
                "alternatives": [["fatigue_margin"]],
                "loss_mode": "DEGRADED_HOLD",
            },
            "participation_quality": {
                "alternatives": [["affected_load"]],
                "loss_mode": "DEGRADED_HOLD",
            },
            "hand_quality": {
                "alternatives": [["left_handle_vertical", "right_handle_vertical"]],
                "loss_mode": "DEGRADED_HOLD",
            },
        },
        "task_requirements": {
            "IDLE": {"required": [], "optional": []},
            "STAND_UP": {
                "required": ["seat_feedback", "stand_phase"],
                "optional": ["participation_quality", "hand_quality", "posture_quality", "fatigue_quality"],
            },
            "SIT_DOWN": {
                "required": ["seat_feedback", "stand_phase"],
                "optional": ["participation_quality", "hand_quality", "posture_quality", "fatigue_quality"],
            },
            "GAIT": {
                "required": ["gait_intent", "wheel_feedback"],
                "optional": ["participation_quality", "hand_quality", "posture_quality", "fatigue_quality"],
            },
            "SEATED_TRANSPORT": {
                "required": ["wheel_feedback"],
                "optional": [],
            },
        },
        "estimators": {
            "phase": [
                {"role": "seat_angle", "input_min": 0.0, "input_max": 1.0, "output_min": 0.0, "output_max": 1.0, "weight": 1.0},
                {"role": "seat_load", "input_min": 600.0, "input_max": 0.0, "output_min": 0.0, "output_max": 1.0, "weight": 0.5},
            ],
            "intent": [
                {"role": "trunk_pitch", "input_min": 0.0, "input_max": 0.5, "output_min": 0.0, "output_max": 1.0, "weight": 1.0},
                {"role": "seat_load", "input_min": 600.0, "input_max": 0.0, "output_min": 0.0, "output_max": 1.0, "weight": 0.5},
            ],
        },
        "quality_margins": {
            "participation": {"roles": ["affected_load"], "aggregation": "MEAN", "boundary": 100.0, "sigma": 100.0, "direction": "ABOVE"},
            "hand": {"roles": ["left_handle_vertical", "right_handle_vertical"], "aggregation": "SUM", "boundary": 100.0, "sigma": 100.0, "direction": "BELOW"},
            "posture": {"roles": ["posture_margin"], "aggregation": "MEAN", "boundary": 0.0, "sigma": 1.0, "direction": "ABOVE"},
            "fatigue": {"roles": ["fatigue_margin"], "aggregation": "MEAN", "boundary": 0.0, "sigma": 1.0, "direction": "ABOVE"},
        },
        "adaptive": {
            "k_up": 0.8,
            "k_down": 0.2,
            "alpha_min": 0.0,
            "alpha_max": 1.0,
            "initial_alpha": 0.4,
            "max_slew_per_s": 0.5,
            "progress_sigma": 0.2,
            "safe_progress_curve": [[0.0, 0.05], [1.0, 0.02]],
        },
        "seat": {
            "device_torque_curve": [[0.0, 1.0], [1.0, 2.0]],
            "support_torque_curve": [[0.0, 12.0], [0.5, 20.0], [1.0, 5.0]],
            "descent_safe_speed_curve": [[0.0, 0.1], [1.0, 0.1]],
            "descent_damping": 4.0,
        },
        "wheel": {
            "virtual_mass": 20.0,
            "linear_damping": 10.0,
            "virtual_inertia": 8.0,
            "yaw_damping": 4.0,
            "wheel_radius_m": 0.15,
            "track_width_m": 0.55,
            "max_linear_m_s": 0.5,
            "max_yaw_rad_s": 0.8,
            "max_wheel_accel_rad_s2": 2.0,
            "controlled_stop_decel_rad_s2": 3.0,
        },
        "limits": {
            "seat_torque_min_nm": -30.0,
            "seat_torque_max_nm": 30.0,
            "wheel_speed_max_rad_s": 4.0,
            "candidate_ttl_s": 0.1,
            "max_dt_s": 0.2,
        },
    }
    return deepcopy(config)


def _binding(signal_id: str, unit: str, *, required: bool = True) -> dict[str, Any]:
    return {
        "signal_id": signal_id,
        "unit": unit,
        "max_age_s": 0.1,
        "min_confidence": 0.8,
        "required": required,
    }


def sample(
    signal_id: str,
    value: float,
    unit: str,
    *,
    timestamp_ns: int = NOW_NS,
    confidence: float = 1.0,
    flags: frozenset[SignalFlag] = frozenset(),
) -> SignalSample:
    return SignalSample(
        signal_id=signal_id,
        value=value,
        unit=unit,
        timestamp_ns=timestamp_ns,
        confidence=confidence,
        calibration_version="synthetic-cal-v1",
        source="synthetic-test",
        flags=flags,
    )


def valid_frame(*, sequence: int = 1, now_ns: int = NOW_NS) -> SensorFrame:
    values = {
        "sim.seat.angle": sample("sim.seat.angle", 0.4, "rad", timestamp_ns=now_ns),
        "sim.seat.velocity": sample("sim.seat.velocity", 0.1, "rad/s", timestamp_ns=now_ns),
        "sim.seat.torque": sample("sim.seat.torque", 3.0, "N*m", timestamp_ns=now_ns),
        "sim.seat.load": sample("sim.seat.load", 300.0, "N", timestamp_ns=now_ns),
        "sim.foot.affected_load": sample("sim.foot.affected_load", 160.0, "N", timestamp_ns=now_ns),
        "sim.foot.unaffected_load": sample("sim.foot.unaffected_load", 300.0, "N", timestamp_ns=now_ns),
        "sim.imu.trunk_pitch": sample("sim.imu.trunk_pitch", 0.2, "rad", timestamp_ns=now_ns),
        "sim.handle.left.forward": sample("sim.handle.left.forward", 5.0, "N", timestamp_ns=now_ns),
        "sim.handle.right.forward": sample("sim.handle.right.forward", 5.0, "N", timestamp_ns=now_ns),
        "sim.handle.left.vertical": sample("sim.handle.left.vertical", 30.0, "N", timestamp_ns=now_ns),
        "sim.handle.right.vertical": sample("sim.handle.right.vertical", 30.0, "N", timestamp_ns=now_ns),
        "sim.wheel.left.speed": sample("sim.wheel.left.speed", 0.1, "rad/s", timestamp_ns=now_ns),
        "sim.wheel.right.speed": sample("sim.wheel.right.speed", 0.1, "rad/s", timestamp_ns=now_ns),
        "camera.posture_margin": sample("camera.posture_margin", 0.8, "1", timestamp_ns=now_ns),
        "wearable.fatigue_margin": sample("wearable.fatigue_margin", 0.8, "1", timestamp_ns=now_ns),
        "sim.actuator.health": sample("sim.actuator.health", 1.0, "bool", timestamp_ns=now_ns),
        "sim.brake.health": sample("sim.brake.health", 1.0, "bool", timestamp_ns=now_ns),
    }
    return SensorFrame(sequence=sequence, captured_at_ns=now_ns, samples=values)


def frame_without(*signal_ids: str, now_ns: int = NOW_NS) -> SensorFrame:
    original = valid_frame(now_ns=now_ns)
    removed = set(signal_ids)
    return SensorFrame(
        sequence=original.sequence,
        captured_at_ns=original.captured_at_ns,
        samples={
            signal_id: value
            for signal_id, value in original.samples.items()
            if signal_id not in removed
        },
    )
