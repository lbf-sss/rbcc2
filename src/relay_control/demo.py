from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from relay_control.adapters import (
    InMemoryActuatorAdapter,
    ListAuditAdapter,
    SequenceSensorAdapter,
)
from relay_control.config import ControlConfig, load_config
from relay_control.model import (
    OperatorInput,
    SafetyContext,
    SensorFrame,
    SignalSample,
    Task,
)
from relay_control.runtime import DeviceRuntime


_SYNTHETIC_VALUES = {
    "seat_angle": 0.4,
    "seat_velocity": 0.1,
    "seat_torque": 3.0,
    "seat_load": 300.0,
    "affected_load": 160.0,
    "unaffected_load": 300.0,
    "trunk_pitch": 0.2,
    "left_handle_forward": 5.0,
    "right_handle_forward": 5.0,
    "left_handle_vertical": 30.0,
    "right_handle_vertical": 30.0,
    "left_wheel_speed": 0.1,
    "right_wheel_speed": 0.1,
    "posture_margin": 0.8,
    "fatigue_margin": 0.8,
    "actuator_health": 1.0,
    "brake_health": 1.0,
}


def run_synthetic_scenario(
    config: ControlConfig,
) -> list[dict[str, object]]:
    start_ns = 1_000_000_000
    step_ns = 20_000_000
    times = [start_ns + index * step_ns for index in range(4)]
    frames = [
        _synthetic_frame(config, 1, times[0]),
        _synthetic_frame(
            config,
            2,
            times[1],
            omit_roles=frozenset({"posture_margin", "fatigue_margin"}),
        ),
        _synthetic_frame(
            config,
            3,
            times[2],
            omit_roles=frozenset(
                {"left_handle_forward", "right_handle_forward"}
            ),
        ),
        _synthetic_frame(config, 4, times[3]),
    ]
    actuator = InMemoryActuatorAdapter()
    runtime = DeviceRuntime(
        config,
        SequenceSensorAdapter(frames),
        actuator,
        ListAuditAdapter(),
    )
    operator = OperatorInput(
        local_confirmation=True,
        active_intent=True,
        transport_linear_m_s=0.2,
    )
    tasks = [Task.STAND_UP, Task.STAND_UP, Task.GAIT, Task.GAIT]
    summaries: list[dict[str, object]] = []

    for index, (task, now_ns) in enumerate(zip(tasks, times)):
        result = runtime.cycle(
            task,
            operator,
            _safety_context(emergency_stop=index == 3),
            now_ns,
            step_ns / 1_000_000_000,
        )
        command = result.safe_command
        values = (
            command.seat_torque_nm,
            command.left_wheel_rad_s,
            command.right_wheel_rad_s,
        )
        limits = config.limits
        summaries.append(
            {
                "cycle": index + 1,
                "task": task.value,
                "mode": command.mode.value,
                "decision_mode": result.decision.mode.value,
                "assistance_alpha": result.decision.assistance_alpha,
                "seat_mode": command.seat_mode.value,
                "seat_torque_nm": command.seat_torque_nm,
                "left_wheel_rad_s": command.left_wheel_rad_s,
                "right_wheel_rad_s": command.right_wheel_rad_s,
                "brake_request": command.brake_request,
                "finite": all(math.isfinite(value) for value in values),
                "within_limits": (
                    limits.seat_torque_min_nm
                    <= command.seat_torque_nm
                    <= limits.seat_torque_max_nm
                    and abs(command.left_wheel_rad_s)
                    <= limits.wheel_speed_max_rad_s
                    and abs(command.right_wheel_rad_s)
                    <= limits.wheel_speed_max_rad_s
                ),
                "reasons": list(command.reasons),
                "rejected_signals": list(
                    result.decision.trace.rejected_signals
                ),
                "actuator_accepted": result.receipt.accepted,
            }
        )
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the RELAY non-clinical synthetic control scenario."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/synthetic.toml"),
        help="complete synthetic TOML configuration",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    print(
        json.dumps(
            {
                "warning": (
                    "SYNTHETIC SOFTWARE DEMO ONLY; NOT VALIDATED OR "
                    "APPROVED FOR HUMAN USE"
                )
            },
            sort_keys=True,
        )
    )
    for summary in run_synthetic_scenario(config):
        print(json.dumps(summary, sort_keys=True))
    return 0


def _synthetic_frame(
    config: ControlConfig,
    sequence: int,
    timestamp_ns: int,
    *,
    omit_roles: frozenset[str] = frozenset(),
) -> SensorFrame:
    samples = {}
    for role, binding in config.signals.items():
        if role in omit_roles:
            continue
        if role not in _SYNTHETIC_VALUES:
            raise ValueError(f"synthetic demo has no value for role {role}")
        sample = SignalSample(
            signal_id=binding.signal_id,
            value=_SYNTHETIC_VALUES[role],
            unit=binding.unit,
            timestamp_ns=timestamp_ns,
            confidence=1.0,
            calibration_version="synthetic-demo-cal-v1",
            source="relay-control-demo",
        )
        samples[sample.signal_id] = sample
    return SensorFrame(sequence, timestamp_ns, samples)


def _safety_context(*, emergency_stop: bool) -> SafetyContext:
    return SafetyContext(
        emergency_stop=emergency_stop,
        actuator_healthy=True,
        brakes_healthy=True,
        seat_locked=True,
        allowed_tasks=frozenset(Task),
    )


if __name__ == "__main__":
    raise SystemExit(main())
