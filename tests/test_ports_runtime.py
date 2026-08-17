from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from relay_control.adapters import (
    FaultInjectingSensorAdapter,
    InMemoryActuatorAdapter,
    JsonlFrameRecorder,
    JsonlReplaySensorAdapter,
    ListAuditAdapter,
    SequenceSensorAdapter,
)
from relay_control.config import parse_config
from relay_control.demo import run_synthetic_scenario
from relay_control.model import (
    OperatorInput,
    RuntimeMode,
    SafeCommand,
    SafetyContext,
    Task,
)
from relay_control.runtime import DeviceRuntime
from tests.helpers import NOW_NS, synthetic_config_dict, valid_frame


def operator_input() -> OperatorInput:
    return OperatorInput(
        local_confirmation=True,
        active_intent=True,
        transport_linear_m_s=0.2,
    )


def safety_context(*, emergency_stop: bool = False) -> SafetyContext:
    return SafetyContext(
        emergency_stop=emergency_stop,
        actuator_healthy=True,
        brakes_healthy=True,
        seat_locked=True,
        allowed_tasks=frozenset(Task),
    )


def runtime_for(sensor) -> tuple[DeviceRuntime, InMemoryActuatorAdapter, ListAuditAdapter]:
    config = parse_config(synthetic_config_dict())
    actuator = InMemoryActuatorAdapter()
    audit = ListAuditAdapter()
    return DeviceRuntime(config, sensor, actuator, audit), actuator, audit


class PortRuntimeTests(unittest.TestCase):
    def test_synthetic_scenario_exercises_all_degradation_modes(self) -> None:
        config = parse_config(synthetic_config_dict())

        summaries = run_synthetic_scenario(config)

        self.assertEqual(
            [summary["mode"] for summary in summaries],
            [
                "NORMAL",
                "DEGRADED_HOLD",
                "CONTROLLED_STOP",
                "SAFE_HOLD",
            ],
        )
        for summary in summaries:
            self.assertTrue(summary["finite"])
            self.assertTrue(summary["within_limits"])

    def test_runtime_calls_actuator_only_with_safe_command(self) -> None:
        runtime, actuator, audit = runtime_for(
            SequenceSensorAdapter([valid_frame()])
        )

        result = runtime.cycle(
            Task.STAND_UP,
            operator_input(),
            safety_context(),
            NOW_NS,
            0.02,
        )

        self.assertTrue(result.receipt.accepted)
        self.assertEqual(len(actuator.commands), 1)
        self.assertIsInstance(actuator.commands[0], SafeCommand)
        self.assertEqual(len(audit.events), 1)

    def test_fault_adapter_removes_optional_signal_without_stopping_cycle(self) -> None:
        source = SequenceSensorAdapter([valid_frame()])
        faulted = FaultInjectingSensorAdapter(
            source,
            remove=frozenset({"camera.posture_margin"}),
        )
        runtime, actuator, _ = runtime_for(faulted)

        result = runtime.cycle(
            Task.STAND_UP,
            operator_input(),
            safety_context(),
            NOW_NS,
            0.02,
        )

        self.assertEqual(result.decision.mode, RuntimeMode.DEGRADED_HOLD)
        self.assertTrue(result.receipt.accepted)
        self.assertEqual(len(actuator.commands), 1)

    def test_fault_adapter_stales_critical_signal_into_safe_hold(self) -> None:
        source = SequenceSensorAdapter([valid_frame()])
        faulted = FaultInjectingSensorAdapter(
            source,
            stale_by_ns={"sim.seat.angle": 500_000_000},
        )
        runtime, _, _ = runtime_for(faulted)

        result = runtime.cycle(
            Task.STAND_UP,
            operator_input(),
            safety_context(),
            NOW_NS,
            0.02,
        )

        self.assertEqual(result.safe_command.mode, RuntimeMode.SAFE_HOLD)
        self.assertTrue(result.safe_command.brake_request)

    def test_recorded_frames_replay_deterministically(self) -> None:
        second_time = NOW_NS + 20_000_000
        frames = [
            valid_frame(sequence=1, now_ns=NOW_NS),
            valid_frame(sequence=2, now_ns=second_time),
        ]
        with tempfile.TemporaryDirectory() as directory:
            recording = Path(directory) / "frames.jsonl"
            first_sensor = JsonlFrameRecorder(
                SequenceSensorAdapter(frames), recording
            )
            first_runtime, _, _ = runtime_for(first_sensor)
            first = [
                first_runtime.cycle(
                    Task.STAND_UP,
                    operator_input(),
                    safety_context(),
                    timestamp,
                    0.02,
                )
                for timestamp in (NOW_NS, second_time)
            ]

            second_runtime, _, _ = runtime_for(
                JsonlReplaySensorAdapter(recording)
            )
            second = [
                second_runtime.cycle(
                    Task.STAND_UP,
                    operator_input(),
                    safety_context(),
                    timestamp,
                    0.02,
                )
                for timestamp in (NOW_NS, second_time)
            ]

        first_summary = [
            (
                item.decision.mode,
                item.decision.assistance_alpha,
                item.safe_command.seat_torque_nm,
                item.decision.reasons,
            )
            for item in first
        ]
        second_summary = [
            (
                item.decision.mode,
                item.decision.assistance_alpha,
                item.safe_command.seat_torque_nm,
                item.decision.reasons,
            )
            for item in second
        ]
        self.assertEqual(first_summary, second_summary)

    def test_sensor_exception_is_converted_to_safe_hold(self) -> None:
        runtime, actuator, _ = runtime_for(SequenceSensorAdapter([]))

        result = runtime.cycle(
            Task.GAIT,
            operator_input(),
            safety_context(),
            NOW_NS,
            0.02,
        )

        self.assertEqual(result.safe_command.mode, RuntimeMode.SAFE_HOLD)
        self.assertIn("sensor_port_error:SensorExhausted", result.safe_command.reasons)
        self.assertEqual(len(actuator.commands), 1)


if __name__ == "__main__":
    unittest.main()
