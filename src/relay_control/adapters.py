from __future__ import annotations

from collections import deque
from dataclasses import replace
import json
from pathlib import Path
from typing import Iterable, Mapping

from relay_control.model import (
    ActuatorReceipt,
    AuditEvent,
    SafeCommand,
    SensorFrame,
    SignalFlag,
    SignalSample,
)
from relay_control.ports import SensorPort


class SensorExhausted(RuntimeError):
    pass


class InMemorySensorAdapter:
    def __init__(self, frame: SensorFrame) -> None:
        self.frame = frame

    def read(self, deadline_ns: int) -> SensorFrame:
        return self.frame


class SequenceSensorAdapter:
    def __init__(self, frames: Iterable[SensorFrame]) -> None:
        self._frames = deque(frames)

    def read(self, deadline_ns: int) -> SensorFrame:
        if not self._frames:
            raise SensorExhausted("sensor frame sequence is exhausted")
        return self._frames.popleft()


class FaultInjectingSensorAdapter:
    def __init__(
        self,
        source: SensorPort,
        *,
        remove: frozenset[str] = frozenset(),
        stale_by_ns: Mapping[str, int] | None = None,
        confidence: Mapping[str, float] | None = None,
        flags: Mapping[str, frozenset[SignalFlag]] | None = None,
        values: Mapping[str, float] | None = None,
    ) -> None:
        self._source = source
        self._remove = remove
        self._stale_by_ns = dict(stale_by_ns or {})
        self._confidence = dict(confidence or {})
        self._flags = dict(flags or {})
        self._values = dict(values or {})

    def read(self, deadline_ns: int) -> SensorFrame:
        frame = self._source.read(deadline_ns)
        samples: dict[str, SignalSample] = {}
        for signal_id, original in frame.samples.items():
            if signal_id in self._remove:
                continue
            sample = original
            if signal_id in self._stale_by_ns:
                sample = replace(
                    sample,
                    timestamp_ns=max(
                        0,
                        sample.timestamp_ns - self._stale_by_ns[signal_id],
                    ),
                )
            if signal_id in self._confidence:
                sample = replace(
                    sample, confidence=self._confidence[signal_id]
                )
            if signal_id in self._flags:
                sample = replace(sample, flags=self._flags[signal_id])
            if signal_id in self._values:
                sample = replace(sample, value=self._values[signal_id])
            samples[signal_id] = sample
        return SensorFrame(
            sequence=frame.sequence,
            captured_at_ns=frame.captured_at_ns,
            samples=samples,
        )


class InMemoryActuatorAdapter:
    def __init__(self, *, accept: bool = True) -> None:
        self.accept = accept
        self.commands: list[SafeCommand] = []
        self.receipts: list[ActuatorReceipt] = []

    def apply(
        self, command: SafeCommand, deadline_ns: int
    ) -> ActuatorReceipt:
        if not isinstance(command, SafeCommand):
            raise TypeError("actuator accepts SafeCommand only")
        self.commands.append(command)
        receipt = ActuatorReceipt(
            sequence=command.sequence,
            accepted=self.accept,
            status="ACCEPTED" if self.accept else "REJECTED",
            reasons=() if self.accept else ("synthetic_rejection",),
        )
        self.receipts.append(receipt)
        return receipt


class ListAuditAdapter:
    def __init__(self) -> None:
        self.events: list[AuditEvent] = []

    def append(self, event: AuditEvent) -> None:
        self.events.append(event)


class JsonlFrameRecorder:
    def __init__(self, source: SensorPort, path: str | Path) -> None:
        self._source = source
        self._path = Path(path)

    def read(self, deadline_ns: int) -> SensorFrame:
        frame = self._source.read(deadline_ns)
        with self._path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(_frame_to_record(frame), sort_keys=True))
            stream.write("\n")
        return frame


class JsonlReplaySensorAdapter(SequenceSensorAdapter):
    def __init__(self, path: str | Path) -> None:
        frames: list[SensorFrame] = []
        with Path(path).open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                    frames.append(_record_to_frame(record))
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"invalid frame recording at line {line_number}"
                    ) from exc
        super().__init__(frames)


def _frame_to_record(frame: SensorFrame) -> dict:
    return {
        "schema_version": 1,
        "sequence": frame.sequence,
        "captured_at_ns": frame.captured_at_ns,
        "samples": [
            {
                "signal_id": sample.signal_id,
                "value": sample.value,
                "unit": sample.unit,
                "timestamp_ns": sample.timestamp_ns,
                "confidence": sample.confidence,
                "calibration_version": sample.calibration_version,
                "source": sample.source,
                "flags": sorted(flag.value for flag in sample.flags),
            }
            for sample in sorted(
                frame.samples.values(), key=lambda item: item.signal_id
            )
        ],
    }


def _record_to_frame(record: dict) -> SensorFrame:
    if record["schema_version"] != 1:
        raise ValueError("unsupported frame schema version")
    samples = {
        item["signal_id"]: SignalSample(
            signal_id=item["signal_id"],
            value=float(item["value"]),
            unit=item["unit"],
            timestamp_ns=int(item["timestamp_ns"]),
            confidence=float(item["confidence"]),
            calibration_version=item["calibration_version"],
            source=item["source"],
            flags=frozenset(SignalFlag(flag) for flag in item["flags"]),
        )
        for item in record["samples"]
    }
    return SensorFrame(
        sequence=int(record["sequence"]),
        captured_at_ns=int(record["captured_at_ns"]),
        samples=samples,
    )
