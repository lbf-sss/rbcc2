from __future__ import annotations

from typing import Protocol

from relay_control.model import (
    ActuatorReceipt,
    AuditEvent,
    SafeCommand,
    SensorFrame,
)


class SensorPort(Protocol):
    def read(self, deadline_ns: int) -> SensorFrame:
        raise NotImplementedError


class ActuatorPort(Protocol):
    def apply(
        self, command: SafeCommand, deadline_ns: int
    ) -> ActuatorReceipt:
        raise NotImplementedError


class AuditPort(Protocol):
    def append(self, event: AuditEvent) -> None:
        raise NotImplementedError
