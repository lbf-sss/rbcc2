"""RELAY deterministic reference control core.

This package is a software reference and is not validated for human use.
"""

__version__ = "0.1.0"

from relay_control.config import ControlConfig, load_config, parse_config
from relay_control.engine import ControlEngine
from relay_control.model import (
    ControlInput,
    OperatorInput,
    RuntimeMode,
    SafeCommand,
    SafetyContext,
    SensorFrame,
    SignalSample,
    Task,
)
from relay_control.ports import ActuatorPort, AuditPort, SensorPort

__all__ = [
    "ActuatorPort",
    "AuditPort",
    "ControlConfig",
    "ControlEngine",
    "ControlInput",
    "OperatorInput",
    "RuntimeMode",
    "SafeCommand",
    "SafetyContext",
    "SensorFrame",
    "SensorPort",
    "SignalSample",
    "Task",
    "load_config",
    "parse_config",
]
