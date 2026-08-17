# RELAY Adaptive Control Core

> **SYNTHETIC SOFTWARE REFERENCE ONLY. NOT VALIDATED OR APPROVED FOR HUMAN USE.**
>
> Do not connect this repository directly to a person or production actuator. Hardware-in-the-loop testing, real-time analysis, actuator and MCU protection, mechanical risk controls, hazard analysis, and supervised clinical validation are outside this milestone and remain mandatory.

This repository implements the deterministic reference algorithm described in the RELAY continuous blended assistance specification. It covers a rotating seat assistance axis, independent left/right wheel following, partial sensor loss, a constrained Agent interface, replay, and fault injection.

## What Is Implemented

- continuous sit-to-stand assistance with faster support increase and slower withdrawal;
- separate device compensation and rehabilitation-assistance terms;
- controlled descent damping;
- force-based wheel admittance and differential steering;
- dynamic semantic signals rather than a fixed vendor sensor record;
- task-specific alternative evidence groups and explicit missing-data behavior;
- `NORMAL`, `DEGRADED_HOLD`, `CONTROLLED_STOP`, and `SAFE_HOLD` modes;
- replaceable sensor, actuator, and audit interfaces;
- a high-level Agent gateway with no actuator command surface;
- synthetic, fault-injection, JSONL recording, and replay adapters.

It does not include production CAN, CANopen, EtherCAT, serial, ROS 2, motor-driver, brake-controller, or medical-device integrations.

## Run the Demo

Python 3.11 or newer is required. The implementation has no runtime third-party dependencies.

```bash
PYTHONPATH=src python3 -m relay_control.demo --config config/synthetic.toml
```

The scenario emits JSON for:

1. complete data in `NORMAL`;
2. missing camera and fatigue data in `DEGRADED_HOLD`;
3. missing walking handle intent in `CONTROLLED_STOP`;
4. emergency stop in `SAFE_HOLD`.

The numeric values in `config/synthetic.toml` are arbitrary software-test values. The parser rejects any repository configuration marked `human_use = true`.

## Run Tests

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 -m compileall -q src tests
```

## Core Interface

`ControlEngine` is deterministic and performs no I/O:

```python
from relay_control import ControlEngine, ControlInput, load_config

config = load_config("config/synthetic.toml")
engine = ControlEngine(config)
decision = engine.step(control_input)
```

`decision` contains the candidate command, runtime mode, assistance state, signal contributors, rejected signals, quality margins, quality gate, and stable reason codes.

The execution path remains separate:

```text
SensorPort.read
-> ControlEngine.step
-> SafetyGuard.review
-> ActuatorPort.apply(SafeCommand)
-> AuditPort.append
```

Only `DeviceRuntime` calls `ActuatorPort.apply`. A concrete actuator adapter accepts `SafeCommand`, not `ControlCandidate`.

## Sensor Integration

A hardware adapter implements one method:

```python
class MySensorAdapter:
    def read(self, deadline_ns: int) -> SensorFrame:
        return frame_from_my_bus(deadline_ns)
```

`SensorFrame.samples` is a mapping keyed by vendor-independent signal IDs. Every `SignalSample` carries value, unit, monotonic timestamp, confidence, calibration version, source, and quality flags. The TOML file binds control roles such as `seat_angle` or `left_handle_forward` to those signal IDs.

Adding a vendor sensor therefore requires an adapter and configuration update, not an engine edit. Raw sensors may also be normalized upstream into the semantic quantities expected by the configured estimator.

Missing values stay absent. They are never silently converted to zero:

- an available alternative evidence group can preserve the capability;
- loss of optional quality evidence selects `DEGRADED_HOLD`, sets the withdrawal gate to zero, and prevents baseline updates;
- loss of gait intent selects `CONTROLLED_STOP`;
- loss of seat or wheel closed-loop feedback selects `SAFE_HOLD` under the sample policy.

Requiredness and loss behavior are task-specific TOML declarations. Safety semantics remain explicit even though device addresses, thresholds, gains, curves, and evidence combinations are configurable.

## Actuator Integration

A future hardware adapter implements:

```python
class MyActuatorAdapter:
    def apply(self, command: SafeCommand, deadline_ns: int) -> ActuatorReceipt:
        return send_validated_command_to_driver(command, deadline_ns)
```

Production integration must independently verify command sequence, deadline, configuration version, units, driver state, limits, watchdog operation, braking, and receipt status. The Python `SafetyGuard` is a testable reference and is not a replacement for an independent safety process, MCU protection, drive limits, or mechanical fail-safe design.

## Agent Integration

`AgentGateway.handle(request)` accepts only:

- `QUERY_STATUS`;
- `REQUEST_TASK`;
- `PAUSE_TASK`;
- `STOP_TASK`;
- `SUBMIT_HEALTH_CONTEXT`;
- `SUBMIT_PARAMETER_SUGGESTION`.

Identity, version, issue/expiry time, request size, JSON structure, and idempotency are checked. Task requests return `PENDING_LOCAL_CONFIRMATION`; parameter suggestions return `PENDING_THERAPIST_REVIEW`.

Keys containing torque, speed, velocity, position, brake, current, PWM, actuator, or setpoint semantics are rejected recursively. The Agent module does not import the runtime, safety, port, or adapter modules.

## Recording and Fault Injection

`JsonlFrameRecorder` wraps any sensor adapter and records versioned frames. `JsonlReplaySensorAdapter` replays those frames through the same engine. `FaultInjectingSensorAdapter` can remove signals, shift timestamps, lower confidence, add flags, or override values.

These tools support deterministic software tests. They do not establish timing correctness, closed-loop stability, or clinical safety.

## Before Hardware Use

At minimum, a separate engineering program must complete:

- calibrated device gravity, friction, support, and descent curves;
- actuator bandwidth and discrete-time stability analysis;
- independent safety process and physical device-node permissions;
- drive and MCU current, torque, speed, temperature, travel, and watchdog limits;
- seat lock, brake, emergency-stop, and power-loss validation;
- bench fault injection and hardware-in-the-loop testing;
- formal hazard analysis, traceability, and applicable regulatory work;
- supervised staged validation before any human interaction.

The software tests in this repository verify only deterministic reference behavior under synthetic inputs.
