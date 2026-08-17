# RELAY Adaptive Control Core Design

**Date:** 2026-08-17

**Status:** Approved for implementation

**Product baseline:** [RELAY continuous blended assistance design](relay连续混合助力设计.md) and [RELAY product overview](relay完整产品功能简介.md)

## 1. Milestone

This milestone delivers a deterministic Python reference implementation of the continuous blended assistance algorithm. It includes replaceable interfaces for external sensors, actuators, and an Agent, plus simulation, recording, replay, and fault-injection adapters.

It does not include production hardware drivers, an independent C++ safety process, hard real-time guarantees, or clinical deployment. Every sample configuration and demo must be marked as synthetic and not for human use.

## 2. Design choices

The implementation uses a typed control core with a dynamic signal registry. This keeps the control interface small while allowing hardware adapters to publish different signal sets. A fully dynamic plugin graph was rejected because configuration errors would be harder to detect. ROS 2 was deferred until a real device integration requires it.

The main interface is pure and deterministic:

```python
ControlEngine.step(ControlInput) -> ControlDecision
```

The engine does not create hardware, network, storage, clock, or Agent dependencies. A runtime module owns those adapters and is the only module that forwards a safety-approved command to an actuator adapter.

## 3. External seams

The external seams are:

- `SensorPort.read(deadline_ns) -> SensorFrame` for timestamped semantic signals;
- `ActuatorPort.apply(SafeCommand, deadline_ns) -> ActuatorReceipt` for seat and wheel outputs;
- `AgentGateway.handle(AgentRequest) -> AgentResponse` for high-level task, stop, health-context, and parameter-suggestion requests;
- `AuditPort.append(AuditEvent)` for non-blocking decision records.

Adapters translate vendor-specific addresses and messages into semantic signal identifiers. The control engine never imports a vendor SDK. Agent requests cannot contain torque, speed, position, brake, or raw actuator setpoints.

## 4. Signal contract

A sensor frame is a mapping rather than a fixed record. Each sample contains:

- semantic signal identifier;
- numeric value and unit;
- monotonic timestamp;
- confidence in `[0, 1]`;
- calibration version;
- source identifier;
- quality flags such as missing, stale, drift, saturation, conflict, and out-of-range.

Missing data remains missing and is never converted to zero. Configuration binds algorithm roles to signal identifiers and declares units, maximum age, minimum confidence, task capabilities, alternative evidence groups, and whether losing a capability requires conservative continuation or stopping.

## 5. State estimation and degradation

Configured evidence sources estimate sit-to-stand intent and continuous phase. Available sources are normalized and fused by configured weights; unavailable or invalid sources are excluded and reduce the estimate confidence. Derived state records its contributing signals.

The runtime modes are:

- `NORMAL`: required evidence is healthy; continuous assistance and withdrawal are allowed;
- `DEGRADED_HOLD`: required control capability remains available but optional or redundant evidence is degraded; the current task may continue conservatively while assistance withdrawal and baseline updates are frozen;
- `CONTROLLED_STOP`: the current task can no longer continue reliably, but powered deceleration or a validated return action remains available;
- `SAFE_HOLD`: active rehabilitation motion is prohibited; wheels are stationary or braking and the seat is held or locked until faults are cleared and checks are repeated.

Agent, camera, heart-rate, temperature, or another configured optional source may disappear without terminating the control cycle. Loss of task-essential seat feedback, wheel feedback, handle intent, interlocks, or actuator health follows the configured controlled-stop or safe-hold policy.

## 6. Continuous assistance

The implementation discretizes the approved adaptive law:

```text
alpha_dot = k_up * max(-m_progress, 0)
            - k_down * max(m_progress, 0) * g_quality
```

`k_up` must be greater than `k_down`. Integration applies elapsed time, assistance bounds, and an independently configured slew-rate limit. Progress shortfall can increase assistance during `DEGRADED_HOLD`; degraded data forces `g_quality` to zero, so the controller cannot withdraw assistance using incomplete evidence.

Quality uses separate participation, hand compensation, posture, fatigue, and data margins. The withdrawal gate is their clipped minimum, preventing a favorable metric from cancelling an unfavorable one.

Seat motor torque is the sum of configured device compensation and intent-scaled phase support. Controlled descent adds damping only when descent speed exceeds the configured safe phase curve. All curves use validated, piecewise-linear configuration rather than embedded clinical values.

## 7. Wheel following

The wheel controller discretizes the approved linear and angular admittance equations. Configured left and right forward handle-force roles produce forward velocity and yaw rate, then differential-drive geometry produces wheel angular velocities. Speed, yaw rate, acceleration, and wheel acceleration are independently limited.

Vertical handle force never produces forward motion. When forward intent returns to zero, damping decays wheel velocity smoothly. Loss of the handle-force capability requests `CONTROLLED_STOP`; loss of wheel feedback or braking capability requests `SAFE_HOLD`.

Seated transport remains a distinct command path and is excluded from rehabilitation adaptation and baseline updates.

## 8. Safety and configuration

The Python safety guard is a testable reference layer, not a substitute for MCU, drive, mechanical, or independent-process safety. It rejects expired commands, invalid modes, non-finite values, prescription violations, interlock failures, emergency stops, and actuator-health faults before an actuator adapter is called.

No clinically meaningful gain, limit, curve, sample period, timeout, or sensor binding has a production default. A complete versioned TOML configuration is required. The repository contains only a synthetic example that cannot be represented as approved for human use.

## 9. Decision trace and Agent access

Every decision includes the runtime mode, candidate and safe commands, contributing and rejected signals, confidence, assistance change, applied clamps, and stable reason codes. This supports deterministic replay and human-readable explanation.

The Agent receives status and decision summaries and may request a high-level task, pause, stop, submit health context, or submit a suggestion for review. It cannot alter a prescription, directly set control values, bypass local intent, or call an actuator adapter.

## 10. Verification

Tests cover:

- configuration validation and absence of production defaults;
- signal age, units, confidence, missing values, and alternative capability groups;
- noncritical data loss continuing in `DEGRADED_HOLD`;
- critical capability loss selecting controlled stop or safe hold;
- assistance integration, bounds, asymmetric rates, and quality gating;
- descent damping and phase curves;
- wheel admittance, steering, decay, and loss of intent feedback;
- safety rejection and actuator single-path execution;
- Agent request authority and forbidden setpoints;
- simulation, fault injection, recording, replay, and end-to-end runtime behavior.

Passing software tests proves deterministic reference behavior only. Hardware-in-the-loop, timing, stability, hazard analysis, and supervised clinical validation remain mandatory before connection to a person.
