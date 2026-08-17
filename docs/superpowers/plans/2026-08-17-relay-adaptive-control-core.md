# RELAY Adaptive Control Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic Python reference control core that implements RELAY continuous seat assistance and wheel following while continuing conservatively when configured noncritical data is missing.

**Architecture:** A pure `ControlEngine.step()` consumes dynamic semantic signals and returns a traced command candidate. A separate `SafetyGuard` is the only path from a candidate to `SafeCommand`; `DeviceRuntime` alone calls replaceable sensor, actuator, and audit adapters. Agent requests terminate at a high-level gateway and have no actuator command surface.

**Tech Stack:** Python 3.12 standard library, `dataclasses`, `enum`, `typing.Protocol`, `tomllib`, `unittest`, TOML, JSON Lines.

---

## File Map

- `pyproject.toml`: package metadata and Python version.
- `.gitignore`: local Python, coverage, and editor artifacts.
- `src/relay_control/model.py`: stable public contracts, enums, signal frames, commands, and traces.
- `src/relay_control/config.py`: required TOML configuration parser and validation.
- `src/relay_control/quality.py`: signal resolution, capability evaluation, and degradation selection.
- `src/relay_control/estimation.py`: configured evidence fusion and quality margins.
- `src/relay_control/controllers.py`: continuous assistance, seat torque, descent damping, and wheel admittance.
- `src/relay_control/engine.py`: the deep deterministic `ControlEngine.step()` interface.
- `src/relay_control/safety.py`: candidate validation and conversion to `SafeCommand`.
- `src/relay_control/ports.py`: external sensor, actuator, audit, and clock interfaces.
- `src/relay_control/adapters.py`: in-memory, sequence, fault-injection, recording, and replay adapters.
- `src/relay_control/runtime.py`: adapter assembly and one-cycle execution.
- `src/relay_control/agent.py`: high-level Agent request validation and authority rules.
- `src/relay_control/demo.py`: synthetic scenario CLI.
- `config/synthetic.toml`: complete non-clinical example configuration.
- `tests/helpers.py`: shared synthetic configuration and frame builders.
- `tests/test_config_quality.py`: configuration, signals, capabilities, and degradation.
- `tests/test_controllers.py`: adaptive seat and wheel control behavior.
- `tests/test_engine_safety.py`: engine decisions, traces, and safety invariants.
- `tests/test_ports_runtime.py`: adapters, recording, replay, and end-to-end runtime.
- `tests/test_agent.py`: Agent authority and forbidden setpoints.
- `README.md`: safety scope, usage, integration contracts, and verification.

## Task 1: Package, Contracts, and Configuration

**Files:**
- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `src/relay_control/__init__.py`
- Create: `src/relay_control/model.py`
- Create: `src/relay_control/config.py`
- Create: `config/synthetic.toml`
- Create: `tests/__init__.py`
- Create: `tests/helpers.py`
- Create: `tests/test_config_quality.py`

- [ ] **Step 1: Write failing contract and configuration tests**

Create tests that require dynamic frames and reject an unsafe or incomplete configuration:

```python
class ConfigContractTests(unittest.TestCase):
    def test_frame_allows_absent_signal(self) -> None:
        frame = SensorFrame(sequence=1, captured_at_ns=10, samples={})
        self.assertIsNone(frame.sample("camera.posture_margin"))

    def test_config_rejects_withdrawal_faster_than_support_increase(self) -> None:
        raw = synthetic_config_dict()
        raw["adaptive"]["k_down"] = raw["adaptive"]["k_up"]
        with self.assertRaisesRegex(ConfigError, "k_up"):
            parse_config(raw)

    def test_config_requires_synthetic_warning(self) -> None:
        raw = synthetic_config_dict()
        raw["metadata"]["human_use"] = True
        with self.assertRaisesRegex(ConfigError, "human use"):
            parse_config(raw)
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_config_quality -v
```

Expected: import failure because `relay_control.model` and `relay_control.config` do not exist.

- [ ] **Step 3: Implement stable contracts and strict configuration**

Implement these public shapes without vendor-specific fields:

```python
class Task(StrEnum):
    IDLE = "IDLE"
    STAND_UP = "STAND_UP"
    SIT_DOWN = "SIT_DOWN"
    GAIT = "GAIT"
    SEATED_TRANSPORT = "SEATED_TRANSPORT"

class RuntimeMode(StrEnum):
    NORMAL = "NORMAL"
    DEGRADED_HOLD = "DEGRADED_HOLD"
    CONTROLLED_STOP = "CONTROLLED_STOP"
    SAFE_HOLD = "SAFE_HOLD"

@dataclass(frozen=True)
class SignalSample:
    signal_id: str
    value: float
    unit: str
    timestamp_ns: int
    confidence: float
    calibration_version: str
    source: str
    flags: frozenset[SignalFlag] = frozenset()

@dataclass(frozen=True)
class SensorFrame:
    sequence: int
    captured_at_ns: int
    samples: Mapping[str, SignalSample]

    def sample(self, signal_id: str) -> SignalSample | None:
        return self.samples.get(signal_id)
```

Define immutable configuration records for signal bindings, capability alternatives, task requirements, evidence sources, margin policies, curves, adaptive gains, seat parameters, wheel parameters, and command limits. `load_config(path)` must call `tomllib.load`, then `parse_config(mapping)`. Validation must require every field used by control, finite numeric values, ordered curves, positive masses/damping/scales, `k_up > k_down > 0`, and `metadata.human_use == false`.

- [ ] **Step 4: Add a complete synthetic TOML configuration**

The file must declare `metadata.configuration_id = "synthetic-demo-v1"`, `metadata.human_use = false`, all semantic bindings, capability alternatives, task requirements, estimator evidence, quality margins, device and support curves, adaptive gains, wheel geometry/admittance, and command limits. It must begin with:

```toml
# SYNTHETIC SOFTWARE DEMO ONLY. NOT VALIDATED OR APPROVED FOR HUMAN USE.
[metadata]
configuration_id = "synthetic-demo-v1"
version = 1
human_use = false
```

- [ ] **Step 5: Run tests, compile the package, and commit**

Run:

```bash
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_config_quality -v
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m compileall -q src tests
```

Expected: tests pass and compilation exits zero.

Commit:

```bash
git add pyproject.toml .gitignore src config tests
git commit -m "feat: define control contracts and configuration"
```

## Task 2: Signal Quality, Capabilities, and Evidence Fusion

**Files:**
- Create: `src/relay_control/quality.py`
- Create: `src/relay_control/estimation.py`
- Modify: `tests/test_config_quality.py`

- [ ] **Step 1: Write failing degradation and fusion tests**

Add these behaviors:

```python
def test_missing_optional_camera_selects_degraded_hold(self) -> None:
    result = evaluate_task_quality(Task.STAND_UP, frame_without("camera.posture_margin"), config, NOW_NS)
    self.assertEqual(result.mode, RuntimeMode.DEGRADED_HOLD)
    self.assertIn("optional_signal_unavailable:posture", result.reasons)

def test_alternative_phase_evidence_keeps_capability_available(self) -> None:
    frame = valid_frame().without("imu.trunk.pitch")
    result = evaluate_task_quality(Task.STAND_UP, frame, config, NOW_NS)
    self.assertNotEqual(result.mode, RuntimeMode.SAFE_HOLD)

def test_missing_seat_feedback_selects_safe_hold(self) -> None:
    frame = valid_frame().without("seat.angle")
    result = evaluate_task_quality(Task.STAND_UP, frame, config, NOW_NS)
    self.assertEqual(result.mode, RuntimeMode.SAFE_HOLD)

def test_fusion_renormalizes_available_sources(self) -> None:
    estimate = fuse_evidence(config.phase_evidence, resolved_without_camera)
    self.assertGreaterEqual(estimate.value, 0.0)
    self.assertLessEqual(estimate.value, 1.0)
    self.assertNotIn("camera.posture_margin", estimate.contributors)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run the four new tests. Expected: import failure because quality and estimation modules do not exist.

- [ ] **Step 3: Implement signal resolution and capability evaluation**

Implement an immutable `ResolvedSignal(role, sample, usable, reason)` record. Add
`resolve_signals(frame, config, now_ns) -> Mapping[str, ResolvedSignal]` and
`evaluate_task_quality(task, frame, config, now_ns) -> QualityDecision` as the
only public functions in this module.

Resolution must reject wrong units, stale timestamps, low confidence, non-finite values, and nonempty hard quality flags. A capability is available when any declared alternative group has all usable roles. Missing required capability selects its configured loss mode. Missing optional capability selects `DEGRADED_HOLD`. Severity order is `NORMAL < DEGRADED_HOLD < CONTROLLED_STOP < SAFE_HOLD`.

- [ ] **Step 4: Implement configured evidence and margin calculation**

Implement weighted evidence fusion by excluding unusable inputs and renormalizing remaining weights. Return value, confidence, and contributors. Implement generic margin policies:

```python
margin = (value - boundary) / sigma       # direction = ABOVE
margin = (boundary - value) / sigma       # direction = BELOW
quality_gate = clip(min(all_available_margins, data_margin), 0.0, 1.0)
```

When quality mode is not `NORMAL`, `data_margin` and `quality_gate` must be zero. Missing optional margins must never be interpreted as favorable evidence.

- [ ] **Step 5: Run tests and commit**

Run the focused tests and the whole current suite. Expected: all pass.

Commit:

```bash
git add src/relay_control/quality.py src/relay_control/estimation.py tests/test_config_quality.py
git commit -m "feat: fuse partial sensor evidence safely"
```

## Task 3: Continuous Assistance and Task Controllers

**Files:**
- Create: `src/relay_control/controllers.py`
- Create: `tests/test_controllers.py`

- [ ] **Step 1: Write failing adaptive assistance tests**

```python
def test_progress_shortfall_increases_assistance(self) -> None:
    next_alpha = update_assistance(alpha=0.4, progress_margin=-0.5, quality_gate=1.0, dt_s=0.1, cfg=ADAPTIVE)
    self.assertGreater(next_alpha, 0.4)

def test_degraded_data_freezes_withdrawal(self) -> None:
    next_alpha = update_assistance(alpha=0.4, progress_margin=0.5, quality_gate=0.0, dt_s=0.1, cfg=ADAPTIVE)
    self.assertEqual(next_alpha, 0.4)

def test_assistance_is_bounded_for_random_finite_inputs(self) -> None:
    rng = random.Random(7)
    for _ in range(1_000):
        result = update_assistance(rng.random(), rng.uniform(-10, 10), rng.random(), rng.uniform(0, 0.2), ADAPTIVE)
        self.assertGreaterEqual(result, ADAPTIVE.alpha_min)
        self.assertLessEqual(result, ADAPTIVE.alpha_max)
```

- [ ] **Step 2: Run and verify RED**

Run `tests.test_controllers`. Expected: import failure because controllers do not exist.

- [ ] **Step 3: Implement curves, adaptive law, and seat control**

Implement clamped piecewise-linear interpolation and:

```python
def update_assistance(alpha, progress_margin, quality_gate, dt_s, cfg):
    raw_rate = cfg.k_up * max(-progress_margin, 0.0)
    raw_rate -= cfg.k_down * max(progress_margin, 0.0) * quality_gate
    rate = clamp(raw_rate, -cfg.max_slew_per_s, cfg.max_slew_per_s)
    return clamp(alpha + rate * dt_s, cfg.alpha_min, cfg.alpha_max)
```

`SeatController.step()` must calculate device compensation by seat angle, phase support by continuous phase, multiply human support by intent confidence and `alpha`, add descent damping only for excessive descent speed, and return separate device-compensation and rehabilitation-assistance trace fields.

- [ ] **Step 4: Write failing wheel tests, then implement admittance**

Tests must prove equal forward forces produce forward wheel speed, right-minus-left force produces yaw, zero force decays speed, vertical force does not propel, and output respects speed and slew limits.

Implement:

```python
v_dot = (left_forward + right_forward - cfg.linear_damping * state.linear_velocity) / cfg.virtual_mass
yaw_dot = (right_forward - left_forward - cfg.yaw_damping * state.yaw_rate) / cfg.virtual_inertia
right = (v + 0.5 * cfg.track_width_m * yaw) / cfg.wheel_radius_m
left = (v - 0.5 * cfg.track_width_m * yaw) / cfg.wheel_radius_m
```

Use a configured stop deceleration during `CONTROLLED_STOP`; return zero wheel speeds and a brake request during `SAFE_HOLD`.

- [ ] **Step 5: Run tests and commit**

Run controller tests and all current tests. Expected: all pass.

Commit:

```bash
git add src/relay_control/controllers.py tests/test_controllers.py
git commit -m "feat: implement seat assistance and wheel admittance"
```

## Task 4: Control Engine and Safety Guard

**Files:**
- Create: `src/relay_control/engine.py`
- Create: `src/relay_control/safety.py`
- Create: `tests/test_engine_safety.py`

- [ ] **Step 1: Write failing engine behavior tests**

```python
def test_optional_loss_still_produces_executable_seat_candidate(self) -> None:
    decision = engine.step(stand_input(frame_without_camera()))
    self.assertEqual(decision.mode, RuntimeMode.DEGRADED_HOLD)
    self.assertEqual(decision.candidate.seat_mode, SeatCommandMode.TORQUE)
    self.assertGreaterEqual(decision.assistance_alpha, INITIAL_ALPHA)

def test_missing_handle_intent_produces_controlled_stop(self) -> None:
    decision = engine.step(gait_input(frame_without_handles()))
    self.assertEqual(decision.mode, RuntimeMode.CONTROLLED_STOP)

def test_transport_does_not_update_rehabilitation_assistance(self) -> None:
    before = engine.state.assistance_alpha
    engine.step(transport_input())
    self.assertEqual(engine.state.assistance_alpha, before)
```

- [ ] **Step 2: Run and verify RED**

Expected: import failure because engine and safety modules do not exist.

- [ ] **Step 3: Implement the deterministic engine pipeline**

`ControlEngine.step(ControlInput)` must execute resolution, task quality, evidence fusion, progress and quality margins, task controller selection, state update, and trace generation in that order. It must return `ControlDecision` without I/O. It must reject nonpositive or excessive `dt_s`, nonmonotonic time, and invalid task transitions by returning a safe-hold candidate with a reason rather than raising from the control loop.

- [ ] **Step 4: Write failing safety tests, then implement the guard**

Tests must require:

```python
def test_emergency_stop_overrides_finite_candidate(self) -> None:
    safe = guard.review(candidate, safety_context(emergency_stop=True), NOW_NS)
    self.assertTrue(safe.brake_request)
    self.assertEqual(safe.left_wheel_rad_s, 0.0)
    self.assertEqual(safe.right_wheel_rad_s, 0.0)

def test_nonfinite_candidate_is_rejected(self) -> None:
    unsafe = replace(candidate, seat_torque_nm=float("nan"))
    result = guard.review(unsafe, healthy_safety_context(), NOW_NS)
    self.assertEqual(result.seat_mode, SeatCommandMode.HOLD)
```

The guard must validate expiry, sequence monotonicity, finite values, interlocks, stop input, actuator health, prescription task permission, and configured torque/wheel limits. It returns a distinct immutable `SafeCommand`; actuator interfaces must not accept `ControlCandidate`.

- [ ] **Step 5: Run tests and commit**

Run engine and safety tests, then the complete suite. Expected: all pass.

Commit:

```bash
git add src/relay_control/engine.py src/relay_control/safety.py tests/test_engine_safety.py
git commit -m "feat: add deterministic engine and safety guard"
```

## Task 5: External Ports, Adapters, Runtime, and Replay

**Files:**
- Create: `src/relay_control/ports.py`
- Create: `src/relay_control/adapters.py`
- Create: `src/relay_control/runtime.py`
- Create: `tests/test_ports_runtime.py`

- [ ] **Step 1: Write failing adapter and runtime tests**

```python
def test_runtime_calls_actuator_only_with_safe_command(self) -> None:
    runtime.cycle(Task.STAND_UP, operator_input(), safety_context(), NOW_NS, 0.02)
    self.assertEqual(len(actuator.receipts), 1)
    self.assertIsInstance(actuator.commands[0], SafeCommand)

def test_fault_adapter_can_remove_optional_signal_without_stopping_cycle(self) -> None:
    result = runtime_with_missing("camera.posture_margin").cycle(
        Task.STAND_UP,
        operator_input(),
        safety_context(),
        NOW_NS,
        0.02,
    )
    self.assertEqual(result.decision.mode, RuntimeMode.DEGRADED_HOLD)
    self.assertTrue(result.receipt.accepted)

def test_recorded_frames_replay_deterministically(self) -> None:
    first = run_and_record(sequence)
    second = run_replay(recording_path)
    self.assertEqual(first.decision_traces, second.decision_traces)
```

- [ ] **Step 2: Run and verify RED**

Expected: import failure because ports, adapters, and runtime do not exist.

- [ ] **Step 3: Define the external interfaces and in-memory adapters**

```python
class SensorPort(Protocol):
    def read(self, deadline_ns: int) -> SensorFrame:
        raise NotImplementedError

class ActuatorPort(Protocol):
    def apply(self, command: SafeCommand, deadline_ns: int) -> ActuatorReceipt:
        raise NotImplementedError

class AuditPort(Protocol):
    def append(self, event: AuditEvent) -> None:
        raise NotImplementedError
```

Implement `InMemorySensorAdapter`, `SequenceSensorAdapter`, `FaultInjectingSensorAdapter`, `InMemoryActuatorAdapter`, and `ListAuditAdapter`. The fault adapter must support remove, stale, confidence, flag, and value overrides without changing engine code.

- [ ] **Step 4: Implement recording, replay, and one-cycle runtime**

Use JSON Lines with explicit schema version. `DeviceRuntime.cycle()` reads one frame, calls the engine, calls the safety guard, sends only the resulting `SafeCommand` to the actuator, appends a bounded audit event, and returns a `RuntimeCycleResult`. Exceptions from sensor or actuator adapters become structured safe-hold or actuator-fault results; they must not escape the runtime loop.

- [ ] **Step 5: Run tests and commit**

Run runtime tests and the complete suite. Expected: all pass.

Commit:

```bash
git add src/relay_control/ports.py src/relay_control/adapters.py src/relay_control/runtime.py tests/test_ports_runtime.py
git commit -m "feat: add replaceable runtime adapters and replay"
```

## Task 6: Agent Gateway and Authority

**Files:**
- Create: `src/relay_control/agent.py`
- Create: `tests/test_agent.py`

- [ ] **Step 1: Write failing Agent authority tests**

```python
def test_agent_cannot_submit_actuator_setpoint(self) -> None:
    response = gateway.handle(request("REQUEST_TASK", {"task": "GAIT", "wheel_speed": 1.0}))
    self.assertEqual(response.status, AgentResponseStatus.REJECTED)
    self.assertIn("forbidden_control_field", response.reasons)

def test_agent_task_request_remains_pending_local_confirmation(self) -> None:
    response = gateway.handle(request("REQUEST_TASK", {"task": "GAIT"}))
    self.assertEqual(response.status, AgentResponseStatus.PENDING_LOCAL_CONFIRMATION)

def test_agent_stop_is_high_priority_but_still_has_no_actuator_access(self) -> None:
    response = gateway.handle(request("STOP_TASK", {}))
    self.assertEqual(response.intent.kind, AgentIntentKind.STOP)
    self.assertFalse(hasattr(gateway, "actuator"))
```

- [ ] **Step 2: Run and verify RED**

Expected: import failure because the Agent module does not exist.

- [ ] **Step 3: Implement the high-level gateway**

Validate protocol major version, request ID, agent ID, issued/expiry times, request type, payload size, and idempotency. Support only `QUERY_STATUS`, `REQUEST_TASK`, `PAUSE_TASK`, `STOP_TASK`, `SUBMIT_HEALTH_CONTEXT`, and `SUBMIT_PARAMETER_SUGGESTION`. Recursively reject keys containing torque, speed, velocity, position, brake, current, PWM, or actuator setpoint semantics. Return an immutable high-level `AgentIntent`; do not import ports or adapters.

- [ ] **Step 4: Run tests and commit**

Run Agent tests and the complete suite. Expected: all pass.

Commit:

```bash
git add src/relay_control/agent.py tests/test_agent.py
git commit -m "feat: add constrained Agent gateway"
```

## Task 7: Demo, Documentation, and Final Verification

**Files:**
- Create: `src/relay_control/demo.py`
- Create: `README.md`
- Modify: `src/relay_control/__init__.py`
- Modify: `tests/test_ports_runtime.py`

- [ ] **Step 1: Add a failing synthetic scenario test**

Require one scenario to run normal standing, remove camera and heart-rate signals while continuing in degraded hold, run gait force input, remove handle intent to reach controlled stop, and finally assert safe hold after an emergency stop. The test must verify every executed actuator value is finite and bounded.

- [ ] **Step 2: Run the scenario test and verify RED**

Expected: failure because the demo scenario entry point does not exist.

- [ ] **Step 3: Implement the demo and README**

The demo command:

```bash
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m relay_control.demo --config config/synthetic.toml
```

must print JSON summaries for `NORMAL`, `DEGRADED_HOLD`, `CONTROLLED_STOP`, and `SAFE_HOLD`, then exit zero. The README must lead with the non-clinical warning, explain module interfaces, list supported degradation behavior, show the demo and test commands, and give concrete adapter integration examples without claiming real-time or medical readiness.

- [ ] **Step 4: Run fresh full verification**

Run:

```bash
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -v
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m compileall -q src tests
PYTHONPATH=src /Users/bofanliu/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m relay_control.demo --config config/synthetic.toml
git diff --check
```

Expected: zero test failures, compilation exit zero, all four runtime modes in demo output, and no whitespace errors.

- [ ] **Step 5: Audit requirements and commit**

Check every requirement in the approved design against a test or README section. Confirm that no module except `DeviceRuntime` calls `ActuatorPort.apply`, no Agent module imports an actuator type, missing values are never converted to zero, and no sample configuration permits human use.

Commit:

```bash
git add README.md src/relay_control/__init__.py src/relay_control/demo.py tests/test_ports_runtime.py
git commit -m "docs: add synthetic demo and integration guide"
```

- [ ] **Step 6: Push the verified main branch**

Run:

```bash
git status --short --branch
git log --oneline --decorate -10
git push -u origin main
git ls-remote --heads origin main
```

Expected: clean tracked worktree, successful push, and remote `refs/heads/main` equal to local `HEAD`.
