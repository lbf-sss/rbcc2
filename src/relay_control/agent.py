from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
import json
from types import MappingProxyType
from typing import Any, Callable, Mapping

from relay_control.model import Task


class AgentRequestType(StrEnum):
    QUERY_STATUS = "QUERY_STATUS"
    REQUEST_TASK = "REQUEST_TASK"
    PAUSE_TASK = "PAUSE_TASK"
    STOP_TASK = "STOP_TASK"
    SUBMIT_HEALTH_CONTEXT = "SUBMIT_HEALTH_CONTEXT"
    SUBMIT_PARAMETER_SUGGESTION = "SUBMIT_PARAMETER_SUGGESTION"


class AgentResponseStatus(StrEnum):
    ACCEPTED = "ACCEPTED"
    PENDING_LOCAL_CONFIRMATION = "PENDING_LOCAL_CONFIRMATION"
    PENDING_THERAPIST_REVIEW = "PENDING_THERAPIST_REVIEW"
    COMPLETED = "COMPLETED"
    REJECTED = "REJECTED"


class AgentIntentKind(StrEnum):
    REQUEST_TASK = "REQUEST_TASK"
    PAUSE = "PAUSE"
    STOP = "STOP"
    HEALTH_CONTEXT = "HEALTH_CONTEXT"
    PARAMETER_SUGGESTION = "PARAMETER_SUGGESTION"


@dataclass(frozen=True)
class AgentIntent:
    request_id: str
    kind: AgentIntentKind
    task: Task | None
    payload: Mapping[str, Any]


@dataclass(frozen=True)
class AgentResponse:
    request_id: str
    status: AgentResponseStatus
    reasons: tuple[str, ...] = ()
    intent: AgentIntent | None = None
    payload: Mapping[str, Any] = field(
        default_factory=lambda: MappingProxyType({})
    )


class AgentGateway:
    _REQUIRED_FIELDS = frozenset(
        {
            "protocol_version",
            "request_id",
            "agent_id",
            "issued_at_ns",
            "expires_at_ns",
            "request_type",
            "payload",
        }
    )
    _FORBIDDEN_KEY_PARTS = (
        "torque",
        "speed",
        "velocity",
        "position",
        "brake",
        "current",
        "pwm",
        "actuator",
        "setpoint",
    )

    def __init__(
        self,
        *,
        now_ns: Callable[[], int],
        status_provider: Callable[[], Mapping[str, Any]],
        allowed_agents: frozenset[str],
        max_payload_bytes: int,
    ) -> None:
        if max_payload_bytes <= 0:
            raise ValueError("max_payload_bytes must be positive")
        self._now_ns = now_ns
        self._status_provider = status_provider
        self._allowed_agents = allowed_agents
        self._max_payload_bytes = max_payload_bytes
        self._responses: dict[str, tuple[str, AgentResponse]] = {}

    def handle(self, request: Mapping[str, Any]) -> AgentResponse:
        request_id = _request_id(request)
        fingerprint = _fingerprint(request)
        cached = self._responses.get(request_id)
        if cached is not None:
            previous_fingerprint, response = cached
            if fingerprint == previous_fingerprint:
                return response
            return _rejected(request_id, "request_id_conflict")

        reasons = self._validation_reasons(request)
        if reasons:
            response = AgentResponse(
                request_id=request_id,
                status=AgentResponseStatus.REJECTED,
                reasons=tuple(reasons),
            )
        else:
            response = self._dispatch(request)
        if request_id and response.status is not AgentResponseStatus.REJECTED:
            self._responses[request_id] = (fingerprint, response)
        return response

    def _validation_reasons(
        self, request: Mapping[str, Any]
    ) -> list[str]:
        if not isinstance(request, Mapping):
            return ["request_must_be_object"]
        missing = self._REQUIRED_FIELDS.difference(request)
        unknown = set(request).difference(self._REQUIRED_FIELDS)
        reasons = [f"missing_field:{field}" for field in sorted(missing)]
        reasons.extend(f"unknown_field:{field}" for field in sorted(unknown))
        if reasons:
            return reasons

        version = request["protocol_version"]
        if not _supported_version(version):
            reasons.append("unsupported_protocol_version")
        if not isinstance(request["request_id"], str) or not request["request_id"]:
            reasons.append("invalid_request_id")
        agent_id = request["agent_id"]
        if not isinstance(agent_id, str) or agent_id not in self._allowed_agents:
            reasons.append("agent_not_allowed")

        issued = request["issued_at_ns"]
        expires = request["expires_at_ns"]
        if not _integer_timestamp(issued) or not _integer_timestamp(expires):
            reasons.append("invalid_request_time")
        else:
            now = self._now_ns()
            if issued > now:
                reasons.append("request_from_future")
            if expires < now:
                reasons.append("request_expired")
            if expires < issued:
                reasons.append("request_time_range_invalid")

        try:
            AgentRequestType(request["request_type"])
        except (TypeError, ValueError):
            reasons.append("unsupported_request_type")

        payload = request["payload"]
        if not isinstance(payload, Mapping):
            reasons.append("payload_must_be_object")
        else:
            try:
                encoded = json.dumps(
                    payload, sort_keys=True, separators=(",", ":")
                ).encode("utf-8")
            except (TypeError, ValueError):
                reasons.append("payload_not_json_serializable")
            else:
                if len(encoded) > self._max_payload_bytes:
                    reasons.append("payload_too_large")
            forbidden = _find_forbidden_key(
                payload, self._FORBIDDEN_KEY_PARTS
            )
            if forbidden is not None:
                reasons.append(f"forbidden_control_field:{forbidden}")
        return reasons

    def _dispatch(self, request: Mapping[str, Any]) -> AgentResponse:
        request_id = request["request_id"]
        request_type = AgentRequestType(request["request_type"])
        payload = MappingProxyType(dict(request["payload"]))

        if request_type is AgentRequestType.QUERY_STATUS:
            return AgentResponse(
                request_id=request_id,
                status=AgentResponseStatus.COMPLETED,
                payload=MappingProxyType(dict(self._status_provider())),
            )
        if request_type is AgentRequestType.REQUEST_TASK:
            try:
                task = Task(payload.get("task"))
            except (TypeError, ValueError):
                return _rejected(request_id, "invalid_task")
            if task is Task.IDLE:
                return _rejected(request_id, "invalid_task")
            intent = AgentIntent(
                request_id,
                AgentIntentKind.REQUEST_TASK,
                task,
                payload,
            )
            return AgentResponse(
                request_id,
                AgentResponseStatus.PENDING_LOCAL_CONFIRMATION,
                intent=intent,
            )
        if request_type is AgentRequestType.PAUSE_TASK:
            intent = AgentIntent(
                request_id, AgentIntentKind.PAUSE, None, payload
            )
            return AgentResponse(
                request_id, AgentResponseStatus.ACCEPTED, intent=intent
            )
        if request_type is AgentRequestType.STOP_TASK:
            intent = AgentIntent(
                request_id, AgentIntentKind.STOP, None, payload
            )
            return AgentResponse(
                request_id, AgentResponseStatus.ACCEPTED, intent=intent
            )
        if request_type is AgentRequestType.SUBMIT_HEALTH_CONTEXT:
            intent = AgentIntent(
                request_id,
                AgentIntentKind.HEALTH_CONTEXT,
                None,
                payload,
            )
            return AgentResponse(
                request_id, AgentResponseStatus.ACCEPTED, intent=intent
            )
        intent = AgentIntent(
            request_id,
            AgentIntentKind.PARAMETER_SUGGESTION,
            None,
            payload,
        )
        return AgentResponse(
            request_id,
            AgentResponseStatus.PENDING_THERAPIST_REVIEW,
            intent=intent,
        )


def _find_forbidden_key(
    value: Any, forbidden_parts: tuple[str, ...]
) -> str | None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            normalized = str(key).lower()
            if any(part in normalized for part in forbidden_parts):
                return str(key)
            nested = _find_forbidden_key(child, forbidden_parts)
            if nested is not None:
                return nested
    elif isinstance(value, (list, tuple)):
        for child in value:
            nested = _find_forbidden_key(child, forbidden_parts)
            if nested is not None:
                return nested
    return None


def _supported_version(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    parts = value.split(".")
    return len(parts) == 2 and parts[0] == "1" and parts[1].isdigit()


def _integer_timestamp(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _request_id(request: Any) -> str:
    if not isinstance(request, Mapping):
        return ""
    value = request.get("request_id")
    return value if isinstance(value, str) else ""


def _fingerprint(request: Any) -> str:
    try:
        return json.dumps(request, sort_keys=True, separators=(",", ":"))
    except (TypeError, ValueError):
        return repr(request)


def _rejected(request_id: str, reason: str) -> AgentResponse:
    return AgentResponse(
        request_id=request_id,
        status=AgentResponseStatus.REJECTED,
        reasons=(reason,),
    )
