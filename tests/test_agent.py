from __future__ import annotations

import unittest

from relay_control.agent import (
    AgentGateway,
    AgentIntentKind,
    AgentResponseStatus,
)
from tests.helpers import NOW_NS


def request(
    request_type: str,
    payload: dict,
    *,
    request_id: str = "req-1",
    agent_id: str = "trusted-agent",
) -> dict:
    return {
        "protocol_version": "1.0",
        "request_id": request_id,
        "agent_id": agent_id,
        "issued_at_ns": NOW_NS - 1,
        "expires_at_ns": NOW_NS + 1_000_000,
        "request_type": request_type,
        "payload": payload,
    }


class AgentGatewayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gateway = AgentGateway(
            now_ns=lambda: NOW_NS,
            status_provider=lambda: {"lifecycle": "READY"},
            allowed_agents=frozenset({"trusted-agent"}),
            max_payload_bytes=4_096,
        )

    def test_agent_cannot_submit_actuator_setpoint(self) -> None:
        response = self.gateway.handle(
            request("REQUEST_TASK", {"task": "GAIT", "wheel_speed": 1.0})
        )

        self.assertEqual(response.status, AgentResponseStatus.REJECTED)
        self.assertIn("forbidden_control_field:wheel_speed", response.reasons)
        self.assertIsNone(response.intent)

    def test_nested_control_setpoint_is_also_rejected(self) -> None:
        response = self.gateway.handle(
            request(
                "SUBMIT_HEALTH_CONTEXT",
                {"fatigue": "mild", "nested": {"seat_torque": 5.0}},
            )
        )

        self.assertEqual(response.status, AgentResponseStatus.REJECTED)
        self.assertIn("forbidden_control_field:seat_torque", response.reasons)

    def test_agent_task_request_remains_pending_local_confirmation(self) -> None:
        response = self.gateway.handle(
            request("REQUEST_TASK", {"task": "GAIT"})
        )

        self.assertEqual(
            response.status, AgentResponseStatus.PENDING_LOCAL_CONFIRMATION
        )
        self.assertEqual(response.intent.kind, AgentIntentKind.REQUEST_TASK)
        self.assertEqual(response.intent.task.value, "GAIT")

    def test_agent_stop_has_no_actuator_access(self) -> None:
        response = self.gateway.handle(request("STOP_TASK", {}))

        self.assertEqual(response.status, AgentResponseStatus.ACCEPTED)
        self.assertEqual(response.intent.kind, AgentIntentKind.STOP)
        self.assertFalse(hasattr(self.gateway, "actuator"))

    def test_untrusted_or_expired_request_is_rejected(self) -> None:
        untrusted = self.gateway.handle(
            request("QUERY_STATUS", {}, agent_id="other-agent")
        )
        expired_request = request("QUERY_STATUS", {}, request_id="req-2")
        expired_request["expires_at_ns"] = NOW_NS - 1
        expired = self.gateway.handle(expired_request)

        self.assertIn("agent_not_allowed", untrusted.reasons)
        self.assertIn("request_expired", expired.reasons)

    def test_rejected_request_cannot_poison_idempotency_cache(self) -> None:
        rejected = self.gateway.handle(
            request("QUERY_STATUS", {}, agent_id="other-agent")
        )

        accepted = self.gateway.handle(request("QUERY_STATUS", {}))

        self.assertEqual(rejected.status, AgentResponseStatus.REJECTED)
        self.assertEqual(accepted.status, AgentResponseStatus.COMPLETED)

    def test_duplicate_request_is_idempotent(self) -> None:
        original = request("PAUSE_TASK", {})

        first = self.gateway.handle(original)
        second = self.gateway.handle(dict(original))

        self.assertEqual(first, second)

    def test_reused_request_id_with_different_body_is_rejected(self) -> None:
        self.gateway.handle(request("PAUSE_TASK", {}))

        response = self.gateway.handle(
            request("STOP_TASK", {}, request_id="req-1")
        )

        self.assertEqual(response.status, AgentResponseStatus.REJECTED)
        self.assertIn("request_id_conflict", response.reasons)

    def test_query_returns_status_without_creating_motion_intent(self) -> None:
        response = self.gateway.handle(request("QUERY_STATUS", {}))

        self.assertEqual(response.status, AgentResponseStatus.COMPLETED)
        self.assertEqual(response.payload["lifecycle"], "READY")
        self.assertIsNone(response.intent)


if __name__ == "__main__":
    unittest.main()
