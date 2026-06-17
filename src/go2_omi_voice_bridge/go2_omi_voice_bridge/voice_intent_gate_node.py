from __future__ import annotations

from dataclasses import dataclass
import json
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from go2_omi_voice_bridge.intent import (
    INTENT_CANCEL,
    INTENT_CONTINUE_TOUR,
    INTENT_FUN_FACT,
    INTENT_NAVIGATE_TO_PLACE,
    INTENT_OBSERVE,
    INTENT_SKIP_CHECKPOINT,
    INTENT_START_TOUR,
    INTENT_STOP,
    INTENT_UNKNOWN,
    INTENT_WHERE_AM_I,
    ParsedIntent,
    decode_json_or_text,
    is_confirmation,
    is_rejection,
    parse_intent,
    speech_payload,
    transcript_confidence,
    transcript_text,
    normalize_text,
)


@dataclass
class PendingCommand:
    intent: ParsedIntent
    created_at: float


class VoiceIntentGateNode(Node):
    """Rule-based safety gate between transcripts and /go2_agent/user_command."""

    def __init__(self) -> None:
        super().__init__("go2_voice_intent_gate")
        self.declare_parameter("transcript_confidence_threshold", 0.70)
        self.declare_parameter("confirmation_timeout_sec", 15.0)
        self.declare_parameter("wake_words", ["sparky", "go2", "robot"])
        self.declare_parameter("require_confirmation_for_motion", True)
        self.declare_parameter("duplicate_suppression_sec", 5.0)
        self.declare_parameter("publish_to_agent_topic", "/go2_agent/user_command")
        self.declare_parameter("tts_topic", "/go2_tts/say")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_out")
        self.declare_parameter("nav_command_topic", "/go2_nav/command")

        self.agent_pub = self.create_publisher(String, str(self.get_parameter("publish_to_agent_topic").value), 10)
        self.verify_pub = self.create_publisher(String, "/go2_voice/verification_request", 10)
        self.state_pub = self.create_publisher(String, "/go2_voice/verification_state", 10)
        self.tts_pub = self.create_publisher(String, str(self.get_parameter("tts_topic").value), 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.nav_pub = self.create_publisher(String, str(self.get_parameter("nav_command_topic").value), 10)
        self.tour_start_pub = self.create_publisher(String, "/go2_tour/start", 10)
        self.tour_continue_pub = self.create_publisher(String, "/go2_tour/continue", 10)
        self.tour_skip_pub = self.create_publisher(String, "/go2_tour/skip", 10)
        self.tour_cancel_pub = self.create_publisher(String, "/go2_tour/cancel", 10)

        self.create_subscription(String, "/go2_voice/transcript", self._on_transcript, 10)
        self.create_subscription(String, "/go2_nav/status", self._on_nav_status, 10)
        self.create_subscription(String, "/collision_monitor_state", self._on_collision_state, 10)
        self.pending: PendingCommand | None = None
        self.latest_nav_status: Any = None
        self.latest_collision_state = ""
        self.last_transcript_text = ""
        self.last_transcript_time = 0.0
        self.timer = self.create_timer(1.0, self._check_timeout)
        self.get_logger().info("Voice intent gate ready; motion and tour commands require confirmation.")

    def _wake_words(self) -> list[str]:
        value = self.get_parameter("wake_words").value
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        return [part.strip() for part in str(value).split(",") if part.strip()]

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _publish_state(self, payload: dict[str, Any]) -> None:
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _say(self, text: str, category: str = "status", priority: str = "normal", interrupt: bool = False) -> None:
        self.tts_pub.publish(String(data=speech_payload(text, category, priority, interrupt)))

    def _on_nav_status(self, msg: String) -> None:
        try:
            self.latest_nav_status = json.loads(msg.data)
        except Exception:
            self.latest_nav_status = msg.data

    def _on_collision_state(self, msg: String) -> None:
        self.latest_collision_state = msg.data.strip()

    def _on_transcript(self, msg: String) -> None:
        payload = decode_json_or_text(msg.data)
        text = transcript_text(payload)
        confidence = transcript_confidence(payload)
        if not text:
            return
        now = time.time()
        if (
            text == self.last_transcript_text
            and now - self.last_transcript_time < float(self.get_parameter("duplicate_suppression_sec").value)
        ):
            return
        self.last_transcript_text = text
        self.last_transcript_time = now
        if self.pending:
            if normalize_text(text, self._wake_words()) == self.pending.intent.normalized_text:
                return
            if is_confirmation(text):
                self._approve_pending(text)
                return
            if is_rejection(text):
                self._reject_pending(text)
                return
            self._publish_state({"state": "waiting_for_confirmation", "ignored_text": text})
            return
        if is_confirmation(text) or is_rejection(text):
            self._publish_state({"state": "no_pending_confirmation_ignored", "text": text})
            return

        threshold = float(self.get_parameter("transcript_confidence_threshold").value)
        if confidence < threshold:
            self._say("I am not sure I heard that correctly. Could you repeat?", "warning")
            self._publish_state({"state": "low_confidence", "text": text, "confidence": confidence})
            return

        intent = parse_intent(text, confidence, self._wake_words())
        if intent.intent in {INTENT_STOP, INTENT_CANCEL}:
            self._handle_immediate_stop(intent)
            return
        if intent.intent == INTENT_UNKNOWN:
            self._say("I do not know how to do that yet. You can ask where we are, request a fun fact, or ask for a saved tour.", "warning")
            self._publish_state({"state": "unknown_intent", "text": text})
            return
        if self._requires_confirmation(intent):
            self.pending = PendingCommand(intent=intent, created_at=time.time())
            prompt = self._confirmation_prompt(intent)
            self.verify_pub.publish(String(data=json.dumps({"prompt": prompt, "intent": intent.intent}, sort_keys=True)))
            self._say(prompt, "confirmation", "high", interrupt=True)
            self._publish_state({"state": "waiting_for_confirmation", "intent": intent.intent, "text": intent.text})
            return
        self._publish_approved(intent, verified=True)

    def _requires_confirmation(self, intent: ParsedIntent) -> bool:
        if not self._param_bool("require_confirmation_for_motion"):
            return False
        return intent.requires_confirmation or intent.requires_motion or intent.intent in {
            INTENT_NAVIGATE_TO_PLACE,
            INTENT_START_TOUR,
            INTENT_CONTINUE_TOUR,
            INTENT_SKIP_CHECKPOINT,
        }

    def _confirmation_prompt(self, intent: ParsedIntent) -> str:
        if intent.intent == INTENT_START_TOUR:
            return "I heard: start the Applied Data Science tour. This may navigate through saved checkpoints. Should I proceed?"
        if intent.intent == INTENT_CONTINUE_TOUR:
            return "I heard: continue to the next checkpoint. Should I proceed?"
        if intent.intent == INTENT_SKIP_CHECKPOINT:
            return "I heard: skip this checkpoint. Should I proceed?"
        if intent.intent == INTENT_NAVIGATE_TO_PLACE:
            target = intent.semantic_target or "that place"
            return f"I heard: navigate to {target}. Should I proceed?"
        return f"I heard: {intent.text}. Should I proceed?"

    def _handle_immediate_stop(self, intent: ParsedIntent) -> None:
        self.pending = None
        self.cmd_pub.publish(Twist())
        command = {"action": "stop_robot", "reason": "voice_stop", "source": "omi_voice", "text": intent.text}
        self.nav_pub.publish(String(data=json.dumps(command, sort_keys=True)))
        self.tour_cancel_pub.publish(String(data=json.dumps(command, sort_keys=True)))
        self.agent_pub.publish(String(data=json.dumps(intent.to_agent_payload(verified=True), sort_keys=True)))
        self._say("Stopping now.", "warning", "high", interrupt=True)
        self._publish_state({"state": "stopped", "intent": intent.intent, "text": intent.text})

    def _approve_pending(self, confirmation_text: str) -> None:
        pending = self.pending
        self.pending = None
        if pending is None:
            return
        self._publish_approved(pending.intent, verified=True, confirmation_text=confirmation_text)

    def _reject_pending(self, rejection_text: str) -> None:
        pending = self.pending
        self.pending = None
        self._say("Canceled. I will stay here.", "status")
        self._publish_state(
            {
                "state": "rejected",
                "intent": pending.intent.intent if pending else None,
                "text": rejection_text,
            }
        )

    def _publish_approved(self, intent: ParsedIntent, verified: bool, confirmation_text: str = "") -> None:
        payload = intent.to_agent_payload(verified=verified)
        if confirmation_text:
            payload["confirmation_text"] = confirmation_text
        if self.latest_nav_status is not None:
            payload["latest_nav_status"] = self.latest_nav_status
        if self.latest_collision_state:
            payload["collision_monitor_state"] = self.latest_collision_state
        self.agent_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))
        self._route_tour_intent(intent, payload)
        if intent.intent in {INTENT_WHERE_AM_I, INTENT_OBSERVE}:
            self._say("I will observe and answer without moving.", "status")
        elif intent.intent == INTENT_FUN_FACT:
            self._say("I will look up a safe fun fact for this place.", "status")
        self._publish_state({"state": "approved", "intent": intent.intent, "verified": verified})

    def _route_tour_intent(self, intent: ParsedIntent, payload: dict[str, Any]) -> None:
        msg = String(data=json.dumps(payload, sort_keys=True, default=str))
        if intent.intent == INTENT_START_TOUR:
            self.tour_start_pub.publish(msg)
        elif intent.intent == INTENT_CONTINUE_TOUR:
            self.tour_continue_pub.publish(msg)
        elif intent.intent == INTENT_SKIP_CHECKPOINT:
            self.tour_skip_pub.publish(msg)

    def _check_timeout(self) -> None:
        if not self.pending:
            return
        timeout = float(self.get_parameter("confirmation_timeout_sec").value)
        if time.time() - self.pending.created_at < timeout:
            return
        intent = self.pending.intent
        self.pending = None
        self._say("I did not get confirmation, so I will stay here.", "confirmation")
        self._publish_state({"state": "confirmation_timeout", "intent": intent.intent, "text": intent.text})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceIntentGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
