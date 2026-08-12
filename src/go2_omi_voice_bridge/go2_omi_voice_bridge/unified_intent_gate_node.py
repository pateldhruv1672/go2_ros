from __future__ import annotations

from dataclasses import dataclass
import json
import re
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
    normalize_text,
    parse_intent,
    speech_payload,
    transcript_confidence,
    transcript_text,
)

from go2_agentic_motion_skills.webrtc_motion_skill_agent_node import (
    CANONICAL_TO_WEBRTC_NAME,
    SCRIPTS,
    normalize_command,
)

INTENT_COUNT_OBJECTS = "count_objects"
INTENT_WHERE_OBJECT = "where_object"
INTENT_FIND_OBJECT = "find_object"
INTENT_MOTION_SKILL = "motion_skill"
INTENT_AGENT_QUERY = "agent_query"
INTENT_TOUR_HOST = "tour_host_script"
PHONE_NO_CONFIRM_MOTION_SKILLS = {"sit", "stand_up", "hello", "dance1", "dance2", "front_flip"}
# Phone aliases normalize as: wave->hello, stand->stand_up, dance->dance1.

# SPARKY_GUEST_TOUR_HOST_V1


@dataclass
class PendingCommand:
    intent: ParsedIntent
    created_at: float


def _strip_prefix(text: str, prefixes: tuple[str, ...]) -> str:
    for prefix in prefixes:
        if text.startswith(prefix):
            return text[len(prefix):].strip()
    return ""


def _clean_object_query(value: str) -> str:
    value = re.sub(r"\b(?:in|inside|within)\s+(?:this|the|current)\s+(?:room|area|lab)\b.*$", "", value).strip()
    value = re.sub(r"\b(?:are|is)\s+(?:in|inside|around)\b.*$", "", value).strip()
    value = re.sub(r"^(?:the|a|an)\s+", "", value).strip()
    return value or "object"


def parse_extended_intent(text: str, confidence: float, wake_words: list[str]) -> ParsedIntent:
    base = parse_intent(text, confidence, wake_words)
    if base.intent != INTENT_UNKNOWN:
        return base

    norm = normalize_text(text, wake_words)
    if not norm:
        return base

    # Guest-facing deterministic Tour Host scripts. Speech-only scripts do not
    # require confirmation; safe_moves contains Sport motion and therefore does.
    host_script = ""
    host_motion = False
    if norm in {
        "welcome guests", "welcome our guests", "welcome everyone",
        "start the introduction", "start introduction", "start intro",
        "give the introduction", "give an introduction", "begin the introduction",
        "host introduction", "full introduction",
    }:
        host_script = "full_intro"
    elif norm in {
        "welcome", "give the welcome", "welcome the guests",
    }:
        host_script = "welcome"
    elif norm in {
        "introduce the lab", "introduce digital twin lab", "introduce the digital twin lab",
        "tell us about the lab", "tell us about this lab", "lab introduction",
    }:
        host_script = "lab_intro"
    elif norm in {
        "tell us about the research", "research introduction", "introduce the research",
        "tell us about the work", "introduce the work",
    }:
        host_script = "research_intro"
    elif norm in {
        "introduce yourself", "introduce sparky", "sparky introduction",
    }:
        host_script = "sparky_intro"
    elif norm in {
        "tell us your capabilities", "what can you do for the tour", "capabilities",
        "explain your capabilities",
    }:
        host_script = "capabilities"
    elif norm in {
        "show some moves", "show us some moves", "show a move", "show us a move",
        "demonstrate some moves", "motion demo", "show your moves",
    }:
        host_script = "safe_moves"
        host_motion = True
    if host_script:
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_TOUR_HOST,
            confidence=confidence, requires_motion=host_motion,
            requires_confirmation=host_motion,
            metadata={"host_script": host_script},
        )

    # Deterministic object-memory operations.
    if norm.startswith("how many "):
        query = norm[len("how many "):]
        query = re.split(r"\b(?:do you|can you|are there|are in|are inside|are around)\b", query, maxsplit=1)[0].strip()
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_COUNT_OBJECTS,
            confidence=confidence, semantic_target=_clean_object_query(query),
            metadata={"object_query": _clean_object_query(query)},
        )
    count_q = _strip_prefix(norm, ("count the ", "count "))
    if count_q:
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_COUNT_OBJECTS,
            confidence=confidence, semantic_target=_clean_object_query(count_q),
            metadata={"object_query": _clean_object_query(count_q)},
        )

    find_q = _strip_prefix(norm, ("find the ", "find ", "locate the ", "locate ", "look for the ", "look for ", "search for the ", "search for "))
    if find_q:
        query = _clean_object_query(find_q)
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_FIND_OBJECT,
            confidence=confidence, semantic_target=query,
            requires_motion=True, requires_resume_mode=True, requires_confirmation=True,
            metadata={"object_query": query},
        )

    where_q = _strip_prefix(norm, ("where is the ", "where is ", "where are the ", "where are ", "where was the ", "where was ", "where did you see the ", "where did you see "))
    if where_q:
        query = _clean_object_query(where_q)
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_WHERE_OBJECT,
            confidence=confidence, semantic_target=query,
            metadata={"object_query": query},
        )

    # Reuse the already-installed WebRTC motion skill vocabulary. This gate is the
    # only transcript consumer in the new stack; omi_skill_intent_router is not launched.
    skill_text = norm
    for prefix in ("perform ", "do ", "please ", "can you "):
        if skill_text.startswith(prefix):
            skill_text = skill_text[len(prefix):].strip()
            break
    skill = normalize_command(skill_text)
    if skill in CANONICAL_TO_WEBRTC_NAME or skill in SCRIPTS:
        return ParsedIntent(
            text=text, normalized_text=norm, intent=INTENT_MOTION_SKILL,
            confidence=confidence, requires_motion=True, requires_confirmation=True,
            metadata={"motion_skill": skill},
        )

    # Unknown language is a read-only agent question, not an execution request.
    return ParsedIntent(
        text=text, normalized_text=norm, intent=INTENT_AGENT_QUERY,
        confidence=confidence, requires_motion=False, requires_confirmation=False,
    )


class UnifiedIntentGateNode(Node):
    """Single transcript safety gate for phone/Omi, semantic-nav, object QA and Sport skills."""

    def __init__(self) -> None:
        super().__init__("go2_unified_intent_gate")
        self.declare_parameter("transcript_confidence_threshold", 0.70)
        self.declare_parameter("confirmation_timeout_sec", 15.0)
        self.declare_parameter("wake_words", ["sparky", "go2", "robot"])
        self.declare_parameter("require_wake_word", True)
        self.declare_parameter("require_confirmation_for_motion", False)
        self.declare_parameter("emit_legacy_tour_topics", True)
        self.declare_parameter("duplicate_suppression_sec", 4.0)
        self.declare_parameter("ignore_transcripts_during_tts", True)
        self.declare_parameter("tts_feedback_cooldown_sec", 3.0)
        self.declare_parameter("tts_feedback_preroll_sec", 0.75)
        self.declare_parameter("agent_command_topic", "/go2_agent/user_command")
        self.declare_parameter("agent_query_topic", "/go2_agent/query")
        self.declare_parameter("vlm_query_topic", "/go2_vlm/query")
        self.declare_parameter("tts_topic", "/go2_tts/say")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_out")
        self.declare_parameter("nav_command_topic", "/semantic_nav/command")
        self.declare_parameter("motion_skill_topic", "/motion_skills/command")

        self.agent_pub = self.create_publisher(String, str(self.get_parameter("agent_command_topic").value), 10)
        self.query_pub = self.create_publisher(String, str(self.get_parameter("agent_query_topic").value), 10)
        self.vlm_query_pub = self.create_publisher(String, str(self.get_parameter("vlm_query_topic").value), 10)
        self.verify_pub = self.create_publisher(String, "/go2_voice/verification_request", 10)
        self.state_pub = self.create_publisher(String, "/go2_voice/verification_state", 10)
        self.tts_pub = self.create_publisher(String, str(self.get_parameter("tts_topic").value), 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.nav_pub = self.create_publisher(String, str(self.get_parameter("nav_command_topic").value), 10)
        self.motion_pub = self.create_publisher(String, str(self.get_parameter("motion_skill_topic").value), 10)
        self.host_pub = self.create_publisher(String, "/go2_tour/host_command", 10)
        self.tour_start_pub = self.create_publisher(String, "/go2_tour/start", 10)
        self.tour_continue_pub = self.create_publisher(String, "/go2_tour/continue", 10)
        self.tour_skip_pub = self.create_publisher(String, "/go2_tour/skip", 10)
        self.tour_cancel_pub = self.create_publisher(String, "/go2_tour/cancel", 10)

        self.create_subscription(String, "/go2_voice/transcript", self._on_transcript, 20)
        self.create_subscription(String, "/go2_tts/status", self._on_tts_status, 10)
        self.create_subscription(String, "/semantic_nav/status", self._on_nav_status, 20)
        self.create_subscription(String, "/go2_nav/status", self._on_nav_status, 10)
        self.create_subscription(String, "/collision_monitor_state", self._on_collision_state, 10)

        self.pending: PendingCommand | None = None
        self.latest_nav_status: Any = None
        self.latest_collision_state = ""
        self.tts_ignore_until = 0.0
        self.last_transcript_text = ""
        self.last_transcript_time = 0.0
        self.timer = self.create_timer(1.0, self._check_timeout)
        self.get_logger().info(
            "Unified intent gate ready: phone/Omi -> one safety gate -> semantic-nav/object QA/WebRTC skills."
        )

    def _wake_words(self) -> list[str]:
        value = self.get_parameter("wake_words").value
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        return [part.strip() for part in str(value).split(",") if part.strip()]

    def _bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _has_wake_word(self, text: str) -> bool:
        norm = normalize_text(text)
        tokens = set(norm.split())
        return any(normalize_text(wake) in tokens for wake in self._wake_words())

    def _state(self, payload: dict[str, Any]) -> None:
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

    def _on_tts_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not self._bool("ignore_transcripts_during_tts") or not payload.get("local_speaker_enabled", True):
            return
        event = str(payload.get("event", ""))
        now = time.time()
        if event == "speech_queued":
            self.tts_ignore_until = max(
                self.tts_ignore_until,
                now + float(self.get_parameter("tts_feedback_preroll_sec").value),
            )
        elif event == "local_speaker_start":
            estimated = float(payload.get("estimated_duration_sec") or 0.0)
            self.tts_ignore_until = max(
                self.tts_ignore_until,
                now + estimated + float(self.get_parameter("tts_feedback_cooldown_sec").value),
            )
        elif event == "local_speaker_done":
            self.tts_ignore_until = max(
                self.tts_ignore_until,
                now + float(self.get_parameter("tts_feedback_cooldown_sec").value),
            )

    def _on_transcript(self, msg: String) -> None:
        payload = decode_json_or_text(msg.data)
        text = transcript_text(payload)
        confidence = transcript_confidence(payload)
        if not text:
            return

        preview = parse_extended_intent(text, confidence, self._wake_words())
        if (
            self._bool("ignore_transcripts_during_tts")
            and time.time() < self.tts_ignore_until
            and preview.intent not in {INTENT_STOP, INTENT_CANCEL}
        ):
            self._state({"state": "ignored_tts_feedback", "text": text})
            return

        now = time.time()
        if text == self.last_transcript_text and now - self.last_transcript_time < float(self.get_parameter("duplicate_suppression_sec").value):
            return
        self.last_transcript_text = text
        self.last_transcript_time = now

        if self.pending:
            if is_confirmation(text):
                self._approve_pending(text)
                return
            if is_rejection(text):
                self._reject_pending(text)
                return
            self._state({"state": "waiting_for_confirmation", "ignored_text": text})
            return

        if is_confirmation(text) or is_rejection(text):
            self._state({"state": "no_pending_confirmation_ignored", "text": text})
            return

        threshold = float(self.get_parameter("transcript_confidence_threshold").value)
        if confidence < threshold:
            self._say("I am not sure I heard that correctly. Could you repeat?", "warning")
            self._state({"state": "low_confidence", "text": text, "confidence": confidence})
            return

        intent = preview
        if self._bool("require_wake_word") and not self._has_wake_word(text) and intent.intent not in {INTENT_STOP, INTENT_CANCEL}:
            self._state({"state": "ignored_no_wake_word", "text": text})
            return

        if intent.intent in {INTENT_STOP, INTENT_CANCEL}:
            self._handle_immediate_stop(intent)
            return

        # SPARKY_PHONE_INSTANT_GESTURES_V12_6
        # Dashboard/phone speech gestures are immediate, but ONLY for this explicit skill allowlist.
        phone_source = str(payload.get("source") or "").strip().lower()
        phone_instant_motion = (
            (phone_source == "phone_web" or phone_source.startswith("phone_"))
            and intent.intent == INTENT_MOTION_SKILL
            and str(intent.metadata.get("motion_skill") or "") in PHONE_NO_CONFIRM_MOTION_SKILLS
        )
        if phone_instant_motion:
            self._state({"state": "phone_instant_motion", "intent": intent.intent, "skill": intent.metadata.get("motion_skill"), "source": phone_source})
            self._publish_approved(intent, verified=True)
            return
        if self._requires_confirmation(intent):
            self.pending = PendingCommand(intent=intent, created_at=time.time())
            prompt = self._confirmation_prompt(intent)
            self.verify_pub.publish(String(data=json.dumps({"prompt": prompt, "intent": intent.intent}, sort_keys=True)))
            self._say(prompt, "confirmation", "high", interrupt=True)
            self._state({"state": "waiting_for_confirmation", "intent": intent.intent, "text": intent.text})
            return

        self._publish_approved(intent, verified=True)

    def _requires_confirmation(self, intent: ParsedIntent) -> bool:
        # SPARKY_NO_CONFIRMATIONS_V12_7
        # Operator requested immediate execution for user commands.
        # Hard runtime safety boundaries (stop/cancel, collision monitor,
        # Nav2, localization checks, and motion-skill nav interlock) remain.
        return False

    def _confirmation_prompt(self, intent: ParsedIntent) -> str:
        if intent.intent == INTENT_TOUR_HOST:
            script = str(intent.metadata.get("host_script") or "full_intro")
            if intent.requires_motion:
                return f"I heard: run the {script.replace('_', ' ')} demonstration. This includes a Sport motion. Should I proceed?"
            return f"I heard: run the {script.replace('_', ' ')} Tour Host script."
        if intent.intent == INTENT_FIND_OBJECT:
            return f"I heard: find {intent.semantic_target or 'that object'}. I will navigate to its remembered safe viewpoint. Should I proceed?"
        if intent.intent == INTENT_MOTION_SKILL:
            skill = str(intent.metadata.get("motion_skill") or intent.text).replace("_", " ")
            return f"I heard: perform {skill}. This uses the Go2 Sport motion API. Should I proceed?"
        if intent.intent == INTENT_START_TOUR:
            return "I heard: start the saved tour. This will navigate through saved tour stops. Should I proceed?"
        if intent.intent == INTENT_CONTINUE_TOUR:
            return "I heard: continue the tour. Should I proceed?"
        if intent.intent == INTENT_SKIP_CHECKPOINT:
            return "I heard: skip this tour stop. Should I proceed?"
        if intent.intent == INTENT_NAVIGATE_TO_PLACE:
            return f"I heard: navigate to {intent.semantic_target or 'that place'}. Should I proceed?"
        return f"I heard: {intent.text}. Should I proceed?"

    def _handle_immediate_stop(self, intent: ParsedIntent) -> None:
        self.pending = None
        self.cmd_pub.publish(Twist())
        cancel = {"type": "cancel", "reason": "unified_voice_stop", "source": "unified_voice", "text": intent.text}
        self.nav_pub.publish(String(data=json.dumps(cancel, sort_keys=True)))
        self.tour_cancel_pub.publish(String(data=json.dumps(cancel, sort_keys=True)))
        self.motion_pub.publish(String(data=json.dumps({"command": "stop_move", "source": "unified_voice_stop"}, sort_keys=True)))
        self._say("Stopping now.", "warning", "high", interrupt=True)
        self._state({"state": "stopped", "intent": intent.intent, "text": intent.text})

    def _approve_pending(self, confirmation_text: str) -> None:
        pending = self.pending
        self.pending = None
        if pending:
            self._publish_approved(pending.intent, verified=True, confirmation_text=confirmation_text)

    def _reject_pending(self, rejection_text: str) -> None:
        pending = self.pending
        self.pending = None
        self._say("Canceled. I will not execute that action.", "status")
        self._state({"state": "rejected", "intent": pending.intent.intent if pending else None, "text": rejection_text})

    def _publish_approved(self, intent: ParsedIntent, verified: bool, confirmation_text: str = "") -> None:
        # SPARKY_TOUR_AUTO_INTRO_V12_7
        if intent.intent == INTENT_START_TOUR:
            self.host_pub.publish(String(data=json.dumps({
                "script": "full_intro", "source": "unified_voice",
                "verified": True, "safety_checked": True,
            }, sort_keys=True)))
        payload = intent.to_agent_payload(verified=verified, source="unified_voice")
        if confirmation_text:
            payload["confirmation_text"] = confirmation_text
        if self.latest_nav_status is not None:
            payload["latest_nav_status"] = self.latest_nav_status
        if self.latest_collision_state:
            payload["collision_monitor_state"] = self.latest_collision_state

        if intent.intent == INTENT_TOUR_HOST:
            script = str(intent.metadata.get("host_script") or "full_intro")
            self.host_pub.publish(String(data=json.dumps({
                "script": script,
                "source": "unified_voice",
                "verified": verified,
                "safety_checked": True,
            }, sort_keys=True)))
        elif intent.intent == INTENT_MOTION_SKILL:
            skill = str(intent.metadata.get("motion_skill") or "")
            self.motion_pub.publish(String(data=json.dumps({
                "command": skill,
                "source": "unified_voice",
                "verified": verified,
                "safety_checked": True,
            }, sort_keys=True)))
            self._say(f"Executing {skill.replace('_', ' ')}.", "status")
        elif intent.intent == INTENT_OBSERVE:
            # Visual questions must use the current camera frame, not persistent memory.
            visual = {
                "request_id": f"vlm_{time.time_ns()}",
                "question": intent.text,
                "text": intent.text,
                "source": "unified_voice",
                "verified": verified,
            }
            self.vlm_query_pub.publish(String(data=json.dumps(visual, sort_keys=True, default=str)))
        elif intent.intent in {INTENT_COUNT_OBJECTS, INTENT_WHERE_OBJECT, INTENT_AGENT_QUERY, INTENT_WHERE_AM_I, INTENT_FUN_FACT}:
            # Read-only path: the supervisor hard-suppresses motion on /go2_agent/query.
            self.query_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))
        else:
            # Verified command path. find_object is handled by world tools; semantic place navigation
            # and all other approved agent commands remain routed through the supervisor.
            self.agent_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

        msg = String(data=json.dumps(payload, sort_keys=True, default=str))
        # Compatibility topics can be disabled when LangGraph is the sole Tour command router.
        if self._bool("emit_legacy_tour_topics"):
            if intent.intent == INTENT_START_TOUR:
                self.tour_start_pub.publish(msg)
            elif intent.intent == INTENT_CONTINUE_TOUR:
                self.tour_continue_pub.publish(msg)
            elif intent.intent == INTENT_SKIP_CHECKPOINT:
                self.tour_skip_pub.publish(msg)

        self._state({"state": "approved", "intent": intent.intent, "verified": verified})

    def _check_timeout(self) -> None:
        if not self.pending:
            return
        timeout = float(self.get_parameter("confirmation_timeout_sec").value)
        if time.time() - self.pending.created_at < timeout:
            return
        intent = self.pending.intent
        self.pending = None
        self._say("I did not get confirmation, so I will not execute that action.", "confirmation")
        self._state({"state": "confirmation_timeout", "intent": intent.intent, "text": intent.text})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnifiedIntentGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
