from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CONFIRM_WORDS = {
    "yes", "yeah", "yep", "confirm", "go ahead", "proceed",
    "do it", "sure", "please do", "okay", "ok",
}

REJECT_WORDS = {
    "no", "cancel", "stop", "not now", "never mind", "abort",
}

CANONICAL_TO_WEBRTC_NAME = {
    "damp": "Damp",
    "balance_stand": "BalanceStand",
    "stop_move": "StopMove",
    "stand_up": "StandUp",
    "stand_down": "StandDown",
    "recovery_stand": "RecoveryStand",
    "sit": "Sit",
    "rise_sit": "RiseSit",
    "hello": "Hello",
    "stretch": "Stretch",
    "content": "Content",
    "dance1": "Dance1",
    "dance2": "Dance2",
    "scrape": "Scrape",
    "front_flip": "FrontFlip",
    "front_jump": "FrontJump",
    "front_pounce": "FrontPounce",
    "heart": "FingerHeart",
    "hand_stand": "Handstand",
    "cross_step": "CrossStep",
}

try:
    from go2_robot_sdk.domain.constants.robot_commands import ROBOT_CMD
except Exception:
    ROBOT_CMD = {}



def clean_text(text: str) -> str:
    text = (text or "").strip().lower()
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def strip_wake_words(text: str, wake_words: list[str]) -> tuple[str, bool]:
    full = clean_text(text)
    value = full

    for wake in wake_words:
        wake_norm = clean_text(wake)
        if not wake_norm:
            continue

        if value == wake_norm:
            return "", True

        if value.startswith(wake_norm + " "):
            return value[len(wake_norm):].strip(), True

    tokens = set(full.split())
    has_wake = any(clean_text(wake) in tokens for wake in wake_words if clean_text(wake))
    return value, has_wake


def decode_transcript(data: str) -> tuple[str, float]:
    raw = (data or "").strip()
    if not raw:
        return "", 0.0

    try:
        payload = json.loads(raw)
        if isinstance(payload, dict):
            for key in ("text", "transcript", "user_text", "data"):
                value = payload.get(key)
                if isinstance(value, str) and value.strip():
                    try:
                        confidence = float(payload.get("confidence", 1.0))
                    except Exception:
                        confidence = 1.0
                    return value.strip(), confidence
    except Exception:
        pass

    return raw, 1.0


def speech_payload(text: str, category: str = "status", priority: str = "normal", interrupt: bool = False) -> str:
    return json.dumps(
        {
            "text": text,
            "category": category,
            "priority": priority,
            "interrupt": interrupt,
            "voice": "default",
        },
        sort_keys=True,
    )


@dataclass
class PendingSkill:
    skill: str
    raw_text: str
    created_at: float


class OmiSkillIntentRouter(Node):
    """
    Routes processed Omi STT into:
      /motion_skills/command
      /semantic_nav/command
      /go2_tts/say

    Also prints processed STT:
      raw_msg
      raw_text
      clean_text
      wake_stripped_text
      command type
      parsed value
      final routed topic
    """

    def __init__(self) -> None:
        super().__init__("omi_skill_intent_router")

        self.declare_parameter("transcript_topic", "/go2_voice/transcript")
        self.declare_parameter("motion_skill_topic", "/motion_skills/command")
        self.declare_parameter("semantic_command_topic", "/semantic_nav/command")
        self.declare_parameter("tts_topic", "/go2_tts/say")
        self.declare_parameter("status_topic", "/go2_voice/skill_router_status")
        self.declare_parameter("processed_stt_topic", "/go2_voice/processed_stt")

        self.declare_parameter("wake_words", ["sparky", "sparkie", "go2", "robot"])
        self.declare_parameter("require_wake_word", True)
        self.declare_parameter("require_confirmation_for_skills", True)
        self.declare_parameter("confirmation_timeout_sec", 12.0)
        self.declare_parameter("confidence_threshold", 0.55)
        self.declare_parameter("duplicate_suppression_sec", 2.0)
        self.declare_parameter("print_processed_stt", True)

        self.motion_pub = self.create_publisher(
            String,
            str(self.get_parameter("motion_skill_topic").value),
            10,
        )
        self.semantic_pub = self.create_publisher(
            String,
            str(self.get_parameter("semantic_command_topic").value),
            10,
        )
        self.tts_pub = self.create_publisher(
            String,
            str(self.get_parameter("tts_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.processed_stt_pub = self.create_publisher(
            String,
            str(self.get_parameter("processed_stt_topic").value),
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("transcript_topic").value),
            self.on_transcript,
            10,
        )

        self.create_subscription(
            String,
            "/agent/reply",
            self.on_agent_reply,
            10,
        )

        self.pending: Optional[PendingSkill] = None
        self.last_text = ""
        self.last_time = 0.0

        self.create_timer(1.0, self.check_timeout)

        self.say("Omi skill router is ready.", "status")
        self.publish_status({"state": "ready"})

    def pbool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def get_wake_words(self) -> list[str]:
        value = self.get_parameter("wake_words").value
        if isinstance(value, (list, tuple)):
            return [str(x) for x in value]
        return [x.strip() for x in str(value).split(",") if x.strip()]

    def say(self, text: str, category: str = "status", priority: str = "normal", interrupt: bool = False) -> None:
        self.tts_pub.publish(String(data=speech_payload(text, category, priority, interrupt)))

    def publish_status(self, payload: dict) -> None:
        text = json.dumps(payload, sort_keys=True, default=str)
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def emit_processed_stt(self, payload: dict) -> None:
        payload = dict(payload)
        payload["stamp_sec"] = time.time()
        text = json.dumps(payload, sort_keys=True, default=str)
        self.processed_stt_pub.publish(String(data=text))

        if self.pbool("print_processed_stt"):
            self.get_logger().info("PROCESSED_STT " + text)

    def on_agent_reply(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if text:
            self.say(text, "motion_skill_result")

    def is_confirmation(self, text: str) -> bool:
        norm = clean_text(text)
        return any(word in norm for word in CONFIRM_WORDS)

    def is_rejection(self, text: str) -> bool:
        norm = clean_text(text)
        return any(word in norm for word in REJECT_WORDS)

    def on_transcript(self, msg: String) -> None:
        raw_text, confidence = decode_transcript(msg.data)
        if not raw_text:
            return

        now = time.time()
        if raw_text == self.last_text and now - self.last_time < float(self.get_parameter("duplicate_suppression_sec").value):
            return

        self.last_text = raw_text
        self.last_time = now

        full_clean = clean_text(raw_text)
        wake_words = self.get_wake_words()
        text, has_wake = strip_wake_words(raw_text, wake_words)

        preview_type, preview_value = self.parse_command(text)

        self.emit_processed_stt(
            {
                "stage": "decoded",
                "raw_msg": msg.data,
                "raw_text": raw_text,
                "confidence": confidence,
                "clean_text": full_clean,
                "wake_words": wake_words,
                "has_wake": has_wake,
                "wake_stripped_text": text,
                "preview_command_type": preview_type,
                "preview_value": preview_value,
                "pending_skill": self.pending.skill if self.pending else "",
            }
        )

        if confidence < float(self.get_parameter("confidence_threshold").value):
            self.say("I am not sure I heard that. Please repeat.", "warning")
            self.emit_processed_stt(
                {
                    "stage": "ignored_low_confidence",
                    "raw_text": raw_text,
                    "confidence": confidence,
                }
            )
            return

        # Important: confirmation/rejection must work without repeating the wake word.
        if self.pending:
            if self.is_confirmation(raw_text):
                pending = self.pending
                self.pending = None
                self.run_skill(pending.skill, pending.raw_text, confirmed=True)
                return

            if self.is_rejection(raw_text):
                old = self.pending
                self.pending = None
                self.say("Canceled. I will stay still.", "status")
                self.emit_processed_stt(
                    {
                        "stage": "confirmation_rejected",
                        "raw_text": raw_text,
                        "pending_skill": old.skill,
                    }
                )
                return

            self.say("Please say yes to proceed, or no to cancel.", "confirmation")
            self.emit_processed_stt(
                {
                    "stage": "waiting_for_confirmation",
                    "raw_text": raw_text,
                    "pending_skill": self.pending.skill,
                }
            )
            return

        if self.pbool("require_wake_word") and not has_wake:
            self.emit_processed_stt(
                {
                    "stage": "ignored_no_wake_word",
                    "raw_text": raw_text,
                    "clean_text": full_clean,
                    "wake_stripped_text": text,
                }
            )
            self.publish_status({"state": "ignored_no_wake_word", "text": raw_text})
            return

        command_type, value = self.parse_command(text)

        if command_type == "stop":
            self.motion_pub.publish(String(data="stop_move"))
            cancel_payload = {"type": "cancel", "source": "omi_skill_router"}
            self.semantic_pub.publish(String(data=json.dumps(cancel_payload, sort_keys=True)))
            self.say("Stopping now.", "warning", "high", interrupt=True)
            self.emit_processed_stt(
                {
                    "stage": "routed_stop",
                    "raw_text": raw_text,
                    "motion_topic": str(self.get_parameter("motion_skill_topic").value),
                    "motion_command": "stop_move",
                    "semantic_topic": str(self.get_parameter("semantic_command_topic").value),
                    "semantic_command": cancel_payload,
                }
            )
            return

        if command_type == "skill":
            skill = value
            if self.pbool("require_confirmation_for_skills"):
                self.pending = PendingSkill(skill=skill, raw_text=raw_text, created_at=time.time())
                self.say(f"I heard {skill.replace('_', ' ')}. Should I proceed?", "confirmation", "high", True)
                self.emit_processed_stt(
                    {
                        "stage": "confirmation_requested",
                        "raw_text": raw_text,
                        "skill": skill,
                    }
                )
            else:
                self.run_skill(skill, raw_text, confirmed=False)
            return

        if command_type == "nav":
            target = value
            payload = {"type": "go", "target": target, "source": "omi_skill_router", "text": raw_text}
            self.semantic_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
            self.say(f"I will try to navigate to {target}.", "navigation")
            self.emit_processed_stt(
                {
                    "stage": "routed_navigation",
                    "raw_text": raw_text,
                    "topic": str(self.get_parameter("semantic_command_topic").value),
                    "command": payload,
                }
            )
            return

        if command_type == "status":
            self.say("I am connected to Omi and listening for robot commands.", "status")
            self.emit_processed_stt(
                {
                    "stage": "routed_status",
                    "raw_text": raw_text,
                }
            )
            return

        self.say("I heard you, but I do not recognize that robot command yet.", "warning")
        self.emit_processed_stt(
            {
                "stage": "unknown",
                "raw_text": raw_text,
                "normalized": text,
            }
        )

    def run_skill(self, skill: str, raw_text: str, confirmed: bool) -> None:
        self.motion_pub.publish(String(data=skill))

        webrtc_name = CANONICAL_TO_WEBRTC_NAME.get(skill, "")
        webrtc_id = ROBOT_CMD.get(webrtc_name) if webrtc_name else None

        self.say(f"Running {skill.replace('_', ' ')}.", "motion_skill", "high", True)
        self.emit_processed_stt(
            {
                "stage": "routed_motion_skill",
                "raw_text": raw_text,
                "topic": str(self.get_parameter("motion_skill_topic").value),
                "command": skill,
                "canonical_command": skill,
                "sdk2_sport_command": skill,
                "webrtc_command_name": webrtc_name,
                "webrtc_command_id": webrtc_id,
                "confirmed": confirmed,
            }
        )

    def check_timeout(self) -> None:
        if not self.pending:
            return

        timeout = float(self.get_parameter("confirmation_timeout_sec").value)
        if time.time() - self.pending.created_at >= timeout:
            expired = self.pending
            self.pending = None
            self.say("I did not get confirmation, so I canceled the motion skill.", "confirmation")
            self.emit_processed_stt(
                {
                    "stage": "confirmation_timeout",
                    "skill": expired.skill,
                }
            )

    def parse_command(self, text: str) -> tuple[str, str]:
        norm = clean_text(text)

        if not norm:
            return "unknown", ""

        if any(x in norm for x in ("emergency stop", "stop", "halt", "freeze", "cancel", "abort")):
            return "stop", "stop_move"

        if any(x in norm for x in ("status", "are you ready", "system check")):
            return "status", ""

        skill = self.parse_motion_skill(norm)
        if skill:
            return "skill", skill

        target = self.parse_nav_target(norm)
        if target:
            return "nav", target

        return "unknown", ""

    def parse_nav_target(self, norm: str) -> str:
        if norm in {"home", "spawn", "start"}:
            return "spawn"

        if any(x in norm for x in ("return to spawn", "go home", "go back to spawn")):
            return "spawn"

        patterns = [
            r"^go to (.+)$",
            r"^navigate to (.+)$",
            r"^take me to (.+)$",
            r"^bring me to (.+)$",
            r"^drive to (.+)$",
        ]

        for pattern in patterns:
            m = re.match(pattern, norm)
            if m:
                target = m.group(1).strip()
                target = re.sub(r"^(the|a|an)\s+", "", target)
                return target

        return ""

    def parse_motion_skill(self, norm: str) -> str:
        phrase_to_skill = {
            "damp": "damp",
            "relax": "damp",

            "stand": "stand_up",
            "stand up": "stand_up",
            "get up": "stand_up",
            "stand ready": "stand_ready",

            "stand down": "stand_down",
            "lie down": "stand_down",
            "sleep": "stand_down",

            "sit": "sit",
            "sit down": "sit",
            "rise sit": "rise_sit",

            "recover": "recovery_stand",
            "recovery": "recovery_stand",
            "recovery stand": "recovery_stand",
            "self recover": "recovery_stand",

            "balance": "balance_stand",
            "balance stand": "balance_stand",

            "hello": "hello",
            "say hello": "hello",
            "wave": "hello",
            "greet": "tour_greet",
            "greet visitor": "tour_greet",

            "stretch": "stretch",
            "content": "content",
            "happy": "content",

            "dance": "dance1",
            "dance one": "dance1",
            "dance two": "dance2",

            "scrape": "scrape",
            "heart": "heart",

            "jump": "front_jump",
            "front jump": "front_jump",
            "pounce": "front_pounce",
            "front pounce": "front_pounce",

            "front flip": "front_flip",
            "back flip": "back_flip",
            "left flip": "left_flip",

            "hand stand": "hand_stand",
            "handstand": "hand_stand",

            "free walk": "free_walk",
            "static walk": "static_walk",
            "trot run": "trot_run",
            "run trot": "trot_run",

            "free bound": "free_bound",
            "free jump": "free_jump",
            "free avoid": "free_avoid",
            "walk upright": "walk_upright",
            "cross step": "cross_step",
            "classic walk": "classic_walk",

            "avoid mode": "switch_avoid_mode",
            "switch avoid mode": "switch_avoid_mode",
            "auto recovery on": "auto_recovery_on",
            "auto recovery off": "auto_recovery_off",

            "tour greet": "tour_greet",
            "tour attention": "tour_attention",
            "tour pause": "tour_pause",
            "tour resume": "tour_resume",
            "tour handoff": "tour_handoff",
            "blocked route recovery": "blocked_route_recovery",
        }

        if norm in phrase_to_skill:
            return phrase_to_skill[norm]

        prefixes = (
            "do ",
            "do a ",
            "perform ",
            "run ",
            "execute ",
            "show me ",
            "can you ",
            "please ",
        )

        for prefix in prefixes:
            if norm.startswith(prefix):
                remainder = norm[len(prefix):].strip()
                if remainder in phrase_to_skill:
                    return phrase_to_skill[remainder]

        return ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmiSkillIntentRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
