from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import json
import re


INTENT_STOP = "stop"
INTENT_OBSERVE = "observe"
INTENT_WHERE_AM_I = "where_am_i"
INTENT_NAVIGATE_TO_PLACE = "navigate_to_place"
INTENT_START_TOUR = "start_tour"
INTENT_CONTINUE_TOUR = "continue_tour"
INTENT_SKIP_CHECKPOINT = "skip_checkpoint"
INTENT_FUN_FACT = "fun_fact"
INTENT_CANCEL = "cancel"
INTENT_UNKNOWN = "unknown"

CONFIRM_WORDS = {
    "yes",
    "confirm",
    "go ahead",
    "proceed",
    "do it",
    "sure",
    "please do",
}
REJECT_WORDS = {"no", "cancel", "stop", "not now", "never mind", "abort"}


@dataclass
class ParsedIntent:
    text: str
    normalized_text: str
    intent: str
    confidence: float = 1.0
    semantic_target: str = ""
    tour_id: str = ""
    requires_motion: bool = False
    requires_resume_mode: bool = False
    requires_confirmation: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_agent_payload(self, verified: bool, source: str = "omi_voice") -> dict[str, Any]:
        payload: dict[str, Any] = {
            "source": source,
            "text": self.text,
            "user_text": self.text,
            "intent": self.intent,
            "intent_hint": self.intent,
            "confidence": self.confidence,
            "verified": verified,
            "requires_confirmation": self.requires_confirmation,
            "requires_motion": self.requires_motion,
            "requires_resume_mode": self.requires_resume_mode,
            "safety_checked": True,
        }
        if self.semantic_target:
            payload["semantic_target"] = self.semantic_target
            payload["destination"] = self.semantic_target
        if self.tour_id:
            payload["tour_id"] = self.tour_id
            payload["mode"] = "tour"
        if self.metadata:
            payload["metadata"] = self.metadata
        return payload


def decode_json_or_text(data: str) -> dict[str, Any]:
    text = (data or "").strip()
    if not text:
        return {"text": "", "confidence": 0.0, "raw": data}
    try:
        parsed = json.loads(text)
        if isinstance(parsed, dict):
            return parsed
    except Exception:
        pass
    return {"text": text, "confidence": 1.0, "raw": data}


def transcript_text(payload: dict[str, Any]) -> str:
    for key in ("text", "transcript", "user_text", "data"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def transcript_confidence(payload: dict[str, Any]) -> float:
    try:
        return float(payload.get("confidence", 1.0))
    except Exception:
        return 1.0


def normalize_text(text: str, wake_words: list[str] | tuple[str, ...] = ()) -> str:
    value = text.lower()
    value = re.sub(r"[^a-z0-9\s]", " ", value)
    value = re.sub(r"\s+", " ", value).strip()
    for wake in wake_words:
        wake_norm = re.sub(r"[^a-z0-9\s]", " ", str(wake).lower()).strip()
        if not wake_norm:
            continue
        if value == wake_norm:
            value = ""
        elif value.startswith(wake_norm + " "):
            value = value[len(wake_norm) :].strip()
    return value


def is_confirmation(text: str) -> bool:
    norm = normalize_text(text)
    return any(word in norm for word in CONFIRM_WORDS)


def is_rejection(text: str) -> bool:
    norm = normalize_text(text)
    return any(word in norm for word in REJECT_WORDS)


def _target_after(text: str, prefixes: tuple[str, ...]) -> str:
    for prefix in prefixes:
        if text.startswith(prefix):
            return text[len(prefix) :].strip()
    return ""


def parse_intent(text: str, confidence: float = 1.0, wake_words: list[str] | tuple[str, ...] = ()) -> ParsedIntent:
    norm = normalize_text(text, wake_words)
    if not norm:
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_UNKNOWN, confidence=confidence)

    if any(phrase in norm for phrase in ("emergency stop", "stop", "halt", "freeze")):
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_STOP, confidence=confidence)
    if any(phrase in norm for phrase in ("cancel navigation", "cancel nav", "cancel goal", "abort navigation")):
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_CANCEL, confidence=confidence)
    if any(phrase in norm for phrase in ("where are we", "where am i", "where are you", "current location")):
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_WHERE_AM_I, confidence=confidence)
    if any(phrase in norm for phrase in ("what do you see", "summarize this place", "what is around", "observe")):
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_OBSERVE, confidence=confidence)
    if any(phrase in norm for phrase in ("fun fact", "something fun", "tell me about this place")):
        return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_FUN_FACT, confidence=confidence)
    if "skip" in norm and "checkpoint" in norm:
        return ParsedIntent(
            text=text,
            normalized_text=norm,
            intent=INTENT_SKIP_CHECKPOINT,
            confidence=confidence,
            requires_confirmation=True,
        )
    if any(phrase in norm for phrase in ("next checkpoint", "continue tour", "continue the tour", "resume tour")):
        return ParsedIntent(
            text=text,
            normalized_text=norm,
            intent=INTENT_CONTINUE_TOUR,
            confidence=confidence,
            requires_motion=True,
            requires_resume_mode=True,
            requires_confirmation=True,
        )
    if "tour" in norm and any(word in norm for word in ("start", "begin", "run")):
        tour_id = "sjsu_ads_department_tour" if "applied data science" in norm or "ads" in norm else ""
        return ParsedIntent(
            text=text,
            normalized_text=norm,
            intent=INTENT_START_TOUR,
            confidence=confidence,
            tour_id=tour_id,
            requires_motion=True,
            requires_resume_mode=True,
            requires_confirmation=True,
        )

    nav_prefixes = (
        "take me to ",
        "navigate to ",
        "go to ",
        "bring me to ",
        "return to ",
    )
    target = _target_after(norm, nav_prefixes)
    if target:
        return ParsedIntent(
            text=text,
            normalized_text=norm,
            intent=INTENT_NAVIGATE_TO_PLACE,
            confidence=confidence,
            semantic_target=target,
            requires_motion=True,
            requires_resume_mode=True,
            requires_confirmation=True,
        )

    return ParsedIntent(text=text, normalized_text=norm, intent=INTENT_UNKNOWN, confidence=confidence)


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
