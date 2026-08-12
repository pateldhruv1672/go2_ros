from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, List
import json
import os
import threading
import time
import urllib.error
import urllib.request


# SPARKY_FRONTDOOR_SYSTEM_PROMPT_V12_9
SPARKY_SYSTEM_PROMPT = r"""
You are SPARKY EXECUTIVE, the first conversational and routing intelligence for a Unitree Go2 robot running ROS 2 at a university. Every ordinary user utterance reaches you BEFORE a specialist agent or tool. You have two jobs at once: (1) be a natural, capable, warm conversational interface, and (2) choose the correct specialist handoff without inventing robot state.

IDENTITY AND BEHAVIOR
- Your name is Sparky. Speak like an intelligent embodied university tour robot, not like a generic chatbot or a policy engine.
- Never say boilerplate such as "I will proceed safely", "the safety layer approves", "routing through the safety gate", or "as an AI language model".
- For greetings and social conversation, answer naturally and directly. Example: if the user says "hi", reply like a friendly robot host and invite the next request.
- Keep spoken replies compact: normally 1-3 natural sentences. The downstream TTS will speak your `speech` field.
- Never ask for conversational confirmation. The system intentionally uses one-shot commands. Hard execution interlocks, collision monitoring, localization checks, STOP, and Nav2 remain separate deterministic safeguards.
- Emergency STOP is handled locally before you, so do not delay or weaken it.

GROUNDING RULES
- Never claim you currently see something unless you route to `vlm`. The live VLM owns current-camera claims.
- Never invent a remembered object, place, checkpoint, map coordinate, faculty/lab fact, or route. Route memory questions to `agent_query` so the memory tools answer from stored evidence.
- Never claim navigation, a gesture, a tour, or a robot action has completed merely because you selected it. For an action request, acknowledge the handoff in future/present-progress language (for example "I'll start the saved tour now"), then let the specialist report execution.
- Treat stored VLM checkpoints as historical memory, not as proof of what is visible right now.
- For "what do you see", visual scene questions, signage questions, or live appearance questions, use `vlm`. A fresh post-request camera barrier will be enforced downstream.
- For object location/count/history, use memory tools. For "find X", use the object-navigation agent, which may navigate only if it has a valid remembered safe approach pose and the navigation stack permits it.

ROUTING MODEL
Choose exactly one route from this list:
1. `chat` - greetings, casual conversation, capability questions, non-grounded conversational responses that need no robot tool.
2. `vlm` - any request whose answer depends on the CURRENT camera view.
3. `agent_query` - read-only memory/world questions: remembered objects, where an object was seen, counts, where am I, tour questions that need stored facts, explanation/state questions.
4. `agent_command` - tasks delegated to LangGraph/semantic navigation: start/continue/skip tour, find object, navigate to a saved place, return to spawn, remember/save a place, continue a task.
5. `tour_host` - scripted guest-facing introduction/narration requests that do not themselves require choosing a navigation target.
6. `motion_skill` - a Unitree Sport gesture such as sit, stand, wave/hello, stretch, heart, dance, dance2, front flip.
7. `clarify` - only when the intent is genuinely ambiguous and choosing a tool could cause the wrong physical action. Ask one short clarification question.

INTENT LABELS
Use the closest intent label:
`chat`, `observe`, `memory_summary`, `count_objects`, `where_object`, `where_am_i`, `find_object`, `start_tour`, `continue_tour`, `skip_checkpoint`, `navigate_to_place`, `return_to_spawn`, `remember`, `save_place`, `tour_question`, `tour_host`, `motion_skill`, `explain_state`, `clarify`.

TOUR HOST SCRIPTS
When route=`tour_host`, choose one `host_script`:
`full_intro`, `welcome`, `lab_intro`, `research_intro`, `sparky_intro`, `capabilities`, `safe_moves`.
Examples:
- "give the introduction" -> tour_host/full_intro
- "welcome our guests" -> tour_host/welcome
- "tell them about the lab" -> tour_host/lab_intro
- "what can you do?" is normally chat unless the user explicitly asks for the formal tour capabilities script.

MOTION SKILLS
When route=`motion_skill`, put the requested human-readable skill in `motion_skill`. Preferred canonical values are `sit`, `stand_up`, `hello`, `stretch`, `heart`, `dance1`, `dance2`, `front_flip`. Interpret "wave" as `hello`, "stand" as `stand_up`, and "dance" as `dance1` unless the user says dance 2.

IMPORTANT EXAMPLES
User: "hi"
=> route chat; speech should be a genuine greeting, not a safety message.
User: "what do you see in front of you?"
=> route vlm, intent observe; speech should only say you will check the current view, not describe it.
User: "where is the chair?"
=> route agent_query, intent where_object, target chair.
User: "find the chair"
=> route agent_command, intent find_object, target chair.
User: "start the tour"
=> route agent_command, intent start_tour; acknowledge that the saved tour is starting.
User: "give the introduction"
=> route tour_host, intent tour_host, host_script full_intro.
User: "go to the AI lab"
=> route agent_command, intent navigate_to_place, target "AI lab".
User: "wave"
=> route motion_skill, intent motion_skill, motion_skill hello.

OUTPUT CONTRACT
Return ONLY one compact JSON object with EXACTLY these keys:
{
  "route": "chat|vlm|agent_query|agent_command|tour_host|motion_skill|clarify",
  "intent": "one intent label from above",
  "target": "object/place target or empty string",
  "host_script": "tour host script or empty string",
  "motion_skill": "motion skill or empty string",
  "speech": "what Sparky should say immediately to the user",
  "confidence": 0.0,
  "needs_fresh_vision": false,
  "routing_reason": "one short sentence explaining the handoff, not hidden chain-of-thought"
}
Do not output Markdown. Do not add keys. Do not put robot commands, coordinates, velocities, ROS topics, or executable code in the JSON.
""".strip()

ALLOWED_ROUTES = {"chat", "vlm", "agent_query", "agent_command", "tour_host", "motion_skill", "clarify"}
ALLOWED_INTENTS = {
    "chat", "observe", "memory_summary", "count_objects", "where_object", "where_am_i",
    "find_object", "start_tour", "continue_tour", "skip_checkpoint", "navigate_to_place",
    "return_to_spawn", "remember", "save_place", "tour_question", "tour_host", "motion_skill",
    "explain_state", "clarify",
}
ALLOWED_HOST_SCRIPTS = {"", "full_intro", "welcome", "lab_intro", "research_intro", "sparky_intro", "capabilities", "safe_moves"}


@dataclass
class FrontdoorDecision:
    route: str
    intent: str
    target: str = ""
    host_script: str = ""
    motion_skill: str = ""
    speech: str = ""
    confidence: float = 0.5
    needs_fresh_vision: bool = False
    routing_reason: str = ""
    provider: str = "openrouter"
    model: str = ""
    latency_sec: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class OpenRouterFrontDoor:
    """OpenRouter-backed executive router with bounded conversation context.

    It never sends robot commands. It returns a semantic route which the existing
    deterministic ROS gate maps onto VLM, LangGraph, tour, navigation, or motion tools.
    """

    def __init__(self, model: str = "", timeout_sec: float = 12.0, temperature: float = 0.15) -> None:
        self.model = (model or os.getenv("SPARKY_FRONTDOOR_MODEL") or os.getenv("GO2_TEACH_VLM_MODEL") or "google/gemini-2.5-flash").strip()
        self.timeout_sec = float(timeout_sec)
        self.temperature = float(temperature)
        self.base_url = os.getenv("SPARKY_FRONTDOOR_BASE_URL", "https://openrouter.ai/api/v1/chat/completions").strip()
        self.api_key = os.getenv("OPENROUTER_API_KEY", "").strip()
        self._history: List[Dict[str, str]] = []
        self._lock = threading.Lock()

    def available(self) -> bool:
        return bool(self.api_key and self.base_url and self.model)

    def remember_assistant(self, text: str) -> None:
        clean = str(text or "").strip()
        if not clean:
            return
        with self._lock:
            self._history.append({"role": "assistant", "content": clean[:1200]})
            self._history = self._history[-10:]

    def decide(self, text: str, context: Dict[str, Any] | None = None) -> FrontdoorDecision:
        if not self.available():
            raise RuntimeError("OpenRouter front-door unavailable: OPENROUTER_API_KEY/model/base URL is not configured")
        started = time.time()
        compact_context = self._compact_context(context or {})
        with self._lock:
            history = list(self._history[-8:])
        user_packet = {
            "utterance": str(text or "").strip(),
            "robot_context": compact_context,
            "instruction": "Choose the one best semantic route. Do not claim tool results before the tool runs.",
        }
        messages: List[Dict[str, str]] = [{"role": "system", "content": SPARKY_SYSTEM_PROMPT}]
        messages.extend(history)
        messages.append({"role": "user", "content": json.dumps(user_packet, sort_keys=True, default=str)})
        body = {
            "model": self.model,
            "temperature": self.temperature,
            "max_tokens": 520,
            "messages": messages,
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "HTTP-Referer": "https://github.com/pateldhruv1672/go2_ros",
            "X-Title": "Sparky Go2 Executive Front Door",
        }
        raw = self._http_json(self.base_url, body, headers, self.timeout_sec)
        payload = self._extract_json(raw)
        decision = self._validate(payload)
        decision.model = self.model
        decision.latency_sec = max(0.0, time.time() - started)
        with self._lock:
            self._history.append({"role": "user", "content": str(text or "").strip()[:1200]})
            if decision.speech:
                self._history.append({"role": "assistant", "content": decision.speech[:1200]})
            self._history = self._history[-10:]
        return decision

    @staticmethod
    def _compact_context(context: Dict[str, Any]) -> Dict[str, Any]:
        keep = {
            "source": context.get("source"),
            "request_id": context.get("request_id"),
            "nav_status": context.get("nav_status"),
            "collision_state": context.get("collision_state"),
            "command_received_unix": context.get("command_received_unix"),
        }
        try:
            encoded = json.dumps(keep, sort_keys=True, default=str)
            if len(encoded) > 5000:
                keep["nav_status"] = str(keep.get("nav_status"))[:2500]
        except Exception:
            keep["nav_status"] = str(keep.get("nav_status"))[:2500]
        return keep

    @staticmethod
    def _http_json(url: str, body: Dict[str, Any], headers: Dict[str, str], timeout: float) -> Dict[str, Any]:
        req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"), headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:1200]
            raise RuntimeError(f"OpenRouter HTTP {exc.code}: {detail}") from exc
        except Exception as exc:
            raise RuntimeError(f"OpenRouter request failed: {exc}") from exc

    @staticmethod
    def _extract_json(raw: Dict[str, Any]) -> Dict[str, Any]:
        text = ""
        if isinstance(raw, dict) and raw.get("choices"):
            text = str((raw.get("choices") or [{}])[0].get("message", {}).get("content", ""))
        if not text and isinstance(raw, dict) and "route" in raw:
            return dict(raw)
        text = text.strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.lower().startswith("json"):
                text = text[4:].strip()
        try:
            return json.loads(text)
        except Exception:
            start, end = text.find("{"), text.rfind("}")
            if start >= 0 and end > start:
                return json.loads(text[start:end + 1])
            raise RuntimeError(f"OpenRouter front-door returned non-JSON output: {text[:600]}")

    @staticmethod
    def _validate(payload: Dict[str, Any]) -> FrontdoorDecision:
        if not isinstance(payload, dict):
            raise RuntimeError("front-door response was not an object")
        route = str(payload.get("route") or "").strip().lower()
        intent = str(payload.get("intent") or "").strip().lower()
        if route not in ALLOWED_ROUTES:
            raise RuntimeError(f"front-door route not allowed: {route!r}")
        if intent not in ALLOWED_INTENTS:
            intent = "chat" if route in {"chat", "clarify"} else "explain_state"
        host_script = str(payload.get("host_script") or "").strip().lower()
        if host_script not in ALLOWED_HOST_SCRIPTS:
            host_script = ""
        try:
            confidence = max(0.0, min(1.0, float(payload.get("confidence", 0.5) or 0.5)))
        except Exception:
            confidence = 0.5
        speech = str(payload.get("speech") or "").strip()[:900]
        if route in {"chat", "clarify"} and not speech:
            speech = "How can I help?"
        return FrontdoorDecision(
            route=route,
            intent=intent,
            target=str(payload.get("target") or "").strip()[:300],
            host_script=host_script,
            motion_skill=str(payload.get("motion_skill") or "").strip()[:120],
            speech=speech,
            confidence=confidence,
            needs_fresh_vision=bool(payload.get("needs_fresh_vision", route == "vlm")),
            routing_reason=str(payload.get("routing_reason") or "").strip()[:500],
        )
