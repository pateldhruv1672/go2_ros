from __future__ import annotations

from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional
import json
import os
import time
import urllib.error
import urllib.request


COUNCIL_AGENTS = [
    "NavigatorAgent",
    "LocalizerAgent",
    "SafetyAgent",
    "PerceptionAgent",
    "SemanticMemoryAgent",
    "TourGuideAgent",
    "SystemsAgent",
]


@dataclass
class CouncilVote:
    agent: str
    vote: str
    confidence: float
    risk_level: str
    final_action: str
    requires_human_interrupt: bool
    rationale: str
    fallback_plan: List[str]
    source: str
    raw: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class LLMVoteClient:
    """Small OpenAI-compatible/OpenRouter/Gemini debate client.

    This intentionally avoids a hard LangChain dependency inside the ROS node.
    The agent can use OpenRouter or any OpenAI-compatible endpoint by setting:

      GO2_DEBATE_LLM_PROVIDER=openrouter|openai_compatible|gemini|offline
      OPENROUTER_API_KEY=...
      OPENAI_API_KEY=... with GO2_DEBATE_BASE_URL=...
      GEMINI_API_KEY or GOOGLE_API_KEY=...

    If no provider/key is configured, callers should fall back to the safety
    heuristic and mark the result as heuristic_fallback. The LLM is never the
    final safety authority; post-processing in debate.py applies hard vetoes.
    """

    def __init__(
        self,
        provider: Optional[str] = None,
        model: Optional[str] = None,
        timeout_sec: float = 8.0,
        temperature: float = 0.1,
        request_fn: Optional[Callable[[str, Dict[str, Any], Dict[str, str], float], Dict[str, Any]]] = None,
    ) -> None:
        self.provider = (provider or os.getenv("GO2_DEBATE_LLM_PROVIDER") or "openrouter").strip().lower()
        self.model = model or os.getenv("GO2_DEBATE_LLM_MODEL") or "openai/gpt-4o-mini"
        self.timeout_sec = float(timeout_sec)
        self.temperature = float(temperature)
        self.request_fn = request_fn or self._http_json

    def available(self) -> bool:
        if self.provider == "offline":
            return False
        if self.provider == "openrouter":
            return bool(os.getenv("OPENROUTER_API_KEY"))
        if self.provider in {"openai", "openai_compatible"}:
            return bool(os.getenv("OPENAI_API_KEY") or os.getenv("GO2_DEBATE_API_KEY"))
        if self.provider == "gemini":
            return bool(os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY"))
        return False

    def vote(self, agent: str, user_intent: str, context: Dict[str, Any], candidate_actions: List[str]) -> CouncilVote:
        if not self.available():
            raise RuntimeError("LLM debate provider/key is not configured")
        prompt = self._prompt(agent, user_intent, context, candidate_actions)
        started = time.time()
        try:
            if self.provider == "gemini":
                raw = self._call_gemini(prompt)
            else:
                raw = self._call_openai_compatible(prompt)
            payload = self._extract_json(raw)
            payload.setdefault("latency_sec", round(time.time() - started, 3))
            return self._vote_from_payload(agent, payload, source=f"llm:{self.provider}:{self.model}")
        except Exception as exc:
            raise RuntimeError(f"{agent} LLM vote failed: {exc}") from exc

    def _prompt(self, agent: str, user_intent: str, context: Dict[str, Any], candidate_actions: List[str]) -> str:
        compact_context = {
            "localization_confidence": context.get("localization_confidence"),
            "safety_blocked": context.get("safety_blocked"),
            "front_clearance_m": context.get("front_clearance_m"),
            "odom": context.get("odom"),
            "scan_summary": context.get("scan_summary"),
            "traversability": context.get("traversability"),
            "dynamic_obstacles": context.get("dynamic_obstacles"),
            "frontier_candidates": context.get("frontier_candidates"),
            "goal_candidate": context.get("goal_candidate"),
            "nav_status": context.get("nav_status"),
            "memory_summary": context.get("memory_summary"),
        }
        return (
            "You are one council member in a safety-critical Unitree Go2 ROS 2 navigation agent. "
            "Act only as the named role. Do not invent sensor data. Nav2 executes motion; you only vote.\n"
            f"ROLE: {agent}\n"
            f"USER_INTENT: {user_intent}\n"
            f"CANDIDATE_ACTIONS: {json.dumps(candidate_actions)}\n"
            f"ROBOT_CONTEXT_JSON: {json.dumps(compact_context, sort_keys=True, default=str)[:8000]}\n"
            "Return ONLY compact JSON with keys: vote, confidence, risk_level, final_action, "
            "requires_human_interrupt, rationale, fallback_plan. "
            "vote must be approve, veto, recover_first, ask_human, or abstain. "
            "risk_level must be low, medium, or high. final_action must be one candidate or stop_robot or recover_localization."
        )

    def _call_openai_compatible(self, prompt: str) -> Dict[str, Any]:
        if self.provider == "openrouter":
            url = os.getenv("GO2_DEBATE_BASE_URL", "https://openrouter.ai/api/v1/chat/completions")
            key = os.getenv("OPENROUTER_API_KEY", "")
            headers = {
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
                "HTTP-Referer": os.getenv("GO2_DEBATE_HTTP_REFERER", "https://github.com/pateldhruv1672/go2_ros"),
                "X-Title": os.getenv("GO2_DEBATE_APP_TITLE", "Go2 LangGraph Debate Council"),
            }
        else:
            base = os.getenv("GO2_DEBATE_BASE_URL", "https://api.openai.com/v1/chat/completions")
            url = base.rstrip("/")
            key = os.getenv("GO2_DEBATE_API_KEY") or os.getenv("OPENAI_API_KEY", "")
            headers = {"Authorization": f"Bearer {key}", "Content-Type": "application/json"}
        body = {
            "model": self.model,
            "temperature": self.temperature,
            "messages": [
                {"role": "system", "content": "You output valid JSON only."},
                {"role": "user", "content": prompt},
            ],
        }
        return self.request_fn(url, body, headers, self.timeout_sec)

    def _call_gemini(self, prompt: str) -> Dict[str, Any]:
        key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY", "")
        model = self.model if self.model.startswith("models/") else f"models/{self.model}"
        url = f"https://generativelanguage.googleapis.com/v1beta/{model}:generateContent?key={key}"
        body = {"contents": [{"parts": [{"text": prompt}]}], "generationConfig": {"temperature": self.temperature}}
        return self.request_fn(url, body, {"Content-Type": "application/json"}, self.timeout_sec)

    def _http_json(self, url: str, body: Dict[str, Any], headers: Dict[str, str], timeout: float) -> Dict[str, Any]:
        req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"), headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:1000]
            raise RuntimeError(f"HTTP {exc.code}: {detail}") from exc

    def _extract_json(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        text = ""
        if "choices" in raw:
            text = raw.get("choices", [{}])[0].get("message", {}).get("content", "")
        elif "candidates" in raw:
            parts = raw.get("candidates", [{}])[0].get("content", {}).get("parts", [])
            text = "\n".join(str(p.get("text", "")) for p in parts)
        if not text and isinstance(raw, dict):
            # Test/mocked clients may directly return the desired payload.
            return dict(raw)
        text = text.strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.lower().startswith("json"):
                text = text[4:].strip()
        try:
            return json.loads(text)
        except Exception:
            start = text.find("{")
            end = text.rfind("}")
            if start >= 0 and end > start:
                return json.loads(text[start : end + 1])
            raise RuntimeError(f"LLM did not return JSON: {text[:500]}")

    def _vote_from_payload(self, agent: str, payload: Dict[str, Any], source: str) -> CouncilVote:
        fallback = payload.get("fallback_plan") or ["stop_robot"]
        if isinstance(fallback, str):
            fallback = [fallback]
        return CouncilVote(
            agent=agent,
            vote=str(payload.get("vote", "abstain")).lower(),
            confidence=max(0.0, min(1.0, float(payload.get("confidence", 0.5) or 0.5))),
            risk_level=str(payload.get("risk_level", "medium")).lower(),
            final_action=str(payload.get("final_action", "stop_robot")),
            requires_human_interrupt=bool(payload.get("requires_human_interrupt", False)),
            rationale=str(payload.get("rationale", ""))[:1000],
            fallback_plan=[str(x) for x in fallback][:5],
            source=source,
            raw=payload,
        )
