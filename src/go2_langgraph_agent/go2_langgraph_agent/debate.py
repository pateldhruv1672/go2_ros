from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional
import uuid

from go2_langgraph_agent.llm_debate import COUNCIL_AGENTS, LLMVoteClient, CouncilVote


MOTION_PREFIXES = ("navigate", "frontier", "coverage", "return_to_spawn")


@dataclass
class DebateDecision:
    decision_id: str
    user_intent: str
    candidate_actions: List[str]
    context_used: Dict[str, Any]
    votes: Dict[str, str]
    final_action: str
    confidence: float
    risk_level: str
    requires_human_interrupt: bool
    fallback_plan: List[str]
    spoken_response: str
    memory_updates: List[Dict[str, Any]] = field(default_factory=list)
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    debate_mode: str = "heuristic"
    council_votes: List[Dict[str, Any]] = field(default_factory=list)
    safety_overrides: List[Dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def run_debate(
    user_intent: str,
    context: Dict[str, Any],
    candidate_actions: List[str],
    *,
    enable_llm: bool = False,
    llm_client: Optional[LLMVoteClient] = None,
    llm_timeout_sec: float = 8.0,
    llm_model: Optional[str] = None,
    llm_provider: Optional[str] = None,
) -> DebateDecision:
    """Run the Go2 safety debate council.

    When enable_llm is true and a provider/key is configured, each council role
    produces a structured LLM vote. Hard safety rules are always applied after
    the LLM votes, so an LLM cannot authorize unsafe motion by itself. When the
    LLM is unavailable, the function returns a deterministic safety fallback and
    marks the mode honestly as heuristic_fallback.
    """
    if enable_llm:
        try:
            client = llm_client or LLMVoteClient(provider=llm_provider, model=llm_model, timeout_sec=llm_timeout_sec)
            if client.available():
                llm_votes = [client.vote(agent, user_intent, context, candidate_actions) for agent in COUNCIL_AGENTS]
                return _aggregate_llm_votes(user_intent, context, candidate_actions, llm_votes)
            fallback = _heuristic_debate(user_intent, context, candidate_actions)
            fallback.debate_mode = "heuristic_fallback_no_llm_provider"
            fallback.safety_overrides.append({"reason": "LLM debate requested but provider/key unavailable"})
            return fallback
        except Exception as exc:
            fallback = _heuristic_debate(user_intent, context, candidate_actions)
            fallback.debate_mode = "heuristic_fallback_llm_error"
            fallback.safety_overrides.append({"reason": "LLM debate failed", "error": str(exc)})
            return fallback
    return _heuristic_debate(user_intent, context, candidate_actions)


def _aggregate_llm_votes(
    user_intent: str,
    context: Dict[str, Any],
    candidate_actions: List[str],
    council_votes: List[CouncilVote],
) -> DebateDecision:
    votes = {vote.agent.lower().replace("agent", ""): vote.vote for vote in council_votes}
    risk_order = {"low": 0, "medium": 1, "high": 2}
    selected = _weighted_action(council_votes, candidate_actions)
    risk = max((vote.risk_level for vote in council_votes), key=lambda r: risk_order.get(r, 1), default="medium")
    requires_human = any(v.requires_human_interrupt or v.vote in {"ask_human", "veto"} for v in council_votes)
    confidence = _average([v.confidence for v in council_votes])
    fallback_plan = _merge_fallbacks(council_votes) or ["stop_robot"]
    spoken = _spoken_from_votes(council_votes, requires_human)
    decision = DebateDecision(
        decision_id="debate_" + uuid.uuid4().hex[:12],
        user_intent=user_intent,
        candidate_actions=candidate_actions,
        context_used=context,
        votes=votes,
        final_action=selected,
        confidence=confidence,
        risk_level=risk,
        requires_human_interrupt=requires_human,
        fallback_plan=fallback_plan,
        spoken_response=spoken,
        debate_mode="llm_multi_agent",
        council_votes=[v.to_dict() for v in council_votes],
    )
    _apply_hard_safety_rules(decision, context)
    return decision


def _weighted_action(council_votes: List[CouncilVote], candidate_actions: List[str]) -> str:
    allowed = set(candidate_actions) | {"stop_robot", "recover_localization", "recover_nav_failure", "query_memory", "speak"}
    scores: Dict[str, float] = {}
    for vote in council_votes:
        action = vote.final_action if vote.final_action in allowed else (candidate_actions[0] if candidate_actions else "stop_robot")
        weight = vote.confidence
        if vote.vote == "approve":
            weight *= 1.0
        elif vote.vote == "recover_first":
            action = "recover_localization"
            weight *= 1.2
        elif vote.vote == "veto":
            action = "stop_robot"
            weight *= 1.5
        elif vote.vote == "ask_human":
            weight *= 0.8
        else:
            weight *= 0.3
        scores[action] = scores.get(action, 0.0) + weight
    if not scores:
        return candidate_actions[0] if candidate_actions else "clarify"
    return max(scores.items(), key=lambda item: item[1])[0]


def _average(values: List[float]) -> float:
    if not values:
        return 0.5
    return max(0.1, min(1.0, sum(values) / len(values)))


def _merge_fallbacks(council_votes: List[CouncilVote]) -> List[str]:
    merged: List[str] = []
    for vote in council_votes:
        for step in vote.fallback_plan:
            if step not in merged:
                merged.append(step)
    return merged[:5]


def _spoken_from_votes(council_votes: List[CouncilVote], requires_human: bool) -> str:
    if requires_human:
        return "The debate council found non-trivial risk. I need approval or more context before motion."
    top_rationales = [v.rationale for v in council_votes if v.rationale][:2]
    if top_rationales:
        return "The debate council approves the next safe step: " + " | ".join(top_rationales)
    return "The debate council approves the next safe step."


def _heuristic_debate(user_intent: str, context: Dict[str, Any], candidate_actions: List[str]) -> DebateDecision:
    localization = float(context.get("localization_confidence", 0.0) or 0.0)
    safety_blocked = bool(context.get("safety_blocked", False))
    front_clearance = _optional_float(context.get("front_clearance_m"))
    risk = "low"
    final_action = candidate_actions[0] if candidate_actions else "clarify"
    requires_human = False
    votes = {
        "navigator": "approve" if candidate_actions else "abstain",
        "localizer": "approve" if localization >= 0.55 or final_action in ("speak", "query_memory", "stop_robot") else "recover_first",
        "safety": "veto" if safety_blocked else "approve",
        "perception": "veto" if front_clearance is not None and front_clearance < 0.45 and _is_motion(final_action) else "approve",
        "semantic_memory": "approve",
        "tour_guide": "approve" if "tour" in user_intent.lower() or final_action != "speak" else "abstain",
        "systems": "approve",
    }
    if safety_blocked:
        final_action = "stop_robot"
        risk = "high"
        requires_human = True
    elif front_clearance is not None and front_clearance < 0.45 and _is_motion(final_action):
        final_action = "stop_robot"
        risk = "high"
        requires_human = True
    elif localization < 0.35 and _is_motion(final_action):
        final_action = "recover_localization"
        risk = "medium"
        requires_human = True
    return DebateDecision(
        decision_id="debate_" + uuid.uuid4().hex[:12],
        user_intent=user_intent,
        candidate_actions=candidate_actions,
        context_used=context,
        votes=votes,
        final_action=final_action,
        confidence=max(0.1, min(1.0, localization if _is_motion(final_action) else 0.75)),
        risk_level=risk,
        requires_human_interrupt=requires_human,
        fallback_plan=["stop_robot", "return_to_spawn"] if risk != "low" else ["stop_robot"],
        spoken_response="I will proceed safely." if not requires_human else "I need help before moving safely.",
        debate_mode="heuristic",
        council_votes=[],
    )


def _apply_hard_safety_rules(decision: DebateDecision, context: Dict[str, Any]) -> None:
    localization = float(context.get("localization_confidence", 0.0) or 0.0)
    safety_blocked = bool(context.get("safety_blocked", False))
    front_clearance = _optional_float(context.get("front_clearance_m"))
    if safety_blocked:
        decision.final_action = "stop_robot"
        decision.risk_level = "high"
        decision.requires_human_interrupt = True
        decision.confidence = min(decision.confidence, 0.65)
        decision.safety_overrides.append({"rule": "safety_blocked", "action": "stop_robot"})
    if front_clearance is not None and front_clearance < 0.45 and _is_motion(decision.final_action):
        decision.final_action = "stop_robot"
        decision.risk_level = "high"
        decision.requires_human_interrupt = True
        decision.safety_overrides.append({"rule": "front_clearance_lt_0_45m", "front_clearance_m": front_clearance})
    if localization < 0.35 and _is_motion(decision.final_action):
        decision.final_action = "recover_localization"
        decision.risk_level = "medium"
        decision.requires_human_interrupt = True
        decision.safety_overrides.append({"rule": "localization_confidence_lt_0_35", "localization_confidence": localization})
    if decision.final_action not in set(decision.candidate_actions) | {"stop_robot", "recover_localization", "recover_nav_failure", "query_memory", "speak", "clarify"}:
        decision.safety_overrides.append({"rule": "invalid_final_action", "requested": decision.final_action})
        decision.final_action = decision.candidate_actions[0] if decision.candidate_actions else "clarify"


def _optional_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _is_motion(action: str) -> bool:
    action = str(action or "")
    return action.startswith(MOTION_PREFIXES) or action in {"return_to_spawn", "navigate_to_pose", "navigate_through_poses"}
