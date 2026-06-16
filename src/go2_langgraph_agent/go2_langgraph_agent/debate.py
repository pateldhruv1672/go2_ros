from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List
import uuid


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

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def run_debate(user_intent: str, context: Dict[str, Any], candidate_actions: List[str]) -> DebateDecision:
    localization = float(context.get('localization_confidence', 0.0) or 0.0)
    safety_blocked = bool(context.get('safety_blocked', False))
    risk = 'low'
    final_action = candidate_actions[0] if candidate_actions else 'clarify'
    requires_human = False
    votes = {
        'navigator': 'approve' if candidate_actions else 'abstain',
        'localizer': 'approve' if localization >= 0.55 or final_action in ('speak', 'query_memory', 'stop_robot') else 'recover_first',
        'safety': 'veto' if safety_blocked else 'approve',
        'perception': 'approve',
        'semantic_memory': 'approve',
        'tour_guide': 'approve' if 'tour' in user_intent.lower() or final_action != 'speak' else 'abstain',
        'systems': 'approve',
    }
    if safety_blocked:
        final_action = 'stop_robot'
        risk = 'high'
        requires_human = True
    elif localization < 0.35 and final_action.startswith('navigate'):
        final_action = 'recover_localization'
        risk = 'medium'
        requires_human = True
    return DebateDecision(
        decision_id='debate_' + uuid.uuid4().hex[:12],
        user_intent=user_intent,
        candidate_actions=candidate_actions,
        context_used=context,
        votes=votes,
        final_action=final_action,
        confidence=max(0.1, min(1.0, localization if final_action.startswith('navigate') else 0.75)),
        risk_level=risk,
        requires_human_interrupt=requires_human,
        fallback_plan=['stop_robot', 'return_to_spawn'] if risk != 'low' else ['stop_robot'],
        spoken_response='I will proceed safely.' if not requires_human else 'I need help before moving safely.',
    )
