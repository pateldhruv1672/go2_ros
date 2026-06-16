from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, MutableMapping, Optional
from typing_extensions import Annotated, TypedDict
import copy
import json
import operator
import re
import uuid


class Go2AgentState(TypedDict, total=False):
    """State carried by the real LangGraph runtime.

    Reducers are used for append-only traces so checkpoints preserve every node
    update instead of replacing the event/history streams on each step.
    """

    run_id: str
    thread_id: str
    raw_command: str
    text: str
    parsed_intent: Dict[str, Any]
    candidate_actions: List[str]
    context: Dict[str, Any]
    mode: str
    feature_flags: Dict[str, bool]
    decision: Dict[str, Any]
    motion_gate: Dict[str, Any]
    route: List[Dict[str, Any]]
    nav_command: Dict[str, Any]
    speech_response: str
    status: Dict[str, Any]
    memory_operation: Dict[str, Any]
    memory_result: Dict[str, Any]
    goal_selection: Dict[str, Any]
    selected_goal: Dict[str, Any]
    explore_strategy: str
    explore_plan: List[Dict[str, Any]]
    recovery_issue: str
    recovery_plan: List[str]
    tour_memory: Dict[str, Any]
    current_tour_stop: Dict[str, Any]
    tour_route: List[Dict[str, Any]]
    pending_interrupt: Dict[str, Any]
    human_approval: Dict[str, Any]
    error: Dict[str, Any]
    history: Annotated[List[Dict[str, Any]], operator.add]
    events: Annotated[List[Dict[str, Any]], operator.add]
    memory_updates: Annotated[List[Dict[str, Any]], operator.add]


AgentState = Go2AgentState


MOTION_ACTIONS = {
    "navigate_to_pose",
    "navigate_through_poses",
    "navigate_tour_route",
    "return_to_spawn",
    "frontier_explore",
    "coverage_explore",
    "manual_assisted_explore",
}


@dataclass
class SubgraphEvent:
    graph: str
    event: str
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    def to_dict(self) -> Dict[str, Any]:
        return {
            "graph": self.graph,
            "event": self.event,
            "data": self.data,
            "timestamp": self.timestamp,
        }


class LangGraphDependencyError(RuntimeError):
    pass


def require_langgraph() -> None:
    try:
        import langgraph  # noqa: F401
    except Exception as exc:  # pragma: no cover - exercised on robot if dependency missing
        raise LangGraphDependencyError(
            "go2_langgraph_agent now requires real LangGraph. Install it in the workspace venv with: "
            "python -m pip install -U langgraph langgraph-checkpoint-sqlite langchain-core"
        ) from exc


def build_required_langgraph(graph_name: str, node_order: List[str], node_funcs: Dict[str, Any]) -> Any:
    """Build a real LangGraph StateGraph.

    No dependency-free fallback is used here. If LangGraph is missing, the robot
    should fail loudly instead of pretending to run a LangGraph agent.
    """

    require_langgraph()
    from langgraph.graph import END, START, StateGraph  # type: ignore

    builder = StateGraph(Go2AgentState)
    for name in node_order:
        builder.add_node(name, node_funcs[name])
    previous = START
    for name in node_order:
        builder.add_edge(previous, name)
        previous = name
    builder.add_edge(previous, END)
    return builder.compile()


# Backwards-compatible symbol used by existing subgraph modules. It now builds
# real LangGraph only and raises if the dependency is missing.
def try_build_langgraph(graph_name: str, node_order: List[str], node_funcs: Dict[str, Any]) -> Any:
    return build_required_langgraph(graph_name, node_order, node_funcs)


def merge_state(base: AgentState, update: MutableMapping[str, Any]) -> AgentState:
    """Deterministic merge used only for compatibility utilities/tests."""

    for key, value in update.items():
        if key in {"events", "memory_updates", "history"}:
            base.setdefault(key, [])
            base[key].extend(value or [])
        elif isinstance(value, dict) and isinstance(base.get(key), dict):
            merged = dict(base[key])
            merged.update(value)
            base[key] = merged
        else:
            base[key] = value
    return base


def append_event(state: AgentState, graph: str, event: str, data: Optional[Dict[str, Any]] = None) -> None:
    # The compiled LangGraph nodes in this package also mutate the state for
    # compatibility with the previous implementation. Main graph checkpoints use
    # reducer-annotated event streams and final status publishes these events.
    state.setdefault("events", []).append(SubgraphEvent(graph, event, data or {}).to_dict())


def event_update(graph: str, event: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {"events": [SubgraphEvent(graph, event, data or {}).to_dict()]}


def extract_text_from_message(raw: str) -> str:
    text = (raw or "").strip()
    if not text:
        return ""
    try:
        payload = json.loads(text)
    except Exception:
        return text
    if isinstance(payload, dict):
        for key in ("text", "command", "utterance", "transcript"):
            value = payload.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip()
    return text


def normalize_command(text: str) -> str:
    return " ".join((text or "").strip().lower().split())


def classify_intent(text: str) -> Dict[str, Any]:
    norm = normalize_command(text)
    if not norm:
        return {"intent": "idle", "entities": {}, "requires_motion": False}

    destination = extract_destination(norm)
    place_name = extract_place_name(norm)
    if any(w in norm for w in ("emergency stop", "e stop", "stop", "freeze")):
        intent = "stop"
    elif "return" in norm and "spawn" in norm:
        intent = "return_to_spawn"
    elif any(p in norm for p in ("start tour", "give")) and "tour" in norm:
        intent = "start_tour"
    elif "tour" in norm:
        intent = "tour_question"
    elif "explore" in norm or "frontier" in norm:
        intent = "explore"
    elif "coverage" in norm or "scan the area" in norm or "cover" in norm:
        intent = "coverage_explore"
    elif destination:
        intent = "navigate"
    elif place_name:
        intent = "save_place"
    elif norm.startswith("remember") or norm.startswith("save"):
        intent = "remember"
    elif "what" in norm and ("remember" in norm or "know" in norm or "saved" in norm):
        intent = "query_memory"
    elif "why" in norm and ("stop" in norm or "stopped" in norm):
        intent = "explain_state"
    elif "continue" in norm or "resume" in norm:
        intent = "continue_task"
    elif "approve" in norm or "yes continue" in norm or "resume motion" in norm:
        intent = "approve_interrupt"
    elif "reject" in norm or "cancel motion" in norm or "do not move" in norm:
        intent = "reject_interrupt"
    else:
        intent = "chat"
    return {
        "intent": intent,
        "entities": {
            "destination": destination,
            "place_name": place_name,
            "raw_text": text,
            "normalized_text": norm,
        },
        "requires_motion": intent in {"navigate", "return_to_spawn", "start_tour", "explore", "coverage_explore", "continue_task"},
    }


def extract_destination(norm: str) -> str:
    patterns = [
        r"(?:go|navigate|take(?: me| guests)?|walk|move|drive) to (.+)",
        r"(?:resume|continue) (?:to|toward) (.+)",
    ]
    for pattern in patterns:
        match = re.search(pattern, norm)
        if match:
            value = _clean_entity(match.group(1))
            if value:
                return value
    return ""


def extract_place_name(norm: str) -> str:
    patterns = [
        r"save this as (.+)",
        r"remember this as (.+)",
        r"mark this as (.+)",
        r"name this (.+)",
    ]
    for pattern in patterns:
        match = re.search(pattern, norm)
        if match:
            value = _clean_entity(match.group(1))
            if value:
                return value
    return ""


def _clean_entity(value: str) -> str:
    value = value.strip(" .!?;:")
    stop_phrases = [" please", " for me", " now"]
    for phrase in stop_phrases:
        if value.endswith(phrase):
            value = value[: -len(phrase)]
    return value.strip(" .!?;:")


def new_run_id(prefix: str = "agent") -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


def summarize_context(context: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "session_name": context.get("session_name"),
        "map_loaded": bool(context.get("map_yaml_path")),
        "has_spawn": bool(context.get("spawn")),
        "latest_checkpoint_id": (context.get("latest_checkpoint") or {}).get("id"),
        "graph_summary": context.get("graph_summary") or {},
        "localization_confidence": context.get("localization_confidence"),
        "safety_blocked": context.get("safety_blocked"),
        "frontier_count": len((context.get("frontier_candidates") or {}).get("candidates", [])) if isinstance(context.get("frontier_candidates"), dict) else 0,
    }


def safe_copy(value: Any) -> Any:
    try:
        return copy.deepcopy(value)
    except Exception:
        return value
