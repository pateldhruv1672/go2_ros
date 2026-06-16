from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, MutableMapping, Optional
import copy
import json
import re
import uuid


AgentState = Dict[str, Any]


MOTION_ACTIONS = {
    "navigate_to_pose",
    "navigate_through_poses",
    "navigate_tour_route",
    "return_to_spawn",
    "frontier_explore",
    "coverage_explore",
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


class SimpleCompiledGraph:
    """Small dependency-free executable graph used when langgraph is unavailable.

    The project can still install real LangGraph later; this runner intentionally
    mirrors the node/edge mental model and keeps deterministic behavior for ROS
    tests and robot bringup where optional Python deps may not be installed.
    """

    def __init__(self, graph_name: str, nodes: List[Any]):
        self.graph_name = graph_name
        self.nodes = nodes

    def invoke(self, state: AgentState) -> AgentState:
        current = copy.deepcopy(state)
        append_event(current, self.graph_name, "start", {"nodes": [getattr(n, "__name__", str(n)) for n in self.nodes]})
        for node in self.nodes:
            update = node(current) or {}
            if update is not current:
                merge_state(current, update)
        append_event(current, self.graph_name, "end", {"status": current.get("status", "ok")})
        return current


def try_build_langgraph(graph_name: str, node_order: List[str], node_funcs: Dict[str, Any]) -> Any:
    """Build a real LangGraph StateGraph if installed, otherwise fallback.

    LangGraph is an optional dependency in this ROS package because many robot
    installs will build from apt/colcon without the PyPI package. The public API
    stays the same: returned object has .invoke(state).
    """

    try:
        from langgraph.graph import END, START, StateGraph  # type: ignore
    except Exception:
        return SimpleCompiledGraph(graph_name, [node_funcs[name] for name in node_order])

    builder = StateGraph(dict)
    for name in node_order:
        builder.add_node(name, node_funcs[name])
    previous = START
    for name in node_order:
        builder.add_edge(previous, name)
        previous = name
    builder.add_edge(previous, END)
    return builder.compile()


def merge_state(base: AgentState, update: MutableMapping[str, Any]) -> AgentState:
    for key, value in update.items():
        if key == "events":
            base.setdefault("events", [])
            base["events"].extend(value or [])
        elif key == "memory_updates":
            base.setdefault("memory_updates", [])
            base["memory_updates"].extend(value or [])
        elif isinstance(value, dict) and isinstance(base.get(key), dict):
            merged = dict(base[key])
            merged.update(value)
            base[key] = merged
        else:
            base[key] = value
    return base


def append_event(state: AgentState, graph: str, event: str, data: Optional[Dict[str, Any]] = None) -> None:
    state.setdefault("events", []).append(SubgraphEvent(graph, event, data or {}).to_dict())


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
    }
