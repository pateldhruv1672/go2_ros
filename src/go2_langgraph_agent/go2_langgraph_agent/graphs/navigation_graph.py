from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph


GRAPH_NAME = "navigation_graph"


def _graph_route(context: Dict[str, Any], destination: str) -> List[Dict[str, Any]]:
    if not destination:
        return []
    graph_hits = context.get("graph_query") or context.get("memory_query") or {}
    route = graph_hits.get("route") if isinstance(graph_hits, dict) else None
    if route:
        return route
    return [{"type": "destination", "label": destination, "source": "semantic_memory"}]


class NavigationGraph:
    """Navigation planning subgraph with explicit safety gates.

    It accepts concrete selected goals from exploration/tour subgraphs, semantic
    destinations from memory, or spawn poses, then emits commands understood by
    go2_nav_tools.nav2_tool_server.
    """

    def __init__(self, nav_publisher: Any = None):
        self.nav_publisher = nav_publisher
        self.graph = try_build_langgraph(
            GRAPH_NAME,
            ["gate_motion", "resolve_route", "make_nav_command", "record_nav_trace"],
            {
                "gate_motion": self.gate_motion,
                "resolve_route": self.resolve_route,
                "make_nav_command": self.make_nav_command,
                "record_nav_trace": self.record_nav_trace,
            },
        )

    def invoke(self, state: AgentState) -> AgentState:
        return self.graph.invoke(state)

    def gate_motion(self, state: AgentState) -> Dict[str, Any]:
        context = state.get("context") or {}
        parsed = state.get("parsed_intent") or {}
        localization = float(context.get("localization_confidence", 0.0) or 0.0)
        safety_blocked = bool(context.get("safety_blocked", False))
        dyn = context.get("dynamic_obstacles") or {}
        dynamic_blocked = bool(isinstance(dyn, dict) and dyn.get("front_blocked"))
        requires_motion = bool(parsed.get("requires_motion")) or (state.get("nav_command") or {}).get("action") in {"frontier_explore", "coverage_explore", "navigate_to_pose", "navigate_through_poses"}
        gate = {"allowed": True, "reason": "non_motion" if not requires_motion else "ok", "localization_confidence": localization}
        if requires_motion and (safety_blocked or dynamic_blocked):
            gate = {"allowed": False, "reason": "safety_blocked", "localization_confidence": localization, "dynamic_blocked": dynamic_blocked}
        elif requires_motion and localization < 0.35:
            gate = {"allowed": False, "reason": "low_localization", "localization_confidence": localization}
        append_event(state, GRAPH_NAME, "motion_gate", gate)
        return {"motion_gate": gate}

    def resolve_route(self, state: AgentState) -> Dict[str, Any]:
        parsed = state.get("parsed_intent") or {}
        entities = parsed.get("entities") or {}
        intent = parsed.get("intent")
        context = state.get("context") or {}
        existing = state.get("nav_command") or {}
        route: List[Dict[str, Any]] = []
        if existing.get("selected_goal"):
            route = [{"type": "selected_goal", "pose": existing["selected_goal"].get("pose", {}), "source": existing.get("action")}]
        elif existing.get("pose"):
            route = [{"type": "pose", "pose": existing.get("pose"), "source": existing.get("action")}]
        elif intent == "navigate":
            route = _graph_route(context, entities.get("destination", ""))
        elif intent == "return_to_spawn":
            route = [{"type": "spawn", "pose": ((context.get("spawn") or {}).get("data") or {}).get("map_pose") or {}}]
        elif intent in {"start_tour", "continue_task"}:
            route = state.get("tour_route") or []
        append_event(state, GRAPH_NAME, "route_resolved", {"route_length": len(route), "intent": intent})
        return {"route": route}

    def make_nav_command(self, state: AgentState) -> Dict[str, Any]:
        gate = state.get("motion_gate") or {"allowed": True}
        parsed = state.get("parsed_intent") or {}
        intent = parsed.get("intent")
        entities = parsed.get("entities") or {}
        existing = dict(state.get("nav_command") or {})
        if not gate.get("allowed", True):
            action = "recover_localization" if gate.get("reason") == "low_localization" else "stop_robot"
            command = {"action": action, "reason": gate.get("reason"), "gate": gate}
        elif existing.get("action") in {"frontier_explore", "coverage_explore", "manual_assisted_explore", "navigate_to_pose", "navigate_through_poses", "stop_robot"}:
            command = existing
            if state.get("route"):
                command.setdefault("route", state.get("route"))
        elif intent == "stop":
            command = {"action": "stop_robot"}
        elif intent == "return_to_spawn":
            command = {"action": "return_to_spawn", "route": state.get("route", [])}
        elif intent == "navigate":
            command = {"action": "navigate_to_place", "destination": entities.get("destination"), "route": state.get("route", [])}
        elif intent in {"start_tour", "continue_task"} and state.get("route"):
            command = {"action": "navigate_tour_route", "route": state.get("route", [])}
        else:
            command = {"action": "none"}
        append_event(state, GRAPH_NAME, "nav_command_built", command)
        return {"nav_command": command}

    def record_nav_trace(self, state: AgentState) -> Dict[str, Any]:
        command = state.get("nav_command") or {}
        if command.get("action") not in {"none", None}:
            append_event(state, GRAPH_NAME, "nav_trace_recorded", {"action": command.get("action"), "route_length": len(state.get("route") or [])})
        return {}


def build_graph(nav_publisher: Any = None) -> NavigationGraph:
    return NavigationGraph(nav_publisher)
