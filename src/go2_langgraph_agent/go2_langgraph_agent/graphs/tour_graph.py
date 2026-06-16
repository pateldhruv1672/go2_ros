from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph
from go2_langgraph_agent.tools.tour_tools import grounded_tour_response


GRAPH_NAME = "tour_graph"


class TourGraph:
    """Grounded tour guide subgraph using stored memory plus live context."""

    def __init__(self, memory_tools: Any):
        self.memory = memory_tools
        self.graph = try_build_langgraph(
            GRAPH_NAME,
            ["load_tour_context", "select_stop", "compose_grounded_speech", "make_tour_route"],
            {
                "load_tour_context": self.load_tour_context,
                "select_stop": self.select_stop,
                "compose_grounded_speech": self.compose_grounded_speech,
                "make_tour_route": self.make_tour_route,
            },
        )

    def invoke(self, state: AgentState) -> AgentState:
        return self.graph.invoke(state)

    def load_tour_context(self, state: AgentState) -> Dict[str, Any]:
        query_text = state.get("text", "tour")
        memory_result = self.memory.query(query_text)
        context = dict(state.get("context") or {})
        context["tour_memory"] = memory_result
        append_event(state, GRAPH_NAME, "tour_context_loaded", {
            "places": len(memory_result.get("places", [])),
            "checkpoints": len(memory_result.get("checkpoints", [])),
        })
        return {"context": context, "tour_memory": memory_result}

    def select_stop(self, state: AgentState) -> Dict[str, Any]:
        memory_result = state.get("tour_memory") or {}
        places = memory_result.get("places") or []
        checkpoints = memory_result.get("checkpoints") or []
        current_stop = None
        if places:
            current_stop = places[-1]
        elif checkpoints:
            current_stop = checkpoints[-1]
        append_event(state, GRAPH_NAME, "stop_selected", {"stop_id": (current_stop or {}).get("id")})
        return {"current_tour_stop": current_stop or {}}

    def compose_grounded_speech(self, state: AgentState) -> Dict[str, Any]:
        context = state.get("context") or {}
        stop = state.get("current_tour_stop") or {}
        stop_data = stop.get("data") or {}
        question = state.get("text", "") if "?" in state.get("text", "") else ""
        if stop_data.get("name") or stop_data.get("description"):
            live = context.get("live_observation") or "I do not have a fresh live camera summary yet."
            name = stop_data.get("name") or stop.get("id", "this stop")
            desc = stop_data.get("description") or "This stop was saved in memory."
            speech = f"This area is saved as {name}. Stored memory says: {desc} Live observation: {live}"
            if question:
                speech += f" For your question, I will answer only from stored or visible evidence: {question}"
        else:
            speech = grounded_tour_response(context, question)
        append_event(state, GRAPH_NAME, "speech_composed", {"length": len(speech)})
        return {"speech_response": speech}

    def make_tour_route(self, state: AgentState) -> Dict[str, Any]:
        memory_result = state.get("tour_memory") or {}
        route: List[Dict[str, Any]] = []
        for record in memory_result.get("places", [])[-5:]:
            data = record.get("data") or {}
            route.append({"type": "place", "id": record.get("id"), "label": data.get("name") or record.get("id"), "map_pose": data.get("map_pose") or {}})
        append_event(state, GRAPH_NAME, "tour_route_built", {"route_length": len(route)})
        return {"tour_route": route}


def build_graph(memory_tools: Any) -> TourGraph:
    return TourGraph(memory_tools)
