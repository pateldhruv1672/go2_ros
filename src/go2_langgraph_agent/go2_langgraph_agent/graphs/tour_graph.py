from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph
from go2_langgraph_agent.tools.tour_tools import grounded_tour_response

GRAPH_NAME = "tour_graph"


class TourGraph:
    """Grounded tour guide using verified world memory + live perception."""

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
        context["world_memory"] = memory_result.get("world_memory") or {}
        append_event(state, GRAPH_NAME, "tour_context_loaded", {
            "places": len(memory_result.get("places", [])),
            "checkpoints": len(memory_result.get("checkpoints", [])),
            "objects": len(memory_result.get("objects", [])),
            "facts": len(memory_result.get("facts", [])),
        })
        return {"context": context, "tour_memory": memory_result}

    def select_stop(self, state: AgentState) -> Dict[str, Any]:
        memory_result = state.get("tour_memory") or {}
        context = state.get("context") or {}
        event = context.get("semantic_nav_event") or {}
        stop_name = str(event.get("stop_name") or event.get("place") or "").strip().lower()
        candidates = (memory_result.get("tour_stops") or []) + (memory_result.get("rooms") or []) + (memory_result.get("places") or [])
        current_stop = None
        if stop_name:
            for rec in candidates:
                data = rec.get("data") or {}
                names = [str(data.get("name") or data.get("label") or rec.get("id") or "")]
                names.extend(str(x) for x in data.get("aliases", []) or [])
                if any(stop_name == n.lower().strip() or stop_name in n.lower() for n in names if n):
                    current_stop = rec
                    break
        if current_stop is None and candidates:
            current_stop = candidates[-1]
        append_event(state, GRAPH_NAME, "stop_selected", {
            "stop_id": (current_stop or {}).get("id"), "stop_name": stop_name,
        })
        return {"current_tour_stop": current_stop or {}}

    def compose_grounded_speech(self, state: AgentState) -> Dict[str, Any]:
        context = dict(state.get("context") or {})
        context["tour_memory"] = state.get("tour_memory") or {}
        context["current_tour_stop"] = state.get("current_tour_stop") or {}
        question = state.get("text", "") if "?" in state.get("text", "") else ""
        speech = grounded_tour_response(context, question=question, use_ollama=True)
        append_event(state, GRAPH_NAME, "speech_composed", {"length": len(speech), "grounding": "world+live"})
        return {"speech_response": speech}

    def make_tour_route(self, state: AgentState) -> Dict[str, Any]:
        memory_result = state.get("tour_memory") or {}
        route: List[Dict[str, Any]] = []
        source = memory_result.get("tour_stops") or memory_result.get("places") or []
        for record in source[-12:]:
            data = record.get("data") or {}
            pose = data.get("map_pose") or {}
            if not pose:
                continue
            route.append({
                "type": "tour_stop" if record.get("type") == "TourStop" else "place",
                "id": record.get("id"),
                "label": data.get("name") or record.get("id"),
                "map_pose": pose,
                "route_order": data.get("route_order"),
            })
        route.sort(key=lambda r: (r.get("route_order") is None, r.get("route_order") or 9999))
        append_event(state, GRAPH_NAME, "tour_route_built", {"route_length": len(route)})
        return {"tour_route": route}


def build_graph(memory_tools: Any) -> TourGraph:
    return TourGraph(memory_tools)
