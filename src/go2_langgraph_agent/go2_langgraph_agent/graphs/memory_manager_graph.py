from __future__ import annotations

from typing import Any, Dict, Optional

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph


GRAPH_NAME = "memory_manager_graph"


def _latest_pose(context: Dict[str, Any]) -> Dict[str, Any]:
    latest = context.get("latest_checkpoint") or {}
    data = latest.get("data") or {}
    if data.get("map_pose"):
        return data["map_pose"]
    spawn = context.get("spawn") or {}
    spawn_data = spawn.get("data") or spawn
    return spawn_data.get("map_pose") or {"frame_id": "map", "x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


class MemoryManagerGraph:
    """Memory read/write/promote subgraph.

    This graph is intentionally side-effect capable: it writes user-approved
    place names and records decisions through MemoryTools. It never commands
    motion directly.
    """

    def __init__(self, memory_tools: Any):
        self.memory = memory_tools
        self.graph = try_build_langgraph(
            GRAPH_NAME,
            ["refresh_context", "resolve_memory_intent", "execute_memory_op", "legacy_export_hint"],
            {
                "refresh_context": self.refresh_context,
                "resolve_memory_intent": self.resolve_memory_intent,
                "execute_memory_op": self.execute_memory_op,
                "legacy_export_hint": self.legacy_export_hint,
            },
        )

    def invoke(self, state: AgentState) -> AgentState:
        return self.graph.invoke(state)

    def refresh_context(self, state: AgentState) -> Dict[str, Any]:
        context = self.memory.get_resume_context()
        context.update(state.get("context") or {})
        append_event(state, GRAPH_NAME, "context_refreshed", {"graph_summary": context.get("graph_summary")})
        return {"context": context}

    def resolve_memory_intent(self, state: AgentState) -> Dict[str, Any]:
        parsed = state.get("parsed_intent") or {}
        intent = parsed.get("intent", "chat")
        entities = parsed.get("entities") or {}
        op: Dict[str, Any] = {"type": "none"}
        if intent == "save_place":
            op = {
                "type": "write_place",
                "name": entities.get("place_name") or "saved place",
                "verified": True,
                "layer": "permanent",
            }
        elif intent == "remember":
            op = {
                "type": "write_place",
                "name": entities.get("place_name") or entities.get("normalized_text") or "remembered note",
                "description": entities.get("raw_text", ""),
                "verified": False,
                "layer": "temporary",
            }
        elif intent in {"query_memory", "tour_question", "explain_state"}:
            op = {"type": "query", "text": entities.get("raw_text") or state.get("text", "")}
        elif "write_checkpoint" in (state.get("candidate_actions") or []):
            op = {"type": "checkpoint_hint"}
        append_event(state, GRAPH_NAME, "memory_op_resolved", op)
        return {"memory_operation": op}

    def execute_memory_op(self, state: AgentState) -> Dict[str, Any]:
        op = state.get("memory_operation") or {"type": "none"}
        context = state.get("context") or {}
        result: Optional[Dict[str, Any]] = None
        speech = state.get("speech_response", "")
        if op.get("type") == "write_place":
            payload = {
                "name": op.get("name"),
                "aliases": [op.get("name")],
                "description": op.get("description") or f"User saved this location as {op.get('name')}.",
                "verified": bool(op.get("verified", False)),
                "layer": op.get("layer", "temporary"),
                "map_pose": _latest_pose(context),
                "source": ["user", "agent"],
                "confidence": {"memory_confidence": 0.95 if op.get("verified") else 0.65},
            }
            result = self.memory.write_place(payload)
            speech = f"Saved this location as {op.get('name')} in {op.get('layer', 'temporary')} memory."
        elif op.get("type") == "query":
            # SPARKY_OBJECT_AWARE_MEMORY_SPEECH_V2
            result = self.memory.query(op.get("text", ""))
            if isinstance(result, dict):
                places = result.get("places") or []
                checkpoints = result.get("checkpoints") or []
                vlm = result.get("vlm_checkpoints") or []
                world = result.get("world_memory") or result
                objects = world.get("objects") or []
                tour_stops = world.get("tour_stops") or []
                object_counts = {}
                for record in objects:
                    data = (record or {}).get("data") or {}
                    if not bool(data.get("confirmed")) or not bool(data.get("countable", True)):
                        continue
                    if str(data.get("status") or "present") not in {"present", "unknown", "seen"}:
                        continue
                    label = str(data.get("label") or "object")
                    object_counts[label] = object_counts.get(label, 0) + 1
                object_total = sum(object_counts.values())
                top = ", ".join(f"{n} {name}" for name, n in sorted(object_counts.items(), key=lambda kv: (-kv[1], kv[0]))[:8])
                place_count = len({str(r.get("id") or i) for i, r in enumerate(places)})
                checkpoint_count = len({str(r.get("id") or i) for i, r in enumerate(checkpoints)})
                vlm_count = len({str(r.get("id") or i) for i, r in enumerate(vlm)})
                tour_count = len({str(r.get("id") or i) for i, r in enumerate(tour_stops)})
                speech = f"I remember {object_total} confirmed mapped objects" + (f" ({top})" if top else "")
                speech += f", {place_count} saved semantic places, {checkpoint_count} background checkpoints, {vlm_count} VLM checkpoints, and {tour_count} unified-memory tour stops."
            else:
                speech = "I could not read the saved session memory."
        elif op.get("type") == "checkpoint_hint":
            append_event(state, GRAPH_NAME, "checkpoint_requested", {"reason": "explore_or_tour_progress"})
        if result:
            append_event(state, GRAPH_NAME, "memory_op_executed", {"type": op.get("type"), "success": result.get("success", True)})
            return {"memory_result": result, "speech_response": speech, "memory_updates": [{"operation": op, "result": result}]}
        return {"speech_response": speech}

    def legacy_export_hint(self, state: AgentState) -> Dict[str, Any]:
        # UnifiedMemoryAPI exports legacy session files after writes. This event
        # makes the contract visible in the agent trace.
        if (state.get("memory_operation") or {}).get("type") in {"write_place", "checkpoint_hint"}:
            append_event(state, GRAPH_NAME, "legacy_export_covered", {"files": ["places.yaml", "session.yaml", "spawn.yaml"]})
        return {}


def build_graph(memory_tools: Any) -> MemoryManagerGraph:
    return MemoryManagerGraph(memory_tools)
