from __future__ import annotations

from typing import Any, Dict

from go2_memory_core import UnifiedMemoryAPI
from go2_memory_core.session_resolution import resolve_semantic_session_name


class MemoryTools:
    def __init__(self, session_root: str, session_name: str):
        self.session_name = resolve_semantic_session_name(session_root, session_name)
        self.api = UnifiedMemoryAPI(
            session_root=session_root,
            enable_graph_memory=True,
            enable_vector_memory=True,
            enable_voxel_memory=True,
        )

    def get_resume_context(self) -> Dict[str, Any]:
        context = self.api.get_resume_context(self.session_name, {})
        context["world_memory"] = self.world_snapshot(limit=100)
        return context

    def save_decision(self, decision: Dict[str, Any]) -> None:
        self.api.store.append_jsonl(self.session_name, "memory/decisions.jsonl", decision)

    def write_place(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_place(self.session_name, payload)

    def write_checkpoint(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_checkpoint(self.session_name, payload)

    def connect_locations(self, from_id: str, to_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.connect_locations(self.session_name, from_id, to_id, payload)

    def promote_memory(self, memory_id: str, reason: str = "agent-approved") -> Dict[str, Any]:
        return self.api.promote_memory(self.session_name, memory_id, reason)

    def query_graph(self, query: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.query_graph(self.session_name, query)

    def write_room(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_room(self.session_name, payload)

    def write_object_observation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_object_observation(self.session_name, payload)

    def upsert_object_instance(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.upsert_object_instance(self.session_name, payload)

    def query_objects(self, label: str = "", room: str = "", confirmed_only: bool = True, limit: int = 100) -> Dict[str, Any]:
        return self.api.query_objects(self.session_name, label=label, room=room, confirmed_only=confirmed_only, limit=limit)

    def count_objects(self, label: str = "", room: str = "", confirmed_only: bool = True) -> Dict[str, Any]:
        return self.api.count_objects(self.session_name, label=label, room=room, confirmed_only=confirmed_only)

    def write_fact(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_fact(self.session_name, payload)

    def write_tour_stop(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_tour_stop(self.session_name, payload)

    def cache_web_result(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.cache_web_result(self.session_name, payload)

    def world_snapshot(self, limit: int = 100) -> Dict[str, Any]:
        return self.api.world_snapshot(self.session_name, limit=limit)

    def query(self, text: str = "", limit: int = 20) -> Dict[str, Any]:
        result = self.api.query_memory(self.session_name, {"type": "all", "text": text, "limit": limit})
        world = self.world_snapshot(limit=max(limit, 50))
        result.update(world)
        result["world_memory"] = world
        return result
