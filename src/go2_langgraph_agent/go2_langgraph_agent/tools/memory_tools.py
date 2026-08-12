from __future__ import annotations

from typing import Any, Dict, Optional

from go2_memory_core import UnifiedMemoryAPI
from go2_memory_core.session_resolution import resolve_semantic_session_name
from .fused_memory_bridge import FusedMemoryBridge, canonical_object_label


class MemoryTools:
    def __init__(self, session_root: str, session_name: str):
        self.session_name = resolve_semantic_session_name(session_root, session_name)
        self.api = UnifiedMemoryAPI(
            session_root=session_root,
            enable_graph_memory=True,
            enable_vector_memory=True,
            enable_voxel_memory=True,
        )
        self.fused = FusedMemoryBridge(self.api, self.session_name)

    def get_resume_context(self) -> Dict[str, Any]:
        context = self.api.get_resume_context(self.session_name, {})
        context["world_memory"] = self.world_snapshot(limit=100)
        context["vlm_checkpoints"] = self.query_vlm_checkpoints(limit=30)
        context["mapper_sql"] = self.fused.query_mapper_objects(label="", confirmed_only=True, limit=25)
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

    def query_objects(
        self,
        label: str = "",
        room: str = "",
        confirmed_only: bool = True,
        limit: int = 100,
        robot_map_pose: Optional[Dict[str, Any]] = None,
        nearest: bool = False,
    ) -> Dict[str, Any]:
        canonical, inferred_nearest = canonical_object_label(label)
        unified = self.api.query_objects(self.session_name, label=canonical, room=room, confirmed_only=confirmed_only, limit=max(limit, 100))
        return self.fused.fuse_object_results(
            unified, canonical, confirmed_only, limit, robot_map_pose=robot_map_pose,
            nearest=bool(nearest or inferred_nearest),
        )

    def count_objects(self, label: str = "", room: str = "", confirmed_only: bool = True) -> Dict[str, Any]:
        result = self.query_objects(label=label, room=room, confirmed_only=confirmed_only, limit=10000)
        counts: Dict[str, int] = {}
        for rec in result.get("objects", []):
            obj_label = str((rec.get("data") or {}).get("label") or "object")
            counts[obj_label] = counts.get(obj_label, 0) + 1
        result["counts"] = counts
        return result

    def query_vlm_checkpoints(self, text: str = "", limit: int = 30):
        return self.fused.vlm_checkpoints(text=text, limit=limit)

    def write_fact(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_fact(self.session_name, payload)

    def write_tour_stop(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.write_tour_stop(self.session_name, payload)

    def cache_web_result(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.api.cache_web_result(self.session_name, payload)

    def world_snapshot(self, limit: int = 100) -> Dict[str, Any]:
        snapshot = self.api.world_snapshot(self.session_name, limit=limit)
        snapshot["mapper_sql"] = self.fused.query_mapper_objects(label="", confirmed_only=True, limit=limit)
        snapshot["vlm_checkpoints"] = self.query_vlm_checkpoints(limit=min(limit, 50))
        return snapshot

    def query(self, text: str = "", limit: int = 20) -> Dict[str, Any]:
        result = self.api.query_memory(self.session_name, {"type": "all", "text": text, "limit": limit})
        result["vlm_checkpoints"] = self.query_vlm_checkpoints(text=text, limit=max(limit, 30))
        world = self.world_snapshot(limit=max(limit, 50))
        result.update(world)
        result["world_memory"] = world
        return result
