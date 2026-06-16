from __future__ import annotations

from typing import Any, Dict

from go2_memory_core import UnifiedMemoryAPI


class MemoryTools:
    def __init__(self, session_root: str, session_name: str):
        self.session_name = session_name
        self.api = UnifiedMemoryAPI(
            session_root=session_root,
            enable_graph_memory=True,
            enable_vector_memory=True,
            enable_voxel_memory=True,
        )

    def get_resume_context(self) -> Dict[str, Any]:
        return self.api.get_resume_context(self.session_name, {})

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

    def query(self, text: str) -> Dict[str, Any]:
        return self.api.query_memory(self.session_name, {"type": "all", "text": text, "limit": 5})
