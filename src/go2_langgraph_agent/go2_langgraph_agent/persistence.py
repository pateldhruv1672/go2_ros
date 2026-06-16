from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
import json
import os
import sqlite3

from go2_langgraph_agent.graphs.agent_state import LangGraphDependencyError


class JsonCheckpointer:
    """Legacy debug checkpointer kept for migration/inspection only."""

    def __init__(self, session_root: str, session_name: str):
        self.path = Path(os.path.expanduser(session_root)).resolve() / session_name / "langgraph" / "checkpoints" / "agent_state.json"
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def save(self, state: Dict[str, Any]) -> None:
        self.path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def load(self) -> Dict[str, Any]:
        if not self.path.exists():
            return {}
        return json.loads(self.path.read_text(encoding="utf-8"))


class LangGraphSQLitePersistence:
    """SQLite-backed LangGraph checkpoint manager plus small sidecar store.

    The checkpointer is the actual LangGraph persistence layer. The sidecar JSON
    file is only a human-readable summary so ROS debugging remains easy.
    """

    def __init__(self, session_root: str, session_name: str, db_path: Optional[str] = None):
        root = Path(os.path.expanduser(session_root)).resolve() / session_name / "langgraph"
        root.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.db_path = Path(os.path.expanduser(db_path)).resolve() if db_path else root / "checkpoints.sqlite"
        self.summary_path = root / "latest_agent_summary.json"
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self._checkpointer = self._make_sqlite_saver(self._conn)
        try:
            setup = getattr(self._checkpointer, "setup", None)
            if callable(setup):
                setup()
        except Exception:
            # Older versions create tables lazily.
            pass

    @property
    def checkpointer(self) -> Any:
        return self._checkpointer

    def _make_sqlite_saver(self, conn: sqlite3.Connection) -> Any:
        try:
            from langgraph.checkpoint.sqlite import SqliteSaver  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise LangGraphDependencyError(
                "Missing langgraph-checkpoint-sqlite. Install with: "
                "python -m pip install -U langgraph-checkpoint-sqlite"
            ) from exc
        return SqliteSaver(conn)

    def write_summary(self, state: Dict[str, Any]) -> None:
        summary = {
            "run_id": state.get("run_id"),
            "thread_id": state.get("thread_id"),
            "text": state.get("text"),
            "parsed_intent": state.get("parsed_intent"),
            "decision": state.get("decision"),
            "motion_gate": state.get("motion_gate"),
            "nav_command": state.get("nav_command"),
            "speech_response": state.get("speech_response"),
            "status": state.get("status"),
            "pending_interrupt": state.get("pending_interrupt"),
            "event_count": len(state.get("events") or []),
        }
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def list_checkpoints(self, thread_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        config = {"configurable": {"thread_id": thread_id}}
        rows: List[Dict[str, Any]] = []
        try:
            for item in self._checkpointer.list(config, limit=limit):
                rows.append({
                    "config": getattr(item, "config", None),
                    "metadata": getattr(item, "metadata", None),
                    "parent_config": getattr(item, "parent_config", None),
                })
        except TypeError:
            # Some versions do not support limit kwarg.
            for idx, item in enumerate(self._checkpointer.list(config)):
                if idx >= limit:
                    break
                rows.append({
                    "config": getattr(item, "config", None),
                    "metadata": getattr(item, "metadata", None),
                    "parent_config": getattr(item, "parent_config", None),
                })
        except Exception as exc:
            rows.append({"error": str(exc)})
        return rows

    def close(self) -> None:
        try:
            self._conn.close()
        except Exception:
            pass
