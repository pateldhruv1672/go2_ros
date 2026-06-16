from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
from datetime import datetime, timezone
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
        self.path.write_text(json.dumps(state, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")

    def load(self) -> Dict[str, Any]:
        if not self.path.exists():
            return {}
        return json.loads(self.path.read_text(encoding="utf-8"))


class NativeLangGraphStoreAdapter:
    """Durable LangGraph Store wrapper with a SQLite mirror for ROS queries.

    This adapter uses LangGraph's native SQLite Store as the primary long-term
    memory implementation when available. A small local mirror is maintained so
    ROS topic queries remain stable across LangGraph version changes and so the
    user can inspect memory with sqlite3. If native store creation fails and
    require_native=True, the supervisor fails loudly instead of silently using a
    fake store.
    """

    def __init__(self, db_path: Path, require_native: bool = True):
        self.db_path = Path(db_path).resolve()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self._conn.execute(
            """
            CREATE TABLE IF NOT EXISTS kv_store_mirror (
                namespace TEXT NOT NULL,
                key TEXT NOT NULL,
                value_json TEXT NOT NULL,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                PRIMARY KEY(namespace, key)
            )
            """
        )
        self._conn.execute("CREATE INDEX IF NOT EXISTS idx_kv_mirror_namespace_updated ON kv_store_mirror(namespace, updated_at)")
        self._conn.commit()
        self.native_store = self._make_native_store(require_native=require_native)
        self.mode = "native_sqlite" if self.native_store is not None else "sqlite_mirror_only"

    def _make_native_store(self, require_native: bool) -> Any:
        errors: List[str] = []
        try:
            from langgraph.store.sqlite import SqliteStore  # type: ignore
        except Exception as exc:
            errors.append(f"import langgraph.store.sqlite.SqliteStore failed: {exc}")
            SqliteStore = None  # type: ignore
        if SqliteStore is not None:  # type: ignore[truthy-function]
            constructors = [
                lambda: SqliteStore.from_conn_string(str(self.db_path)),
                lambda: SqliteStore(sqlite3.connect(str(self.db_path), check_same_thread=False)),
                lambda: SqliteStore(str(self.db_path)),
            ]
            for ctor in constructors:
                try:
                    store = ctor()
                    setup = getattr(store, "setup", None)
                    if callable(setup):
                        setup()
                    return store
                except Exception as exc:
                    errors.append(str(exc))
        if require_native:
            raise LangGraphDependencyError(
                "Native LangGraph SQLite Store is required but could not be created. "
                "Install/update with: python -m pip install -U langgraph langgraph-checkpoint-sqlite. "
                f"Errors: {' | '.join(errors[-3:])}"
            )
        try:
            from langgraph.store.memory import InMemoryStore  # type: ignore
            return InMemoryStore()
        except Exception:
            return None

    def _namespace_tuple(self, namespace: Iterable[str] | str) -> tuple[str, ...]:
        if isinstance(namespace, str):
            parts = [part for part in namespace.replace(".", "/").split("/") if part]
        else:
            parts = [str(part) for part in namespace]
        return tuple(parts or ["default"])

    def _namespace_str(self, namespace: Iterable[str] | str) -> str:
        return "/".join(self._namespace_tuple(namespace))

    def put(self, namespace: Iterable[str] | str, key: str, value: Dict[str, Any]) -> None:
        ns_tuple = self._namespace_tuple(namespace)
        ns = "/".join(ns_tuple)
        if self.native_store is not None:
            self.native_store.put(ns_tuple, str(key), value)
        now = datetime.now(timezone.utc).isoformat()
        payload = json.dumps(value, sort_keys=True, default=str)
        self._conn.execute(
            """
            INSERT INTO kv_store_mirror(namespace, key, value_json, created_at, updated_at)
            VALUES(?, ?, ?, ?, ?)
            ON CONFLICT(namespace, key) DO UPDATE SET
                value_json=excluded.value_json,
                updated_at=excluded.updated_at
            """,
            (ns, str(key), payload, now, now),
        )
        self._conn.commit()

    def get(self, namespace: Iterable[str] | str, key: str) -> Optional[Dict[str, Any]]:
        ns_tuple = self._namespace_tuple(namespace)
        if self.native_store is not None:
            try:
                item = self.native_store.get(ns_tuple, str(key))
                if item is not None:
                    value = getattr(item, "value", None)
                    return value if isinstance(value, dict) else {"value": value}
            except Exception:
                pass
        row = self._conn.execute(
            "SELECT value_json FROM kv_store_mirror WHERE namespace=? AND key=?",
            ("/".join(ns_tuple), str(key)),
        ).fetchone()
        if not row:
            return None
        try:
            return json.loads(row[0])
        except Exception:
            return {"raw": row[0]}

    def search(self, namespace_prefix: Iterable[str] | str, query: str = "", limit: int = 20) -> List[Dict[str, Any]]:
        ns_tuple = self._namespace_tuple(namespace_prefix)
        if self.native_store is not None:
            try:
                items = self.native_store.search(ns_tuple, query=query or None, limit=int(limit))
                converted = [self._item_to_dict(item) for item in items]
                if converted:
                    return converted
            except Exception:
                pass
        prefix = "/".join(ns_tuple)
        like_ns = prefix + "%"
        q = f"%{query.lower()}%"
        rows = self._conn.execute(
            """
            SELECT namespace, key, value_json, created_at, updated_at
            FROM kv_store_mirror
            WHERE namespace LIKE ?
              AND (? = '%%' OR lower(key) LIKE ? OR lower(value_json) LIKE ?)
            ORDER BY updated_at DESC
            LIMIT ?
            """,
            (like_ns, q, q, q, int(limit)),
        ).fetchall()
        out: List[Dict[str, Any]] = []
        for ns, key, value_json, created_at, updated_at in rows:
            try:
                value = json.loads(value_json)
            except Exception:
                value = {"raw": value_json}
            out.append({"namespace": ns, "key": key, "value": value, "created_at": created_at, "updated_at": updated_at, "source": "sqlite_mirror"})
        return out

    def list_namespaces(self, limit: int = 100) -> List[str]:
        rows = self._conn.execute(
            "SELECT DISTINCT namespace FROM kv_store_mirror ORDER BY namespace LIMIT ?",
            (int(limit),),
        ).fetchall()
        return [str(row[0]) for row in rows]

    def append_event(self, thread_id: str, event: Dict[str, Any]) -> None:
        run_id = str(event.get("run_id") or event.get("data", {}).get("run_id") or "unknown")
        key = f"{datetime.now(timezone.utc).isoformat()}_{run_id}_{abs(hash(json.dumps(event, sort_keys=True, default=str))) % 1000000}"
        self.put(("threads", thread_id, "events"), key, event)

    def _item_to_dict(self, item: Any) -> Dict[str, Any]:
        namespace = getattr(item, "namespace", None)
        key = getattr(item, "key", None)
        value = getattr(item, "value", None)
        created_at = getattr(item, "created_at", None)
        updated_at = getattr(item, "updated_at", None)
        return {
            "namespace": "/".join(namespace) if isinstance(namespace, (tuple, list)) else str(namespace),
            "key": str(key),
            "value": value if isinstance(value, dict) else {"value": value},
            "created_at": str(created_at) if created_at is not None else None,
            "updated_at": str(updated_at) if updated_at is not None else None,
            "source": "langgraph_native_store",
        }

    def close(self) -> None:
        try:
            close = getattr(self.native_store, "close", None)
            if callable(close):
                close()
        except Exception:
            pass
        try:
            self._conn.close()
        except Exception:
            pass


# Backwards-compatible name used by older tests/imports.
SQLiteAgentStore = NativeLangGraphStoreAdapter


class LangGraphSQLitePersistence:
    """SQLite-backed LangGraph checkpoint manager plus native durable Store."""

    def __init__(
        self,
        session_root: str,
        session_name: str,
        db_path: Optional[str] = None,
        store_path: Optional[str] = None,
        require_native_store: bool = True,
    ):
        root = Path(os.path.expanduser(session_root)).resolve() / session_name / "langgraph"
        root.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.db_path = Path(os.path.expanduser(db_path)).resolve() if db_path else root / "checkpoints.sqlite"
        self.store_path = Path(os.path.expanduser(store_path)).resolve() if store_path else root / "store.sqlite"
        self.summary_path = root / "latest_agent_summary.json"
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self._checkpointer = self._make_sqlite_saver(self._conn)
        self.store = NativeLangGraphStoreAdapter(self.store_path, require_native=require_native_store)
        try:
            setup = getattr(self._checkpointer, "setup", None)
            if callable(setup):
                setup()
        except Exception:
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
            "checkpoint_db": str(self.db_path),
            "store_db": str(self.store_path),
            "store_mode": getattr(self.store, "mode", "unknown"),
        }
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
        thread_id = str(state.get("thread_id") or "default")
        run_id = str(state.get("run_id") or datetime.now(timezone.utc).timestamp())
        self.store.put(("threads", thread_id, "runs"), run_id, summary)
        if state.get("history"):
            self.store.put(("threads", thread_id, "history"), run_id, {"history": state.get("history")})
        if state.get("decision"):
            self.store.put(("threads", thread_id, "decisions"), run_id, state.get("decision") or {})

    def list_checkpoints(self, thread_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        config = {"configurable": {"thread_id": thread_id}}
        rows: List[Dict[str, Any]] = []
        try:
            iterator = self._checkpointer.list(config, limit=limit)
        except TypeError:
            iterator = self._checkpointer.list(config)
        except Exception as exc:
            return [{"error": str(exc)}]
        try:
            for idx, item in enumerate(iterator):
                if idx >= limit:
                    break
                cfg = getattr(item, "config", None)
                metadata = getattr(item, "metadata", None)
                parent = getattr(item, "parent_config", None)
                checkpoint_id = (((cfg or {}).get("configurable") or {}).get("checkpoint_id") if isinstance(cfg, dict) else None)
                rows.append({
                    "config": cfg,
                    "metadata": metadata,
                    "parent_config": parent,
                    "checkpoint_id": checkpoint_id,
                })
        except Exception as exc:
            rows.append({"error": str(exc)})
        return rows

    def get_state(self, graph: Any, thread_id: str, checkpoint_id: Optional[str] = None) -> Dict[str, Any]:
        config: Dict[str, Any] = {"configurable": {"thread_id": thread_id}}
        if checkpoint_id:
            config["configurable"]["checkpoint_id"] = checkpoint_id
        try:
            snapshot = graph.get_state(config)
        except Exception as exc:
            return {"error": str(exc), "thread_id": thread_id, "checkpoint_id": checkpoint_id}
        values = getattr(snapshot, "values", None)
        next_nodes = getattr(snapshot, "next", None)
        tasks = getattr(snapshot, "tasks", None)
        return {
            "thread_id": thread_id,
            "checkpoint_id": checkpoint_id,
            "values": values,
            "next": next_nodes,
            "tasks": [str(t) for t in tasks] if tasks is not None else [],
        }

    def close(self) -> None:
        try:
            self.store.close()
        except Exception:
            pass
        try:
            self._conn.close()
        except Exception:
            pass
