from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
import json
import time


class GraphStoreKuzu:
    """Persistent graph store with a real Kuzu backend and JSONL mirror.

    The JSONL mirror is always written so the session remains inspectable on
    machines that do not have the optional ``kuzu`` Python wheel installed. When
    ``enable_kuzu=True`` and the wheel is available, nodes and edges are also
    written to an embedded on-disk Kuzu database at ``graph_memory/kuzu_db``.
    """

    NODE_TABLE = "RobotNode"
    EDGE_TABLE = "RobotEdge"

    def __init__(self, session_dir: Path, enable_kuzu: bool = False):
        self.session_dir = Path(session_dir)
        self.graph_dir = self.session_dir / "graph_memory"
        self.graph_dir.mkdir(parents=True, exist_ok=True)
        self.nodes_path = self.graph_dir / "nodes.jsonl"
        self.edges_path = self.graph_dir / "edges.jsonl"
        self.kuzu_available = False
        self.kuzu_error = ""
        self.db = None
        self.conn = None
        if enable_kuzu:
            self._open_kuzu()

    def add_node(self, node: Dict[str, Any]) -> Dict[str, Any]:
        node = dict(node)
        node.setdefault("created_at", self._now())
        self._append_unique(self.nodes_path, node, "id")
        if self.kuzu_available:
            self._upsert_kuzu_node(node)
        return node

    def add_edge(self, edge: Dict[str, Any]) -> Dict[str, Any]:
        edge = dict(edge)
        edge.setdefault("created_at", self._now())
        self._append_unique(self.edges_path, edge, "id")
        if self.kuzu_available:
            self._upsert_kuzu_edge(edge)
        return edge

    def nodes(self, node_type: Optional[str] = None, limit: int = 1000) -> List[Dict[str, Any]]:
        if self.kuzu_available:
            rows = self._query_kuzu_nodes(node_type=node_type, limit=limit)
            if rows:
                return rows
        records = self._read(self.nodes_path)
        if node_type:
            records = [r for r in records if r.get("type") == node_type]
        return records[-limit:]

    def edges(self, edge_type: Optional[str] = None, limit: int = 1000) -> List[Dict[str, Any]]:
        if self.kuzu_available:
            rows = self._query_kuzu_edges(edge_type=edge_type, limit=limit)
            if rows:
                return rows
        records = self._read(self.edges_path)
        if edge_type:
            records = [r for r in records if r.get("type") == edge_type]
        return records[-limit:]

    def neighbors(self, node_id: str, limit: int = 1000) -> List[Dict[str, Any]]:
        if self.kuzu_available:
            rows = self._query_kuzu_neighbors(node_id, limit=limit)
            if rows:
                return rows
        return [
            e for e in self.edges(limit=limit)
            if e.get("from_id") == node_id or e.get("to_id") == node_id
        ][:limit]

    def latest_node(self, node_type: str) -> Dict[str, Any] | None:
        nodes = self.nodes(node_type, limit=1_000_000)
        return nodes[-1] if nodes else None

    def route(self, start_id: str, goal_id: str, edge_type: str = "CONNECTED_TO", max_depth: int = 64) -> Dict[str, Any]:
        """Return a deterministic shortest-hop route using persisted edges.

        This is deliberately independent of Kuzu's variable length path syntax so
        it works the same in JSONL fallback and in CI. Kuzu still provides the
        durable graph persistence layer; this method provides a safe route query
        contract for Nav2 wrappers and tests.
        """
        adjacency: Dict[str, List[Dict[str, Any]]] = {}
        for edge in self.edges(edge_type=edge_type, limit=1_000_000):
            if edge.get("properties", {}).get("blocked") is True:
                continue
            a, b = edge.get("from_id"), edge.get("to_id")
            if not a or not b:
                continue
            adjacency.setdefault(a, []).append(edge)
            if edge.get("properties", {}).get("bidirectional", True):
                reverse = dict(edge)
                reverse["from_id"], reverse["to_id"] = b, a
                adjacency.setdefault(b, []).append(reverse)
        queue: List[tuple[str, List[str], List[Dict[str, Any]]]] = [(start_id, [start_id], [])]
        seen = {start_id}
        while queue:
            node_id, path, used_edges = queue.pop(0)
            if len(path) > max_depth:
                continue
            if node_id == goal_id:
                return {"success": True, "node_ids": path, "edges": used_edges, "hop_count": len(path) - 1}
            for edge in sorted(adjacency.get(node_id, []), key=lambda e: str(e.get("to_id"))):
                nxt = str(edge.get("to_id"))
                if nxt in seen:
                    continue
                seen.add(nxt)
                queue.append((nxt, path + [nxt], used_edges + [edge]))
        return {"success": False, "node_ids": [], "edges": [], "message": f"no route from {start_id} to {goal_id}"}

    def query(self, query: Dict[str, Any]) -> Dict[str, Any]:
        node_type = query.get("node_type")
        edge_type = query.get("edge_type")
        near_id = query.get("near_id")
        limit = int(query.get("limit", 1000))
        result: Dict[str, Any] = {
            "kuzu_available": self.kuzu_available,
            "kuzu_db_path": str(self.graph_dir / "kuzu_db"),
            "kuzu_error": self.kuzu_error,
        }
        if query.get("route"):
            result["route"] = self.route(str(query.get("start_id", "")), str(query.get("goal_id", "")), edge_type=edge_type or "CONNECTED_TO")
        result["nodes"] = self.nodes(node_type, limit=limit)
        result["edges"] = self.neighbors(str(near_id), limit=limit) if near_id else self.edges(edge_type, limit=limit)
        return result

    def validate_persistence(self) -> Dict[str, Any]:
        """Write, reopen, and query a validation node/edge.

        Returns a structured result instead of raising so this can be called from
        launch smoke tests on machines where Kuzu is intentionally absent.
        """
        if not self.kuzu_available:
            return {"success": False, "kuzu_available": False, "message": self.kuzu_error or "kuzu disabled or unavailable"}
        probe_a = {"id": "kuzu_validation_a", "type": "Validation", "session_name": "validation", "label": "a", "layer": "temporary", "properties": {}}
        probe_b = {"id": "kuzu_validation_b", "type": "Validation", "session_name": "validation", "label": "b", "layer": "temporary", "properties": {}}
        probe_e = {"id": "kuzu_validation_edge", "type": "CONNECTED_TO", "session_name": "validation", "from_id": probe_a["id"], "to_id": probe_b["id"], "properties": {"distance_m": 1.0}}
        self.add_node(probe_a)
        self.add_node(probe_b)
        self.add_edge(probe_e)
        reopened = GraphStoreKuzu(self.session_dir, enable_kuzu=True)
        route = reopened.route(probe_a["id"], probe_b["id"])
        return {"success": bool(route.get("success")), "kuzu_available": reopened.kuzu_available, "route": route, "kuzu_error": reopened.kuzu_error}

    def _open_kuzu(self) -> None:
        try:
            import kuzu  # type: ignore
            self.db = kuzu.Database(str(self.graph_dir / "kuzu_db"))
            self.conn = kuzu.Connection(self.db)
            self._ensure_schema()
            self.kuzu_available = True
        except Exception as exc:  # pragma: no cover - depends on optional native wheel
            self.kuzu_available = False
            self.kuzu_error = str(exc)
            self.db = None
            self.conn = None

    def _ensure_schema(self) -> None:
        statements = [
            f"CREATE NODE TABLE {self.NODE_TABLE}(id STRING, node_type STRING, session_name STRING, label STRING, layer STRING, map_pose_json STRING, properties_json STRING, created_at STRING, PRIMARY KEY(id))",
            f"CREATE REL TABLE {self.EDGE_TABLE}(FROM {self.NODE_TABLE} TO {self.NODE_TABLE}, edge_id STRING, edge_type STRING, session_name STRING, properties_json STRING, created_at STRING)",
        ]
        for statement in statements:
            try:
                self.conn.execute(statement)
            except Exception as exc:  # table already exists on reopen
                if "already exists" not in str(exc).lower() and "duplicated" not in str(exc).lower():
                    raise

    def _upsert_kuzu_node(self, node: Dict[str, Any]) -> None:
        if self._kuzu_node_exists(str(node.get("id", ""))):
            return
        q = (
            f"CREATE (:{self.NODE_TABLE} {{"
            f"id: {self._lit(node.get('id', ''))}, "
            f"node_type: {self._lit(node.get('type', ''))}, "
            f"session_name: {self._lit(node.get('session_name', ''))}, "
            f"label: {self._lit(node.get('label', ''))}, "
            f"layer: {self._lit(node.get('layer', ''))}, "
            f"map_pose_json: {self._lit(json.dumps(node.get('map_pose', {}), sort_keys=True))}, "
            f"properties_json: {self._lit(json.dumps(node.get('properties', {}), sort_keys=True))}, "
            f"created_at: {self._lit(node.get('created_at', self._now()))}"
            "})"
        )
        self._execute(q)

    def _upsert_kuzu_edge(self, edge: Dict[str, Any]) -> None:
        if self._kuzu_edge_exists(str(edge.get("id", ""))):
            return
        from_id = str(edge.get("from_id", ""))
        to_id = str(edge.get("to_id", ""))
        if not self._kuzu_node_exists(from_id) or not self._kuzu_node_exists(to_id):
            return
        q = (
            f"MATCH (a:{self.NODE_TABLE}), (b:{self.NODE_TABLE}) "
            f"WHERE a.id = {self._lit(from_id)} AND b.id = {self._lit(to_id)} "
            f"CREATE (a)-[:{self.EDGE_TABLE} {{"
            f"edge_id: {self._lit(edge.get('id', ''))}, "
            f"edge_type: {self._lit(edge.get('type', ''))}, "
            f"session_name: {self._lit(edge.get('session_name', ''))}, "
            f"properties_json: {self._lit(json.dumps(edge.get('properties', {}), sort_keys=True))}, "
            f"created_at: {self._lit(edge.get('created_at', self._now()))}"
            "}]->(b)"
        )
        self._execute(q)

    def _kuzu_node_exists(self, node_id: str) -> bool:
        rows = self._rows(f"MATCH (n:{self.NODE_TABLE}) WHERE n.id = {self._lit(node_id)} RETURN n.id LIMIT 1")
        return bool(rows)

    def _kuzu_edge_exists(self, edge_id: str) -> bool:
        rows = self._rows(f"MATCH ()-[e:{self.EDGE_TABLE}]->() WHERE e.edge_id = {self._lit(edge_id)} RETURN e.edge_id LIMIT 1")
        return bool(rows)

    def _query_kuzu_nodes(self, node_type: Optional[str], limit: int) -> List[Dict[str, Any]]:
        where = f" WHERE n.node_type = {self._lit(node_type)}" if node_type else ""
        q = f"MATCH (n:{self.NODE_TABLE}){where} RETURN n.id, n.node_type, n.session_name, n.label, n.layer, n.map_pose_json, n.properties_json, n.created_at LIMIT {int(limit)}"
        return [self._node_from_row(row) for row in self._rows(q)]

    def _query_kuzu_edges(self, edge_type: Optional[str], limit: int) -> List[Dict[str, Any]]:
        where = f" WHERE e.edge_type = {self._lit(edge_type)}" if edge_type else ""
        q = f"MATCH (a:{self.NODE_TABLE})-[e:{self.EDGE_TABLE}]->(b:{self.NODE_TABLE}){where} RETURN e.edge_id, e.edge_type, e.session_name, a.id, b.id, e.properties_json, e.created_at LIMIT {int(limit)}"
        return [self._edge_from_row(row) for row in self._rows(q)]

    def _query_kuzu_neighbors(self, node_id: str, limit: int) -> List[Dict[str, Any]]:
        q = (
            f"MATCH (a:{self.NODE_TABLE})-[e:{self.EDGE_TABLE}]->(b:{self.NODE_TABLE}) "
            f"WHERE a.id = {self._lit(node_id)} OR b.id = {self._lit(node_id)} "
            f"RETURN e.edge_id, e.edge_type, e.session_name, a.id, b.id, e.properties_json, e.created_at LIMIT {int(limit)}"
        )
        return [self._edge_from_row(row) for row in self._rows(q)]

    def _execute(self, query: str) -> None:
        try:
            self.conn.execute(query)
        except Exception as exc:  # pragma: no cover - optional native wheel
            self.kuzu_error = str(exc)
            raise

    def _rows(self, query: str) -> List[List[Any]]:
        try:
            result = self.conn.execute(query)
            rows: List[List[Any]] = []
            if hasattr(result, "has_next") and hasattr(result, "get_next"):
                while result.has_next():
                    row = result.get_next()
                    rows.append(list(row) if isinstance(row, (tuple, list)) else [row])
                return rows
            if hasattr(result, "get_as_df"):
                df = result.get_as_df()
                return df.values.tolist()
            return []
        except Exception as exc:  # pragma: no cover - optional native wheel
            self.kuzu_error = str(exc)
            return []

    @staticmethod
    def _node_from_row(row: Iterable[Any]) -> Dict[str, Any]:
        values = list(row)
        return {
            "id": values[0],
            "type": values[1],
            "session_name": values[2],
            "label": values[3],
            "layer": values[4],
            "map_pose": GraphStoreKuzu._json(values[5], {}),
            "properties": GraphStoreKuzu._json(values[6], {}),
            "created_at": values[7] if len(values) > 7 else "",
        }

    @staticmethod
    def _edge_from_row(row: Iterable[Any]) -> Dict[str, Any]:
        values = list(row)
        return {
            "id": values[0],
            "type": values[1],
            "session_name": values[2],
            "from_id": values[3],
            "to_id": values[4],
            "properties": GraphStoreKuzu._json(values[5], {}),
            "created_at": values[6] if len(values) > 6 else "",
        }

    @staticmethod
    def _append_unique(path: Path, record: Dict[str, Any], key: str) -> None:
        existing = GraphStoreKuzu._read(path)
        if any(str(row.get(key)) == str(record.get(key)) for row in existing):
            return
        GraphStoreKuzu._append(path, record)

    @staticmethod
    def _append(path: Path, record: Dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, sort_keys=True) + "\n")

    @staticmethod
    def _read(path: Path) -> List[Dict[str, Any]]:
        if not path.exists():
            return []
        rows: List[Dict[str, Any]] = []
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
        return rows

    @staticmethod
    def _json(value: Any, default: Any) -> Any:
        if value in (None, ""):
            return default
        if isinstance(value, (dict, list)):
            return value
        try:
            return json.loads(str(value))
        except Exception:
            return default

    @staticmethod
    def _lit(value: Any) -> str:
        return "'" + str(value).replace("\\", "\\\\").replace("'", "\\'") + "'"

    @staticmethod
    def _now() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
