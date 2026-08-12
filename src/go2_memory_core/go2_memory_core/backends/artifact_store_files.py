from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List
import json
import os
import shutil

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


class ArtifactStoreFiles:
    """File-backed session/artifact store.

    The store is intentionally boring: JSONL for append-only robotics events and
    YAML/JSON exports for human inspection. This keeps teach/resume memory usable
    even when optional graph/vector dependencies are not installed.
    """

    def __init__(self, session_root: str | Path):
        self.session_root = Path(os.path.expanduser(str(session_root))).resolve()
        self.session_root.mkdir(parents=True, exist_ok=True)

    def session_dir(self, session_name: str) -> Path:
        if not session_name or session_name == "latest":
            sessions = [p for p in self.session_root.glob('*') if p.is_dir()]
            if sessions:
                return max(sessions, key=lambda p: p.stat().st_mtime)
            session_name = "default"
        path = self.session_root / session_name
        path.mkdir(parents=True, exist_ok=True)
        for sub in [
            'memory', 'graph_memory', 'graph_memory/kuzu_db', 'voxel_memory',
            'voxel_memory/voxel_blocks', 'vector_memory', 'artifacts/images',
            'artifacts/pointclouds', 'artifacts/scans', 'artifacts/vlm_raw',
            'artifacts/nav_logs', 'langgraph/checkpoints', 'langgraph/store', 'exports'
        ]:
            (path / sub).mkdir(parents=True, exist_ok=True)
        return path

    def append_jsonl(self, session_name: str, relative_path: str, record: Dict[str, Any]) -> Path:
        path = self.session_dir(session_name) / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, sort_keys=True) + '\n')
        return path

    def write_json(self, session_name: str, relative_path: str, record: Dict[str, Any]) -> Path:
        path = self.session_dir(session_name) / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        return path

    def read_json(self, session_name: str, relative_path: str, default: Any = None) -> Any:
        path = self.session_dir(session_name) / relative_path
        if not path.exists():
            return default
        return json.loads(path.read_text(encoding='utf-8'))

    def read_jsonl(self, session_name: str, relative_path: str) -> List[Dict[str, Any]]:
        # SPARKY_JSONL_RESILIENCE_V12_8
        # Append-only memory is fail-soft: preserve the file and skip only malformed rows.
        path = self.session_dir(session_name) / relative_path
        if not path.exists():
            return []
        records: List[Dict[str, Any]] = []
        with path.open("r", encoding="utf-8", errors="replace") as f:
            for line_no, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    value = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError, ValueError, TypeError):
                    continue
                if isinstance(value, dict):
                    records.append(value)
        return records

    def write_yaml(self, session_name: str, relative_path: str, record: Dict[str, Any]) -> Path:
        path = self.session_dir(session_name) / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        if yaml is not None:
            path.write_text(yaml.safe_dump(record, sort_keys=False), encoding='utf-8')
        else:
            path.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        return path

    def copy_artifact(self, session_name: str, src_path: str | Path, dest_relative: str) -> str:
        src = Path(os.path.expanduser(str(src_path))).resolve()
        dest = self.session_dir(session_name) / dest_relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
        return str(dest)

    def latest_record(self, session_name: str, relative_path: str) -> Dict[str, Any] | None:
        records = self.read_jsonl(session_name, relative_path)
        return records[-1] if records else None
