from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


AUTO_SESSION_NAMES = {"", "auto", "latest", "latest_usable"}


def _route_stop_count(route_path: Path) -> int:
    if not route_path.is_file():
        return 0
    try:
        data = yaml.safe_load(route_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return 0
    route = data.get("route") if isinstance(data.get("route"), dict) else data
    stops = route.get("stops") if isinstance(route, dict) else []
    return len(stops or []) if isinstance(stops, list) else 0


def _is_semantic_session(path: Path) -> bool:
    if not path.is_dir():
        return False
    if (path / "map.yaml").is_file() and ((path / "places.yaml").is_file() or (path / "route.yaml").is_file()):
        return True
    return _route_stop_count(path / "route.yaml") > 0


def resolve_semantic_session_name(session_root: str | Path, requested: Any = "") -> str:
    """Resolve an agent/memory session to the latest usable semantic-nav session.

    The Omi/LangGraph stack historically defaulted to ``default``, which is useful
    for scratch VLM artifacts but often has no map or route. Resume-mode commands
    need the semantic session that owns ``map.yaml``, ``places.yaml``, and
    ``route.yaml``.
    """

    root = Path(str(session_root)).expanduser()
    name = str(requested or "").strip()
    if name == "__latest_created__":
        # Teach has a valid session directory before it has map.yaml/route.yaml. This
        # special internal selector is for Teach-time writers only; Resume/Tour should
        # use an explicit session name (or the normal resume-ready auto resolver).
        candidates = [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
        if not candidates:
            return "__latest_created__"
        return max(candidates, key=lambda p: p.stat().st_mtime_ns).name
    if name and name.lower() not in AUTO_SESSION_NAMES | {"default"}:
        return name

    default_dir = root / "default"
    if name.lower() == "default" and _is_semantic_session(default_dir):
        return "default"

    candidates = [p for p in root.iterdir() if _is_semantic_session(p)] if root.is_dir() else []
    if not candidates:
        return name or "default"

    def score(path: Path) -> tuple[int, float]:
        has_map = 1 if (path / "map.yaml").is_file() else 0
        stops = _route_stop_count(path / "route.yaml")
        try:
            mtime = max((child.stat().st_mtime for child in path.iterdir()), default=path.stat().st_mtime)
        except Exception:
            mtime = 0.0
        return (has_map * 1000 + stops, mtime)

    return max(candidates, key=score).name
