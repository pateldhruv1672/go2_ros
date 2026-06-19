from pathlib import Path

import yaml

from go2_memory_core.session_resolution import resolve_semantic_session_name


def _write_yaml(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")


def test_resolve_semantic_session_skips_empty_default(tmp_path: Path):
    default = tmp_path / "default"
    default.mkdir()
    _write_yaml(default / "places.yaml", {"places": []})

    usable = tmp_path / "lab_session"
    usable.mkdir()
    _write_yaml(usable / "map.yaml", {"image": "map.pgm"})
    _write_yaml(
        usable / "route.yaml",
        {"route": {"name": "lab_session", "stops": [{"name": "checkpoint_1", "place_name": "checkpoint_1"}]}},
    )

    assert resolve_semantic_session_name(tmp_path, "auto") == "lab_session"
    assert resolve_semantic_session_name(tmp_path, "default") == "lab_session"


def test_resolve_semantic_session_keeps_explicit_name(tmp_path: Path):
    assert resolve_semantic_session_name(tmp_path, "operator_selected") == "operator_selected"
