from __future__ import annotations

from typing import Any, Dict, List

from .backends.artifact_store_files import ArtifactStoreFiles


def export_legacy_files(store: ArtifactStoreFiles, session_name: str) -> None:
    checkpoints = store.read_jsonl(session_name, 'memory/checkpoints.jsonl')
    places = store.read_jsonl(session_name, 'memory/places.jsonl')
    spawn = store.read_json(session_name, 'memory/spawn.json', default={}) or {}

    legacy_places: List[Dict[str, Any]] = []
    for p in places:
        data = p.get('data', {})
        legacy_places.append({
            'name': data.get('name') or p.get('id'),
            'id': p.get('id'),
            'aliases': data.get('aliases', []),
            'description': data.get('description', ''),
            'verified': data.get('verified', False),
            'confidence': data.get('confidence', p.get('confidence', {})),
            'map_pose': data.get('map_pose') or data.get('pose'),
            'checkpoint_refs': data.get('checkpoint_refs', []),
        })

    store.write_yaml(session_name, 'places.yaml', {'places': legacy_places})
    if spawn:
        store.write_yaml(session_name, 'spawn.yaml', spawn)
    store.write_yaml(session_name, 'session.yaml', {
        'session_name': session_name,
        'format': 'go2_agentic_session_v1',
        'checkpoint_count': len(checkpoints),
        'place_count': len(legacy_places),
        'has_spawn': bool(spawn),
    })


def import_legacy_summary(store: ArtifactStoreFiles, session_name: str) -> Dict[str, Any]:
    session_dir = store.session_dir(session_name)
    return {
        'session_dir': str(session_dir),
        'map_yaml_exists': (session_dir / 'map.yaml').exists(),
        'map_pgm_exists': (session_dir / 'map.pgm').exists(),
        'places_yaml_exists': (session_dir / 'places.yaml').exists(),
        'session_yaml_exists': (session_dir / 'session.yaml').exists(),
        'spawn_yaml_exists': (session_dir / 'spawn.yaml').exists(),
    }
