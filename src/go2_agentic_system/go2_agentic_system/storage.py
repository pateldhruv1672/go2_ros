from __future__ import annotations

import json
import os
import re
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sanitize_name(name: str) -> str:
    cleaned = ''.join(ch if ch.isalnum() or ch in ('_', '-') else '_' for ch in name.strip().lower())
    return cleaned or 'unnamed'


def tokenize(text: str) -> set[str]:
    return set(re.findall(r'[a-z0-9]+', text.lower()))


@dataclass
class StoragePaths:
    root: Path
    maps: Path
    memory: Path
    logs: Path
    images: Path


def expand_storage_root(path: str) -> StoragePaths:
    root = Path(os.path.expanduser(path)).resolve()
    paths = StoragePaths(
        root=root,
        maps=root / 'maps',
        memory=root / 'memory',
        logs=root / 'logs',
        images=root / 'images',
    )
    for item in (paths.root, paths.maps, paths.memory, paths.logs, paths.images):
        item.mkdir(parents=True, exist_ok=True)
    return paths


def read_yaml(path: Path, default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    if not path.exists():
        return {} if default is None else default
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or ({} if default is None else default)


def write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(payload, ensure_ascii=False) + '\n')


class MemoryStore:
    def __init__(self, storage_root: str) -> None:
        self.paths = expand_storage_root(storage_root)
        self.state_path = self.paths.memory / 'state.yaml'
        self.places_path = self.paths.memory / 'places.yaml'
        self.guardrails_path = self.paths.memory / 'guardrails.yaml'
        self.maps_index_path = self.paths.maps / 'index.yaml'
        self.observations_path = self.paths.memory / 'observations.jsonl'
        self.timeline_path = self.paths.logs / 'timeline.jsonl'
        self.ensure_defaults()

    def ensure_defaults(self) -> None:
        if not self.state_path.exists():
            write_yaml(
                self.state_path,
                {
                    'mode': 'idle',
                    'active_map': None,
                    'active_survey': None,
                    'last_map_pose': None,
                    'last_command': None,
                    'updated_at': utc_now(),
                },
            )
        if not self.places_path.exists():
            write_yaml(self.places_path, {'maps': {}})
        if not self.maps_index_path.exists():
            write_yaml(self.maps_index_path, {'maps': {}})
        if not self.guardrails_path.exists():
            write_yaml(
                self.guardrails_path,
                {
                    'guardrails': {
                        'min_front_clearance_m': 0.75,
                        'min_rear_clearance_m': 0.45,
                        'front_arc_deg': 40.0,
                        'rear_arc_deg': 50.0,
                        'localization_timeout_sec': 2.0,
                        'require_localization_for_navigation': True,
                        'allow_manual_without_localization': True,
                        'capture_distance_m': 1.5,
                        'capture_interval_sec': 8.0,
                        'remember_by_default': True,
                    }
                },
            )
        if not self.observations_path.exists():
            self.observations_path.touch()

    def read_state(self) -> Dict[str, Any]:
        return read_yaml(self.state_path, {})

    def write_state(self, state: Dict[str, Any]) -> None:
        state['updated_at'] = utc_now()
        write_yaml(self.state_path, state)

    def update_state(self, **updates: Any) -> Dict[str, Any]:
        state = self.read_state()
        state.update(updates)
        self.write_state(state)
        return state

    def read_guardrails(self) -> Dict[str, Any]:
        return read_yaml(self.guardrails_path, {})

    def remember_place(self, map_name: str, place_name: str, payload: Dict[str, Any]) -> None:
        data = read_yaml(self.places_path, {'maps': {}})
        data.setdefault('maps', {})
        data['maps'].setdefault(map_name, {'places': {}})
        data['maps'][map_name].setdefault('places', {})
        entry = data['maps'][map_name]['places'].setdefault(place_name, {})
        entry.update(payload)
        entry.setdefault('aliases', [])
        entry['updated_at'] = utc_now()
        write_yaml(self.places_path, data)

    def get_place(self, map_name: str, place_name: str) -> Optional[Dict[str, Any]]:
        data = read_yaml(self.places_path, {'maps': {}})
        return data.get('maps', {}).get(map_name, {}).get('places', {}).get(place_name)

    def list_places(self, map_name: Optional[str] = None) -> Dict[str, Any]:
        data = read_yaml(self.places_path, {'maps': {}})
        if map_name is None:
            return data
        return data.get('maps', {}).get(map_name, {'places': {}})

    def register_map_artifacts(self, map_name: str, map_yaml: str, notes: Optional[str] = None) -> None:
        index = read_yaml(self.maps_index_path, {'maps': {}})
        index.setdefault('maps', {})
        entry = index['maps'].setdefault(map_name, {})
        entry.update({'map_yaml': map_yaml, 'updated_at': utc_now()})
        if notes:
            entry['notes'] = notes
        write_yaml(self.maps_index_path, index)

    def add_observation(self, payload: Dict[str, Any]) -> None:
        append_jsonl(self.observations_path, payload)

    def list_observations(self, map_name: Optional[str] = None) -> List[Dict[str, Any]]:
        items: List[Dict[str, Any]] = []
        if not self.observations_path.exists():
            return items
        with self.observations_path.open('r', encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if map_name is None or obj.get('map_name') == map_name:
                    items.append(obj)
        return items

    def count_memories(self, map_name: Optional[str] = None) -> Dict[str, int]:
        places = self.list_places(map_name).get('places', {})
        observations = self.list_observations(map_name)
        return {'places': len(places), 'observations': len(observations)}

    def rename_map(self, old_name: str, new_name: str) -> None:
        if old_name == new_name:
            return
        old_name = sanitize_name(old_name)
        new_name = sanitize_name(new_name)
        places = read_yaml(self.places_path, {'maps': {}})
        if old_name in places.get('maps', {}):
            places['maps'][new_name] = places['maps'].pop(old_name)
            write_yaml(self.places_path, places)
        maps_index = read_yaml(self.maps_index_path, {'maps': {}})
        if old_name in maps_index.get('maps', {}):
            maps_index['maps'][new_name] = maps_index['maps'].pop(old_name)
            write_yaml(self.maps_index_path, maps_index)
        old_dir = self.paths.images / old_name
        new_dir = self.paths.images / new_name
        if old_dir.exists() and not new_dir.exists():
            shutil.move(str(old_dir), str(new_dir))
        obs = self.list_observations()
        if obs:
            with self.observations_path.open('w', encoding='utf-8') as handle:
                for item in obs:
                    if item.get('map_name') == old_name:
                        item['map_name'] = new_name
                    handle.write(json.dumps(item, ensure_ascii=False) + '\n')
        state = self.read_state()
        if state.get('active_map') == old_name:
            self.update_state(active_map=new_name)

    def log_event(self, event_type: str, payload: Dict[str, Any]) -> None:
        append_jsonl(self.timeline_path, {'ts': utc_now(), 'event': event_type, 'payload': payload})

    def _score_text_fields(self, query: str, fields: List[str], exact_score: float, contains_score: float, token_weight: float) -> float:
        q = sanitize_name(query)
        q_tokens = tokenize(query)
        score = 0.0
        for field in fields:
            sfield = sanitize_name(str(field))
            ftokens = tokenize(str(field))
            if q == sfield:
                score = max(score, exact_score)
            if q and (q in sfield or sfield in q):
                score = max(score, contains_score)
            score = max(score, float(len(q_tokens & ftokens) * token_weight))
        return score

    def resolve_destination(self, map_name: Optional[str], query: str) -> Optional[Dict[str, Any]]:
        if not map_name:
            return None
        best = None
        best_score = -1.0

        places = self.list_places(map_name).get('places', {})
        for name, payload in places.items():
            score = self._score_text_fields(query, [name] + (payload.get('aliases', []) or []) + [payload.get('summary', '')], 100.0, 60.0, 10.0)
            if score > best_score:
                best_score = score
                best = {'type': 'place', 'name': name, 'pose': payload, 'score': score}

        for obs in self.list_observations(map_name):
            fields = [obs.get('label', ''), obs.get('summary', ''), ' '.join(obs.get('aliases', []) or []), ' '.join(obs.get('objects', []) or [])]
            score = self._score_text_fields(query, fields, 85.0, 50.0, 8.0)
            if score > best_score and obs.get('pose'):
                best_score = score
                best = {'type': 'observation', 'name': obs.get('label') or obs.get('id'), 'pose': obs['pose'], 'score': score, 'summary': obs.get('summary', '')}
        return best if best_score > 0 else None

    def search_memories(self, query: str, map_name: Optional[str] = None, limit: int = 5) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        for name, payload in self.list_places(map_name).get('places', {}).items():
            score = self._score_text_fields(query, [name] + (payload.get('aliases', []) or []) + [payload.get('summary', '')], 100.0, 60.0, 10.0)
            if score > 0:
                results.append(
                    {
                        'type': 'place',
                        'name': name,
                        'score': score,
                        'summary': payload.get('summary', ''),
                        'aliases': payload.get('aliases', []) or [],
                        'pose': {'x': payload.get('x'), 'y': payload.get('y'), 'yaw': payload.get('yaw'), 'frame_id': payload.get('frame_id')},
                    }
                )
        for obs in self.list_observations(map_name):
            fields = [obs.get('label', ''), obs.get('summary', ''), ' '.join(obs.get('aliases', []) or []), ' '.join(obs.get('objects', []) or [])]
            score = self._score_text_fields(query, fields, 85.0, 50.0, 8.0)
            if score > 0:
                results.append(
                    {
                        'type': 'observation',
                        'name': obs.get('label') or obs.get('id'),
                        'score': score,
                        'summary': obs.get('summary', ''),
                        'aliases': obs.get('aliases', []) or [],
                        'objects': obs.get('objects', []) or [],
                        'image_path': obs.get('image_path'),
                        'pose': obs.get('pose', {}),
                    }
                )
        results.sort(key=lambda item: float(item.get('score', 0.0)), reverse=True)
        return results[:max(1, limit)]
