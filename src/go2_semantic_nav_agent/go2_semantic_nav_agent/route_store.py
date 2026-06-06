from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
import json
import os
import re

import yaml

from go2_agentic_system.patrol_events import PatrolEvent


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def slugify(text: str) -> str:
    text = (text or '').strip().lower()
    text = re.sub(r'[^a-z0-9]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_')
    return text or 'route'


@dataclass
class RouteStop:
    name: str
    place_name: str = ''
    script: str = ''
    fact: str = ''
    navigation_hint: str = ''
    resume_hook: str = ''
    safety_notes: str = ''
    scene_context: str = ''
    capture_kind: str = ''
    sample_index: int = 0
    sample_group: str = ''
    pause_seconds: float = 4.0
    safe_anchor: str = ''
    kind: str = 'tour'
    confidence: float = 1.0
    aliases: List[str] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)
    status: str = 'pending'


@dataclass
class RoutePlan:
    name: str
    mode: str = 'tour'
    state: str = 'idle'
    current_stop_index: int = 0
    stops: List[RouteStop] = field(default_factory=list)
    safe_anchors: Dict[str, str] = field(default_factory=dict)
    facts: List[str] = field(default_factory=list)
    last_failure: Dict[str, Any] = field(default_factory=dict)
    guest_prompt: str = ''
    created_at: str = field(default_factory=_utc_now)
    updated_at: str = field(default_factory=_utc_now)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['stops'] = [asdict(stop) for stop in self.stops]
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RoutePlan':
        payload = dict(data or {})
        stops = [RouteStop(**dict(stop or {})) for stop in payload.get('stops', []) or []]
        payload['stops'] = stops
        payload['current_stop_index'] = int(payload.get('current_stop_index', 0))
        payload['safe_anchors'] = dict(payload.get('safe_anchors') or {})
        payload['facts'] = [str(x) for x in (payload.get('facts') or [])]
        payload['last_failure'] = dict(payload.get('last_failure') or {})
        payload['created_at'] = str(payload.get('created_at') or _utc_now())
        payload['updated_at'] = str(payload.get('updated_at') or _utc_now())
        return cls(**payload)


class RouteStore:
    def __init__(self, session_dir: str | Path) -> None:
        self.session_dir = Path(session_dir)
        self.route_path = self.session_dir / 'route.yaml'
        self.events_path = self.session_dir / 'route_events.jsonl'
        self.session_dir.mkdir(parents=True, exist_ok=True)
        if not self.events_path.exists():
            self.events_path.touch()

    def load(self) -> Optional[RoutePlan]:
        if not self.route_path.exists():
            return None
        with self.route_path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        if 'route' in data and isinstance(data['route'], dict):
            data = data['route']
        return RoutePlan.from_dict(data)

    def save(self, route: RoutePlan) -> None:
        route.updated_at = _utc_now()
        payload = {'route': route.to_dict()}
        with self.route_path.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(payload, handle, sort_keys=False)

    def load_or_create(self, *, name: str, mode: str, stop_candidates: Iterable[Dict[str, Any]]) -> RoutePlan:
        route = self.load()
        if route is not None:
            return route
        stops: List[RouteStop] = []
        for item in stop_candidates:
            place_name = str(item.get('place_name') or item.get('name') or '').strip()
            if not place_name:
                continue
            stop_name = slugify(str(item.get('stop_name') or place_name))
            stops.append(
                RouteStop(
                    name=stop_name,
                    place_name=place_name,
                    script=str(item.get('script') or ''),
                    fact=str(item.get('fact') or item.get('description') or ''),
                    navigation_hint=str(item.get('navigation_hint') or ''),
                    resume_hook=str(item.get('resume_hook') or ''),
                    safety_notes=str(item.get('safety_notes') or ''),
                    scene_context=str(item.get('scene_context') or ''),
                    capture_kind=str(item.get('capture_kind') or ''),
                    sample_index=int(item.get('sample_index', 0) or 0),
                    sample_group=str(item.get('sample_group') or ''),
                    pause_seconds=float(item.get('pause_seconds', 4.0)),
                    safe_anchor=str(item.get('safe_anchor') or ''),
                    kind=str(item.get('kind') or mode or 'tour'),
                    confidence=float(item.get('confidence', 1.0)),
                    aliases=[str(x) for x in (item.get('aliases') or [])],
                    tags=[str(x) for x in (item.get('tags') or [])],
                    status='pending',
                )
            )
        route = RoutePlan(name=slugify(name), mode=mode, stops=stops)
        self.save(route)
        return route

    def append_event(self, event: PatrolEvent) -> None:
        with self.events_path.open('a', encoding='utf-8') as handle:
            handle.write(event.to_json() + '\n')

    def current_stop(self, route: RoutePlan) -> Optional[RouteStop]:
        if not route.stops:
            return None
        idx = max(0, min(route.current_stop_index, len(route.stops) - 1))
        return route.stops[idx]

    def advance(self, route: RoutePlan) -> Optional[RouteStop]:
        if not route.stops:
            return None
        if route.current_stop_index < len(route.stops) - 1:
            route.current_stop_index += 1
            self.save(route)
            return route.stops[route.current_stop_index]
        route.state = 'complete'
        self.save(route)
        return None

    def set_state(self, route: RoutePlan, state: str) -> None:
        route.state = state
        self.save(route)

    def set_last_failure(self, route: RoutePlan, failure: Dict[str, Any]) -> None:
        route.last_failure = dict(failure)
        self.save(route)

    def upsert_stop(self, route: RoutePlan, stop: RouteStop) -> None:
        for idx, existing in enumerate(route.stops):
            if existing.name == stop.name or existing.place_name == stop.place_name:
                route.stops[idx] = stop
                self.save(route)
                return
        route.stops.append(stop)
        self.save(route)

    def ensure_safe_anchor(self, route: RoutePlan, key: str, place_name: str) -> None:
        route.safe_anchors[key] = place_name
        self.save(route)

    def summary(self, route: RoutePlan) -> Dict[str, Any]:
        stop = self.current_stop(route)
        return {
            'route_name': route.name,
            'mode': route.mode,
            'state': route.state,
            'current_stop_index': route.current_stop_index,
            'stop_count': len(route.stops),
            'current_stop': asdict(stop) if stop else None,
            'safe_anchors': dict(route.safe_anchors),
            'facts': list(route.facts),
            'last_failure': dict(route.last_failure),
            'guest_prompt': route.guest_prompt,
        }
