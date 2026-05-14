from __future__ import annotations

from dataclasses import dataclass
import glob
import os
from datetime import datetime
import yaml


@dataclass
class SessionPaths:
    session_name: str
    session_dir: str
    map_prefix: str
    places_path: str
    session_yaml_path: str


class SessionStore:
    def __init__(self, root: str) -> None:
        self.root = os.path.expanduser(root)
        os.makedirs(self.root, exist_ok=True)

    def create(self, label: str) -> SessionPaths:
        safe = ''.join(c if c.isalnum() or c in ('_', '-') else '_' for c in (label or 'session')).strip('_') or 'session'
        name = f'{safe}_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        return self.for_name(name, create=True)

    def for_name(self, session_name: str, create: bool = False) -> SessionPaths:
        session_dir = os.path.join(self.root, session_name)
        if create:
            os.makedirs(session_dir, exist_ok=True)
        return SessionPaths(
            session_name=session_name,
            session_dir=session_dir,
            map_prefix=os.path.join(session_dir, 'map'),
            places_path=os.path.join(session_dir, 'places.yaml'),
            session_yaml_path=os.path.join(session_dir, 'session.yaml'),
        )

    def latest(self) -> SessionPaths | None:
        items = sorted(glob.glob(os.path.join(self.root, '*')), key=os.path.getmtime, reverse=True)
        for d in items:
            if os.path.isdir(d):
                return self.for_name(os.path.basename(d))
        return None

    def latest_usable(self) -> SessionPaths | None:
        items = sorted(glob.glob(os.path.join(self.root, '*')), key=os.path.getmtime, reverse=True)
        for d in items:
            if not os.path.isdir(d):
                continue
            if all(os.path.isfile(os.path.join(d, f)) for f in ('map.yaml', 'map.pgm', 'places.yaml', 'session.yaml')):
                return self.for_name(os.path.basename(d))
        return None

    def load_session_yaml(self, session: SessionPaths) -> dict:
        if not os.path.isfile(session.session_yaml_path):
            return {}
        with open(session.session_yaml_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def save_session_yaml(self, session: SessionPaths, data: dict) -> None:
        os.makedirs(session.session_dir, exist_ok=True)
        with open(session.session_yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
