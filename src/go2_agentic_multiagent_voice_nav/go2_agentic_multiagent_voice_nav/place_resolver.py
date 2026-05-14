from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Optional

import yaml


def norm(text: str) -> str:
    text = (text or '').strip().lower()
    text = re.sub(r'[^a-z0-9 ]+', ' ', text)
    text = re.sub(r'\s+', ' ', text).strip()
    return text


def slugify(text: str) -> str:
    text = norm(text).replace(' ', '_')
    return text or 'place'


@dataclass
class Place:
    name: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    room: str = ''
    category: str = ''
    aliases: list[str] = field(default_factory=list)
    tags: list[str] = field(default_factory=list)
    description: str = ''


class PlaceResolver:
    def __init__(self, session_root: str, session_name: str = ''):
        self.session_root = os.path.expanduser(session_root)
        self.session_name = session_name.strip()
        self.places: list[Place] = []
        self.places_yaml_path = ''
        self.reload()

    def _latest_session(self) -> str:
        if not os.path.isdir(self.session_root):
            return ''
        dirs = [
            os.path.join(self.session_root, d)
            for d in os.listdir(self.session_root)
            if os.path.isdir(os.path.join(self.session_root, d))
        ]
        if not dirs:
            return ''
        dirs.sort(key=os.path.getmtime, reverse=True)
        return os.path.basename(dirs[0])

    def reload(self) -> None:
        session = self.session_name or self._latest_session()
        self.places = []
        self.places_yaml_path = ''
        if not session:
            return
        path = os.path.join(self.session_root, session, 'places.yaml')
        self.places_yaml_path = path
        if not os.path.isfile(path):
            return
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        for p in data.get('places', []) or []:
            self.places.append(
                Place(
                    name=str(p.get('name', '')),
                    x=float(p.get('x', 0.0)),
                    y=float(p.get('y', 0.0)),
                    yaw=float(p.get('yaw', 0.0)),
                    room=str(p.get('room', '')),
                    category=str(p.get('category', '')),
                    aliases=list(p.get('aliases', []) or []),
                    tags=list(p.get('tags', []) or []),
                    description=str(p.get('description', '')),
                )
            )

    def list_names(self) -> list[str]:
        return sorted([p.name for p in self.places])

    def resolve(self, query: str) -> Optional[Place]:
        q = norm(query)
        if not q:
            return None
        best = None
        best_score = -1

        for p in self.places:
            score = 0
            fields = [p.name, p.room, p.category, p.description, *p.aliases, *p.tags]
            for field in fields:
                s = norm(field)
                if not s:
                    continue
                if s == q:
                    score = max(score, 100)
                elif q in s or s in q:
                    score = max(score, 75)
                else:
                    overlap = len(set(q.split()) & set(s.split()))
                    if overlap:
                        score = max(score, overlap * 20)
            if score > best_score:
                best_score = score
                best = p

        return best if best_score >= 20 else None
