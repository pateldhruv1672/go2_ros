from __future__ import annotations

import glob
import os
import re
from dataclasses import dataclass, field
from typing import Optional

import yaml


def norm(text: str) -> str:
    text = (text or "").strip().lower()
    text = re.sub(r"[^a-z0-9 ]+", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text


@dataclass
class Place:
    name: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    room: str = ""
    category: str = ""
    aliases: list[str] = field(default_factory=list)
    tags: list[str] = field(default_factory=list)
    description: str = ""


def latest_session_name(session_root: str) -> str:
    session_root = os.path.expanduser(session_root)
    dirs = [d for d in glob.glob(os.path.join(session_root, "*")) if os.path.isdir(d)]
    if not dirs:
        raise RuntimeError(f"No sessions found in {session_root}")
    dirs.sort(key=os.path.getmtime, reverse=True)
    return os.path.basename(dirs[0])


class PlaceResolver:
    def __init__(self, places_yaml_path: str):
        self.places_yaml_path = os.path.expanduser(places_yaml_path)
        self.places: list[Place] = []
        self.load()

    def load(self) -> None:
        self.places = []
        if not os.path.isfile(self.places_yaml_path):
            return
        with open(self.places_yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        for p in data.get("places", []):
            self.places.append(
                Place(
                    name=str(p.get("name", "")),
                    x=float(p.get("x", 0.0)),
                    y=float(p.get("y", 0.0)),
                    yaw=float(p.get("yaw", 0.0)),
                    room=str(p.get("room", "")),
                    category=str(p.get("category", "")),
                    aliases=list(p.get("aliases", []) or []),
                    tags=list(p.get("tags", []) or []),
                    description=str(p.get("description", "")),
                )
            )

    def list_names(self) -> list[str]:
        return [p.name for p in self.places]

    def resolve(self, query: str) -> Optional[Place]:
        q = norm(query)
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
                    score = max(score, 80)
                else:
                    q_tokens = set(q.split())
                    s_tokens = set(s.split())
                    overlap = len(q_tokens & s_tokens)
                    if overlap:
                        score = max(score, overlap * 20)
            if score > best_score:
                best_score = score
                best = p
        return best if best_score >= 20 else None
