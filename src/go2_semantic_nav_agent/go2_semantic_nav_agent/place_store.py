from __future__ import annotations

from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional
import math
import os
import yaml


@dataclass
class Place:
    name: str
    x: float
    y: float
    yaw: float
    room: str = ''
    category: str = ''
    aliases: List[str] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)
    description: str = ''
    confidence: float = 1.0
    source: str = 'manual'


class PlaceStore:
    def __init__(self, path: str) -> None:
        self.path = os.path.expanduser(path)
        self.places: Dict[str, Place] = {}
        self.load()

    def load(self) -> None:
        self.places = {}
        if not os.path.isfile(self.path):
            return
        with open(self.path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        for row in data.get('places', []) or []:
            p = Place(**row)
            self.places[p.name] = p

    def save(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'places': [asdict(p) for p in self.places.values()]}, f, sort_keys=False)

    def upsert(self, place: Place) -> None:
        self.places[place.name] = place

    def get(self, name: str) -> Optional[Place]:
        return self.places.get(name)

    def unique_name(self, base: str) -> str:
        if base not in self.places:
            return base
        idx = 2
        while f'{base}_{idx}' in self.places:
            idx += 1
        return f'{base}_{idx}'

    def nearest_within(self, x: float, y: float, radius_m: float) -> Optional[Place]:
        best = None
        best_d = radius_m
        for p in self.places.values():
            d = math.hypot(x - p.x, y - p.y)
            if d <= best_d:
                best = p
                best_d = d
        return best

    def list_lines(self) -> List[str]:
        out = []
        for p in sorted(self.places.values(), key=lambda z: z.name):
            out.append(
                f'{p.name} @ ({p.x:.2f}, {p.y:.2f}, yaw={p.yaw:.2f}) '
                f'[room={p.room or "-"} | category={p.category or "-"} | aliases={",".join(p.aliases) or "-"} | tags={",".join(p.tags) or "-"} | desc={p.description or "-"}]'
            )
        return out
