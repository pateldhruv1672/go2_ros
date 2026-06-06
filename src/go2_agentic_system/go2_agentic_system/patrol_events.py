from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional
import json


@dataclass
class PatrolEvent:
    location: Optional[Dict[str, Any]] = None
    confidence: float = 0.0
    request: str = ''
    status: str = ''
    completion: float = 0.0
    event_worthy: bool = False
    label: str = ''
    replay_variations: List[str] = field(default_factory=list)
    update_strength: float = 0.0
    route_name: str = ''
    stop_name: str = ''
    speech: str = ''
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        if self.location is None:
            data.pop('location', None)
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'PatrolEvent':
        payload = dict(data or {})
        payload['replay_variations'] = list(payload.get('replay_variations') or [])
        payload['details'] = dict(payload.get('details') or {})
        if payload.get('location') is not None and not isinstance(payload['location'], dict):
            payload['location'] = None
        try:
            payload['confidence'] = float(payload.get('confidence', 0.0))
        except Exception:
            payload['confidence'] = 0.0
        try:
            payload['completion'] = float(payload.get('completion', 0.0))
        except Exception:
            payload['completion'] = 0.0
        try:
            payload['update_strength'] = float(payload.get('update_strength', 0.0))
        except Exception:
            payload['update_strength'] = 0.0
        payload['event_worthy'] = bool(payload.get('event_worthy', False))
        return cls(**payload)

