from __future__ import annotations

from typing import Any, Dict


def make_safe_anchor(label: str, map_pose: Dict[str, Any], radius_m: float = 0.75) -> Dict[str, Any]:
    return {
        'type': 'SafeAnchor',
        'label': label,
        'map_pose': map_pose,
        'safe_radius_m': radius_m,
        'verified': False,
        'layer': 'temporary',
    }
