from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List
import json
import math


class VoxelStoreAdapter:
    def __init__(self, session_dir: Path):
        self.path = session_dir / 'voxel_memory' / 'semantic_voxels.jsonl'
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write_observation(self, observation: Dict[str, Any]) -> None:
        with self.path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(observation, sort_keys=True) + '\n')

    def query_near_pose(self, x: float, y: float, radius_m: float = 2.0, limit: int = 50) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for obs in self._read():
            center = obs.get('center_xyz') or obs.get('center') or {}
            cx = float(center.get('x', center[0] if isinstance(center, list) and center else 0.0))
            cy = float(center.get('y', center[1] if isinstance(center, list) and len(center) > 1 else 0.0))
            if math.hypot(cx - x, cy - y) <= radius_m:
                out.append(obs)
                if len(out) >= limit:
                    break
        return out

    def _read(self) -> List[Dict[str, Any]]:
        if not self.path.exists():
            return []
        rows: List[Dict[str, Any]] = []
        with self.path.open('r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
        return rows
