from __future__ import annotations

from typing import Any, Dict, List, Sequence

from .voxel_store import SemanticVoxelStore


LANDMARK_LABELS = {"doorway", "corner", "sign", "poster", "lab_entrance", "hallway", "elevator", "stairwell"}


def query_localization_landmarks(store: SemanticVoxelStore, x: float, y: float, radius_m: float = 3.0) -> List[Dict[str, Any]]:
    return [
        v for v in store.query_landmarks(x, y, radius_m=radius_m)
        if LANDMARK_LABELS.intersection(set(v.get("semantic_labels", [])))
    ]


def query_objects_near_route(store: SemanticVoxelStore, waypoints: Sequence[Dict[str, float]], radius_m: float = 1.0) -> Dict[str, Any]:
    seen: Dict[str, Dict[str, Any]] = {}
    for waypoint in waypoints:
        for voxel in store.query_near(float(waypoint.get("x", 0.0)), float(waypoint.get("y", 0.0)), radius_m=radius_m, limit=100):
            labels = set(voxel.get("semantic_labels", []))
            if labels.difference({"free_space", "pointcloud_observed", "hallway"}):
                seen[voxel["voxel_id"]] = voxel
    return {"count": len(seen), "voxels": list(seen.values())}


def summarize_voxels(voxels: List[Dict[str, Any]]) -> Dict[str, Any]:
    labels: Dict[str, int] = {}
    traversability = []
    for voxel in voxels:
        traversability.append(float(voxel.get("traversability_score", 0.5)))
        for label in voxel.get("semantic_labels", []):
            labels[label] = labels.get(label, 0) + 1
    return {
        "count": len(voxels),
        "labels": labels,
        "average_traversability_score": (sum(traversability) / len(traversability)) if traversability else 0.0,
    }
