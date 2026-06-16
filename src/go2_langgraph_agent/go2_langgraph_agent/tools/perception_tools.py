from __future__ import annotations

from typing import Any, Dict


def merge_perception(scan_summary: Dict[str, Any], pointcloud_summary: Dict[str, Any]) -> Dict[str, Any]:
    return {
        'scan': scan_summary,
        'pointcloud': pointcloud_summary,
        'safety_blocked': (scan_summary.get('sector_clearance_m') or {}).get('front', 10.0) is not None and (scan_summary.get('sector_clearance_m') or {}).get('front', 10.0) < 0.45,
    }
