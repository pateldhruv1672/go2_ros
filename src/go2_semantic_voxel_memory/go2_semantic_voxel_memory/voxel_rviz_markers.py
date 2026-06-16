from __future__ import annotations

from typing import Any, Dict, Iterable

from visualization_msgs.msg import Marker, MarkerArray


def markers_from_voxels(voxels: Iterable[Dict[str, Any]], frame_id: str = 'map') -> MarkerArray:
    arr = MarkerArray()
    for idx, voxel in enumerate(voxels):
        center = voxel.get('center_xyz', {})
        size = float(voxel.get('size_m', 0.25))
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'semantic_voxel_memory'
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center.get('x', 0.0))
        marker.pose.position.y = float(center.get('y', 0.0))
        marker.pose.position.z = float(center.get('z', 0.0))
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 0.35
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        arr.markers.append(marker)
    return arr
