from __future__ import annotations

import json
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .voxel_query import query_localization_landmarks, summarize_voxels
from .voxel_rviz_markers import markers_from_voxels
from .voxel_store import SemanticVoxelStore
from rclpy.qos import qos_profile_sensor_data

try:  # ROS 2 package; optional in pure Python tests
    from sensor_msgs_py import point_cloud2
except Exception:  # pragma: no cover
    point_cloud2 = None


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class SemanticVoxelNode(Node):
    """ROS bridge for persistent semantic voxel memory.

    Inputs:
      * /go2_voxel/write_observation: JSON single observation.
      * /go2_voxel/query: JSON query request.
      * /point_cloud2: optional live point cloud ingestion when enabled.

    Outputs:
      * /go2_voxel/summary: JSON responses and status.
      * /go2_voxel/markers: RViz MarkerArray for recently updated voxels.
    """

    def __init__(self) -> None:
        super().__init__("go2_semantic_voxel_node")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("voxel_size_m", 0.25)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("enable_pointcloud_ingest", False)
        self.declare_parameter("pointcloud_topic", "/point_cloud2")
        self.declare_parameter("pointcloud_stride", 25)
        self.declare_parameter("pointcloud_max_points", 2000)
        self.declare_parameter("pointcloud_min_period_sec", 2.0)
        self.store = SemanticVoxelStore(
            self.get_parameter("session_root").value,
            self.get_parameter("session_name").value,
            float(self.get_parameter("voxel_size_m").value),
        )
        self.latest_cloud_points: List[Dict[str, float]] = []
        self.last_cloud_ingest_time = 0.0
        self.create_subscription(String, "/go2_voxel/query", self._on_query, 10)
        if _as_bool(self.get_parameter("enable_pointcloud_ingest").value):
            self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), self._on_pointcloud, qos_profile_sensor_data)
        self.summary_pub = self.create_publisher(String, "/go2_voxel/summary", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/go2_voxel/markers", 10)
        self.create_timer(2.0, self._publish_markers)
        self.get_logger().info("Semantic voxel memory node ready")

    def _on_write(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if data.get("points"):
                result = self.store.write_pointcloud_observation(
                    data.get("points", []),
                    labels=data.get("semantic_labels") or ["pointcloud_observed"],
                    properties=data.get("properties", {}),
                    max_points=int(data.get("max_points", self.get_parameter("pointcloud_max_points").value)),
                )
                self._publish({"success": True, "result": result})
            else:
                record = self.store.write_observation(data.get("center_xyz", {}), data.get("semantic_labels", []), data.get("properties", {}))
                self._publish({"success": True, "record": record})
        except Exception as exc:
            self._publish({"success": False, "message": str(exc)})

    def _on_query(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or "{}")
            mode = data.get("mode", "near")
            if mode == "summary":
                result = self.store.export_summary()
            elif mode == "landmarks":
                result = {"landmarks": query_localization_landmarks(self.store, float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("radius_m", 3.0)))}
            elif mode == "traversability":
                result = self.store.get_traversability(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("radius_m", 0.75)))
            elif mode == "compare_current_pointcloud":
                result = self.store.compare_pointcloud(self.latest_cloud_points, int(data.get("match_radius_voxels", 0)))
            elif mode == "compare_points":
                result = self.store.compare_pointcloud(data.get("points", []), int(data.get("match_radius_voxels", 0)))
            else:
                voxels = self.store.query_near(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("radius_m", 2.0)), int(data.get("limit", 100)))
                result = summarize_voxels(voxels)
                result["voxels"] = voxels
            self._publish(result)
        except Exception as exc:
            self._publish({"success": False, "message": str(exc)})

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_cloud_ingest_time < float(self.get_parameter("pointcloud_min_period_sec").value):
            return
        self.last_cloud_ingest_time = now
        points = self._points_from_cloud(msg)
        self.latest_cloud_points = points
        if not points:
            return
        result = self.store.write_pointcloud_observation(
            points,
            labels=["pointcloud_observed"],
            properties={"source": "pointcloud2", "frame_id": msg.header.frame_id, "source_confidence": {"pointcloud": 0.8}},
            max_points=int(self.get_parameter("pointcloud_max_points").value),
        )
        self._publish({"success": True, "pointcloud_ingest": result})

    def _points_from_cloud(self, msg: PointCloud2) -> List[Dict[str, float]]:
        if point_cloud2 is None:
            self.get_logger().warn("sensor_msgs_py.point_cloud2 unavailable; cannot ingest PointCloud2")
            return []
        stride = max(1, int(self.get_parameter("pointcloud_stride").value))
        max_points = int(self.get_parameter("pointcloud_max_points").value)
        out: List[Dict[str, float]] = []
        for idx, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if idx % stride != 0:
                continue
            out.append({"x": float(point[0]), "y": float(point[1]), "z": float(point[2])})
            if len(out) >= max_points:
                break
        return out

    def _publish_markers(self) -> None:
        if _as_bool(self.get_parameter("publish_markers").value):
            self.marker_pub.publish(markers_from_voxels(self.store.read_all(limit=500), frame_id=str(self.get_parameter("marker_frame_id").value)))

    def _publish(self, payload: Dict[str, Any]) -> None:
        self.summary_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticVoxelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
