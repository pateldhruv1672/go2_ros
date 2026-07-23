from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from go2_semantic_nav_agent.semantic_demo_utils import SemanticPose, load_semantic_poses


class SemanticDemoMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("semantic_demo_marker_publisher")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "latest")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("marker_topic", "/semantic_nav/demo_markers")
        self.declare_parameter("status_topic", "/semantic_nav/demo_status")
        self.declare_parameter("z_offset", 0.15)

        marker_topic = self.get_parameter("marker_topic").value
        status_topic = self.get_parameter("status_topic").value
        period = float(self.get_parameter("publish_period_sec").value)

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.timer = self.create_timer(max(0.2, period), self._on_timer)
        self._last_count = -1
        self.get_logger().info(f"Publishing semantic demo markers on {marker_topic}")

    def _load(self) -> tuple[str, List[SemanticPose]]:
        root = str(self.get_parameter("session_root").value)
        name = str(self.get_parameter("session_name").value)
        return load_semantic_poses(root, name)

    def _make_sphere(self, idx: int, pose: SemanticPose) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose.frame_id or "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic_places"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = float(self.get_parameter("z_offset").value)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        if pose.name.lower() == "spawn":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 0.95
        else:
            marker.color.r = 0.1
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.90
        return marker

    def _make_arrow(self, idx: int, pose: SemanticPose) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose.frame_id or "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic_place_yaw"
        marker.id = idx
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = float(self.get_parameter("z_offset").value) + 0.10
        marker.pose.orientation.z = math.sin(pose.yaw * 0.5)
        marker.pose.orientation.w = math.cos(pose.yaw * 0.5)
        marker.scale.x = 0.45
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        marker.color.a = 0.90
        return marker

    def _make_text(self, idx: int, pose: SemanticPose) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose.frame_id or "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic_place_labels"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = float(self.get_parameter("z_offset").value) + 0.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = pose.name
        return marker

    def _delete_all(self) -> Marker:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def _on_timer(self) -> None:
        try:
            session_dir, poses = self._load()
        except Exception as exc:
            self.get_logger().warn(f"Could not load semantic session: {exc}")
            return

        arr = MarkerArray()
        arr.markers.append(self._delete_all())
        mid = 1
        for pose in poses:
            arr.markers.append(self._make_sphere(mid, pose)); mid += 1
            arr.markers.append(self._make_arrow(mid, pose)); mid += 1
            arr.markers.append(self._make_text(mid, pose)); mid += 1
        self.marker_pub.publish(arr)

        if len(poses) != self._last_count:
            self._last_count = len(poses)
            msg = String()
            msg.data = f"semantic_demo_markers session={session_dir} places={len(poses)}"
            self.status_pub.publish(msg)
            self.get_logger().info(msg.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticDemoMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
