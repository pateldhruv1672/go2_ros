from __future__ import annotations

import json
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

try:
    from sensor_msgs_py import point_cloud2
except Exception:  # pragma: no cover
    point_cloud2 = None


def _sample_points(msg: PointCloud2, limit: int = 4000) -> Iterable[Tuple[float, float, float]]:
    if point_cloud2 is None:
        return []
    pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    out = []
    for idx, p in enumerate(pts):
        if idx >= limit:
            break
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


def summarize_pointcloud(msg: PointCloud2) -> dict:
    pts = list(_sample_points(msg))
    if not pts:
        return {'point_count_sampled': 0, 'message': 'no points sampled or sensor_msgs_py unavailable'}
    xs, ys, zs = zip(*pts)
    return {
        'point_count_sampled': len(pts),
        'bounds': {
            'min_x': min(xs), 'max_x': max(xs),
            'min_y': min(ys), 'max_y': max(ys),
            'min_z': min(zs), 'max_z': max(zs),
        },
        'centroid': {'x': sum(xs) / len(xs), 'y': sum(ys) / len(ys), 'z': sum(zs) / len(zs)},
        'height_span_m': max(zs) - min(zs),
        'width_span_m': max(ys) - min(ys),
        'depth_span_m': max(xs) - min(xs),
        'object_candidate_hint': len(pts) > 50 and (max(zs) - min(zs)) > 0.25,
    }


class PointCloudAnalyzerNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_pointcloud_analyzer_node')
        self.declare_parameter('pointcloud_topic', '/point_cloud2')
        self.pub = self.create_publisher(String, '/go2_perception/pointcloud_summary', 10)
        self.create_subscription(PointCloud2, self.get_parameter('pointcloud_topic').value, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info('Point-cloud analyzer ready')

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.pub.publish(String(data=json.dumps(summarize_pointcloud(msg), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudAnalyzerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
