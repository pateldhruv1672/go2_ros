from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TraversabilityNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_traversability_node')
        self.scan_summary = {}
        self.cloud_summary = {}
        self.pub = self.create_publisher(String, '/go2_perception/traversability_summary', 10)
        self.create_subscription(String, '/go2_perception/scan_summary', self._on_scan, 10)
        self.create_subscription(String, '/go2_perception/pointcloud_summary', self._on_cloud, 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info('Traversability fusion node ready')

    def _on_scan(self, msg: String) -> None:
        self.scan_summary = json.loads(msg.data)

    def _on_cloud(self, msg: String) -> None:
        self.cloud_summary = json.loads(msg.data)

    def _publish(self) -> None:
        front = (self.scan_summary.get('sector_clearance_m') or {}).get('front')
        score = 0.5
        reasons = []
        if front is not None:
            score = min(1.0, max(0.0, float(front) / 2.0))
            reasons.append(f'front_clearance={front:.2f}m')
        if self.cloud_summary.get('object_candidate_hint'):
            score = min(score, 0.55)
            reasons.append('object_candidate_from_pointcloud')
        self.pub.publish(String(data=json.dumps({'traversability_score': score, 'reasons': reasons}, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraversabilityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
