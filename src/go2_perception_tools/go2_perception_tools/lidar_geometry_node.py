from __future__ import annotations

import json
import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


def summarize_scan(msg: LaserScan, opening_threshold_m: float = 1.8) -> dict:
    finite = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
    if not finite:
        return {'valid_count': 0, 'message': 'no valid ranges'}
    n = len(msg.ranges)
    sectors = {}
    for name, start, end in [
        ('front', int(n * 0.45), int(n * 0.55)),
        ('left', int(n * 0.70), int(n * 0.90)),
        ('right', int(n * 0.10), int(n * 0.30)),
        ('rear', 0, int(n * 0.08)),
    ]:
        vals = [r for r in msg.ranges[start:end] if math.isfinite(r)]
        sectors[name] = min(vals) if vals else None
    openings: List[dict] = []
    start_idx = None
    for i, r in enumerate(msg.ranges):
        is_open = math.isfinite(r) and r >= opening_threshold_m
        if is_open and start_idx is None:
            start_idx = i
        if (not is_open or i == n - 1) and start_idx is not None:
            end_idx = i if not is_open else i + 1
            if end_idx - start_idx > max(3, int(n * 0.02)):
                center = (start_idx + end_idx) / 2.0
                angle = msg.angle_min + center * msg.angle_increment
                openings.append({'start_index': start_idx, 'end_index': end_idx, 'center_angle_rad': angle})
            start_idx = None
    return {
        'valid_count': len(finite),
        'min_range_m': min(finite),
        'max_range_m': max(finite),
        'mean_range_m': sum(finite) / len(finite),
        'sector_clearance_m': sectors,
        'opening_count': len(openings),
        'openings': openings[:8],
        'corridor_width_hint_m': (sectors.get('left') or 0.0) + (sectors.get('right') or 0.0),
    }


class LidarGeometryNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_lidar_geometry_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('opening_threshold_m', 1.8)
        self.pub = self.create_publisher(String, '/go2_perception/scan_summary', 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('LiDAR geometry analyzer ready')

    def _on_scan(self, msg: LaserScan) -> None:
        summary = summarize_scan(msg, float(self.get_parameter('opening_threshold_m').value))
        self.pub.publish(String(data=json.dumps(summary, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarGeometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
