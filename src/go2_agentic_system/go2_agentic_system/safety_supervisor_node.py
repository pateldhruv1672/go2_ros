from __future__ import annotations

import math
from datetime import datetime, timezone

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .command_utils import make_event
from .storage import MemoryStore


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_supervisor_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('command_topic', '/agent/commands')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        guardrails = self.store.read_guardrails().get('guardrails', {})
        self.stop_distance = float(guardrails.get('min_lidar_stop_distance_m', 0.55))
        self.warn_distance = float(guardrails.get('min_lidar_warn_distance_m', 0.9))
        self.max_pose_age_sec = float(guardrails.get('max_pose_age_sec', 2.5))
        self.autonomy_requires_fresh_pose = bool(guardrails.get('autonomy_requires_fresh_pose', True))

        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.command_pub = self.create_publisher(String, self.get_parameter('command_topic').value, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 20)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.localization_timer = self.create_timer(1.0, self.check_pose_freshness)
        self.last_odom_wall_time = None
        self.last_stop_reason = ''

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom_wall_time = datetime.now(timezone.utc)

    def on_scan(self, msg: LaserScan) -> None:
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.01]
        if not valid_ranges:
            return
        min_range = min(valid_ranges)
        if min_range <= self.stop_distance:
            reason = f'imminent obstacle at {min_range:.2f}m'
            if reason != self.last_stop_reason:
                self.command_pub.publish(String(data=make_event('cancel_navigation', reason=reason)))
                self.status(f'safety: canceled navigation due to {reason}')
                self.store.log_event('safety_cancel', {'reason': reason})
                self.last_stop_reason = reason
        elif min_range <= self.warn_distance:
            self.status(f'safety: warning obstacle at {min_range:.2f}m')
            self.last_stop_reason = ''
        else:
            self.last_stop_reason = ''

    def check_pose_freshness(self) -> None:
        if not self.autonomy_requires_fresh_pose or self.last_odom_wall_time is None:
            return
        age = (datetime.now(timezone.utc) - self.last_odom_wall_time).total_seconds()
        if age > self.max_pose_age_sec:
            reason = f'pose stale for {age:.1f}s'
            if reason != self.last_stop_reason:
                self.command_pub.publish(String(data=make_event('cancel_navigation', reason=reason)))
                self.status(f'safety: canceled navigation due to {reason}')
                self.store.log_event('safety_cancel', {'reason': reason})
                self.last_stop_reason = reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
