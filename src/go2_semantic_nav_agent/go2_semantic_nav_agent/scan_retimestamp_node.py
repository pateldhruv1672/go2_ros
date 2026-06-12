from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRetimestampNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_retimestamp_node')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('stamp_offset_sec', 0.30)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.stamp_offset_sec = float(self.get_parameter('stamp_offset_sec').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(LaserScan, output_topic, qos)
        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_cb, qos)
        self.get_logger().info(f'Retimestamping {input_topic} -> {output_topic} with BEST_EFFORT QoS')

    def scan_cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        stamp = self.get_clock().now() + rclpy.duration.Duration(seconds=self.stamp_offset_sec)
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.frame_id or msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRetimestampNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
