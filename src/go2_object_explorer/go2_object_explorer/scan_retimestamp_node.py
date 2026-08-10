#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRelayNode(Node):
    def __init__(self):
        super().__init__("object_explorer_scan_retimestamp_node")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_nav")

        # Retained so older launch files do not fail, but intentionally ignored.
        self.declare_parameter("frame_id", "")
        self.declare_parameter("stamp_offset_sec", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Forwarding {input_topic} -> {output_topic} while preserving "
            "the original frame and acquisition timestamp"
        )

    def on_scan(self, msg: LaserScan):
        out = LaserScan()

        # Preserve real acquisition metadata.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ScanRelayNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
