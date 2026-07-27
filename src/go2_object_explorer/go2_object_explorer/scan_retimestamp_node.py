#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class ScanRetimestampNode(Node):
    def __init__(self):
        super().__init__("object_explorer_scan_retimestamp_node")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_nav")

        # Kept only for compatibility with older launch files.
        # The frame must never be overwritten without transforming scan points.
        self.declare_parameter("frame_id", "")

        # Use a slightly past timestamp to avoid requesting TF in the future.
        self.declare_parameter("stamp_offset_sec", -0.08)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )

        requested_frame = str(self.get_parameter("frame_id").value)
        if requested_frame:
            self.get_logger().warn(
                f"Ignoring frame_id override '{requested_frame}'. "
                "LaserScan frame is preserved from the source message."
            )

        self.get_logger().info(
            f"Retimestamping {input_topic} -> {output_topic} "
            "while preserving the original scan frame"
        )

    def on_scan(self, msg: LaserScan):
        out = LaserScan()

        # Preserve the physical sensor frame.
        out.header.frame_id = msg.header.frame_id

        offset = float(self.get_parameter("stamp_offset_sec").value)
        stamp_ns = self.get_clock().now().nanoseconds + int(offset * 1e9)
        out.header.stamp = Time(nanoseconds=max(0, stamp_ns)).to_msg()

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


def main():
    rclpy.init()
    node = ScanRetimestampNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
