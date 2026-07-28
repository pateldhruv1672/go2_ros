#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class AnnotatedImageQosRelay(Node):
    def __init__(self):
        super().__init__("annotated_image_qos_relay")

        self.declare_parameter(
            "input_topic",
            "/object_explorer/annotated_image",
        )
        self.declare_parameter(
            "output_topic",
            "/object_explorer/annotated_image_rviz",
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        # Best-effort subscription is compatible with both:
        #   best-effort image publishers
        #   reliable image publishers
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # RViz can consume this reliably.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            Image,
            output_topic,
            output_qos,
        )

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.on_image,
            input_qos,
        )

        self.frame_count = 0

        self.get_logger().info(
            f"Image QoS relay: {input_topic} -> {output_topic}; "
            "input=best_effort, output=reliable"
        )

    def on_image(self, msg: Image):
        # Preserve timestamp, frame, encoding, dimensions, and image bytes.
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info(
                f"First annotated image relayed: "
                f"{msg.width}x{msg.height}, encoding={msg.encoding}"
            )


def main():
    rclpy.init()
    node = AnnotatedImageQosRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
