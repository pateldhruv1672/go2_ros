from __future__ import annotations

import copy
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class PaddedMapNode(Node):
    def __init__(self) -> None:
        super().__init__("padded_map_node")

        self.declare_parameter("input_map_topic", "/map")
        self.declare_parameter("output_map_topic", "/map_padded")
        self.declare_parameter("padding_m", 2.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("unknown_value", -1)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.latest_map: OccupancyGrid | None = None
        self.last_pub = 0.0
        self.last_bad_size_warn = 0.0

        self.pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("output_map_topic").value),
            map_qos,
        )

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("input_map_topic").value),
            self.on_map,
            map_qos,
        )

        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self.publish_padded,
        )

        self.get_logger().info(
            f"padded_map_node ready: "
            f"{self.get_parameter('input_map_topic').value} -> "
            f"{self.get_parameter('output_map_topic').value}"
        )

    def on_map(self, msg: OccupancyGrid) -> None:
        # Keep the latest message, but never mutate msg.info later.
        self.latest_map = msg

    def publish_padded(self) -> None:
        msg = self.latest_map
        if msg is None:
            return

        res = float(msg.info.resolution)
        if res <= 0.0:
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        expected = width * height
        actual = len(msg.data)

        if width <= 0 or height <= 0:
            return

        if actual != expected:
            now = time.time()
            if now - self.last_bad_size_warn > 2.0:
                self.last_bad_size_warn = now
                self.get_logger().warn(
                    f"Skipping malformed map: width={width} height={height} "
                    f"expected_data={expected} actual_data={actual}"
                )
            return

        padding_m = float(self.get_parameter("padding_m").value)
        pad_cells = int(math.ceil(padding_m / res))

        if pad_cells <= 0:
            self.pub.publish(msg)
            return

        src = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        unknown = int(self.get_parameter("unknown_value").value)

        dst_h = height + 2 * pad_cells
        dst_w = width + 2 * pad_cells

        dst = np.full((dst_h, dst_w), unknown, dtype=np.int16)
        dst[pad_cells:pad_cells + height, pad_cells:pad_cells + width] = src

        out = OccupancyGrid()
        out.header = copy.deepcopy(msg.header)
        out.header.stamp = self.get_clock().now().to_msg()

        # Critical: deep-copy info. Do NOT do out.info = msg.info.
        out.info = copy.deepcopy(msg.info)
        out.info.width = int(dst_w)
        out.info.height = int(dst_h)
        out.info.origin.position.x = float(msg.info.origin.position.x) - pad_cells * res
        out.info.origin.position.y = float(msg.info.origin.position.y) - pad_cells * res

        out.data = [int(v) for v in dst.reshape(-1)]

        self.pub.publish(out)

        now = time.time()
        if now - self.last_pub > 3.0:
            self.last_pub = now
            self.get_logger().info(
                f"published padded map: original=({width},{height}) "
                f"padded=({dst_w},{dst_h}) "
                f"origin=({out.info.origin.position.x:.2f},{out.info.origin.position.y:.2f}) "
                f"data={len(out.data)}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaddedMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
