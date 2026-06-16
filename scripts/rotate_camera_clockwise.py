#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class RotateCameraClockwise(Node):
    def __init__(self):
        super().__init__("rotate_camera_clockwise")

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", reliable_qos)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", reliable_qos)

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw_raw",
            self.image_cb,
            reliable_qos,
        )

        self.count = 0
        self.last_info = None
        self.timer = self.create_timer(0.1, self.publish_info)

        self.get_logger().info("READY reliable relay: /camera/image_raw_raw -> rotate CW + flip vertical -> /camera/image_raw")

    def channels_for_encoding(self, encoding):
        e = encoding.lower()
        if e in ["mono8", "8uc1"]:
            return 1
        if e in ["rgb8", "bgr8", "8uc3"]:
            return 3
        if e in ["rgba8", "bgra8", "8uc4"]:
            return 4
        return None

    def image_cb(self, msg):
        channels = self.channels_for_encoding(msg.encoding)
        if channels is None:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return

        try:
            if channels == 1:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                rot = np.flipud(np.rot90(arr, k=3))
                step = rot.shape[1]
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
                rot = np.flipud(np.rot90(arr, k=3))
                step = rot.shape[1] * channels

            out = Image()
            out.header = msg.header
            out.header.frame_id = "camera_link"
            out.height = int(rot.shape[0])
            out.width = int(rot.shape[1])
            out.encoding = msg.encoding
            out.is_bigendian = msg.is_bigendian
            out.step = int(step)
            out.data = rot.tobytes()

            self.image_pub.publish(out)
            self.last_info = self.make_info(out)

            self.count += 1
            if self.count % 30 == 0:
                self.get_logger().info(
                    f"rotated {self.count} frames: {msg.width}x{msg.height} -> {out.width}x{out.height}, encoding={msg.encoding}"
                )

        except Exception as e:
            self.get_logger().error(f"Rotation failed: {repr(e)}")

    def make_info(self, img):
        fx = 554.0
        fy = 554.0
        cx = img.width / 2.0
        cy = img.height / 2.0

        info = CameraInfo()
        info.header = img.header
        info.header.frame_id = "camera_link"
        info.width = img.width
        info.height = img.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_info(self):
        if self.last_info is not None:
            self.last_info.header.stamp = self.get_clock().now().to_msg()
            self.info_pub.publish(self.last_info)


def main():
    rclpy.init()
    node = RotateCameraClockwise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
