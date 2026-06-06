#!/usr/bin/env python3
from __future__ import annotations

import io
import sys

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_msg_to_jpeg_bytes(msg: Image) -> bytes | None:
    try:
        width = int(msg.width)
        height = int(msg.height)
        enc = (msg.encoding or '').lower()
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ('rgb8', '8uc3'):
            arr = raw.reshape((height, msg.step // 3, 3))[:, :width, :]
            pil = PILImage.fromarray(arr, mode='RGB')
        elif enc == 'bgr8':
            arr = raw.reshape((height, msg.step // 3, 3))[:, :width, :]
            pil = PILImage.fromarray(arr[:, :, ::-1], mode='RGB')
        elif enc in ('rgba8', '8uc4'):
            arr = raw.reshape((height, msg.step // 4, 4))[:, :width, :]
            pil = PILImage.fromarray(arr, mode='RGBA').convert('RGB')
        elif enc == 'bgra8':
            arr = raw.reshape((height, msg.step // 4, 4))[:, :width, :]
            arr = arr[:, :, [2, 1, 0, 3]]
            pil = PILImage.fromarray(arr, mode='RGBA').convert('RGB')
        elif enc in ('mono8', '8uc1'):
            arr = raw.reshape((height, msg.step))[:, :width]
            pil = PILImage.fromarray(arr, mode='L').convert('RGB')
        else:
            return None
        output = io.BytesIO()
        pil.save(output, format='JPEG', quality=85)
        return output.getvalue()
    except Exception:
        return None


class CameraProbe(Node):
    def __init__(self) -> None:
        super().__init__('camera_probe')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)
        self.got = False

    def cb(self, msg: Image) -> None:
        jpeg = image_msg_to_jpeg_bytes(msg)
        self.get_logger().info(
            f"encoding={msg.encoding} width={msg.width} height={msg.height} step={msg.step} "
            f"data_len={len(msg.data)} jpeg_bytes={len(jpeg) if jpeg else 0}"
        )
        self.got = True
        rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = CameraProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
    return 0 if node.got else 1


if __name__ == '__main__':
    raise SystemExit(main())
