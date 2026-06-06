#!/usr/bin/env python3
from __future__ import annotations

import base64
import io
import json

import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from go2_agentic_system.openrouter_client import OpenRouterClient


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


class SceneProbe(Node):
    def __init__(self) -> None:
        super().__init__('scene_probe')
        self.client = OpenRouterClient()
        self.result: dict | None = None
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)

    def cb(self, msg: Image) -> None:
        jpeg = image_msg_to_jpeg_bytes(msg)
        if not jpeg:
            self.get_logger().error('jpeg conversion failed')
            self.result = {'ok': False, 'error': 'jpeg conversion failed'}
            rclpy.shutdown()
            return
        url = f"data:image/jpeg;base64,{base64.b64encode(jpeg).decode('ascii')}"
        self.result = self.client.describe_scene(url, hint='Return a stable place label and nonzero confidence when the scene is clear.')
        self.get_logger().info(json.dumps(self.result, ensure_ascii=False))
        rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = SceneProbe()
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
    result = node.result or {'ok': False, 'error': 'no result'}
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
