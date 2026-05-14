from __future__ import annotations

import base64
import io
import os
from typing import Optional

import numpy as np
from PIL import Image as PILImage
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def image_msg_to_jpeg_bytes(msg: Image) -> Optional[bytes]:
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
        elif enc in ('mono8', '8uc1'):
            arr = raw.reshape((height, msg.step))[:, :width]
            pil = PILImage.fromarray(arr, mode='L').convert('RGB')
        else:
            return None
        buf = io.BytesIO()
        pil.save(buf, format='JPEG', quality=85)
        return buf.getvalue()
    except Exception:
        return None


class CameraAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_agent_node")
        self.declare_parameter("camera_image_topic", "/camera/image_raw")
        self.declare_parameter("camera_compressed_topic", "/camera/image_raw/compressed")
        self.declare_parameter("openrouter_model", "google/gemini-2.5-flash")
        self.declare_parameter("openrouter_base_url", "https://openrouter.ai/api/v1/chat/completions")

        self.latest_image_bytes: Optional[bytes] = None
        self.req_sub = self.create_subscription(String, "/agent/camera/request", self.request_cb, 10)
        self.resp_pub = self.create_publisher(String, "/agent/camera/response", 10)
        self.create_subscription(Image, str(self.get_parameter("camera_image_topic").value), self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, str(self.get_parameter("camera_compressed_topic").value), self.compressed_cb, qos_profile_sensor_data)
        self.get_logger().info("ready")

    def image_cb(self, msg: Image) -> None:
        maybe = image_msg_to_jpeg_bytes(msg)
        if maybe:
            self.latest_image_bytes = maybe

    def compressed_cb(self, msg: CompressedImage) -> None:
        self.latest_image_bytes = bytes(msg.data)

    def request_cb(self, msg: String) -> None:
        text = (msg.data or '').strip()
        if not text:
            return
        reply = self.describe_scene(text)
        out = String()
        out.data = reply
        self.resp_pub.publish(out)
        self.get_logger().info(f"camera_response: {reply}")

    def describe_scene(self, user_prompt: str) -> str:
        if not self.latest_image_bytes:
            return "I do not have a camera frame yet."
        key = os.environ.get("OPENROUTER_API_KEY", "")
        if not key:
            return "OPENROUTER_API_KEY is missing, so I cannot use the camera agent yet."

        prompt = (
            "You are Sparky's camera agent. "
            "Describe the robot's current indoor view in one concise helpful paragraph. "
            "Answer the user's request directly. "
            f"User request: {user_prompt}"
        )
        try:
            b64 = base64.b64encode(self.latest_image_bytes).decode("ascii")
            payload = {
                "model": str(self.get_parameter("openrouter_model").value),
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                    ],
                }],
                "temperature": 0.2,
            }
            headers = {
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            }
            resp = requests.post(
                str(self.get_parameter("openrouter_base_url").value),
                headers=headers, json=payload, timeout=30,
            )
            resp.raise_for_status()
            data = resp.json()
            text = data["choices"][0]["message"]["content"]
            if isinstance(text, list):
                text = " ".join(str(x.get("text", "")) for x in text if isinstance(x, dict))
            return str(text).strip() or "I could not describe the scene."
        except Exception as exc:
            return f"Camera agent failed: {exc}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
