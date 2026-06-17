from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String


class OmiBleBridgeNode(Node):
    """Connection/status shim for Omi-style voice input.

    In transcript_only mode, simulated or mobile-app transcripts are published
    directly to /omi/transcript_raw. In BLE modes, go2_voice_stt_node owns the
    Omi audio connection and converts local STT into /go2_voice/transcript.
    """

    def __init__(self) -> None:
        super().__init__("go2_omi_bridge")
        self.declare_parameter("adapter_mode", "transcript_only")
        self.declare_parameter("ble_device_name", "Omi")
        self.declare_parameter("ble_device_address", "")
        self.declare_parameter("ble_scan_timeout_sec", 10.0)
        self.declare_parameter("reconnect_on_drop", True)
        self.status_pub = self.create_publisher(String, "/omi/status", 10)
        self.connection_pub = self.create_publisher(String, "/omi/connection_state", 10)
        self.battery_pub = self.create_publisher(Float32, "/omi/battery", 10)
        self.create_subscription(String, "/omi/connect", self._on_connect, 10)
        self.create_subscription(Empty, "/omi/disconnect", self._on_disconnect, 10)
        self.create_subscription(Empty, "/omi/reconnect", self._on_reconnect, 10)
        self.connected = False
        self._publish_state("ready")
        mode = str(self.get_parameter("adapter_mode").value)
        self.get_logger().info(f"Omi bridge ready in adapter_mode={mode}")
        if mode != "transcript_only":
            self.get_logger().info("BLE audio is handled by go2_voice_stt_node; this node only publishes Omi status.")

    def _publish_state(self, state: str, extra: dict | None = None) -> None:
        payload = {
            "adapter_mode": str(self.get_parameter("adapter_mode").value),
            "device_name": str(self.get_parameter("ble_device_name").value),
            "device_address": str(self.get_parameter("ble_device_address").value),
            "connected": self.connected,
            "state": state,
        }
        if extra:
            payload.update(extra)
        data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=data))
        self.connection_pub.publish(String(data=data))

    def _on_connect(self, msg: String) -> None:
        target = msg.data.strip() or str(self.get_parameter("ble_device_name").value)
        self.connected = True
        self._publish_state("connected", {"target": target})

    def _on_disconnect(self, _msg: Empty) -> None:
        self.connected = False
        self._publish_state("disconnected")

    def _on_reconnect(self, _msg: Empty) -> None:
        self.connected = True
        self._publish_state("reconnected")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmiBleBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
