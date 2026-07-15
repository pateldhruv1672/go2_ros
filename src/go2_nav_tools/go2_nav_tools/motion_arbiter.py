#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


@dataclass
class SourceState:
    msg: Twist
    stamp_sec: float


def _clip(v: float, lo: float, hi: float) -> float:
    if math.isnan(v) or math.isinf(v):
        return 0.0
    return max(lo, min(hi, v))


def _zero() -> Twist:
    return Twist()


class MotionArbiter(Node):
    """
    One pre-safety velocity owner.

    Inputs:
      /cmd_vel_escape  highest priority local recovery
      /cmd_vel_omi     direct voice/manual micro motion
      /cmd_vel_nav2    Nav2 DWB output

    Output:
      /cmd_vel_nav     goes into collision_monitor

    Final robot chain:
      /cmd_vel_nav -> collision_monitor -> /cmd_vel_out -> go2_driver_node
    """

    def __init__(self) -> None:
        super().__init__("go2_motion_arbiter")

        self.declare_parameter("nav2_topic", "/cmd_vel_nav2")
        self.declare_parameter("omi_topic", "/cmd_vel_omi")
        self.declare_parameter("escape_topic", "/cmd_vel_escape")
        self.declare_parameter("output_topic", "/cmd_vel_nav")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("source_timeout_sec", 0.40)

        self.declare_parameter("nav2_max_x", 0.75)
        self.declare_parameter("nav2_max_y", 0.0)
        self.declare_parameter("nav2_max_theta", 0.90)

        self.declare_parameter("omi_max_x", 0.25)
        self.declare_parameter("omi_max_y", 0.20)
        self.declare_parameter("omi_max_theta", 0.60)

        self.declare_parameter("escape_max_x", 0.30)
        self.declare_parameter("escape_max_y", 0.25)
        self.declare_parameter("escape_max_theta", 0.70)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sources: Dict[str, SourceState] = {}

        self._pub = self.create_publisher(
            Twist,
            str(self.get_parameter("output_topic").value),
            qos,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("nav2_topic").value),
            lambda msg: self._update("nav2", msg),
            qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("omi_topic").value),
            lambda msg: self._update("omi", msg),
            qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("escape_topic").value),
            lambda msg: self._update("escape", msg),
            qos,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(1.0, rate), self._tick)

        self.get_logger().info(
            f"motion arbiter ready: "
            f"nav2={self.get_parameter('nav2_topic').value} "
            f"omi={self.get_parameter('omi_topic').value} "
            f"escape={self.get_parameter('escape_topic').value} "
            f"-> output={self.get_parameter('output_topic').value}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _update(self, name: str, msg: Twist) -> None:
        self._sources[name] = SourceState(msg=msg, stamp_sec=self._now_sec())

    def _fresh(self, name: str) -> Optional[Twist]:
        state = self._sources.get(name)
        if state is None:
            return None
        timeout = float(self.get_parameter("source_timeout_sec").value)
        if self._now_sec() - state.stamp_sec > timeout:
            return None
        return state.msg

    def _select(self) -> tuple[str, Twist]:
        # Priority order.
        for name in ("escape", "omi", "nav2"):
            msg = self._fresh(name)
            if msg is not None:
                return name, msg
        return "none", _zero()

    def _limits(self, source: str) -> tuple[float, float, float]:
        if source == "escape":
            return (
                float(self.get_parameter("escape_max_x").value),
                float(self.get_parameter("escape_max_y").value),
                float(self.get_parameter("escape_max_theta").value),
            )
        if source == "omi":
            return (
                float(self.get_parameter("omi_max_x").value),
                float(self.get_parameter("omi_max_y").value),
                float(self.get_parameter("omi_max_theta").value),
            )
        return (
            float(self.get_parameter("nav2_max_x").value),
            float(self.get_parameter("nav2_max_y").value),
            float(self.get_parameter("nav2_max_theta").value),
        )

    def _sanitize(self, source: str, msg: Twist) -> Twist:
        max_x, max_y, max_th = self._limits(source)

        out = Twist()
        out.linear.x = _clip(msg.linear.x, -max_x, max_x)
        out.linear.y = _clip(msg.linear.y, -max_y, max_y)
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = _clip(msg.angular.z, -max_th, max_th)
        return out

    def _tick(self) -> None:
        source, msg = self._select()
        self._pub.publish(self._sanitize(source, msg))


def main() -> None:
    rclpy.init()
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
