#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool


def magnitude(msg: Twist) -> float:
    return abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class MotionArbiter(Node):
    def __init__(self) -> None:
        super().__init__("go2_motion_arbiter")
        self.declare_parameter("nav2_topic", "/cmd_vel_nav2")
        self.declare_parameter("omi_topic", "/cmd_vel_omi")
        self.declare_parameter("escape_topic", "/cmd_vel_escape")
        self.declare_parameter("stop_topic", "/go2_motion/stop")
        self.declare_parameter("output_topic", "/cmd_vel_nav")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("source_timeout_sec", 0.40)
        self.declare_parameter("stop_hold_sec", 1.00)
        self.declare_parameter("nonzero_deadband", 0.01)
        self.declare_parameter("nav2_max_x", 0.40)
        self.declare_parameter("nav2_max_y", 0.0)
        self.declare_parameter("nav2_max_theta", 0.70)
        self.declare_parameter("omi_max_x", 0.25)
        self.declare_parameter("omi_max_y", 0.20)
        self.declare_parameter("omi_max_theta", 0.60)
        self.declare_parameter("escape_max_x", 0.25)
        self.declare_parameter("escape_max_y", 0.20)
        self.declare_parameter("escape_max_theta", 0.60)

        self.timeout = float(self.get_parameter("source_timeout_sec").value)
        self.deadband = float(self.get_parameter("nonzero_deadband").value)
        self.stop_until: Optional[Time] = None
        self.latest = {
            "nav2": {"twist": None, "stamp": None},
            "omi": {"twist": None, "stamp": None},
            "escape": {"twist": None, "stamp": None},
        }
        self.last_source = None
        self.pub = self.create_publisher(Twist, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(Twist, str(self.get_parameter("nav2_topic").value), lambda m: self.store("nav2", m, True), 10)
        self.create_subscription(Twist, str(self.get_parameter("omi_topic").value), lambda m: self.store("omi", m, False), 10)
        self.create_subscription(Twist, str(self.get_parameter("escape_topic").value), lambda m: self.store("escape", m, False), 10)
        self.create_subscription(Bool, str(self.get_parameter("stop_topic").value), self.on_stop, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.get_logger().info("motion arbiter ready with explicit stop latch")

    def on_stop(self, msg: Bool) -> None:
        if msg.data:
            hold = float(self.get_parameter("stop_hold_sec").value)
            self.stop_until = self.get_clock().now() + rclpy.duration.Duration(seconds=hold)
        else:
            self.stop_until = None

    def store(self, source: str, msg: Twist, accept_zero: bool) -> None:
        if not accept_zero and magnitude(msg) < self.deadband:
            self.latest[source] = {"twist": None, "stamp": None}
            return
        self.latest[source] = {"twist": msg, "stamp": self.get_clock().now()}

    def fresh(self, source: str) -> bool:
        stamp = self.latest[source]["stamp"]
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 <= self.timeout

    def stop_active(self) -> bool:
        return self.stop_until is not None and self.get_clock().now() <= self.stop_until

    def select(self):
        if self.stop_active():
            return "stop", Twist()
        for source in ("escape", "omi", "nav2"):
            if self.fresh(source) and self.latest[source]["twist"] is not None:
                return source, self.latest[source]["twist"]
        return "none", Twist()

    def clipped(self, source: str, msg: Twist) -> Twist:
        if source in {"none", "stop"}:
            return Twist()
        out = Twist()
        out.linear.x = clamp(msg.linear.x, float(self.get_parameter(f"{source}_max_x").value))
        out.linear.y = clamp(msg.linear.y, float(self.get_parameter(f"{source}_max_y").value))
        out.angular.z = clamp(msg.angular.z, float(self.get_parameter(f"{source}_max_theta").value))
        return out

    def tick(self) -> None:
        source, msg = self.select()
        if source != self.last_source:
            self.get_logger().info(f"active_source={source}")
            self.last_source = source
        self.pub.publish(self.clipped(source, msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
