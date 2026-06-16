#!/usr/bin/env python3

import json
import math
import socket
from typing import Any, Dict, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def read_xyz(state: Dict[str, Any]) -> Tuple[float, float, float]:
    if "position" in state and isinstance(state["position"], (list, tuple)) and len(state["position"]) >= 3:
        return safe_float(state["position"][0]), safe_float(state["position"][1]), safe_float(state["position"][2])

    if "pos" in state and isinstance(state["pos"], (list, tuple)) and len(state["pos"]) >= 3:
        return safe_float(state["pos"][0]), safe_float(state["pos"][1]), safe_float(state["pos"][2])

    return safe_float(state.get("x")), safe_float(state.get("y")), safe_float(state.get("z"))


def read_velocity(state: Dict[str, Any]) -> Tuple[float, float, float]:
    if "linear_velocity" in state and isinstance(state["linear_velocity"], (list, tuple)) and len(state["linear_velocity"]) >= 2:
        vx = safe_float(state["linear_velocity"][0])
        vy = safe_float(state["linear_velocity"][1])
    elif "vel" in state and isinstance(state["vel"], (list, tuple)) and len(state["vel"]) >= 2:
        vx = safe_float(state["vel"][0])
        vy = safe_float(state["vel"][1])
    else:
        vx = safe_float(state.get("vx"))
        vy = safe_float(state.get("vy"))

    wz = safe_float(state.get("wz", state.get("yaw_rate", 0.0)))
    return vx, vy, wz


def read_quat_or_yaw(state: Dict[str, Any]) -> Tuple[float, float, float, float]:
    if "orientation" in state and isinstance(state["orientation"], (list, tuple)) and len(state["orientation"]) >= 4:
        return (
            safe_float(state["orientation"][0]),
            safe_float(state["orientation"][1]),
            safe_float(state["orientation"][2]),
            safe_float(state["orientation"][3], 1.0),
        )

    if "quat" in state and isinstance(state["quat"], (list, tuple)) and len(state["quat"]) >= 4:
        return (
            safe_float(state["quat"][0]),
            safe_float(state["quat"][1]),
            safe_float(state["quat"][2]),
            safe_float(state["quat"][3], 1.0),
        )

    yaw = safe_float(state.get("yaw", state.get("heading", 0.0)))
    return yaw_to_quat(yaw)


class Go2IsaacUdpRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_isaac_udp_ros_bridge")

        self.declare_parameter("state_host", "127.0.0.1")
        self.declare_parameter("state_port", 15001)
        self.declare_parameter("cmd_host", "127.0.0.1")
        self.declare_parameter("cmd_port", 15000)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("timer_hz", 60.0)

        state_host = str(self.get_parameter("state_host").value)
        state_port = int(self.get_parameter("state_port").value)

        self.cmd_host = str(self.get_parameter("cmd_host").value)
        self.cmd_port = int(self.get_parameter("cmd_port").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind((state_host, state_port))
        self.state_sock.setblocking(False)

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        timer_hz = safe_float(self.get_parameter("timer_hz").value, 60.0)
        self.create_timer(1.0 / max(timer_hz, 1.0), self.tick)

        self.last_state_time = self.get_clock().now()

        self.get_logger().info(
            f"IsaacLab UDP bridge ready. State UDP {state_host}:{state_port} -> /odom + /tf, "
            f"/cmd_vel_out or /cmd_vel -> UDP {self.cmd_host}:{self.cmd_port}"
        )

    def cmd_cb(self, msg: Twist) -> None:
        payload = {
            "vx": float(msg.linear.x),
            "vy": float(msg.linear.y),
            "wz": float(msg.angular.z),
        }
        self.cmd_sock.sendto(
            json.dumps(payload).encode("utf-8"),
            (self.cmd_host, self.cmd_port),
        )

    def tick(self) -> None:
        latest = None

        while True:
            try:
                data, _ = self.state_sock.recvfrom(65535)
                latest = json.loads(data.decode("utf-8"))
            except BlockingIOError:
                break
            except Exception as exc:
                self.get_logger().warn(f"Bad IsaacLab UDP state packet: {exc}")
                break

        if latest is None:
            return

        now = self.get_clock().now().to_msg()

        x, y, z = read_xyz(latest)
        vx, vy, wz = read_velocity(latest)
        qx, qy, qz, qw = read_quat_or_yaw(latest)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = Go2IsaacUdpRosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
