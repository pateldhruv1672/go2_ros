#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


GO2_JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def yaw_to_quat_xyzw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


class Go2IsaacBridge(Node):
    def __init__(self):
        super().__init__("go2_isaac_bridge_node")

        self.declare_parameter("cmd_host", "127.0.0.1")
        self.declare_parameter("cmd_port", 15000)
        self.declare_parameter("state_host", "127.0.0.1")
        self.declare_parameter("state_port", 15001)
        self.declare_parameter("cmd_topic", "/cmd_vel_out")
        self.declare_parameter("publish_clear_scan", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("scan_frame_id", "base_scan")
        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("imu_frame_id", "imu_link")

        self.cmd_addr = (
            self.get_parameter("cmd_host").value,
            int(self.get_parameter("cmd_port").value),
        )
        self.state_addr = (
            self.get_parameter("state_host").value,
            int(self.get_parameter("state_port").value),
        )

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.publish_clear_scan = as_bool(self.get_parameter("publish_clear_scan").value)

        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value
        self.scan_frame = self.get_parameter("scan_frame_id").value
        self.camera_frame = self.get_parameter("camera_frame_id").value
        self.imu_frame = self.get_parameter("imu_frame_id").value

        self.last_sec = 0
        self.last_nanosec = 0

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind(self.state_addr)
        self.state_sock.setblocking(False)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_wall = time.time()

        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.scan_pub = self.create_publisher(LaserScan, self.get_parameter("scan_topic").value, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        self.create_timer(0.05, self.send_cmd_timer)
        self.create_timer(0.01, self.recv_state_timer)
        self.create_timer(0.05, self.publish_joint_state_timer)

        self.get_logger().info("READY")
        self.get_logger().info(f"Subscribing: {self.cmd_topic}")
        self.get_logger().info("Publishing : /clock, /odom, /tf, /tf_static, /imu, /joint_states")
        self.get_logger().info(f"UDP cmd -> {self.cmd_addr[0]}:{self.cmd_addr[1]}")
        self.get_logger().info(f"UDP state <- {self.state_addr[0]}:{self.state_addr[1]}")

        if self.publish_clear_scan:
            self.get_logger().warn("publish_clear_scan=true: placeholder clear /scan only, not real Isaac lidar.")

    def cmd_cb(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self.last_cmd_wall = time.time()

    def send_cmd_timer(self):
        if time.time() - self.last_cmd_wall > 0.5:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.vx, self.vy, self.wz

        payload = json.dumps({"vx": vx, "vy": vy, "wz": wz}).encode("utf-8")
        self.cmd_sock.sendto(payload, self.cmd_addr)

    def recv_state_timer(self):
        latest = None

        while True:
            try:
                data, _ = self.state_sock.recvfrom(4096)
                latest = json.loads(data.decode("utf-8"))
            except BlockingIOError:
                break
            except Exception as exc:
                self.get_logger().warn(f"bad UDP state: {exc!r}")
                break

        if latest is None:
            return

        t = float(latest.get("t", 0.0))
        x = float(latest.get("x", 0.0))
        y = float(latest.get("y", 0.0))
        z = float(latest.get("z", 0.0))
        yaw = float(latest.get("yaw", 0.0))
        vx = float(latest.get("vx", 0.0))
        vy = float(latest.get("vy", 0.0))
        wz = float(latest.get("wz", 0.0))

        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        self.last_sec = sec
        self.last_nanosec = nanosec

        clock_msg = Clock()
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec
        self.clock_pub.publish(clock_msg)

        qx, qy, qz, qw = yaw_to_quat_xyzw(yaw)

        odom = Odometry()
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec
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

        tf = TransformStamped()
        tf.header.stamp.sec = sec
        tf.header.stamp.nanosec = nanosec
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

        imu = Imu()
        imu.header.stamp.sec = sec
        imu.header.stamp.nanosec = nanosec
        imu.header.frame_id = self.imu_frame
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = wz
        self.imu_pub.publish(imu)

        if self.publish_clear_scan:
            self.publish_placeholder_scan(sec, nanosec)

    def publish_static_transforms(self):
        transforms = []

        def add_tf(parent, child, x, y, z, yaw=0.0):
            qx, qy, qz, qw = yaw_to_quat_xyzw(yaw)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)

        add_tf(self.base_frame, self.scan_frame, 0.20, 0.0, 0.25)
        add_tf(self.base_frame, self.camera_frame, 0.25, 0.0, 0.30)
        add_tf(self.base_frame, self.imu_frame, 0.0, 0.0, 0.10)

        self.static_tf_broadcaster.sendTransform(transforms)

    def publish_joint_state_timer(self):
        msg = JointState()
        msg.header.stamp.sec = self.last_sec
        msg.header.stamp.nanosec = self.last_nanosec
        msg.name = GO2_JOINT_NAMES
        msg.position = [0.0] * len(GO2_JOINT_NAMES)
        msg.velocity = [0.0] * len(GO2_JOINT_NAMES)
        msg.effort = [0.0] * len(GO2_JOINT_NAMES)
        self.joint_pub.publish(msg)

    def publish_placeholder_scan(self, sec, nanosec):
        msg = LaserScan()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = self.scan_frame
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 3.14159 / 360.0
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 8.0
        count = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [msg.range_max] * count
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = Go2IsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
