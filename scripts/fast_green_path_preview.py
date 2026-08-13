#!/usr/bin/env python3
import argparse
import copy
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class FastGreenPathPreview(Node):
    def __init__(self, hz: float):
        super().__init__('sparky_fast_green_path_preview')
        self.hz = max(1.0, min(float(hz), 30.0))
        self.plan = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Path, '/plan_fast', 1)
        self.create_subscription(Path, '/plan', self._plan_cb, 1)
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'Fast green path preview: /plan -> /plan_fast at {self.hz:.1f} Hz; display-only, no replanning')

    def _plan_cb(self, msg: Path):
        self.plan = msg

    def _tick(self):
        if self.plan is None or not self.plan.poses:
            return
        src = self.plan
        frame = src.header.frame_id or 'map'
        out = Path()
        out.header = copy.deepcopy(src.header)
        out.header.stamp = self.get_clock().now().to_msg()

        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_link', Time(), timeout=Duration(seconds=0.02))
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            closest = min(
                range(len(src.poses)),
                key=lambda i: math.hypot(src.poses[i].pose.position.x - x, src.poses[i].pose.position.y - y),
            )
            robot = PoseStamped()
            robot.header.frame_id = frame
            robot.header.stamp = out.header.stamp
            robot.pose.position.x = x
            robot.pose.position.y = y
            robot.pose.position.z = float(tf.transform.translation.z)
            robot.pose.orientation = copy.deepcopy(tf.transform.rotation)
            out.poses = [robot]
            # Keep one nearby segment ahead so the displayed line remains smooth.
            out.poses.extend(copy.deepcopy(src.poses[closest:]))
        except TransformException:
            out.poses = copy.deepcopy(src.poses)
            for pose in out.poses:
                pose.header.stamp = out.header.stamp

        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hz', type=float, default=15.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = FastGreenPathPreview(args.hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
