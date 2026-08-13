#!/usr/bin/env python3

import math
import sys
import fcntl
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist


def twist_mag(t: Twist) -> float:
    return (
        abs(t.linear.x)
        + abs(t.linear.y)
        + abs(t.angular.z)
    )


def clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MotionArbiter(Node):
    def __init__(self):
        super().__init__('go2_motion_arbiter')

        self.declare_parameter('nav2_topic', '/cmd_vel_nav2')
        self.declare_parameter('omi_topic', '/cmd_vel_omi')
        self.declare_parameter('escape_topic', '/cmd_vel_escape')
        self.declare_parameter('output_topic', '/cmd_vel_nav')

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('source_timeout_sec', 0.40)
        self.declare_parameter('nonzero_deadband', 0.01)

        self.declare_parameter('nav2_max_x', 0.75)
        self.declare_parameter('nav2_max_y', 0.0)
        self.declare_parameter('nav2_max_theta', 0.90)
        self.declare_parameter('nav2_min_effective_x', 0.30)
        self.declare_parameter('nav2_zero_subfloor_x', False)

        self.declare_parameter('omi_max_x', 0.25)
        self.declare_parameter('omi_max_y', 0.20)
        self.declare_parameter('omi_max_theta', 0.60)

        self.declare_parameter('escape_max_x', 0.30)
        self.declare_parameter('escape_max_y', 0.25)
        self.declare_parameter('escape_max_theta', 0.70)

        self.timeout = float(self.get_parameter('source_timeout_sec').value)
        self.deadband = float(self.get_parameter('nonzero_deadband').value)

        self.latest = {
            'nav2': {'twist': None, 'stamp': None},
            'omi': {'twist': None, 'stamp': None},
            'escape': {'twist': None, 'stamp': None},
        }

        self.last_source = None

        self.pub = self.create_publisher(
            Twist,
            str(self.get_parameter('output_topic').value),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter('nav2_topic').value),
            lambda msg: self._store('nav2', msg, accept_zero=True),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter('omi_topic').value),
            lambda msg: self._store('omi', msg, accept_zero=False),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter('escape_topic').value),
            lambda msg: self._store('escape', msg, accept_zero=False),
            10,
        )

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"motion arbiter ready: "
            f"nav2={self.get_parameter('nav2_topic').value} "
            f"omi={self.get_parameter('omi_topic').value} "
            f"escape={self.get_parameter('escape_topic').value} "
            f"-> output={self.get_parameter('output_topic').value}"
        )

    def _store(self, source: str, msg: Twist, accept_zero: bool):
        if not accept_zero and twist_mag(msg) < self.deadband:
            self.latest[source]['twist'] = None
            self.latest[source]['stamp'] = None
            return

        self.latest[source]['twist'] = msg
        self.latest[source]['stamp'] = self.get_clock().now()

    def _fresh(self, source: str) -> bool:
        stamp: Optional[Time] = self.latest[source]['stamp']
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= self.timeout

    def _select(self):
        for source in ('escape', 'omi', 'nav2'):
            if self._fresh(source) and self.latest[source]['twist'] is not None:
                return source, self.latest[source]['twist']
        return 'none', Twist()

    def _clip_twist(self, source: str, msg: Twist) -> Twist:
        # SPARKY_CONTROLLER_COMMAND_FIDELITY_V13_1
        # Do not rewrite a DWB trajectory after it has been scored. The Go2
        # executable floor belongs in the controller/physical calibration, not
        # as a downstream component mutation that turns arcs into pure spins.
        out = Twist()
        max_x = float(self.get_parameter(f'{source}_max_x').value) if source != 'none' else 0.0
        max_y = float(self.get_parameter(f'{source}_max_y').value) if source != 'none' else 0.0
        max_theta = float(self.get_parameter(f'{source}_max_theta').value) if source != 'none' else 0.0
        out.linear.x = clip(msg.linear.x, -max_x, max_x)
        out.linear.y = clip(msg.linear.y, -max_y, max_y)
        out.angular.z = clip(msg.angular.z, -max_theta, max_theta)
        return out

    def _tick(self):
        source, msg = self._select()

        if source != self.last_source:
            self.get_logger().info(f"active_source={source}")
            self.last_source = source

        if source == 'none':
            self.pub.publish(Twist())
            return

        self.pub.publish(self._clip_twist(source, msg))


def main(args=None):
    # SPARKY_MOTION_ARBITER_SINGLETON_V13_4
    # One and only one process may own /cmd_vel_nav2 + /cmd_vel_escape -> /cmd_vel_nav.
    lock_handle = open('/tmp/go2_motion_arbiter.lock', 'a+')
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('ERROR: another go2_motion_arbiter already owns the motion command path', file=sys.stderr)
        return
    rclpy.init(args=args)
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
