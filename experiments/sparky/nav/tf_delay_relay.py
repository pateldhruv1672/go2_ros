#!/usr/bin/env python3
"""Republish odom->base_footprint TF from /odom with a simulated-time delay, preserving source stamps.
Delay 0 = control arm (same apparatus, immediate republish). The EKF's own TF publication is disabled."""
import sys, csv, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class Relay(Node):
    def __init__(self, delay_s, log_path, mode):
        super().__init__('tf_delay_relay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.delay_cfg = delay_s; self.mode = mode
        self.active = (mode == 'start')
        self.delay = delay_s if self.active else 0.0
        self.create_subscription(Bool, '/tf_delay/active', self.on_active, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.q = collections.deque()
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 50)
        self.timer = self.create_timer(0.005, self.tick)
        self.f = open(log_path, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['source_stamp', 'delivery_sim_time', 'age_s', 'intended_delay_s', 'fault_active'])
        self.n = 0
    def on_active(self, m):
        self.active = bool(m.data); self.delay = self.delay_cfg if self.active else 0.0
        self.get_logger().info(f'tf delay active={self.active} delay={self.delay}')
    def cb(self, m):
        self.q.append((self.get_clock().now().nanoseconds * 1e-9 + self.delay, m))
        if self.delay <= 0: self.tick()
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.q and self.q[0][0] <= now + 1e-9:
            _, m = self.q.popleft()
            t = TransformStamped(); t.header = m.header
            t.header.frame_id = m.header.frame_id or 'odom'; t.child_frame_id = m.child_frame_id or 'base_footprint'
            p, o = m.pose.pose.position, m.pose.pose.orientation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = o
            self.pub.publish(TFMessage(transforms=[t]))
            src = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.w.writerow([f'{src:.4f}', f'{now:.4f}', f'{now - src:.4f}', self.delay, int(self.active)])
            self.n += 1
            if self.n % 50 == 0: self.f.flush()

def main():
    rclpy.init()
    n = Relay(float(sys.argv[1]) / 1000.0, sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else 'start')
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.f.close()
main()
