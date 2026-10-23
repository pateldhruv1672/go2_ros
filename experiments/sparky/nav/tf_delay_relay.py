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

class Relay(Node):
    def __init__(self, delay_s, log_path):
        super().__init__('tf_delay_relay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.delay = delay_s
        self.q = collections.deque()
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 50)
        self.timer = self.create_timer(0.005, self.tick)
        self.f = open(log_path, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['source_stamp', 'delivery_sim_time', 'age_s', 'intended_delay_s'])
        self.n = 0
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
            self.w.writerow([f'{src:.4f}', f'{now:.4f}', f'{now - src:.4f}', self.delay])
            self.n += 1
            if self.n % 50 == 0: self.f.flush()

def main():
    rclpy.init()
    n = Relay(float(sys.argv[1]) / 1000.0, sys.argv[2])
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.f.close()
main()
