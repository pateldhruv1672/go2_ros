#!/usr/bin/env python3
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from rclpy.node import Node

class Trace(Node):
    def __init__(self):
        super().__init__('sparky_rpp_trace_v13_11')
        self.data = {k: [] for k in ('nav2', 'nav', 'out')}
        self.odom = []
        self.limits = []
        self.create_subscription(Twist, '/cmd_vel_nav2', lambda m: self.add('nav2', m), 30)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.add('nav', m), 30)
        self.create_subscription(Twist, '/cmd_vel_out', lambda m: self.add('out', m), 30)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 30)
        self.create_subscription(SpeedLimit, '/speed_limit', self.limit_cb, 10)

    def add(self, key, msg):
        self.data[key].append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom.append((time.monotonic(), float(p.x), float(p.y)))

    def limit_cb(self, msg):
        self.limits.append((time.monotonic(), float(msg.speed_limit), bool(msg.percentage)))

def summarize(values):
    if not values:
        return 'NO DATA'
    nz = [(x, w) for _, x, w in values if abs(x) > 1e-4 or abs(w) > 1e-4]
    xs = [abs(x) for x, _ in nz if abs(x) > 1e-4]
    ws = [abs(w) for _, w in nz if abs(w) > 1e-4]
    def q(v):
        if not v: return 'n/a'
        return f'min={min(v):.3f} med={statistics.median(v):.3f} max={max(v):.3f}'
    return f'samples={len(values)} nonzero={len(nz)} vx[{q(xs)}] wz[{q(ws)}] first={nz[:8]}'

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=10.0)
a = ap.parse_args()
rclpy.init()
n = Trace()
print(f'Tracing RPP for {a.seconds:.1f}s. Set an RViz goal or start the Tour NOW.')
end = time.monotonic() + a.seconds
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
for key in ('nav2', 'nav', 'out'):
    print(f'{key:>4}: {summarize(n.data[key])}')
if len(n.odom) >= 2:
    _, x0, y0 = n.odom[0]
    _, x1, y1 = n.odom[-1]
    print(f'odom displacement={math.hypot(x1-x0, y1-y0):.3f} m')
else:
    print('odom displacement=NO DATA')
print(f'speed_limits={[(round(v,3), pct) for _,v,pct in n.limits[-5:]]}')
n.destroy_node()
rclpy.shutdown()
