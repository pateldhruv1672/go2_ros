#!/usr/bin/env python3
import math
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DURATION = 12.0
if len(sys.argv) >= 2:
    try:
        DURATION = float(sys.argv[1])
    except ValueError:
        pass

class Trace(Node):
    def __init__(self):
        super().__init__('sparky_rotation_shim_rpp_trace')
        self.data = defaultdict(list)
        self.odom = []
        for topic in ('/cmd_vel_nav2', '/cmd_vel_nav', '/cmd_vel_out'):
            self.create_subscription(Twist, topic, lambda m, t=topic: self.cb(t, m), 20)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)

    def cb(self, topic, msg):
        self.data[topic].append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom.append((time.monotonic(), float(p.x), float(p.y)))

rclpy.init()
n = Trace()
print(f'Tracing {DURATION:.1f}s. Set a Nav2 goal now; for U-turn testing place the goal behind the robot.')
end = time.monotonic() + DURATION
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)

print('\n=== COMMAND SUMMARY ===')
for t in ('/cmd_vel_nav2', '/cmd_vel_nav', '/cmd_vel_out'):
    a = n.data[t]
    if not a:
        print(t, 'NO DATA')
        continue
    moving = [(x,w) for _,x,w in a if abs(x) > 0.01 or abs(w) > 0.01]
    rotations = [(x,w) for x,w in moving if abs(x) < 0.05 and abs(w) >= 0.40]
    translating = [(x,w) for x,w in moving if abs(x) >= 0.30]
    max_x = max((abs(x) for _,x,w in a), default=0.0)
    max_w = max((abs(w) for _,x,w in a), default=0.0)
    print(f'{t}: samples={len(a)} moving={len(moving)} rotate_in_place={len(rotations)} useful_translation={len(translating)} max|vx|={max_x:.3f} max|wz|={max_w:.3f}')
    if moving:
        print('  first moving:', [(round(x,3), round(w,3)) for x,w in moving[:12]])

if len(n.odom) >= 2:
    x0,y0 = n.odom[0][1], n.odom[0][2]
    x1,y1 = n.odom[-1][1], n.odom[-1][2]
    print(f'odom displacement={math.hypot(x1-x0,y1-y0):.3f} m')
else:
    print('odom displacement=NO DATA')

nav = n.data['/cmd_vel_nav2']
if nav:
    rotations = [(x,w) for _,x,w in nav if abs(x) < 0.05 and abs(w) >= 0.40]
    useful = [(x,w) for _,x,w in nav if abs(x) >= 0.30]
    print('initial_alignment_detected=', bool(rotations))
    print('useful_translation_detected=', bool(useful))

n.destroy_node()
rclpy.shutdown()
