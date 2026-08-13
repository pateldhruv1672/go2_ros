#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Trace(Node):
    def __init__(self):
        super().__init__('sparky_nav_cmd_trace_v13_10')
        self.data = {k: [] for k in ('nav2','nav','out')}
        self.create_subscription(Twist, '/cmd_vel_nav2', lambda m: self.add('nav2', m), 20)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.add('nav', m), 20)
        self.create_subscription(Twist, '/cmd_vel_out', lambda m: self.add('out', m), 20)
    def add(self, k, m):
        self.data[k].append((float(m.linear.x), float(m.angular.z)))

def stats(vals):
    if not vals: return 'NO DATA'
    nz = [(x,w) for x,w in vals if abs(x) > 1e-4 or abs(w) > 1e-4]
    trans = [abs(x) for x,w in nz if abs(x) > 1e-4]
    exec_trans = [x for x in trans if x >= 0.35 - 1e-3]
    tiny = [x for x in trans if x < 0.30]
    return f'samples={len(vals)} nonzero={len(nz)} trans={len(trans)} exec_x>=0.35={len(exec_trans)} sub0.30={len(tiny)} first={nz[:8]}'

rclpy.init(); n=Trace(); end=time.monotonic()+8.0
print('Trace for 8 seconds. Set an RViz Nav2 goal NOW.')
while time.monotonic()<end:
    rclpy.spin_once(n, timeout_sec=0.05)
for k in ('nav2','nav','out'):
    print(f'{k:>4}: {stats(n.data[k])}')
n.destroy_node(); rclpy.shutdown()
