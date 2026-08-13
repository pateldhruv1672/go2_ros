#!/usr/bin/env python3
import sys, time
import rclpy
from geometry_msgs.msg import Twist

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
rclpy.init()
node = rclpy.create_node('sparky_dwb_reactive_trace_v13_15')
streams = {k: [] for k in ('nav2','nav','out','sdk')}

def cb(name):
    def _cb(msg):
        streams[name].append((time.monotonic(), float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)))
    return _cb

subs = [
    node.create_subscription(Twist, '/cmd_vel_nav2', cb('nav2'), 20),
    node.create_subscription(Twist, '/cmd_vel_nav', cb('nav'), 20),
    node.create_subscription(Twist, '/cmd_vel_out', cb('out'), 20),
    node.create_subscription(Twist, '/cmd_vel_sdk', cb('sdk'), 20),
]
end = time.monotonic() + seconds
print(f'Tracing {seconds:.1f}s. Set an RViz goal now.')
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

for name, vals in streams.items():
    nonzero = [(x,y,w) for _,x,y,w in vals if abs(x)>1e-3 or abs(y)>1e-3 or abs(w)>1e-3]
    print(f'\n{name}: samples={len(vals)} nonzero={len(nonzero)}')
    print('  first_nonzero=', nonzero[:12])
    if nonzero:
        xs=[abs(v[0]) for v in nonzero]
        ws=[abs(v[2]) for v in nonzero]
        print(f'  |vx| min/median/max={min(xs):.3f}/{sorted(xs)[len(xs)//2]:.3f}/{max(xs):.3f}')
        print(f'  |wz| max={max(ws):.3f}')
node.destroy_node(); rclpy.shutdown()
