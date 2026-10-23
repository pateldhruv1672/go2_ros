#!/usr/bin/env python3
"""Observe actual ROS messages; no commanded movement and no navigation claims."""
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('sparky_simulator_smoke_observer')
    counts = {'clock': 0, 'ground_truth': 0, 'estimated_odom': 0, 'joint_states': 0}
    last = {}
    stream = (out / 'observations.jsonl').open('w')

    def odom(key):
        def callback(msg):
            counts[key] += 1
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row = {'topic': key, 'receipt_monotonic_ns': time.monotonic_ns(),
                   'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                   'x': p.x, 'y': p.y, 'z': p.z,
                   'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
            last[key] = row
            stream.write(json.dumps(row) + '\n')
        return callback

    def clock(msg):
        counts['clock'] += 1
        last['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def joints(msg):
        counts['joint_states'] += 1
        last['joint_names'] = list(msg.name)

    subs = [node.create_subscription(Odometry, '/odom/ground_truth', odom('ground_truth'), qos_profile_sensor_data),
            node.create_subscription(Odometry, '/odom', odom('estimated_odom'), qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
            node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)]
    start = time.monotonic()
    while time.monotonic() - start < 65 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    # Readiness only, not a gait-stability or navigation result.
    ready = all(v >= 10 for v in counts.values())
    result = {'status': 'MESSAGE_FLOW_OBSERVED' if ready else 'BLOCKED_MISSING_MESSAGE_FLOW',
              'counts': counts, 'last': last, 'navigation_runs': 0, 'controller_executed': None}
    (out / 'smoke_status.json').write_text(json.dumps(result, indent=2) + '\n')
    stream.close()
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if ready else 2


if __name__ == '__main__':
    raise SystemExit(main())
