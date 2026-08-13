#!/usr/bin/env python3
"""Verify /scan_nav is a geometry/time-faithful QoS copy of /scan."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def key(msg):
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


class Probe(Node):
    def __init__(self):
        super().__init__('scan_nav_fidelity_probe')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.raw = {}
        self.matches = 0
        self.deadline = time.monotonic() + 8.0
        self.create_subscription(LaserScan, '/scan', self.on_raw, qos)
        self.create_subscription(LaserScan, '/scan_nav', self.on_nav, qos)

    def on_raw(self, msg):
        self.raw[key(msg)] = msg
        if len(self.raw) > 100:
            self.raw.pop(next(iter(self.raw)))

    def on_nav(self, msg):
        k = key(msg)
        raw = self.raw.get(k)
        if raw is None:
            return
        same_frame = raw.header.frame_id == msg.header.frame_id
        same_len = len(raw.ranges) == len(msg.ranges)
        max_diff = float('inf')
        if same_len:
            diffs = []
            for a, b in zip(raw.ranges, msg.ranges):
                if math.isinf(a) and math.isinf(b):
                    diffs.append(0.0)
                elif math.isnan(a) and math.isnan(b):
                    diffs.append(0.0)
                elif math.isfinite(a) and math.isfinite(b):
                    diffs.append(abs(a - b))
                else:
                    diffs.append(float('inf'))
            max_diff = max(diffs, default=0.0)
        self.matches += 1
        print(
            f'MATCH {self.matches}: stamp={k[0]}.{k[1]:09d} '
            f'frame_raw={raw.header.frame_id} frame_nav={msg.header.frame_id} '
            f'ranges={len(msg.ranges)} max_range_diff={max_diff:.9f}'
        )
        if not (same_frame and same_len and max_diff == 0.0):
            print('FAIL: /scan_nav changed the measurement geometry/header')
            raise SystemExit(2)
        if self.matches >= 5:
            print('PASS: 5 scans matched exactly; /scan_nav is a timestamp/frame-faithful QoS copy of /scan')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = Probe()
    code = 3
    try:
        while rclpy.ok() and time.monotonic() < node.deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'FAIL: only matched {node.matches} scan pairs in 8 seconds')
    except SystemExit as e:
        code = int(e.code or 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
