#!/usr/bin/env python3
import argparse
import math
import statistics
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TWIST_TOPICS = ["/cmd_vel_nav2", "/cmd_vel_nav", "/cmd_vel_out", "/cmd_vel_sdk"]

class Monitor(Node):
    def __init__(self, duration):
        super().__init__("sparky_command_response_v11_6_1")
        self.duration = duration
        self.t0 = time.monotonic()
        self.events = defaultdict(list)

        for topic in TWIST_TOPICS:
            self.create_subscription(
                Twist, topic,
                lambda msg, t=topic: self.twist_cb(t, msg),
                50,
            )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 100)

    def now(self):
        return time.monotonic() - self.t0

    def twist_cb(self, topic, msg):
        self.events[topic].append(
            (self.now(), float(msg.linear.x), float(msg.angular.z))
        )

    def odom_cb(self, msg):
        self.events["/odom"].append(
            (
                self.now(),
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
        )

def rate(events):
    if len(events) < 2:
        return 0.0
    dt = events[-1][0] - events[0][0]
    return (len(events) - 1) / dt if dt > 0 else 0.0

def peak(events, idx):
    return max((abs(e[idx]) for e in events), default=0.0)

def rms(events, idx):
    if not events:
        return 0.0
    return math.sqrt(sum(e[idx] * e[idx] for e in events) / len(events))

def sample_at(events, t, idx):
    # latest event at or before t
    if not events:
        return 0.0
    lo, hi = 0, len(events) - 1
    if events[0][0] > t:
        return 0.0
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if events[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return events[lo][idx]

def chain_diff(a, b, idx):
    if not a or not b:
        return None
    vals = []
    for t, _, _ in a:
        va = sample_at(a, t, idx)
        vb = sample_at(b, t, idx)
        vals.append(va - vb)
    if not vals:
        return None
    return (
        math.sqrt(sum(v*v for v in vals) / len(vals)),
        max(abs(v) for v in vals),
    )

def correlation(a, b):
    if len(a) < 10:
        return None
    ma = statistics.fmean(a)
    mb = statistics.fmean(b)
    da = [x - ma for x in a]
    db = [x - mb for x in b]
    va = sum(x*x for x in da)
    vb = sum(x*x for x in db)
    if va < 1e-9 or vb < 1e-9:
        return None
    return sum(x*y for x, y in zip(da, db)) / math.sqrt(va * vb)

def lag_gain(cmd, odom, idx, max_lag=1.0, step=0.02):
    # Only evaluate when both command and measured response contain meaningful motion.
    cmd_peak = peak(cmd, idx)
    odom_peak = peak(odom, idx)
    cmd_min = 0.025 if idx == 1 else 0.025
    obs_min = 0.015 if idx == 1 else 0.015
    if cmd_peak < cmd_min or odom_peak < obs_min:
        return None, (
            f"insufficient physical response: cmd_peak={cmd_peak:.4f}, "
            f"odom_peak={odom_peak:.4f}"
        )

    t_start = max(cmd[0][0], odom[0][0])
    t_end = min(cmd[-1][0], odom[-1][0])
    if t_end - t_start < 2.0:
        return None, "insufficient overlapping duration"

    best = None
    lag = 0.0
    while lag <= max_lag + 1e-9:
        xs, ys = [], []
        t = t_start
        while t <= t_end - lag:
            x = sample_at(cmd, t, idx)
            y = sample_at(odom, t + lag, idx)
            if abs(x) >= cmd_min:
                xs.append(x)
                ys.append(y)
            t += step
        c = correlation(xs, ys)
        if c is not None:
            denom = sum(x*x for x in xs)
            gain = sum(x*y for x, y in zip(xs, ys)) / denom if denom > 1e-9 else 0.0
            candidate = (abs(c), lag, c, gain, len(xs))
            if best is None or candidate[0] > best[0]:
                best = candidate
        lag += step

    if best is None:
        return None, "not enough correlated excitation"
    return best, None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = Monitor(args.duration)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    events = node.events
    node.destroy_node()
    rclpy.shutdown()

    print("\n=== SPARKY V11.6.1 COMMAND / RESPONSE REPORT ===")

    print("\nMESSAGE RATES")
    for topic in TWIST_TOPICS + ["/odom"]:
        ev = events[topic]
        print(
            f"  {topic:15s} rate={rate(ev):6.2f} Hz "
            f"n={len(ev):4d} peak_vx={peak(ev,1):.4f} peak_wz={peak(ev,2):.4f}"
        )

    print("\nCOMMAND CHAIN FIDELITY")
    for a, b in [
        ("/cmd_vel_nav2", "/cmd_vel_nav"),
        ("/cmd_vel_nav", "/cmd_vel_out"),
        ("/cmd_vel_out", "/cmd_vel_sdk"),
    ]:
        dx = chain_diff(events[a], events[b], 1)
        dz = chain_diff(events[a], events[b], 2)
        print(f"  {a} -> {b}")
        print(f"    linear  {dx}")
        print(f"    angular {dz}")

    # The V11.6 live failure was specifically cmd_vel_out.x > 0 but SDK x == 0.
    out_peak_x = peak(events["/cmd_vel_out"], 1)
    sdk_peak_x = peak(events["/cmd_vel_sdk"], 1)
    if out_peak_x >= 0.02 and sdk_peak_x < 0.005:
        print("\nRED FLAG: linear command is still being deleted at the driver boundary.")
    elif out_peak_x >= 0.02:
        print("\nOK: nonzero linear command reaches /cmd_vel_sdk.")

    for idx, label in [(1, "LINEAR"), (2, "ANGULAR")]:
        result, reason = lag_gain(events["/cmd_vel_sdk"], events["/odom"], idx)
        print(f"\n{label} SDK COMMAND -> ODOM")
        if result is None:
            print(f"  lag/gain NOT ESTIMATED: {reason}")
            print("  This is intentionally not reported as a sign or latency failure.")
        else:
            _, lag, corr, gain, n = result
            print(f"  best_lag_sec={lag:.3f}")
            print(f"  correlation={corr:.3f}")
            print(f"  observed_gain={gain:.3f}")
            print(f"  samples_used={n}")
            if corr < -0.4:
                print("  RED FLAG: likely command/odometry sign mismatch")
            if abs(gain) > 1.5:
                print("  RED FLAG: observed response much larger than requested")
            elif abs(gain) < 0.5:
                print("  RED FLAG: observed response much smaller than requested")
            if lag > 0.35:
                print("  RED FLAG: large command-response delay")

    print("\nNOTES")
    print("  /cmd_vel_sdk is the command prepared at the WebRTC adapter boundary.")
    print("  It is not a network acknowledgement from the robot.")
    print("  A real transport-latency conclusion requires meaningful odom excitation.")
    print("  For the next test use a 0.5-1.0 m goal, not a 5 m goal.")

if __name__ == "__main__":
    main()
