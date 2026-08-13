#!/usr/bin/env python3
import argparse
import math
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TOPICS = [
    "/cmd_vel_nav2",
    "/cmd_vel_nav",
    "/cmd_vel_out",
    "/cmd_vel_sdk",
]

def corr(a, b):
    if len(a) < 8:
        return 0.0
    ma = sum(a) / len(a)
    mb = sum(b) / len(b)
    xa = [x - ma for x in a]
    xb = [x - mb for x in b]
    da = math.sqrt(sum(x*x for x in xa))
    db = math.sqrt(sum(x*x for x in xb))
    if da < 1e-9 or db < 1e-9:
        return 0.0
    return sum(x*y for x, y in zip(xa, xb)) / (da * db)

class Monitor(Node):
    def __init__(self, duration, hz):
        super().__init__("sparky_command_response_monitor")
        self.duration = duration
        self.dt = 1.0 / hz
        self.started = time.monotonic()
        self.latest = defaultdict(lambda: {"x": 0.0, "z": 0.0, "seen": False})
        self.rows = []

        for topic in TOPICS:
            self.create_subscription(
                Twist, topic,
                lambda msg, t=topic: self.on_twist(t, msg),
                20
            )
        self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        self.create_timer(self.dt, self.sample)

    def on_twist(self, topic, msg):
        self.latest[topic] = {
            "x": float(msg.linear.x),
            "z": float(msg.angular.z),
            "seen": True,
        }

    def on_odom(self, msg):
        self.latest["/odom"] = {
            "x": float(msg.twist.twist.linear.x),
            "z": float(msg.twist.twist.angular.z),
            "seen": True,
        }

    def sample(self):
        t = time.monotonic() - self.started
        row = {"t": t}
        for topic in TOPICS + ["/odom"]:
            d = self.latest[topic]
            row[topic + ".x"] = d["x"]
            row[topic + ".z"] = d["z"]
            row[topic + ".seen"] = d["seen"]
        self.rows.append(row)
        if t >= self.duration:
            raise KeyboardInterrupt

def mse_diff(rows, a, b, axis):
    vals = [
        (r[f"{a}.{axis}"], r[f"{b}.{axis}"])
        for r in rows
        if r[f"{a}.seen"] and r[f"{b}.seen"]
    ]
    if not vals:
        return None, None
    diffs = [x-y for x,y in vals]
    rms = math.sqrt(sum(d*d for d in diffs) / len(diffs))
    mx = max(abs(d) for d in diffs)
    return rms, mx

def lag_gain(rows, cmd_topic, axis, dt, max_lag=1.2):
    cmd = [r[f"{cmd_topic}.{axis}"] for r in rows]
    obs = [r[f"/odom.{axis}"] for r in rows]
    max_k = min(int(max_lag / dt), max(0, len(rows)//3))
    best = None
    for k in range(max_k + 1):
        if k == 0:
            c = cmd
            o = obs
        else:
            c = cmd[:-k]
            o = obs[k:]
        pairs = [(x,y) for x,y in zip(c,o) if abs(x) >= (0.03 if axis=="z" else 0.02)]
        if len(pairs) < 8:
            continue
        aa = [x for x,_ in pairs]
        bb = [y for _,y in pairs]
        cc = corr(aa, bb)
        denom = sum(x*x for x in aa)
        gain = (sum(x*y for x,y in pairs) / denom) if denom > 1e-9 else 0.0
        score = abs(cc)
        if best is None or score > best[0]:
            best = (score, k*dt, cc, gain, len(pairs))
    return best

def peak(rows, topic, axis):
    vals = [abs(r[f"{topic}.{axis}"]) for r in rows if r[f"{topic}.seen"]]
    return max(vals) if vals else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=50.0)
    args = ap.parse_args()

    rclpy.init()
    node = Monitor(args.duration, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    print("\n=== SPARKY V11.6 COMMAND -> RESPONSE REPORT ===")
    print(f"samples={len(rows)} duration={rows[-1]['t'] if rows else 0:.2f}s")

    for a,b in [
        ("/cmd_vel_nav2","/cmd_vel_nav"),
        ("/cmd_vel_nav","/cmd_vel_out"),
        ("/cmd_vel_out","/cmd_vel_sdk"),
    ]:
        xrms,xmax = mse_diff(rows,a,b,"x")
        zrms,zmax = mse_diff(rows,a,b,"z")
        print(f"\n{a} -> {b}")
        print(f"  linear  rms_diff={xrms!r} max_diff={xmax!r}")
        print(f"  angular rms_diff={zrms!r} max_diff={zmax!r}")

    print("\nPEAK ABSOLUTE VELOCITIES")
    for topic in TOPICS + ["/odom"]:
        print(
            f"  {topic:15s} "
            f"vx={peak(rows,topic,'x')!r} "
            f"wz={peak(rows,topic,'z')!r}"
        )

    dt = 1.0 / args.hz
    for axis,name in [("z","ANGULAR"),("x","LINEAR")]:
        best = lag_gain(rows, "/cmd_vel_sdk", axis, dt)
        print(f"\n{name} SDK COMMAND -> ODOM RESPONSE")
        if best is None:
            print("  insufficient excitation; move/rotate the robot during the capture")
        else:
            _, lag, cc, gain, n = best
            print(f"  best_lag_sec={lag:.3f}")
            print(f"  correlation={cc:.3f}")
            print(f"  observed_gain={gain:.3f}")
            print(f"  samples_used={n}")
            if cc < 0:
                print("  RED FLAG: command and observed response have opposite signs")
            if abs(gain) > 1.25:
                print("  RED FLAG: robot/odometry response is materially larger than SDK command")
            elif 0 < abs(gain) < 0.70:
                print("  RED FLAG: robot/odometry response is materially smaller than SDK command")
            if lag > 0.25:
                print("  RED FLAG: control-response delay is high for a 10 Hz controller")

    print("\nINTERPRETATION")
    print("  /cmd_vel_out != /cmd_vel_sdk  -> driver adapter is modifying commands")
    print("  /cmd_vel_out ~= /cmd_vel_sdk but odom gain != 1 -> robot response / odom scaling mismatch")
    print("  good gain but high lag -> WebRTC / odom latency; controller frequency must account for it")
    print("  negative correlation -> sign/frame convention bug")
    print("  No commands during capture -> rerun while doing one supervised short navigation test")

if __name__ == "__main__":
    main()
