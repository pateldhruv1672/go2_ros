#!/usr/bin/env python3
"""Real Nav2 goal execution + raw recording. No synthetic data: every row comes from a received ROS message."""
import sys, json, csv, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

scn = json.loads(Path(sys.argv[1]).read_text()); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
GOAL_TIMEOUT_SIM = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def st(h): return h.stamp.sec + h.stamp.nanosec * 1e-9

class Rec(Node):
    def __init__(self):
        super().__init__('sparky_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.f = {k: open(out / f'{k}.csv', 'w', newline='') for k in ['ground_truth', 'est_odom', 'cmd_vel', 'amcl']}
        self.w = {k: csv.writer(v) for k, v in self.f.items()}
        for k in ['ground_truth', 'est_odom']: self.w[k].writerow(['stamp', 'x', 'y', 'yaw', 'vx', 'wz'])
        self.w['cmd_vel'].writerow(['sim_time', 'vx', 'wz'])
        self.w['amcl'].writerow(['stamp', 'x', 'y', 'yaw'])
        self.plans = open(out / 'plans.jsonl', 'w')
        self.counts = {'clock': 0, 'scan': 0, 'est_odom': 0, 'ground_truth': 0}
        self.recording = False
        self.clock = 0.0
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/ground_truth', lambda m: self.odom('ground_truth', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom('est_odom', m), qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel/smooth', self.on_cmd, 50)
        self.create_subscription(LaserScan, '/scan', lambda m: self.counts.__setitem__('scan', self.counts['scan'] + 1), qos_profile_sensor_data)
        self.create_subscription(PathMsg, '/plan', self.on_plan, 10)
        self.amcl_n = 0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
    def on_clock(self, m): self.counts['clock'] += 1; self.clock = m.clock.sec + m.clock.nanosec * 1e-9
    def odom(self, k, m):
        self.counts[k] += 1
        if self.recording:
            p = m.pose.pose; self.w[k].writerow([f'{st(m.header):.4f}', p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z])
    def on_cmd(self, m):
        if self.recording: self.w['cmd_vel'].writerow([f'{self.clock:.4f}', m.linear.x, m.angular.z])
    def on_amcl(self, m):
        self.amcl_n += 1; p = m.pose.pose
        if self.recording: self.w['amcl'].writerow([f'{st(m.header):.4f}', p.position.x, p.position.y, yaw_of(p.orientation)])
    def on_plan(self, m):
        if self.recording:
            self.plans.write(json.dumps({'stamp': st(m.header), 'recv_sim': self.clock, 'xy': [[p.pose.position.x, p.pose.position.y] for p in m.poses]}) + '\n')

def spin_until(node, cond, timeout_wall, label):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_wall:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cond(): return True
    node.get_logger().error('TIMEOUT waiting for ' + label); return False

def main():
    rclpy.init(); n = Rec(); manifest = {'scenario': scn, 'gates': {}}
    def gate(name, ok, **kw): manifest['gates'][name] = {'ok': bool(ok), **kw}; print('GATE', name, ok, kw, flush=True); return ok
    def finish(status):
        manifest['status'] = status; (out / 'run_manifest.json').write_text(json.dumps(manifest, indent=1))
        for f in n.f.values(): f.close()
        n.plans.close(); rclpy.shutdown(); sys.exit(0 if status == 'GOAL_SUCCEEDED' else 3)
    if not gate('clock_advancing', spin_until(n, lambda: n.counts['clock'] > 50 and n.clock > 5, 120, 'clock'), sim_time=n.clock): finish('BLOCKED_NO_CLOCK')
    if not gate('scan_flowing', spin_until(n, lambda: n.counts['scan'] > 5, 120, 'scan'), count=n.counts['scan']): finish('BLOCKED_NO_SCAN')
    for node in ['lifecycle_manager_localization', 'lifecycle_manager_navigation']:
        cli = n.create_client(Trigger, f'/{node}/is_active'); ok = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 240 and not ok:
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
                ok = bool(fut.done() and fut.result().success)
            time.sleep(1)
        if not gate('active_' + node, ok): finish('BLOCKED_LIFECYCLE')
    if not gate('amcl_pose', spin_until(n, lambda: n.amcl_n > 0, 120, 'amcl_pose'), count=n.amcl_n): finish('BLOCKED_NO_AMCL')
    def tf_ok(a, b):
        try: n.tfbuf.lookup_transform(a, b, rclpy.time.Time()); return True
        except Exception: return False
    if not gate('tf_map_base', spin_until(n, lambda: tf_ok('map', 'base_link'), 60, 'map->base_link')): finish('BLOCKED_NO_TF')
    # let a few seconds of sim time pass so the robot has settled and AMCL has converged
    t_settle = n.clock + 5.0; spin_until(n, lambda: n.clock > t_settle, 120, 'settle')
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not gate('action_server', ac.wait_for_server(timeout_sec=60)): finish('BLOCKED_NO_ACTION_SERVER')
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; gl = scn['goal']
    g.pose.pose.position.x, g.pose.pose.position.y = gl['x'], gl['y']
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gl['yaw']/2), math.cos(gl['yaw']/2)
    g.pose.header.stamp = n.get_clock().now().to_msg()
    n.recording = True; t_goal = n.clock
    manifest['goal_sent_sim_time'] = t_goal
    fut = ac.send_goal_async(g); spin_until(n, fut.done, 30, 'goal accept')
    gh = fut.result()
    if not gate('goal_accepted', gh is not None and gh.accepted): finish('BLOCKED_GOAL_REJECTED')
    rf = gh.get_result_async()
    spin_until(n, lambda: rf.done() or (n.clock - t_goal) > GOAL_TIMEOUT_SIM, GOAL_TIMEOUT_SIM * 6 + 60, 'goal result')
    if rf.done():
        res = rf.result(); status = res.status; manifest['action_status'] = int(status)  # 4 = SUCCEEDED
        manifest['error_code'] = int(getattr(res.result, 'error_code', -1))
    else:
        manifest['action_status'] = None; gh.cancel_goal_async(); spin_until(n, lambda: False, 3, 'cancel')
    manifest['goal_end_sim_time'] = n.clock; n.recording = False
    # final ground-truth distance recorded after the loop from csv by analysis; capture last values here too
    ok = manifest.get('action_status') == 4
    finish('GOAL_SUCCEEDED' if ok else 'GOAL_NOT_SUCCEEDED')
main()
