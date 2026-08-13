#!/usr/bin/env python3
import argparse, csv, json, math, os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
try:
    from dwb_msgs.msg import LocalPlanEvaluation
    HAVE_DWB = True
except Exception:
    HAVE_DWB = False

def stamp_sec(s): return float(s.sec) + float(s.nanosec)*1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class Trace(Node):
    def __init__(self, outdir):
        super().__init__('sparky_trajectory_trace_v11_7')
        self.outdir=outdir; self.t0=time.monotonic(); self.latest={}; self.paths={}; self.counts={}
        self.tf=open(os.path.join(outdir,'trace.csv'),'w',newline='')
        self.tw=csv.DictWriter(self.tf,fieldnames=['t','nav2_vx','nav2_wz','arb_vx','arb_wz','out_vx','out_wz','sdk_vx','sdk_wz','odom_x','odom_y','odom_yaw','odom_vx','odom_vy','odom_wz','odom_age','amcl_x','amcl_y','amcl_yaw','amcl_var_x','amcl_var_y','amcl_var_yaw','amcl_age','scan_age','scan_nav_age','global_plan_n','transformed_plan_n','local_plan_n'])
        self.tw.writeheader()
        self.df=open(os.path.join(outdir,'dwb_summary.csv'),'w',newline='')
        self.dw=csv.DictWriter(self.df,fieldnames=['t','n_total','n_valid','n_illegal','best_index','best_vx','best_vy','best_wz','best_total','best_scores_json','illegal_critics_json'])
        self.dw.writeheader()
        for topic in ['/cmd_vel_nav2','/cmd_vel_nav','/cmd_vel_out','/cmd_vel_sdk']:
            self.create_subscription(Twist,topic,lambda m,t=topic:self.cmd(t,m),50)
        self.create_subscription(Odometry,'/odom',self.odom,100)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl,50)
        self.create_subscription(LaserScan,'/scan',lambda m:self.scan('/scan',m),20)
        self.create_subscription(LaserScan,'/scan_nav',lambda m:self.scan('/scan_nav',m),20)
        for topic in ['/plan','/transformed_global_plan','/local_plan']:
            self.create_subscription(Path,topic,lambda m,t=topic:self.path(t,m),10)
        if HAVE_DWB:
            self.create_subscription(LocalPlanEvaluation,'/evaluation',self.eval,20)
        self.create_timer(0.1,self.sample)
    def rel(self): return time.monotonic()-self.t0
    def bump(self,k): self.counts[k]=self.counts.get(k,0)+1
    def cmd(self,t,m): self.bump(t); self.latest[t]=(float(m.linear.x),float(m.angular.z))
    def odom(self,m):
        self.bump('/odom'); p=m.pose.pose.position; q=m.pose.pose.orientation; v=m.twist.twist
        self.latest['/odom']={'x':p.x,'y':p.y,'yaw':yaw(q),'vx':v.linear.x,'vy':v.linear.y,'wz':v.angular.z,'stamp':stamp_sec(m.header.stamp)}
    def amcl(self,m):
        self.bump('/amcl_pose'); p=m.pose.pose.position; q=m.pose.pose.orientation; c=m.pose.covariance
        self.latest['/amcl_pose']={'x':p.x,'y':p.y,'yaw':yaw(q),'var_x':c[0],'var_y':c[7],'var_yaw':c[35],'stamp':stamp_sec(m.header.stamp)}
    def scan(self,t,m): self.bump(t); self.latest[t]={'stamp':stamp_sec(m.header.stamp)}
    def path(self,t,m): self.bump(t); self.paths[t]=m
    def age(self,k):
        d=self.latest.get(k)
        if not d or 'stamp' not in d: return float('nan')
        return max(0.0,self.get_clock().now().nanoseconds*1e-9-d['stamp'])
    def eval(self,m):
        self.bump('/evaluation'); total=len(m.twists); valid=sum(1 for x in m.twists if x.total>=0); illegal=total-valid
        bad={}
        for tr in m.twists:
            if tr.total<0:
                for sc in tr.scores:
                    if sc.raw_score<0: bad[sc.name]=bad.get(sc.name,0)+1
        bi=int(m.best_index); bvx=bvy=bw=bt=float('nan'); bs={}
        if total and 0<=bi<total:
            b=m.twists[bi]; bvx=b.traj.velocity.x; bvy=b.traj.velocity.y; bw=b.traj.velocity.theta; bt=b.total
            for sc in b.scores: bs[sc.name]={'raw':float(sc.raw_score),'scale':float(sc.scale),'weighted':float(sc.raw_score*sc.scale)}
        self.dw.writerow({'t':self.rel(),'n_total':total,'n_valid':valid,'n_illegal':illegal,'best_index':bi,'best_vx':bvx,'best_vy':bvy,'best_wz':bw,'best_total':bt,'best_scores_json':json.dumps(bs,sort_keys=True),'illegal_critics_json':json.dumps(bad,sort_keys=True)})
        self.df.flush()
    def sample(self):
        def c(k): return self.latest.get(k,(0.0,0.0))
        n,a,o,s=c('/cmd_vel_nav2'),c('/cmd_vel_nav'),c('/cmd_vel_out'),c('/cmd_vel_sdk')
        od=self.latest.get('/odom',{}); am=self.latest.get('/amcl_pose',{})
        self.tw.writerow({'t':self.rel(),'nav2_vx':n[0],'nav2_wz':n[1],'arb_vx':a[0],'arb_wz':a[1],'out_vx':o[0],'out_wz':o[1],'sdk_vx':s[0],'sdk_wz':s[1],'odom_x':od.get('x',float('nan')),'odom_y':od.get('y',float('nan')),'odom_yaw':od.get('yaw',float('nan')),'odom_vx':od.get('vx',float('nan')),'odom_vy':od.get('vy',float('nan')),'odom_wz':od.get('wz',float('nan')),'odom_age':self.age('/odom'),'amcl_x':am.get('x',float('nan')),'amcl_y':am.get('y',float('nan')),'amcl_yaw':am.get('yaw',float('nan')),'amcl_var_x':am.get('var_x',float('nan')),'amcl_var_y':am.get('var_y',float('nan')),'amcl_var_yaw':am.get('var_yaw',float('nan')),'amcl_age':self.age('/amcl_pose'),'scan_age':self.age('/scan'),'scan_nav_age':self.age('/scan_nav'),'global_plan_n':len(self.paths.get('/plan',Path()).poses),'transformed_plan_n':len(self.paths.get('/transformed_global_plan',Path()).poses),'local_plan_n':len(self.paths.get('/local_plan',Path()).poses)})
        self.tf.flush()
    def close(self):
        with open(os.path.join(self.outdir,'message_counts.json'),'w') as f: json.dump(self.counts,f,indent=2,sort_keys=True)
        self.tf.close(); self.df.close()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--outdir',required=True); a=ap.parse_args(); os.makedirs(a.outdir,exist_ok=True)
    rclpy.init(); n=Trace(a.outdir)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.close(); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
