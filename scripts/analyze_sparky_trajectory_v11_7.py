#!/usr/bin/env python3
import argparse,csv,json,math,os
from collections import Counter

def num(x):
    try:return float(x)
    except:return float('nan')
def vals(rows,k): return [num(r[k]) for r in rows if math.isfinite(num(r[k]))]
def peak(rows,k): return max([abs(x) for x in vals(rows,k)],default=0.0)
def med(xs):
    xs=sorted([x for x in xs if math.isfinite(x)])
    if not xs:return float('nan')
    n=len(xs); return xs[n//2] if n%2 else (xs[n//2-1]+xs[n//2])/2
def disp(rows,x,y):
    p=[(num(r[x]),num(r[y])) for r in rows]; p=[q for q in p if all(math.isfinite(z) for z in q)]
    return math.hypot(p[-1][0]-p[0][0],p[-1][1]-p[0][1]) if len(p)>1 else float('nan')

ap=argparse.ArgumentParser(); ap.add_argument('outdir'); a=ap.parse_args()
with open(os.path.join(a.outdir,'trace.csv'),newline='') as f: rows=list(csv.DictReader(f))
dwb=[]; dp=os.path.join(a.outdir,'dwb_summary.csv')
if os.path.exists(dp):
    with open(dp,newline='') as f:dwb=list(csv.DictReader(f))
L=[]; A=L.append
A('SPARKY V11.7 TRAJECTORY FLIGHT-RECORDER SUMMARY'); A('='*58)
A(f'trace_samples={len(rows)}'); A(f'dwb_evaluations={len(dwb)}'); A(''); A('COMMAND PEAKS')
for p,n in [('nav2','DWB/controller'),('arb','motion arbiter'),('out','collision output'),('sdk','Go2 SDK boundary')]: A(f'  {n:20s} vx={peak(rows,p+"_vx"):.4f} wz={peak(rows,p+"_wz"):.4f}')
A(f'  odom observed        vx={peak(rows,"odom_vx"):.4f} wz={peak(rows,"odom_wz"):.4f}')
A(''); A('PHYSICAL PROGRESS'); A(f'  odom displacement = {disp(rows,"odom_x","odom_y"):.3f} m'); A(f'  AMCL displacement = {disp(rows,"amcl_x","amcl_y"):.3f} m')
A(''); A('TIMING')
for k in ['odom_age','amcl_age','scan_age','scan_nav_age']:
    x=vals(rows,k); A(f'  {k:14s} median={med(x):.3f}s peak={max(x,default=float("nan")):.3f}s')
A(''); A('AMCL COVARIANCE')
for k in ['amcl_var_x','amcl_var_y','amcl_var_yaw']: A(f'  {k:14s} median={med(vals(rows,k)):.4f}')
A(''); A('AUTOMATIC FAULT LOCALIZATION')
nx,ax,ox,sx,dx=[peak(rows,k) for k in ['nav2_vx','arb_vx','out_vx','sdk_vx','odom_vx']]
if nx<0.015:A('  RED: controller/DWB produced essentially no forward command.')
elif ax<0.4*nx:A('  RED: motion arbiter suppressed most controller forward command.')
elif ox<0.4*ax:A('  RED: collision-monitor stage suppressed most forward command.')
elif sx<0.4*ox:A('  RED: Go2 driver/adapter suppressed most forward command.')
elif dx<0.25*sx:A('  RED: command reaches SDK boundary but odometry/physical response is much smaller.')
else:A('  Command chain and odometry show meaningful forward response.')
if peak(rows,'sdk_wz')>0.03 and peak(rows,'odom_wz')>1.5*peak(rows,'sdk_wz'): A('  RED: observed yaw peak is >1.5x requested SDK yaw peak.')
if max(vals(rows,'scan_age'),default=0)>0.75:A('  RED: raw /scan becomes >0.75 s old relative to ROS time.')
A(''); A('DWB TRAJECTORY LEGALITY')
if dwb:
    zero=0; C=Counter()
    for r in dwb:
        if int(r['n_total'])>0 and int(r['n_valid'])==0: zero+=1
        try:C.update(json.loads(r['illegal_critics_json']))
        except:pass
    A(f'  zero-valid evaluations = {zero}/{len(dwb)}')
    for k,v in C.most_common():A(f'  illegal {k:28s} {v}')
else:A('  No /evaluation messages captured.')
A(''); A('TRACE ORDER'); A('  DWB -> arbiter -> collision monitor -> SDK -> odometry -> AMCL/scan -> critic legality')
out='\n'.join(L)+'\n'; print(out)
with open(os.path.join(a.outdir,'SUMMARY.txt'),'w') as f:f.write(out)
