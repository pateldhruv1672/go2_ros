#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math, os
from pathlib import Path


def derive(data):
    obj = data.get('object_pose') or data.get('map_pose') or {}
    if not isinstance(obj, dict) or obj.get('x') is None or obj.get('y') is None:
        return
    data['object_pose'] = dict(obj)
    data['map_pose'] = dict(obj)
    observer = data.get('last_observer_pose') or data.get('observation_viewpoint')
    if not isinstance(observer, dict) or observer.get('x') is None or observer.get('y') is None:
        legacy = data.get('approach_pose') or {}
        if isinstance(legacy, dict) and legacy.get('x') is not None and legacy.get('y') is not None:
            observer = dict(legacy)
            data['last_observer_pose'] = observer
    if not isinstance(observer, dict) or observer.get('x') is None or observer.get('y') is None:
        return
    try:
        ox,oy=float(obj['x']),float(obj['y']); vx,vy=float(observer['x']),float(observer['y'])
        dx,dy=vx-ox,vy-oy; n=math.hypot(dx,dy)
        if n < .08: return
        extent=max(float(data.get('extent_x') or .30),float(data.get('extent_y') or .30))
        standoff=min(max(.85,.5*extent+.55),max(.65,n))
        gx,gy=ox+dx/n*standoff,oy+dy/n*standoff
        yaw=math.atan2(oy-gy,ox-gx)
        nav={'frame_id':'map','x':gx,'y':gy,'z':0.0,'qx':0.0,'qy':0.0,
             'qz':math.sin(.5*yaw),'qw':math.cos(.5*yaw),'yaw':yaw,
             'standoff_m':standoff,'source':'derived_from_object_pose_and_observer'}
        data['navigation_approach_pose']=nav
        data['approach_pose']=nav
        data['location_semantics']='object_pose/map_pose=physical object centroid; last_observer_pose=robot observation pose; navigation_approach_pose=derived Nav2 standoff'
    except Exception:
        pass


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('session', nargs='?', default='')
    ap.add_argument('--root', default=os.path.expanduser('~/.ros/go2_semantic_nav_sessions'))
    a=ap.parse_args(); root=Path(a.root).expanduser()
    if a.session:
        session=a.session
    else:
        dirs=sorted((p for p in root.iterdir() if p.is_dir()), key=lambda p:p.stat().st_mtime, reverse=True)
        if not dirs: raise SystemExit('No sessions found')
        session=dirs[0].name
    path=root/session/'memory'/'objects.jsonl'
    if not path.is_file(): raise SystemExit(f'Missing {path}')
    rows=[]
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip(): continue
        try: rec=json.loads(line)
        except Exception: continue
        data=rec.get('data') if isinstance(rec,dict) and isinstance(rec.get('data'),dict) else None
        if data is not None: derive(data)
        rows.append(rec)
    backup=path.with_suffix(path.suffix+'.pre_v6')
    if not backup.exists(): backup.write_text(path.read_text(encoding='utf-8'),encoding='utf-8')
    tmp=path.with_suffix('.jsonl.tmp')
    tmp.write_text(''.join(json.dumps(r,sort_keys=True,default=str)+'\n' for r in rows),encoding='utf-8')
    tmp.replace(path)
    print(f'Upgraded {len(rows)} object records in {path}')
    print(f'Backup: {backup}')

if __name__=='__main__': main()
