from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def _yaw_to_quat(yaw: float) -> Tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class ObjectNavigationTools:
    """Select safe standoff poses near a physical remembered object.

    Candidate generation is deterministic.  OccupancyGrid rejects obviously unsafe
    base poses; Nav2 ComputePathToPose then validates reachability without moving.
    """

    def __init__(self, node: Any, planner_action: str = '/compute_path_to_pose') -> None:
        self.node = node
        self.client = ActionClient(node, ComputePathToPose, planner_action)
        self.robot_radius = 0.33
        self.margin = 0.12
        self.occupied_threshold = 50

    @staticmethod
    def _obj_pose(obj: Dict[str, Any]) -> Dict[str, Any]:
        return obj.get('object_pose') or obj.get('map_pose') or {}

    @staticmethod
    def _extent(obj: Dict[str, Any]) -> float:
        vals=[]
        for k in ('extent_x','extent_y'):
            try:
                v=float(obj.get(k) or 0.0)
                if v > 0: vals.append(v)
            except Exception: pass
        return max(vals or [0.30])

    def _cell(self, grid: Any, x: float, y: float) -> Optional[Tuple[int,int]]:
        try:
            ox=float(grid.info.origin.position.x); oy=float(grid.info.origin.position.y)
            res=float(grid.info.resolution); w=int(grid.info.width); h=int(grid.info.height)
            ix=int(math.floor((x-ox)/res)); iy=int(math.floor((y-oy)/res))
            if ix < 0 or iy < 0 or ix >= w or iy >= h: return None
            return ix,iy
        except Exception:
            return None

    def _footprint_free(self, grid: Any, x: float, y: float) -> bool:
        if grid is None: return True
        c=self._cell(grid,x,y)
        if c is None: return False
        ix,iy=c
        try:
            res=float(grid.info.resolution); w=int(grid.info.width); h=int(grid.info.height)
            cells=int(math.ceil((self.robot_radius+self.margin)/max(res,1e-3)))
            data=grid.data
            for dy in range(-cells,cells+1):
                for dx in range(-cells,cells+1):
                    if math.hypot(dx*res,dy*res) > self.robot_radius+self.margin: continue
                    xx,yy=ix+dx,iy+dy
                    if xx < 0 or yy < 0 or xx >= w or yy >= h: return False
                    v=int(data[yy*w+xx])
                    if v < 0 or v >= self.occupied_threshold: return False
            return True
        except Exception:
            return False

    def candidates(self, obj: Dict[str, Any], grid: Any, robot_pose: Dict[str, Any], max_candidates: int = 10) -> List[Dict[str, Any]]:
        p=self._obj_pose(obj)
        if not p: return []
        try: ox,oy=float(p['x']),float(p['y'])
        except Exception: return []
        extent=self._extent(obj)
        base=max(0.85, 0.5*extent + 0.62)
        radii=(base, base+0.25, min(base+0.55, 1.8))
        try: rx,ry=float(robot_pose.get('x')),float(robot_pose.get('y'))
        except Exception: rx,ry=ox-base,oy
        robot_angle=math.atan2(ry-oy,rx-ox)
        scored=[]
        for radius in radii:
            for i in range(16):
                angle=-math.pi + i*(2*math.pi/16.0)
                gx,gy=ox+radius*math.cos(angle),oy+radius*math.sin(angle)
                if not self._footprint_free(grid,gx,gy): continue
                yaw=math.atan2(oy-gy,ox-gx)
                qz,qw=_yaw_to_quat(yaw)
                angle_cost=abs(math.atan2(math.sin(angle-robot_angle),math.cos(angle-robot_angle)))
                travel=math.hypot(gx-rx,gy-ry)
                score=travel + 0.20*angle_cost + 0.10*(radius-base)
                scored.append((score,{
                    'frame_id':'map','x':gx,'y':gy,'z':0.0,
                    'qx':0.0,'qy':0.0,'qz':qz,'qw':qw,'yaw':yaw,
                    'standoff_m':radius,'object_x':ox,'object_y':oy,
                    'source':'occupancy_grid_candidate','candidate_score':round(score,4),
                }))
        scored.sort(key=lambda x:x[0])
        return [p for _,p in scored[:max_candidates]]

    def _wait_future(self, future: Any, timeout: float) -> bool:
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        return future.done()

    def _path_reachable(self, pose: Dict[str, Any], timeout: float = 1.1) -> Tuple[bool,float]:
        if not self.client.wait_for_server(timeout_sec=0.15):
            return False,float('inf')
        goal=ComputePathToPose.Goal()
        goal.goal=PoseStamped()
        goal.goal.header.frame_id='map'
        goal.goal.header.stamp=self.node.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(pose['x']); goal.goal.pose.position.y=float(pose['y'])
        goal.goal.pose.orientation.z=float(pose.get('qz',0.0)); goal.goal.pose.orientation.w=float(pose.get('qw',1.0))
        goal.use_start=False
        send=self.client.send_goal_async(goal)
        if not self._wait_future(send,timeout): return False,float('inf')
        handle=send.result()
        if handle is None or not handle.accepted: return False,float('inf')
        result_future=handle.get_result_async()
        if not self._wait_future(result_future,timeout):
            try: handle.cancel_goal_async()
            except Exception: pass
            return False,float('inf')
        wrapped=result_future.result()
        if wrapped is None: return False,float('inf')
        path=getattr(wrapped.result,'path',None)
        poses=list(getattr(path,'poses',[]) or [])
        if not poses: return False,float('inf')
        length=0.0
        for a,b in zip(poses,poses[1:]):
            length += math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
        return True,length

    def select(self, obj: Dict[str, Any], grid: Any, robot_pose: Dict[str, Any]) -> Dict[str, Any]:
        candidates=self.candidates(obj,grid,robot_pose,max_candidates=10)
        if not candidates:
            fallback=obj.get('navigation_approach_pose') or {}
            if fallback:
                return {'success':True,'pose':fallback,'planning_verified':False,'reason':'stored_standoff_fallback','candidates':[]}
            return {'success':False,'error':'no occupancy-safe standoff candidate'}
        verified=[]
        for pose in candidates[:5]:
            ok,length=self._path_reachable(pose)
            if ok:
                q=dict(pose); q['path_length_m']=round(length,3); q['planning_verified']=True
                verified.append(q)
        if verified:
            verified.sort(key=lambda p:(float(p.get('path_length_m',9999)),float(p.get('candidate_score',9999))))
            return {'success':True,'pose':verified[0],'planning_verified':True,'candidates':verified}
        # Planner may still be starting. Occupancy-safe fallback is preferable to the object centroid.
        return {'success':True,'pose':candidates[0],'planning_verified':False,'reason':'planner_unavailable_or_no_verified_path','candidates':candidates[:3]}
