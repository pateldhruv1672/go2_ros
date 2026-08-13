#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo '=== P1 YOLO + SAM2 ==='
for n in /fast_sam2_tracker_overlay_node /semantic_nav_rviz2; do
  ros2 node list 2>/dev/null | grep -qx "$n" && echo "PRESENT $n" || echo "MISSING $n"
done
for t in /camera/image_raw /object_explorer/annotated_image /object_explorer/sam2_detections; do
  echo "-- $t"
  ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:|Reliability:' || echo MISSING
done
echo '-- annotated image rate (5 sec) --'
timeout 5 ros2 topic hz /object_explorer/annotated_image 2>/dev/null || true

if ros2 node list 2>/dev/null | grep -qx /fast_sam2_tracker_overlay_node; then
  for p in publish_annotated_image enable_sam2 yolo_imgsz sam2_imgsz sam2_every_n inference_period_sec; do
    printf '%-30s ' "$p"
    ros2 param get /fast_sam2_tracker_overlay_node "$p" 2>/dev/null || echo MISSING
  done
fi

echo
echo '=== P2 TOUR ==='
ros2 param get /semantic_nav_node tour_auto_advance 2>/dev/null || true
ros2 topic info /semantic_nav/command 2>/dev/null || true
python3 - <<'PY'
import os, yaml
from pathlib import Path
root=Path(os.path.expanduser(os.environ.get('SESSION_ROOT','~/.ros/go2_semantic_nav_sessions')))
name=os.environ.get('SESSION_NAME','')
if not name or name in {'latest','auto'}:
    dirs=[p for p in root.iterdir() if p.is_dir() and (p/'route.yaml').is_file() and (p/'map.yaml').is_file()]
    session=max(dirs,key=lambda p:p.stat().st_mtime) if dirs else None
else:
    session=root/name
if session and (session/'route.yaml').is_file():
    d=yaml.safe_load((session/'route.yaml').read_text()) or {}
    r=d.get('route',d)
    print('session=',session.name)
    print('route=',r.get('name'),'state=',r.get('state'),'stops=',len(r.get('stops') or []))
    print('guest_prompt=',r.get('guest_prompt',''))
    for i,s in enumerate((r.get('stops') or [])[:3],1):
        print(f'stop{i} name={s.get("name")} place={s.get("place_name")} status={s.get("status")} script={s.get("script","")[:100]}')
else:
    print('route.yaml not found')
PY

echo
echo '=== NAV BASELINE ==='
printf '%-28s ' 'planner:'
ros2 param get /planner_server GridBased.plugin 2>/dev/null || true
printf '%-28s ' 'controller:'
ros2 param get /controller_server FollowPath.plugin 2>/dev/null || true

echo
echo '=== FAST GREEN PATH ==='
ros2 node list 2>/dev/null | grep -qx /sparky_fast_green_path_preview && echo 'PRESENT /sparky_fast_green_path_preview' || echo 'MISSING /sparky_fast_green_path_preview'
ros2 topic info /plan_fast 2>/dev/null || true
echo '-- /plan_fast rate (4 sec) --'
timeout 4 ros2 topic hz /plan_fast 2>/dev/null || true

echo
echo 'Expected:'
echo '  annotated image: publisher=1, RELIABLE, nonzero Hz'
echo '  tour_auto_advance: True; route has the existing 3 stops with welcome/NOVA/xArm scripts'
echo '  planner NavFn + controller DWB; no RPP/RotationShim'
echo '  /plan_fast: around 15 Hz when a /plan exists'
