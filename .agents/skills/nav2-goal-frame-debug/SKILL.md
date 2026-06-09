# nav2-goal-frame-debug

## When to use
Use when Nav2 reports:
- `Failed to transform from  to map`
- planner cannot transform start or goal pose
- goals are going to `(0,0)` unexpectedly
- Nav2 rejects goals because `bt_navigator` or `controller_server` is inactive
- semantic resume mode is restoring places/spawn, but navigation still aborts immediately
- the controller log mentions stale TF, old transforms, or `odom -> map` timing issues

## What to inspect
1. The goal pose topic:
```bash
ros2 topic echo --once /goal_pose
```
2. Saved places:
```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
grep -n -A12 -B2 "name:" ~/.ros/go2_semantic_nav_sessions/$SESSION/places.yaml | head -120
```
3. Semantic nav route/session state:
```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
sed -n '1,220p' ~/.ros/go2_semantic_nav_sessions/$SESSION/session.yaml
sed -n '1,220p' ~/.ros/go2_semantic_nav_sessions/$SESSION/route.yaml
```
4. Nav2 lifecycle and action availability:
```bash
ros2 lifecycle get /bt_navigator --disable-daemon
ros2 lifecycle get /controller_server --disable-daemon
ros2 action info /navigate_to_pose --disable-daemon
```

## Code rules
- every generated `PoseStamped` must set:
```python
pose.header.frame_id = "map"
pose.header.stamp = self.get_clock().now().to_msg()
```
- default missing saved `frame_id` to `map`
- reject unknown places instead of using `(0,0)`
- if a goal is rejected immediately, classify it before retrying:
  - `missing_tf`
  - `stale_tf`
  - `inactive_nav2`
  - `localization_lost`
  - `goal_unreachable`
- when resume mode loads the session, rehydrate places from shared memory if `places.yaml` is empty
- if the session has no saved spawn, publish `/initialpose` from the current TF pose once TF is ready
- do not merge distinct places that are physically different, but collapse near-duplicate captures from the same spot
- preserve route/tour state in `session.yaml` and `route.yaml`, not only in memory logs

## Success check
- outgoing goal has non-empty `frame_id`
- logs show `frame=map`
- `bt_navigator` and `controller_server` are active before a resume goal is sent
- `resume_tour` can advance from one taught node to another without replaying duplicate place captures
