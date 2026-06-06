# nav2-goal-frame-debug

## When to use
Use when Nav2 reports:
- `Failed to transform from  to map`
- planner cannot transform start or goal pose
- goals are going to `(0,0)` unexpectedly

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

## Code rules
- every generated `PoseStamped` must set:
```python
pose.header.frame_id = "map"
pose.header.stamp = self.get_clock().now().to_msg()
```
- default missing saved `frame_id` to `map`
- reject unknown places instead of using `(0,0)`

## Success check
- outgoing goal has non-empty `frame_id`
- logs show `frame=map`
