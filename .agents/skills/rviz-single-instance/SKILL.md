# rviz-single-instance

## When to use
Use when TF debugging is noisy or multiple RViz instances appear.

## Rules
- run only one main RViz instance
- do not launch semantic-nav RViz if base stack already launched RViz
- prefer `rviz2:=false` in resume mode unless explicitly testing the semantic-nav RViz

## Checks
```bash
ros2 node list | grep rviz
ps -ef | grep -E 'rviz2|go2_rviz2' | grep -v grep
```

## Cleanup
```bash
pkill -f rviz2 || true
sleep 2
```
