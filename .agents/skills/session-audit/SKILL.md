# session-audit

## When to use
Use before resume mode or when a saved session fails to load.

## Check latest session
```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
echo "$SESSION"
find ~/.ros/go2_semantic_nav_sessions/$SESSION -maxdepth 1 -type f | sort
```

## Resume-ready session files
- `map.yaml`
- `map.pgm`
- `places.yaml`
- `session.yaml`

## Inspect files
```bash
cat ~/.ros/go2_semantic_nav_sessions/$SESSION/session.yaml
cat ~/.ros/go2_semantic_nav_sessions/$SESSION/map.yaml
ls -lh ~/.ros/go2_semantic_nav_sessions/$SESSION/map.pgm
```

## Recovery pattern
If latest labels exist but map files are missing, copy a known-good `map.yaml` and `map.pgm` from an older session while keeping latest `places.yaml` and `session.yaml`.
