# Go2 Agentic Session Format V1

Existing session files remain supported:

```text
map.yaml
map.pgm
places.yaml
session.yaml
```

The additive session layout is:

```text
~/.ros/go2_semantic_nav_sessions/<session_name>/
  map.yaml
  map.pgm
  places.yaml
  session.yaml
  spawn.yaml
  memory/
    checkpoints.jsonl
    places.jsonl
    spawn.json
    decisions.jsonl
  graph_memory/
    nodes.jsonl
    edges.jsonl
    kuzu_db/
  voxel_memory/
    semantic_voxels.jsonl
    voxel_blocks/
  vector_memory/
    embeddings.jsonl
  artifacts/
    images/
    pointclouds/
    scans/
    vlm_raw/
    nav_logs/
  langgraph/
    checkpoints/
    store/
  exports/
    route_graph.geojson
    tour.yaml
    department_knowledge.yaml
    memory_summary.yaml
```

## Memory layers

- `session`: current mission state.
- `temporary`: uncertain or time-limited facts.
- `permanent`: verified map-frame memories.

## Backward compatibility

`go2_memory_core.legacy_import_export` keeps legacy `places.yaml`, `session.yaml`, and `spawn.yaml` exports synchronized when possible.
