# Real LangGraph Agentic System Patch

This patch replaces the previous deterministic "LangGraph-style" supervisor with a real `langgraph` `StateGraph` runtime and adds the project-required LangGraph capabilities as robot-facing ROS APIs.

## Capability coverage

| Capability | Implemented in this patch | Notes / honest limits |
|---|---|---|
| Persistence | Yes | Per-session persistent runtime under `~/.ros/go2_semantic_nav_sessions/<session>/langgraph/`. |
| Checkpointers | Yes | SQLite checkpointer via `langgraph-checkpoint-sqlite` at `checkpoints.sqlite`. |
| Stores | Yes | Durable SQLite long-term store at `store.sqlite`, plus best-effort attachment of LangGraph `InMemoryStore` to `compile(store=...)` when supported by the installed LangGraph version. SQLite is the durable source of truth. |
| Fault tolerance | Yes | Graph node errors route to `fault_handler`; ROS adapter publishes `stop_robot` on LangGraph exceptions. This is software-level fault tolerance, not proof of hardware safety. |
| Event streaming | Yes | Graph result events publish to `/go2_agent/events`; per-superstep stream updates publish to `/go2_agent/stream`. |
| Streaming | Yes | `AgentOrchestrator.run_with_stream()` consumes `graph.stream(..., stream_mode="updates")` and publishes updates over ROS. |
| Interrupts | Yes | `human_interrupt_gate` uses LangGraph `interrupt()` when `enable_human_interrupts:=true`; resume is on `/go2_agent/resume`. |
| Time travel | Yes | Checkpoint listing/state query on `/go2_agent/checkpoints/request`; branch/re-run from a checkpoint on `/go2_agent/time_travel`. No RViz UI yet. |
| Memory | Yes | Short-term state memory via checkpointer; long-term agent store via `store.sqlite`; robot semantic memory remains in `go2_memory_core` graph/vector/artifact/voxel backends. |
| Subgraphs | Yes | Memory, exploration, navigation, recovery, and tour subgraphs are compiled/executed from the main `StateGraph` with conditional routing. |

## Implemented

- Real `langgraph.graph.StateGraph` supervisor compiled with conditional edges.
- Real SQLite LangGraph checkpointing through `langgraph-checkpoint-sqlite`.
- One persistent `thread_id` per map/session so command state survives process restarts.
- Durable long-term store for cross-run agent facts, summaries, events, manual writes, and time-travel metadata.
- Conditional graph routing:
  - input router
  - context builder
  - memory refresh subgraph
  - intent parser
  - candidate-action builder
  - debate council
  - human interrupt gate
  - memory subgraph
  - exploration subgraph
  - tour subgraph
  - navigation subgraph
  - recovery subgraph
  - memory writeback
  - final response
  - fault handler
- Real compiled subgraphs for memory, exploration, navigation, recovery, and tour execution.
- Optional LangGraph `interrupt()` gate before risky motion when `enable_human_interrupts:=true`.
- ROS resume topic `/go2_agent/resume` using `langgraph.types.Command(resume=...)`.
- ROS event stream topic `/go2_agent/events` containing graph results/events.
- ROS streaming topic `/go2_agent/stream` containing LangGraph update chunks.
- ROS interrupt topic `/go2_agent/interrupts` for UI / terminal approval workflows.
- ROS checkpoint/time-travel topics:
  - `/go2_agent/checkpoints/request`
  - `/go2_agent/checkpoints`
  - `/go2_agent/time_travel`
- ROS store topics:
  - `/go2_agent/store/query`
  - `/go2_agent/store/write`
  - `/go2_agent/store_results`
- Durable human-readable summary written to:

```text
~/.ros/go2_semantic_nav_sessions/<session>/langgraph/latest_agent_summary.json
```

- Actual LangGraph checkpoint DB written to:

```text
~/.ros/go2_semantic_nav_sessions/<session>/langgraph/checkpoints.sqlite
```

- Durable long-term store DB written to:

```text
~/.ros/go2_semantic_nav_sessions/<session>/langgraph/store.sqlite
```

## Dependencies

Install in the workspace venv:

```bash
cd ~/Dhruv/sparky/ros2_ws
source src/.venv/bin/activate
python -m pip install -U langgraph langgraph-checkpoint-sqlite langchain-core
```

## Launch example

```bash
ros2 launch go2_agentic_system explore_mode.launch.py \
  enable_motion:=false \
  enable_human_interrupts:=false \
  enable_open_vocab_detector:=false \
  enable_dynamic_obstacle_tracking:=false \
  voice_input_mode:=text_topic
```

For human approval before risky motion:

```bash
ros2 launch go2_agentic_system explore_mode.launch.py \
  enable_motion:=false \
  enable_human_interrupts:=true
```

When interrupted, approve with:

```bash
ros2 topic pub --once /go2_agent/resume std_msgs/msg/String "{data: '{\"approved\": true}'}"
```

## Debug examples

List recent checkpoints:

```bash
ros2 topic pub --once /go2_agent/checkpoints/request std_msgs/msg/String "{data: '{\"limit\": 5}'}"
ros2 topic echo /go2_agent/checkpoints --once
```

Inspect one checkpoint:

```bash
ros2 topic pub --once /go2_agent/checkpoints/request std_msgs/msg/String "{data: '{\"checkpoint_id\": \"<ID>\"}'}"
```

Time-travel / branch from a checkpoint with a new command:

```bash
ros2 topic pub --once /go2_agent/time_travel std_msgs/msg/String \
  "{data: '{\"checkpoint_id\": \"<ID>\", \"command\": \"explain why you selected that goal\"}'}"
```

Query durable store:

```bash
ros2 topic pub --once /go2_agent/store/query std_msgs/msg/String \
  "{data: '{\"namespace\": [\"threads\", \"default\"], \"query\": \"explore\", \"limit\": 10}'}"
ros2 topic echo /go2_agent/store_results --once
```

## Still not fully implemented / not yet validated

- The debate council is still deterministic and safety-biased; it is not yet an LLM-powered multi-agent debate for every vote.
- The durable store supports structured JSON search, but it is not yet a vector-indexed LangGraph Store with semantic embedding search.
- Time travel is implemented through checkpoint query/branch topics, but no RViz/UI tool exists yet for selecting and replaying a previous checkpoint.
- Hardware validation on the Go2 is still required before enabling `enable_motion:=true`.
- The ROS graph is still sensitive to CycloneDDS participant limits, so lean launches may still be needed on the current machine.
