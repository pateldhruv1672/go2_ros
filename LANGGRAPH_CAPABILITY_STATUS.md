# LangGraph capability status

This branch now distinguishes between validated runtime behavior and features that still require robot-side validation.

## Implemented in code

- Real LangGraph StateGraph supervisor with conditional subgraph routing.
- SQLite checkpointer for graph state persistence.
- Native LangGraph Store adapter using `langgraph.store.sqlite.SqliteStore` when available, with a SQLite mirror for ROS topic inspection.
- LLM-powered debate council with seven council roles: Navigator, Localizer, Safety, Perception, SemanticMemory, TourGuide, and Systems.
- Hard safety post-processing after LLM votes so the LLM cannot authorize unsafe motion.
- Event streaming on `/go2_agent/events` and LangGraph update streaming on `/go2_agent/stream`.
- Durable interrupts and resume via `/go2_agent/interrupts` and `/go2_agent/resume`.
- Time-travel checkpoint listing, checkpoint state inspection, branch/re-run support, and RViz-friendly checkpoint markers.
- Store query/write topics for long-term LangGraph memory inspection.

## Runtime configuration

LLM debate is opt-in:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_langgraph_agent:=true \
  enable_debate_layer:=true \
  enable_llm_debate:=true \
  debate_llm_provider:=openrouter \
  debate_llm_model:=openai/gpt-4o-mini
```

Set one of these before launch:

```bash
export OPENROUTER_API_KEY=...
# or
export GO2_DEBATE_LLM_PROVIDER=gemini
export GEMINI_API_KEY=...
```

Time-travel RViz bridge:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_langgraph_agent:=true \
  enable_time_travel_rviz:=true
```

Display `/go2_agent/time_travel_markers` in RViz as a MarkerArray. If interactive markers are available, also add an InteractiveMarkers display for `go2_time_travel_checkpoints`.

## Still needs real-system validation

- LLM API latency and failure behavior during live navigation.
- Long-duration checkpoint recovery after process kill/restart.
- Interactive marker click behavior on your installed RViz/ROS Jazzy image.
- Motion with `enable_motion:=true`; dry-run should be validated first.
