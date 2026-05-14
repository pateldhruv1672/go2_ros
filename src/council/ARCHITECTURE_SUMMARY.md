# Council AI Architecture: Cognitive Refactor Summary

## Overview
This document captures the major architectural changes implemented to strengthen the Go2 Council multi-agent system with cognitive memory, inter-agent collaboration, and camera-priority navigation for visual tasks.

**Session Goals Addressed:**
1. ✅ Fix AI mode immediate-exit bug ("[No task]")
2. ✅ Prevent blind backward motion (LiDAR/SLAM constraints)
3. ✅ Enhance camera usage for object-seeking tasks
4. ✅ Implement LangChain-style planner/executor memory
5. ✅ Reduce response truncation and logging noise

---

## Key Changes

### 1. **Task Planner Integration** (`task_planner.py`)
- **Checkpoint Decomposition**: LLM breaks task into 2-5 concrete, sequential checkpoints
- **Progress Tracking**: Monitors steps, distance, stuck conditions per checkpoint
- **Completion Logic**: Task is complete only when all checkpoints done (fixed: uninitialized ≠ complete)
- **Stuck Detection**: Skips checkpoint if same action repeated 3+ times or max steps exceeded

**Impact**: AI mode no longer exits prematurely; robots make measurable progress toward goals.

### 2. **Cognitive Memory + Reasoning Trace** (`orchestrator.py`)

#### Reasoning History
- Stores compact traces of each decision cycle:
  - `timestamp`, `task`, `objective`, `checkpoint`
  - `agent_reports` (sensor analysis summary)
  - `decision` (action, confidence, risk, decided-by)

#### Compressed Memory
- **Rolling compression**: After 40 traces, older entries are summarized into ~200-char compressed string
- **Short-term window**: Last 8 traces remain detailed for immediate context
- **Injection into agents**: Both compressed + recent summaries fed into agent and meta-agent contexts

**Pattern**: Follows LangChain ReAct memory compression (history → summary → compact memory)

### 3. **Peer Reflection Round** (`orchestrator.py`, agent collaboration)
- **When**: Triggered on agent disagreement, low confidence, or visual tasks
- **How**: Agents see peer summaries + can revise recommendations
- **Safeguard**: New recommendations only accepted if risk-equal or risk-improved
- **Timeout**: Reflection limited to ~8s to avoid stalling perception

**Pattern**: Multi-agent discussion (agents see each other's reports via orchestrator)

### 4. **Structured Task Context** (`main.py` → `orchestrator.py`)
Instead of embedding raw task string, planner sends structured context:
```python
orchestrator.task_context = {
    "current_objective": "Approach water bottle from 2m to 0.5m distance",
    "checkpoint": "Move forward while tracking target",
    "task_progress": "Step 3/5: Navigating toward target",
    "task_planner_status": "[3/5] Move forward...",
}
```
Agents receive this context + can coordinate based on shared checkpoint state.

### 5. **Camera-Priority Voting** (`orchestrator.py`, `_weighted_vote()`)

#### Visual Task Detection
- Keywords: "find", "look", "bottle", "person", "door", "near", etc.
- When detected: Camera weight raised from 1.5 → 2.2
- Meta-agent escalation: Triggers review if camera sees target but decision is stop/reverse

#### Smart Camera Override
- **Condition**: Visual task, camera confirms target-visible+actionable, no high-risk stop present
- **Action**: Override preliminary stop/reverse vote with camera recommendation
- **Confidence adjustment**: Camera confidence upgraded toward meta-agent confidence floor (0.88)

**Effect**: Robot pursues visual targets instead of getting stuck in conservative blocking patterns.

### 6. **Reverse-Motion Safety Tiers** 

#### Agent Level
- **LiDAR Agent**: Reverse only justified if (front_danger=HIGH) AND (rear_clear=TRUE)
- **SLAM Agent**: Avoids reverse unless localization lost
- **Camera Agent**: Enforces defer on low-confidence visual guesses
- Fallback: Turn instead of reverse when justification insufficient

#### Orchestrator Level
- **Hard constraint** in `_constrain_motion_action()`:
  - Reverse requires: explicit LiDAR confirmation + front threat + rear clear
  - Otherwise: Stop or turn recommendation

#### Session Logging
- Attribution: `decided_by` tracks which agent/process made final call
- `planner_blocked` flags nav2 safety overrides
- Enables post-run diagnosis of why action was chosen

### 7. **Sensor Data Enhancement** (`sensor_hub.py`)

#### LiDAR Completeness
- `bev_b64`: Base64 JPEG bird's-eye view image (vision-model friendly)
- `points_raw`: Raw point cloud before filtering (for temporal analysis)
- `points`: Filtered cloud (robot legs + ground removed)
- `stats`: Min/max/mean distance, point counts, height levels

#### Impact
- LiDAR agent receives enriched multimodal input (image + statistics)
- Allows temporal consistency checking and sector-based obstacle analysis

### 8. **Compact Output Formatting** (`base_agent.py`)

Changed from:
```
{
  "context": {
    "key1": "value1",
    ...
  }
}
```

To:
```
context_json: {"key1": "value1", ...}  // Single line, compact
```

**Result**: Reduced console spam; easier log parsing.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       User Task                                 │
│         "Go near the water bottle and stop there"               │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
              ┌──────────────────────────┐
              │   Task Planner           │
              │ ┌──────────────────────┐ │
              │ │ Checkpoint 1: Scan   │ │  (LLM decomposition)
              │ │ Checkpoint 2: Move   │ │
              │ │ Checkpoint 3: Stop   │ │
              │ └──────────────────────┘ │
              └──────────┬───────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
╔════════════════╕   ╔════════════════╕   ╔════════════════╕
║  Orchestrator  ║   ║  Memory State  ║   ║ Planner Safety ║
║                ║   ║                ║   ║ (nav2 map)     ║
║ Phase 1: Perc. ║   ║ • reasoning_   ║   ║                ║
║ (sensors only) ║   ║   history (8)  ║───┤ • Grid check   ║
║                ║   ║ • compressed   ║   ║ • Direction    ║
║ Phase 2: Vote  ║   ║   memory (~700 ║   ║   validation   ║
║ (w/ context)   ║   ║   chars)       ║   ║                ║
║                ║   ╚════════════════╝   ╚════════════════╝
║ Phase 3: Meta  ║
║ (if disputed)  ║
║                ║
╚───────┬────────╝
        │
        ├─ Perception (parallel agents)
        │  ├─ Camera: visual scene + target
        │  ├─ LiDAR: BEV image + sector analysis
        │  ├─ IMU: stability assessment
        │  └─ SLAM: map + localization quality
        │
        ├─ Peer Reflection (if disagreement/low-conf)
        │  └─ Agents revise seeing peer reports
        │
        ├─ Weighted Vote (role-based + dynamic)
        │  ├─ Navigation: camera (2.2), lidar (1.2)
        │  ├─ Safety: IMU, SLAM veto high-risk
        │  └─ Visual task: camera boost ↑ 46%
        │
        ├─ Meta-Agent Review (if conflict)
        │  └─ Final decision with reasoning trace
        │
        └─ Recording Trace (for compression)
           └─ Store in reasoning_history
              (compress after 40 entries)
```

---

## Testing & Validation

### Unit Tests Passing
- ✅ LiDAR BEV encoding and agent multimodal input
- ✅ Task context injection into agent context
- ✅ Reasoning memory compression + recent-window maintenance  
- ✅ Visual-task escalation to meta-agent
- ✅ Agent peer reporting (summarization)

### Integration Tests
- ✅ Full perception → vote → decision cycle
- ✅ Camera-priority override on visual tasks
- ✅ Planner safety constraints on nav2 data
- ✅ Task completion semantics (uninitialized ≠ complete)

### Not Yet Validated (requires real robot/ROS)
- Behavioral validation: actual forward/stop/turn sequences on real Go2
- Camera inference latency + frame-drop resilience
- Perception quality with noisy real lidar + imu
- Long-run memory compression stability (>100 cycles)

---

## Known Limitations & Future Improvements

### Limitations
1. **Reflection latency**: Optional peer-reflection round adds ~8s overhead (gated on disagreement)
2. **Memory compression lossiness**: Summaries are compact → some nuance lost
3. **Camera fallback**: No image = agent defers (room 307 w/ no overhead camera not ideal)
4. **Reverse logic complexity**: Multiple tiers of constraints can be hard to debug

### Future Work
1. **Batched reflection**: Reflect on sub-group of agents (e.g., navigation only) vs all
2. **Adaptive compression**: Adjust summary detail based on task type (e.g., verbose for novel tasks)
3. **Multi-camera support**: External camera input + internal depth for better visual coverage
4. **Learned task decomposition**: Replace LLM decomposition w/ learned checkpoints from demo
5. **Persistent memory**: Store compressed memory across sessions (ontology of places/obstacles)

---

## Debugging Guide

### Console Logs to Watch

**[AI] Decision line:**
```
[AI] turn_left | council_vote:lidar+camera | 🗺️⚠️blocked:forward
     Votes: imu→defer lidar→turn_left camera→forward slam→defer
     [3/5] Move forward until target in view (2/20 steps)
```
Read as:
- Action: `turn_left`
- Decided by: `council_vote` weighted by `lidar` + `camera`
- Map available?, planner blocked?: emoji indicators
- Final step count in checkpoint

**[Council] Decision line:**
```
[Council] Decision: forward | conf=0.82 | risk=low
```
Confirms orchestrator result before execution.

**[Agent] Error line:**
```
[LidarAgent] Error: {error details}
```
Check agent state + last response using `orchestrator.agents["lidar"].get_last_response()`.

### Common Issues

| Symptom | Check |
|---------|-------|
| Robot stops despite clear path | Navigation planner has nav2 map + cost blocked direction |
| Camera never consulted | Check `Is visual task:` detect in `orchestrator.py` keyword list |
| Backward drift | Verify LiDAR is reporting rear clear + front is actually unsafe |
| Endless "thinking" | Reflection timeout or LLM hang (check OpenRouter rate limit) |
| Weird compressed memory | Check `_recent_reasoning_window = 8` and `_max_reasoning_records = 40` |

---

## File Map (Cognitive Architecture)

- **`orchestrator.py`** (1150 lines)
  - Deliberation loop (Phase 1/2/3)
  - Reasoning trace + memory compression
  - Weighted voting + camera override
  - Meta-agent review prompt + context building
  - Helper: `_build_perception_task`, `_record_reasoning_trace`, `_compress_reasoning_history`, `_peer_reflection_round`

- **`task_planner.py`** (400 lines)
  - Checkpoint decomposition (LLM)
  - Progress tracking + stuck detection
  - Completion semantics (uninitialized check)
  - Status helpers (get_current_objective, get_agent_context)

- **`main.py`** (750 lines)
  - AI loop integration (process_ai_step)
  - Planner context handoff (task_context update)
  - Task completion guard (_task_decomposed check)
  - Checkpoint status display

- **`agents/base_agent.py`** (310 lines)
  - Multimodal message support (image + text blocks)
  - Compact logging (one-line response preview)
  - Context injection (JSON flattened to single line)

- **`agents/lidar_agent.py`** (380 lines)
  - BEV multimodal input (image + sector stats)
  - Reverse safeguard (front danger + rear clear only)
  - Temporal consistency analysis

- **`agents/camera_agent.py`** (310 lines)
  - Defer on no-image (defer instead of guessing)
  - Low-confidence abstention (<0.35 conf)
  - Target visibility + position tracking

- **`agents/slam_agent.py`** (540 lines)
  - Backward suppression (unless lost localization)
  - Map summary + exploration status
  - Pose trajectory history

- **`ros_interfaces/sensor_hub.py`** (740 lines)
  - BEV encoding (bird's-eye view JPEG)
  - point_raw + points + stats structure
  - Nav2 map subscription + occupancy grid exposure

---

## Next Steps (User Action Items)

### Immediate (Validation)
1. **Run on real Go2**: Execute `council_node` in your lab with robot
   - Expected: Smooth visually-guided navigation, no blind reversals
   - If stuck: Check agent responses in logs (format: `[AgentName] Raw response: ...`)

2. **Test visual task** (object-seeking):
   ```bash
   # In voice controller or direct ROS2 call:
   ros2 topic pub /council/voice_task std_msgs/msg/String "data: 'Go near the water bottle and stop there'"
   ```
   - Expected: Camera visibility drives action (not just LiDAR)

3. **Monitor memory growth**:
   - Check `orchestrator.reasoning_history` length over 200+ cycles
   - Should stabilize at ~8 recent traces + compressed summary

### Medium-term (Tuning)
1. **Adjust dynamic weights** if camera still underused:
   - Increase `weights["camera"]` from 2.2 → 2.5 for visual tasks
   - Lower `weights["lidar"]` from 1.2 → 1.0 if too conservative

2. **Tune reflection threshold**:
   - Currently: reflection if disagreement OR conf < 0.72
   - May skip visual tasks if too costly (add `gating_mode = "disagreement_only"`)

3. **Checkpoint max_steps tuning**:
   - If checkpoints timeout too early: increase from 15-20 → 25-30
   - If stuck detection triggers too late: lower 5-cycle pattern match

### Advanced (Architecture)
1. **Add external camera feed**: Modify `sensor_hub.py` to subscribe to additional `/external_camera` topic
2. **Persistent memory**: Save `compressed_reasoning_memory` to disk at task end
3. **Learned decomposition**: Replace LLM checkpoint generation w/ vision-based milestone detection

---

## References & Patterns

### LangChain Concepts Used
- **ReAct Loop**: Reason (decompose) → Act (execute) → Observe (track) → Decide (next step)
- **Compressed Memory**: Short-term traces + compact historical summary
- **Multi-agent Tool Use**: Agents share observations (peer reports) asynchronously

### Council Pattern (Original)
- Sensor agents (perception specialists) → Orchestrator (voting) → Meta-agent (dispute resolution)

### Additions This Session
- **Memory layer**: Traces + compression between cycles
- **Structured task context**: Planner → agents (not free-form string)
- **Dynamic weighting**: Task type drives voting preferences
- **Peer reflection**: Asynchronous agent revision after initial responses

---

## Summary

The refactored council now exhibits:
1. **Memory persistence**: Reasoning traces compound into understanding
2. **Collaborative perception**: Agents see each other via orchestrator summaries
3. **Balanced safety**: Reverse requires justification; forward encouraged for visual tasks
4. **Robust task semantics**: AI mode stays engaged until task completion
5. **Transparent decision-making**: Every action attributed to agent(s) or planner

**Test Score**: 6/6 architecture features validated in integration test.  
**Build Status**: Clean compilation, no syntax errors.  
**Ready for**: Real-world behavioral validation on Go2 robot (Room 307 arena recommended for camera coverage).

---

**Document Version**: 1.0  
**Date**: 2026-02-26  
**Author**: AI Architecture Refactor Session
