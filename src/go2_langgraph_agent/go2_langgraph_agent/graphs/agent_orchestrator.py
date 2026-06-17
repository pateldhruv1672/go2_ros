from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import json
import traceback

from go2_langgraph_agent.debate import run_debate
from go2_langgraph_agent.graphs.agent_state import (
    AgentState,
    MOTION_ACTIONS,
    append_event,
    classify_intent,
    compact_langgraph_value,
    event_update,
    extract_text_from_message,
    new_run_id,
    normalize_command,
    require_langgraph,
    summarize_context,
)
from go2_langgraph_agent.graphs.exploration_graph import build_graph as build_exploration_graph
from go2_langgraph_agent.graphs.memory_manager_graph import build_graph as build_memory_graph
from go2_langgraph_agent.graphs.navigation_graph import build_graph as build_navigation_graph
from go2_langgraph_agent.graphs.recovery_graph import build_graph as build_recovery_graph
from go2_langgraph_agent.graphs.tour_graph import build_graph as build_tour_graph
from go2_langgraph_agent.persistence import LangGraphSQLitePersistence
from go2_langgraph_agent.tools.memory_tools import MemoryTools


def _compact_for_checkpoint(value, *, depth=0, max_depth=6, max_list=40, max_str=12000):
    """Return a JSON-safe, bounded-size object for LangGraph checkpoint/state.

    ROS topic payloads can contain large maps, coverage plans, pointcloud summaries,
    or repeated stream/event histories. LangGraph checkpointers should persist
    decisions and compact summaries, not unbounded raw sensor/map data.
    """
    if depth > max_depth:
        return {"__truncated__": "max_depth"}

    if value is None or isinstance(value, (bool, int, float)):
        return value

    if isinstance(value, str):
        if len(value) > max_str:
            return value[:max_str] + f"...<truncated {len(value) - max_str} chars>"
        return value

    if isinstance(value, bytes):
        return f"<bytes:{len(value)}>"

    if isinstance(value, (list, tuple)):
        out = [_compact_for_checkpoint(v, depth=depth + 1, max_depth=max_depth, max_list=max_list, max_str=max_str) for v in list(value)[:max_list]]
        if len(value) > max_list:
            out.append({"__truncated_items__": len(value) - max_list})
        return out

    if isinstance(value, dict):
        heavy_keys = {
            "data", "ranges", "intensities", "raw", "raw_message", "raw_map",
            "occupancy_grid", "pointcloud", "points", "image", "image_b64",
            "camera_frame", "map_cells", "costmap", "scan_ranges",
        }
        out = {}
        for k, v in value.items():
            key = str(k)
            if key in heavy_keys:
                if isinstance(v, (list, tuple, bytes, str)):
                    out[key] = f"<omitted:{key}:len={len(v)}>"
                else:
                    out[key] = f"<omitted:{key}:{type(v).__name__}>"
                continue
            out[key] = _compact_for_checkpoint(v, depth=depth + 1, max_depth=max_depth, max_list=max_list, max_str=max_str)
        return out

    return str(value)[:max_str]


def _compact_agent_state(state):
    compact = _compact_for_checkpoint(state, max_list=40, max_str=12000)
    if isinstance(compact, dict):
        # Event history can grow fast; keep only recent events.
        events = compact.get("events")
        if isinstance(events, list) and len(events) > 30:
            compact["events"] = events[-30:]
            compact["events_truncated_count"] = len(events) - 30

        # Keep only top candidates/waypoints.
        ctx = compact.get("context")
        if isinstance(ctx, dict):
            for key in ("frontier_candidates", "coverage_plan", "candidate_frontiers", "coverage_waypoints"):
                val = ctx.get(key)
                if isinstance(val, list) and len(val) > 20:
                    ctx[key] = val[:20]
                    ctx[key + "_truncated_count"] = len(val) - 20

        # Same at top-level, if present.
        for key in ("frontier_candidates", "coverage_plan", "candidate_frontiers", "coverage_waypoints"):
            val = compact.get(key)
            if isinstance(val, list) and len(val) > 20:
                compact[key] = val[:20]
                compact[key + "_truncated_count"] = len(val) - 20

    return compact


def _tiny_state_for_graph(state):
    """Minimal checkpoint-safe state for LangGraph.

    Do not checkpoint raw ROS context, map data, point clouds, scan arrays,
    large frontier JSON, large coverage plans, or event history.
    """
    if not isinstance(state, dict):
        return {"raw_command": str(state)[:2000]}

    context = state.get("context") if isinstance(state.get("context"), dict) else {}

    # Keep only compact graph/memory/navigation summaries.
    compact_context = {
        "graph_summary": context.get("graph_summary", {}),
        "localization_confidence": context.get("localization_confidence"),
        "has_spawn": context.get("has_spawn"),
        "latest_checkpoint_id": context.get("latest_checkpoint_id"),
    }

    # Keep only top frontier/coverage summaries if already parsed into lists.
    for key in ("frontier_candidates", "candidate_frontiers"):
        val = context.get(key)
        if isinstance(val, list):
            compact_context[key] = val[:5]
        elif isinstance(val, dict):
            candidates = val.get("candidates")
            compact_context[key] = {"candidate_count": val.get("candidate_count"), "candidates": candidates[:5] if isinstance(candidates, list) else []}

    for key in ("coverage_plan", "coverage_waypoints"):
        val = context.get(key)
        if isinstance(val, list):
            compact_context[key] = val[:10]
        elif isinstance(val, dict):
            waypoints = val.get("waypoints") or val.get("poses") or val.get("path")
            compact_context[key] = {"waypoint_count": val.get("waypoint_count") or val.get("count"), "waypoints": waypoints[:10] if isinstance(waypoints, list) else []}

    # Keep selected goal/decision but omit giant histories.
    return {
        "run_id": str(state.get("run_id", ""))[:128],
        "thread_id": str(state.get("thread_id", ""))[:128],
        "raw_command": str(state.get("raw_command", ""))[:2000],
        "text": str(state.get("text", ""))[:2000],
        "intent": state.get("intent", {}),
        "candidate_actions": state.get("candidate_actions", [])[:10] if isinstance(state.get("candidate_actions"), list) else [],
        "decision": state.get("decision", {}),
        "selected_goal": state.get("selected_goal"),
        "nav_command": state.get("nav_command"),
        "speech": str(state.get("speech", ""))[:4000],
        "context": compact_context,
        "events": [],
        "errors": [],
    }


class AgentOrchestrator:
    """Real LangGraph supervisor for Go2.

    This class owns one compiled LangGraph StateGraph with conditional routing,
    SQLite checkpointing, event streaming, subgraph execution, safety gating, and
    optional human interrupts before risky motion. The ROS node remains only an
    adapter around this runtime.
    """

    def __init__(
        self,
        memory: MemoryTools,
        persistence: LangGraphSQLitePersistence,
        feature_flags: Dict[str, bool],
        live_context_provider: Any = None,
        thread_id: str = "default",
    ):
        require_langgraph()
        self.memory = memory
        self.persistence = persistence
        self.feature_flags = feature_flags
        self.live_context_provider = live_context_provider
        self.thread_id = thread_id
        self.memory_graph = build_memory_graph(memory)
        self.navigation_graph = build_navigation_graph()
        self.exploration_graph = build_exploration_graph()
        self.recovery_graph = build_recovery_graph()
        self.tour_graph = build_tour_graph(memory)
        self.graph = self._build_graph()
        self.config = {"configurable": {"thread_id": self.thread_id}}

    def _build_graph(self) -> Any:
        from langgraph.graph import END, START, StateGraph  # type: ignore

        builder = StateGraph(AgentState)
        builder.add_node("input_router", self.input_router)
        builder.add_node("context_builder", self.context_builder)
        builder.add_node("memory_refresh", self.memory_refresh)
        builder.add_node("intent_parser", self.intent_parser)
        builder.add_node("candidate_action_builder", self.candidate_action_builder)
        builder.add_node("debate_council", self.debate_council)
        builder.add_node("human_interrupt_gate", self.human_interrupt_gate)
        builder.add_node("memory_subgraph", self.memory_subgraph)
        builder.add_node("exploration_subgraph", self.exploration_subgraph)
        builder.add_node("tour_subgraph", self.tour_subgraph)
        builder.add_node("navigation_subgraph", self.navigation_subgraph)
        builder.add_node("recovery_subgraph", self.recovery_subgraph)
        builder.add_node("memory_writeback", self.memory_writeback)
        builder.add_node("final_response", self.final_response)
        builder.add_node("fault_handler", self.fault_handler)

        builder.add_edge(START, "input_router")
        builder.add_edge("input_router", "context_builder")
        builder.add_edge("context_builder", "memory_refresh")
        builder.add_edge("memory_refresh", "intent_parser")
        builder.add_edge("intent_parser", "candidate_action_builder")
        builder.add_edge("candidate_action_builder", "debate_council")
        builder.add_edge("debate_council", "human_interrupt_gate")
        builder.add_conditional_edges(
            "human_interrupt_gate",
            self.route_after_gate,
            {
                "memory": "memory_subgraph",
                "explore": "exploration_subgraph",
                "tour": "tour_subgraph",
                "navigate": "navigation_subgraph",
                "recovery": "recovery_subgraph",
                "final": "final_response",
                "fault": "fault_handler",
            },
        )
        builder.add_edge("memory_subgraph", "memory_writeback")
        builder.add_edge("exploration_subgraph", "navigation_subgraph")
        builder.add_conditional_edges("tour_subgraph", self.route_after_tour, {"navigate": "navigation_subgraph", "final": "memory_writeback"})
        builder.add_edge("navigation_subgraph", "memory_writeback")
        builder.add_edge("recovery_subgraph", "memory_writeback")
        builder.add_edge("memory_writeback", "final_response")
        builder.add_edge("fault_handler", "final_response")
        builder.add_edge("final_response", END)
        
        # Compile with both LangGraph persistence concepts when supported:
        # checkpointer = per-thread graph state; store = cross-thread durable memory.
        native_store = getattr(self.persistence.store, "native_store", None) or self.persistence.store
        try:
            return builder.compile(checkpointer=self.persistence.checkpointer, store=native_store)
        except TypeError:
            # Older LangGraph versions may not expose compile(store=...). The agent
            # still uses the durable SQLite store directly in graph nodes.
            return builder.compile(checkpointer=self.persistence.checkpointer)

    def invoke(self, raw_command: str) -> AgentState:
        input_state: AgentState = {
            "run_id": new_run_id("agent"),
            "thread_id": self.thread_id,
            "raw_command": raw_command,
            "feature_flags": dict(self.feature_flags),
            "events": [],
            "memory_updates": [],
            "history": [],
            "store_events": [],
        }
        self.persistence.store.put(("threads", self.thread_id, "commands"), input_state["run_id"], _compact_for_checkpoint({"raw_command": raw_command}))
        input_state = _tiny_state_for_graph(input_state)
        result = self.graph.invoke(input_state, self.config)
        self.persistence.write_summary(compact_langgraph_value(result))
        return result

    def stream(self, raw_command: str) -> List[Dict[str, Any]]:
        """Run the graph and return LangGraph update events for ROS publication."""
        updates: List[Dict[str, Any]] = []
        input_state: AgentState = {
            "run_id": new_run_id("agent"),
            "thread_id": self.thread_id,
            "raw_command": raw_command,
            "feature_flags": dict(self.feature_flags),
            "events": [],
            "memory_updates": [],
            "history": [],
            "store_events": [],
        }
        self.persistence.store.put(("threads", self.thread_id, "commands"), input_state["run_id"], _compact_for_checkpoint({"raw_command": raw_command}))
        final_state: Optional[AgentState] = None
        for event in self.graph.stream(input_state, self.config, stream_mode="updates"):
            updates.append(compact_langgraph_value(event, max_depth=5))
            try:
                self.persistence.store.append_event(self.thread_id, {"type": "stream_update", "run_id": input_state["run_id"], "payload": compact_langgraph_value(event, max_depth=5)})
            except Exception:
                pass
            if isinstance(event, dict):
                for value in event.values():
                    if isinstance(value, dict) and value.get("status"):
                        final_state = value  # best-effort; invoke-style state may not appear in update mode
        # Retrieve final checkpointed state after streaming.
        if final_state is None:
            try:
                snapshot = self.graph.get_state(self.config)
                values = getattr(snapshot, "values", None)
                if isinstance(values, dict):
                    final_state = values
            except Exception:
                final_state = None
        if final_state:
            self.persistence.write_summary(compact_langgraph_value(final_state))
        return updates

    def run_with_stream(self, raw_command: str) -> Tuple[AgentState, List[Dict[str, Any]]]:
        """Run once with LangGraph streaming and return final checkpointed state."""
        events = self.stream(raw_command)
        snapshot = self.graph.get_state(self.config)
        state = getattr(snapshot, "values", None) or {}
        if isinstance(state, dict):
            self.persistence.write_summary(compact_langgraph_value(state))
            return state, events
        return {"error": {"message": "stream_completed_without_state"}, "events": []}, events

    def resume_interrupt(self, resume_payload: Any) -> AgentState:
        """Resume a paused LangGraph interrupt with human input."""
        from langgraph.types import Command  # type: ignore

        result = self.graph.invoke(Command(resume=resume_payload), self.config)
        self.persistence.write_summary(compact_langgraph_value(result))
        return result

    def list_checkpoints(self, limit: int = 10) -> List[Dict[str, Any]]:
        return self.persistence.list_checkpoints(self.thread_id, limit=limit)

    def get_checkpoint_state(self, checkpoint_id: Optional[str] = None) -> Dict[str, Any]:
        """Inspect current or historical checkpoint state for time-travel debugging."""
        return self.persistence.get_state(self.graph, self.thread_id, checkpoint_id=checkpoint_id)

    def time_travel_invoke(self, checkpoint_id: str, raw_command: Optional[str] = None) -> AgentState:
        """Branch/re-run from a previous checkpoint.

        The checkpoint_id is added to the runnable config. If raw_command is
        provided, the branch receives that command; otherwise LangGraph resumes
        from the checkpoint with no new input.
        """
        config = {"configurable": {"thread_id": self.thread_id, "checkpoint_id": checkpoint_id}}
        if raw_command:
            input_state: AgentState = {
                "run_id": new_run_id("time_travel"),
                "thread_id": self.thread_id,
                "raw_command": raw_command,
                "feature_flags": dict(self.feature_flags),
                "events": [],
                "memory_updates": [],
                "history": [],
                "store_events": [],
                "time_travel": {"from_checkpoint_id": checkpoint_id},
            }
            result = self.graph.invoke(input_state, config)
        else:
            result = self.graph.invoke(None, config)
        self.persistence.write_summary(compact_langgraph_value(result))
        self.persistence.store.put(("threads", self.thread_id, "time_travel"), str(result.get("run_id") or checkpoint_id), {
            "checkpoint_id": checkpoint_id,
            "raw_command": raw_command,
            "result_status": result.get("status"),
        })
        return result

    # ------------------------- graph nodes -------------------------

    def input_router(self, state: AgentState) -> Dict[str, Any]:
        text = normalize_command(extract_text_from_message(state.get("raw_command", "")))
        return {
            "text": text,
            "events": event_update("main_supervisor", "command_received", {"text": text})["events"],
        }

    def context_builder(self, state: AgentState) -> Dict[str, Any]:
        context: Dict[str, Any] = {}
        try:
            context = self.memory.get_resume_context()
        except Exception as exc:
            context = {"memory_context_error": str(exc)}
        live: Dict[str, Any] = {}
        if self.live_context_provider is not None:
            try:
                live = dict(self.live_context_provider() or {})
            except Exception as exc:
                live = {"live_context_error": str(exc)}
        context.update(live)
        try:
            context["langgraph_store_recent"] = compact_langgraph_value(self.persistence.store.search(("threads", self.thread_id), limit=3), max_depth=3)
        except Exception as exc:
            context["langgraph_store_error"] = str(exc)
        context.setdefault("localization_confidence", 0.75)
        context.setdefault("safety_blocked", False)
        if "scan_summary" in context:
            front = ((context.get("scan_summary") or {}).get("sector_clearance_m") or {}).get("front")
            try:
                if front is not None and float(front) < 0.45:
                    context["safety_blocked"] = True
            except Exception:
                pass
        dyn = context.get("dynamic_obstacles")
        if isinstance(dyn, dict) and dyn.get("front_blocked"):
            context["safety_blocked"] = True
        context.setdefault("live_observation", "No live VLM observation has been attached to this command yet.")
        context = compact_langgraph_value(context)
        return {"context": context, "events": event_update("main_supervisor", "context_built", summarize_context(context))["events"]}

    def memory_refresh(self, state: AgentState) -> Dict[str, Any]:
        return self.memory_graph.invoke(state)

    def intent_parser(self, state: AgentState) -> Dict[str, Any]:
        parsed = classify_intent(state.get("text", ""))
        return {"parsed_intent": parsed, "events": event_update("main_supervisor", "intent_parsed", parsed)["events"]}

    def candidate_action_builder(self, state: AgentState) -> Dict[str, Any]:
        actions = self._candidate_actions(state)
        return {"candidate_actions": actions, "events": event_update("main_supervisor", "candidate_actions", {"actions": actions})["events"]}

    def debate_council(self, state: AgentState) -> Dict[str, Any]:
        try:
            if self.feature_flags.get("enable_debate_layer", True):
                decision = run_debate(
                    state.get("text", ""),
                    state.get("context", {}),
                    state.get("candidate_actions", []),
                    enable_llm=bool(self.feature_flags.get("enable_llm_debate", False)),
                    llm_provider=self.feature_flags.get("debate_llm_provider") or None,
                    llm_model=self.feature_flags.get("debate_llm_model") or None,
                    llm_timeout_sec=float(self.feature_flags.get("debate_llm_timeout_sec", 8.0) or 8.0),
                )
                decision_dict = decision.to_dict()
            else:
                actions = state.get("candidate_actions") or ["speak"]
                decision_dict = {
                    "decision_id": "debate_disabled",
                    "user_intent": state.get("text", ""),
                    "candidate_actions": actions,
                    "context_used": state.get("context", {}),
                    "votes": {"systems": "debate_disabled"},
                    "final_action": actions[0],
                    "confidence": 0.5,
                    "risk_level": "low",
                    "requires_human_interrupt": False,
                    "fallback_plan": ["stop_robot"],
                    "spoken_response": "Debate layer is disabled; using first safe candidate action.",
                    "memory_updates": [],
                }
            decision_dict = compact_langgraph_value(decision_dict)
            self.memory.save_decision(decision_dict)
            try:
                self.persistence.store.put(("threads", self.thread_id, "debate_decisions"), decision_dict.get("decision_id", str(state.get("run_id"))), decision_dict)
            except Exception:
                pass
            return {
                "decision": decision_dict,
                "events": event_update("main_supervisor", "debate_complete", {
                    "final_action": decision_dict.get("final_action"),
                    "risk": decision_dict.get("risk_level"),
                    "mode": decision_dict.get("debate_mode"),
                    "vote_count": len(decision_dict.get("council_votes") or []),
                })["events"],
            }
        except Exception as exc:
            return {
                "error": {"node": "debate_council", "message": str(exc), "traceback": traceback.format_exc(limit=6)},
                "events": event_update("main_supervisor", "debate_error", {"error": str(exc)})["events"],
            }

    def human_interrupt_gate(self, state: AgentState) -> Dict[str, Any]:
        """Optional durable human approval before risky motion.

        In normal dry-run workflows this returns immediately. If
        enable_human_interrupts is true and the debate requires human review,
        LangGraph's interrupt() persists the state and pauses graph execution.
        """
        decision = state.get("decision") or {}
        final_action = decision.get("final_action")
        requires_motion = final_action in MOTION_ACTIONS or bool((state.get("parsed_intent") or {}).get("requires_motion"))
        needs_interrupt = bool(self.feature_flags.get("enable_human_interrupts", False) and requires_motion and decision.get("requires_human_interrupt"))
        if not needs_interrupt:
            return {"pending_interrupt": {}, "events": event_update("main_supervisor", "interrupt_not_required", {"final_action": final_action})["events"]}
        try:
            from langgraph.types import interrupt  # type: ignore
            payload = {
                "type": "motion_approval",
                "run_id": state.get("run_id"),
                "thread_id": self.thread_id,
                "risk_level": decision.get("risk_level"),
                "reason": decision.get("spoken_response"),
                "candidate_action": final_action,
                "fallback_plan": decision.get("fallback_plan"),
                "instruction": "Reply with approve/resume motion or reject/cancel motion.",
            }
            approval = interrupt(payload)
            approved = self._approval_is_positive(approval)
            if not approved:
                return {
                    "human_approval": {"approved": False, "raw": approval},
                    "nav_command": {"action": "stop_robot", "reason": "human_rejected_or_no_approval"},
                    "speech_response": "I will not move. Human approval was not provided.",
                    "events": event_update("main_supervisor", "human_interrupt_rejected", {"approval": str(approval)})["events"],
                }
            return {
                "human_approval": {"approved": True, "raw": approval},
                "events": event_update("main_supervisor", "human_interrupt_approved", {"approval": str(approval)})["events"],
            }
        except Exception as exc:
            return {
                "pending_interrupt": {"error": str(exc), "decision": decision},
                "nav_command": {"action": "stop_robot", "reason": "interrupt_failed"},
                "speech_response": "I need human approval before moving, but interrupt handling failed, so I am stopping.",
                "events": event_update("main_supervisor", "human_interrupt_error", {"error": str(exc)})["events"],
            }

    def memory_subgraph(self, state: AgentState) -> Dict[str, Any]:
        return self.memory_graph.invoke(state)

    def exploration_subgraph(self, state: AgentState) -> Dict[str, Any]:
        if not self.feature_flags.get("enable_explore_mode", False):
            return {
                "speech_response": "Explore mode is available but disabled by launch flag. Re-launch with enable_explore_mode:=true.",
                "nav_command": {"action": "stop_robot", "reason": "explore_disabled"},
                "events": event_update("main_supervisor", "explore_disabled", {})["events"],
            }
        return self.exploration_graph.invoke(state)

    def tour_subgraph(self, state: AgentState) -> Dict[str, Any]:
        intent = (state.get("parsed_intent") or {}).get("intent")
        if intent == "start_tour" and not self.feature_flags.get("enable_tour_mode", False):
            return {
                "speech_response": "Tour mode is available but disabled by launch flag. Re-launch with enable_tour_mode:=true.",
                "nav_command": {"action": "stop_robot", "reason": "tour_disabled"},
                "events": event_update("main_supervisor", "tour_disabled", {})["events"],
            }
        return self.tour_graph.invoke(state)

    def navigation_subgraph(self, state: AgentState) -> Dict[str, Any]:
        return self.navigation_graph.invoke(state)

    def recovery_subgraph(self, state: AgentState) -> Dict[str, Any]:
        return self.recovery_graph.invoke(state)

    def memory_writeback(self, state: AgentState) -> Dict[str, Any]:
        # Write a mission checkpoint request after any concrete nav/explore/tour
        # command. This is metadata only; VLM/voxel artifact writing remains in
        # the dedicated ROS checkpoint nodes.
        command = state.get("nav_command") or {}
        try:
            self.persistence.store.put(("threads", self.thread_id, "nav_commands"), str(state.get("run_id") or new_run_id("nav")), {
                "command": command,
                "decision": compact_langgraph_value(state.get("decision") or {}),
                "intent": state.get("parsed_intent") or {},
            })
        except Exception:
            pass
        if command.get("action") in {None, "none", "stop_robot"}:
            return {"events": event_update("main_supervisor", "memory_writeback_skipped", {"action": command.get("action")})["events"]}
        payload = {
            "layer": "temporary",
            "source": "langgraph_agent",
            "command": command,
            "decision": compact_langgraph_value(state.get("decision") or {}),
            "context_summary": summarize_context(compact_langgraph_value(state.get("context") or {})),
        }
        try:
            result = self.memory.write_checkpoint(payload)
        except Exception as exc:
            return {
                "memory_updates": [{"type": "checkpoint_write_failed", "error": str(exc), "payload": payload}],
                "events": event_update("main_supervisor", "memory_writeback_failed", {"error": str(exc)})["events"],
            }
        return {
            "memory_updates": [{"type": "checkpoint_written", "result": result}],
            "events": event_update("main_supervisor", "memory_writeback_done", {"result": result})["events"],
        }

    def final_response(self, state: AgentState) -> Dict[str, Any]:
        command = state.get("nav_command") or {}
        if command.get("action") in {"none", None}:
            command = {}
        speech = state.get("speech_response") or (state.get("decision") or {}).get("spoken_response") or "Done."
        status = {
            "run_id": state.get("run_id"),
            "thread_id": self.thread_id,
            "intent": (state.get("parsed_intent") or {}).get("intent"),
            "decision": (state.get("decision") or {}).get("final_action"),
            "nav_action": command.get("action"),
            "context": summarize_context(state.get("context") or {}),
            "pending_interrupt": state.get("pending_interrupt") or {},
            "speech": speech,
            "checkpoint_db": str(self.persistence.db_path),
            "store_db": str(self.persistence.store_path),
            "store_namespaces": self.persistence.store.list_namespaces(limit=20),
        }
        history_entry = {
            "run_id": state.get("run_id"),
            "text": state.get("text"),
            "parsed_intent": state.get("parsed_intent"),
            "decision": compact_langgraph_value(state.get("decision")),
            "nav_command": command,
            "speech_response": speech,
        }
        return {
            "nav_command": command,
            "speech_response": speech,
            "status": status,
            "history": [history_entry],
            "events": event_update("main_supervisor", "finalized", {"nav_action": command.get("action"), "speech_len": len(speech)})["events"],
        }

    def fault_handler(self, state: AgentState) -> Dict[str, Any]:
        err = state.get("error") or {"message": "unknown graph fault"}
        return {
            "nav_command": {"action": "stop_robot", "reason": "graph_fault", "error": err},
            "speech_response": "I hit an internal agent error and am stopping instead of moving.",
            "events": event_update("main_supervisor", "fault_handled", err)["events"],
        }

    # ------------------------- routers -------------------------

    def route_after_gate(self, state: AgentState) -> str:
        if state.get("error"):
            return "fault"
        command = state.get("nav_command") or {}
        if command.get("action") == "stop_robot" and command.get("reason") in {"human_rejected_or_no_approval", "interrupt_failed"}:
            return "final"
        decision = state.get("decision") or {}
        final_action = decision.get("final_action")
        intent = (state.get("parsed_intent") or {}).get("intent", "chat")
        if final_action in {"recover_localization", "recover_nav_failure"}:
            return "recovery"
        if intent in {"save_place", "remember", "query_memory", "explain_state"}:
            return "memory"
        if intent in {"explore", "coverage_explore"} or final_action in {"frontier_explore", "coverage_explore"}:
            return "explore"
        if intent in {"start_tour", "tour_question"} or final_action == "navigate_tour_route":
            return "tour"
        if intent in {"navigate", "return_to_spawn", "stop", "continue_task"}:
            return "navigate"
        return "final"

    def route_after_tour(self, state: AgentState) -> str:
        return "navigate" if (state.get("parsed_intent") or {}).get("intent") == "start_tour" else "final"

    # ------------------------- helpers -------------------------

    def _candidate_actions(self, state: AgentState) -> List[str]:
        intent = (state.get("parsed_intent") or {}).get("intent", "chat")
        if intent == "stop":
            return ["stop_robot"]
        if intent == "return_to_spawn":
            return ["return_to_spawn", "stop_robot"]
        if intent == "navigate":
            return ["navigate_to_pose", "query_memory", "recover_localization"]
        if intent == "start_tour":
            return ["navigate_tour_route", "speak", "query_memory"]
        if intent == "tour_question":
            return ["speak", "query_memory"]
        if intent == "explore":
            return ["frontier_explore", "write_checkpoint", "return_to_spawn"]
        if intent == "coverage_explore":
            return ["coverage_explore", "write_checkpoint", "return_to_spawn"]
        if intent in {"save_place", "remember"}:
            return ["write_memory", "speak"]
        if intent == "query_memory":
            return ["query_memory", "speak"]
        if intent == "continue_task":
            return ["continue_previous_task", "recover_localization"]
        if intent == "explain_state":
            return ["speak", "query_memory"]
        if intent == "approve_interrupt":
            return ["continue_previous_task"]
        if intent == "reject_interrupt":
            return ["stop_robot"]
        return ["speak"]

    def _approval_is_positive(self, approval: Any) -> bool:
        if isinstance(approval, bool):
            return approval
        if isinstance(approval, dict):
            value = approval.get("approved", approval.get("decision", approval.get("text", "")))
            if isinstance(value, bool):
                return value
            approval = str(value)
        text = str(approval).lower()
        return any(token in text for token in ("approve", "approved", "yes", "continue", "resume"))
