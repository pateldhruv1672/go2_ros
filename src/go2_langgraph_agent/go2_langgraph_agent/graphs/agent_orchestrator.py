from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.debate import run_debate
from go2_langgraph_agent.graphs.agent_state import (
    AgentState,
    append_event,
    classify_intent,
    extract_text_from_message,
    new_run_id,
    normalize_command,
    summarize_context,
)
from go2_langgraph_agent.graphs.exploration_graph import build_graph as build_exploration_graph
from go2_langgraph_agent.graphs.memory_manager_graph import build_graph as build_memory_graph
from go2_langgraph_agent.graphs.navigation_graph import build_graph as build_navigation_graph
from go2_langgraph_agent.graphs.recovery_graph import build_graph as build_recovery_graph
from go2_langgraph_agent.graphs.tour_graph import build_graph as build_tour_graph
from go2_langgraph_agent.persistence import JsonCheckpointer
from go2_langgraph_agent.tools.memory_tools import MemoryTools


class AgentOrchestrator:
    """LangGraph-style supervisor that composes specialized subgraphs.

    It keeps one persisted state per session, routes commands through context,
    memory, debate, and the right task subgraph, and only emits Nav2 commands
    after safety/localization gates have run.
    """

    def __init__(self, memory: MemoryTools, checkpointer: JsonCheckpointer, feature_flags: Dict[str, bool], live_context_provider: Any = None):
        self.memory = memory
        self.checkpointer = checkpointer
        self.feature_flags = feature_flags
        self.live_context_provider = live_context_provider
        self.memory_graph = build_memory_graph(memory)
        self.navigation_graph = build_navigation_graph()
        self.exploration_graph = build_exploration_graph()
        self.recovery_graph = build_recovery_graph()
        self.tour_graph = build_tour_graph(memory)
        self.state = self.checkpointer.load() or {"mode": "idle", "history": [], "events": []}

    def invoke(self, raw_command: str) -> AgentState:
        text = normalize_command(extract_text_from_message(raw_command))
        parsed = classify_intent(text)
        context = self._build_context()
        base_state: AgentState = {
            "run_id": new_run_id("agent"),
            "text": text,
            "raw_command": raw_command,
            "parsed_intent": parsed,
            "context": context,
            "mode": self.state.get("mode", "idle"),
            "history": list(self.state.get("history", []))[-50:],
            "events": [],
            "feature_flags": dict(self.feature_flags),
        }
        append_event(base_state, "main_supervisor", "command_received", {"intent": parsed.get("intent"), "text": text})

        # Context/memory graph always runs first so route/tour/debate can use the
        # freshest persisted data and so place-save/query commands are handled.
        state = self.memory_graph.invoke(base_state)
        state["candidate_actions"] = self._candidate_actions(state)
        state = self._debate(state)
        state = self._route_to_task_graph(state)
        state = self._finalize(state)
        self._persist(state)
        return state

    def _build_context(self) -> Dict[str, Any]:
        context = self.memory.get_resume_context()
        live: Dict[str, Any] = {}
        if self.live_context_provider is not None:
            try:
                live = dict(self.live_context_provider() or {})
            except Exception as exc:
                live = {"live_context_error": str(exc)}
        context.update(live)
        # Conservative defaults when no live monitors have populated context yet.
        context.setdefault("localization_confidence", 0.75)
        context.setdefault("safety_blocked", False)
        if "scan_summary" in context:
            front = ((context.get("scan_summary") or {}).get("sector_clearance_m") or {}).get("front")
            try:
                if front is not None and float(front) < 0.45:
                    context["safety_blocked"] = True
            except Exception:
                pass
        if "dynamic_obstacles" in context and isinstance(context["dynamic_obstacles"], dict):
            if context["dynamic_obstacles"].get("front_blocked"):
                context["safety_blocked"] = True
        context.setdefault("live_observation", "No live VLM observation has been attached to this command yet.")
        return context

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
        return ["speak"]

    def _debate(self, state: AgentState) -> AgentState:
        if self.feature_flags.get("enable_debate_layer", True):
            decision = run_debate(state.get("text", ""), state.get("context", {}), state.get("candidate_actions", []))
        else:
            decision = run_debate(state.get("text", ""), state.get("context", {}), state.get("candidate_actions", []))
        decision_dict = decision.to_dict()
        self.memory.save_decision(decision_dict)
        state["decision"] = decision_dict
        append_event(state, "main_supervisor", "debate_complete", {"final_action": decision.final_action, "risk": decision.risk_level})
        return state

    def _route_to_task_graph(self, state: AgentState) -> AgentState:
        intent = (state.get("parsed_intent") or {}).get("intent", "chat")
        final_action = (state.get("decision") or {}).get("final_action")
        if final_action in {"recover_localization", "recover_nav_failure"}:
            return self.recovery_graph.invoke(state)
        if intent in {"explore", "coverage_explore"} or final_action in {"frontier_explore", "coverage_explore"}:
            if not self.feature_flags.get("enable_explore_mode", False):
                append_event(state, "main_supervisor", "explore_disabled", {"enable_explore_mode": False})
                state["speech_response"] = "Explore mode is available but disabled by launch flag. Re-launch with enable_explore_mode:=true."
                return state
            explored = self.exploration_graph.invoke(state)
            # Run navigation gates after exploration produces the candidate command.
            return self.navigation_graph.invoke(explored)
        if intent in {"start_tour", "tour_question"} or final_action == "navigate_tour_route":
            if not self.feature_flags.get("enable_tour_mode", False) and intent == "start_tour":
                append_event(state, "main_supervisor", "tour_disabled", {"enable_tour_mode": False})
                state["speech_response"] = "Tour mode is available but disabled by launch flag. Re-launch with enable_tour_mode:=true."
                return state
            toured = self.tour_graph.invoke(state)
            if intent == "start_tour":
                return self.navigation_graph.invoke(toured)
            return toured
        if intent in {"navigate", "return_to_spawn", "stop", "continue_task"}:
            return self.navigation_graph.invoke(state)
        if intent in {"save_place", "remember", "query_memory", "explain_state"}:
            return state
        if not state.get("speech_response"):
            state["speech_response"] = "I heard you. I can save places, query memory, navigate, explore, give tours, stop, or return to spawn."
        return state

    def _finalize(self, state: AgentState) -> AgentState:
        command = state.get("nav_command") or {}
        speech = state.get("speech_response") or (state.get("decision") or {}).get("spoken_response") or "Done."
        if command.get("action") in {"none", None}:
            command = {}
        state["nav_command"] = command
        state["speech_response"] = speech
        state["status"] = {
            "run_id": state.get("run_id"),
            "intent": (state.get("parsed_intent") or {}).get("intent"),
            "decision": (state.get("decision") or {}).get("final_action"),
            "nav_action": command.get("action"),
            "context": summarize_context(state.get("context") or {}),
            "speech": speech,
        }
        append_event(state, "main_supervisor", "finalized", {"nav_action": command.get("action"), "speech_len": len(speech)})
        return state

    def _persist(self, state: AgentState) -> None:
        history_entry = {
            "run_id": state.get("run_id"),
            "text": state.get("text"),
            "parsed_intent": state.get("parsed_intent"),
            "decision": state.get("decision"),
            "nav_command": state.get("nav_command"),
            "speech_response": state.get("speech_response"),
            "events": state.get("events", []),
        }
        history = list(self.state.get("history", []))[-99:]
        history.append(history_entry)
        self.state = {
            "mode": self._next_mode(state),
            "history": history,
            "last_state": history_entry,
            "last_status": state.get("status", {}),
        }
        self.checkpointer.save(self.state)

    def _next_mode(self, state: AgentState) -> str:
        intent = (state.get("parsed_intent") or {}).get("intent")
        if intent in {"explore", "coverage_explore"}:
            return "explore"
        if intent in {"start_tour", "tour_question"}:
            return "tour"
        if intent == "stop":
            return "idle"
        return self.state.get("mode", "idle")
