from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph


GRAPH_NAME = "recovery_graph"


class RecoveryGraph:
    """Safe recovery and relocalization subgraph."""

    def __init__(self):
        self.graph = try_build_langgraph(
            GRAPH_NAME,
            ["diagnose", "build_recovery_plan", "make_recovery_command"],
            {
                "diagnose": self.diagnose,
                "build_recovery_plan": self.build_recovery_plan,
                "make_recovery_command": self.make_recovery_command,
            },
        )

    def invoke(self, state: AgentState) -> AgentState:
        return self.graph.invoke(state)

    def diagnose(self, state: AgentState) -> Dict[str, Any]:
        context = state.get("context") or {}
        localization = float(context.get("localization_confidence", 0.0) or 0.0)
        nav_status = context.get("nav_status") or state.get("nav_status") or {}
        if context.get("safety_blocked"):
            issue = "safety_blocked"
        elif localization < 0.35:
            issue = "low_localization"
        elif nav_status.get("last_result") == "failed":
            issue = "nav_failure"
        else:
            issue = "unknown_or_user_requested"
        append_event(state, GRAPH_NAME, "diagnosed", {"issue": issue, "localization_confidence": localization})
        return {"recovery_issue": issue}

    def build_recovery_plan(self, state: AgentState) -> Dict[str, Any]:
        issue = state.get("recovery_issue")
        if issue == "safety_blocked":
            plan: List[str] = ["stop_robot", "wait_for_clearance", "ask_human_before_motion"]
        elif issue == "low_localization":
            plan = ["stop_robot", "publish_initial_pose_from_spawn", "compare_scan_to_map", "ask_human_if_confidence_low"]
        elif issue == "nav_failure":
            plan = ["stop_robot", "clear_costmaps", "try_safe_anchor", "reroute_or_return_to_spawn"]
        else:
            plan = ["stop_robot", "report_state", "ask_for_next_command"]
        append_event(state, GRAPH_NAME, "plan_built", {"steps": plan})
        return {"recovery_plan": plan}

    def make_recovery_command(self, state: AgentState) -> Dict[str, Any]:
        issue = state.get("recovery_issue")
        action = "stop_robot" if issue == "safety_blocked" else "recover_localization" if issue == "low_localization" else "recover_nav_failure"
        command = {"action": action, "issue": issue, "plan": state.get("recovery_plan", [])}
        speech = "I am stopping and running a safe recovery plan."
        if issue == "low_localization":
            speech = "Localization confidence is low. I will stop and relocalize from the saved spawn/checkpoints before moving."
        append_event(state, GRAPH_NAME, "recovery_command_built", command)
        return {"nav_command": command, "speech_response": speech}


def build_graph() -> RecoveryGraph:
    return RecoveryGraph()
