from __future__ import annotations

from typing import Any, Dict, List

from go2_langgraph_agent.graphs.agent_state import AgentState, append_event, try_build_langgraph
from go2_langgraph_agent.tools.goal_selection import select_exploration_goal


GRAPH_NAME = "exploration_graph"


class ExplorationGraph:
    """Multimodal exploration mission subgraph.

    This is not a placeholder: it consumes the latest frontier candidates,
    coverage candidates, LiDAR/traversability summaries, odom, VLM/live camera
    summaries, and dynamic obstacle tracks from the supervisor context. It then
    emits a concrete Nav2 tool command with a selected goal and a memory write
    policy for checkpoint growth.
    """

    def __init__(self):
        self.graph = try_build_langgraph(
            GRAPH_NAME,
            ["select_strategy", "score_multimodal_goals", "make_explore_plan", "request_memory_growth", "make_explore_command"],
            {
                "select_strategy": self.select_strategy,
                "score_multimodal_goals": self.score_multimodal_goals,
                "make_explore_plan": self.make_explore_plan,
                "request_memory_growth": self.request_memory_growth,
                "make_explore_command": self.make_explore_command,
            },
        )

    def invoke(self, state: AgentState) -> AgentState:
        return self.graph.invoke(state)

    def select_strategy(self, state: AgentState) -> Dict[str, Any]:
        text = (state.get("text") or "").lower()
        context = state.get("context") or {}
        frontier_count = len((context.get("frontier_candidates") or {}).get("candidates", [])) if isinstance(context.get("frontier_candidates"), dict) else 0
        if "coverage" in text or "scan" in text or "cover" in text:
            strategy = "coverage"
        elif "manual" in text:
            strategy = "manual_assisted"
        elif frontier_count > 0:
            strategy = "frontier"
        else:
            strategy = "coverage" if context.get("coverage_waypoints") else "frontier"
        append_event(state, GRAPH_NAME, "strategy_selected", {"strategy": strategy, "frontier_count": frontier_count})
        return {"explore_strategy": strategy}

    def score_multimodal_goals(self, state: AgentState) -> Dict[str, Any]:
        context = state.get("context") or {}
        strategy = state.get("explore_strategy", "frontier")
        if strategy == "manual_assisted":
            result = {"strategy": strategy, "candidate_count": 0, "best_goal": None, "scored_candidates": []}
        else:
            result = select_exploration_goal(context, strategy=strategy, goal_text=state.get("text", ""))
        append_event(state, GRAPH_NAME, "goals_scored", {"strategy": strategy, "candidate_count": result.get("candidate_count"), "has_best": bool(result.get("best_goal"))})
        return {"goal_selection": result}

    def make_explore_plan(self, state: AgentState) -> Dict[str, Any]:
        context = state.get("context") or {}
        strategy = state.get("explore_strategy", "frontier")
        safe_anchor = ((context.get("spawn") or {}).get("data") or {}).get("map_pose") or context.get("last_safe_pose") or {}
        best_goal = (state.get("goal_selection") or {}).get("best_goal")
        plan: List[Dict[str, Any]] = [
            {"step": "snapshot_pose_camera_lidar_odom"},
            {"step": "score_multimodal_candidates", "uses": ["frontier_map", "camera_vlm", "lidar", "pointcloud", "odom", "dynamic_obstacles"]},
            {"step": "navigate_selected_goal" if best_goal else "request_more_context", "safe_anchor": safe_anchor, "goal": best_goal},
            {"step": "write_checkpoint_graph_voxel_vector_memory"},
            {"step": "replan_or_return_to_spawn_on_failure"},
        ]
        append_event(state, GRAPH_NAME, "plan_created", {"steps": len(plan), "has_goal": bool(best_goal)})
        return {"explore_plan": plan, "selected_goal": best_goal}

    def request_memory_growth(self, state: AgentState) -> Dict[str, Any]:
        selected = state.get("selected_goal") or {}
        request = {
            "write_checkpoint": True,
            "update_graph": True,
            "update_voxels": True,
            "store_artifacts": True,
            "link_selected_goal": selected.get("goal_id"),
            "layer": "temporary",
            "promotion_policy": "ask_user",
            "source_modalities": ["camera", "vlm", "lidar", "pointcloud", "odom", "frontier", "dynamic_obstacle_tracker"],
        }
        append_event(state, GRAPH_NAME, "memory_growth_requested", request)
        return {"memory_request": request}

    def make_explore_command(self, state: AgentState) -> Dict[str, Any]:
        strategy = state.get("explore_strategy", "frontier")
        selected = state.get("selected_goal")
        action = "coverage_explore" if strategy == "coverage" else "frontier_explore"
        command = {
            "action": action,
            "strategy": strategy,
            "selected_goal": selected,
            "pose": (selected or {}).get("pose", {}),
            "plan": state.get("explore_plan", []),
            "memory_request": state.get("memory_request", {}),
        }
        if not selected and strategy != "manual_assisted":
            command["action"] = "stop_robot"
            command["reason"] = "no_safe_multimodal_goal_available"
            speech = "I do not have a safe exploration goal yet. I am stopping and waiting for a better map, LiDAR, or camera context."
        elif strategy == "manual_assisted":
            command["action"] = "manual_assisted_explore"
            speech = "Manual-assisted explore is active. I will keep writing memory while you guide the robot."
        else:
            speech = f"I selected a {strategy} goal using camera, LiDAR, odom, traversability, and dynamic obstacle context."
        append_event(state, GRAPH_NAME, "explore_command_built", {"action": command["action"], "selected_goal": (selected or {}).get("goal_id")})
        return {"nav_command": command, "speech_response": speech}


def build_graph() -> ExplorationGraph:
    return ExplorationGraph()
