from pathlib import Path

import pytest

from go2_langgraph_agent.graphs.agent_state import classify_intent, extract_text_from_message


langgraph = pytest.importorskip("langgraph")
pytest.importorskip("langgraph.checkpoint.sqlite")

from go2_langgraph_agent.graphs.agent_orchestrator import AgentOrchestrator
from go2_langgraph_agent.persistence import LangGraphSQLitePersistence
from go2_langgraph_agent.tools.memory_tools import MemoryTools


def make_orchestrator(tmp_path, session="test_session", explore=True, tour=True, live_context=None):
    memory = MemoryTools(str(tmp_path), session)
    persistence = LangGraphSQLitePersistence(str(tmp_path), session)
    return AgentOrchestrator(
        memory,
        persistence,
        {
            "enable_debate_layer": True,
            "enable_explore_mode": explore,
            "enable_tour_mode": tour,
            "enable_human_interrupts": False,
        },
        live_context_provider=(lambda: live_context or {}),
        thread_id=session,
    )


def test_json_voice_command_text_is_extracted():
    assert extract_text_from_message('{"text":"Start tour","source":"omi"}') == "Start tour"


def test_intent_classification_save_and_nav():
    assert classify_intent("save this as ai lab entrance")["intent"] == "save_place"
    parsed = classify_intent("go to robotics lab")
    assert parsed["intent"] == "navigate"
    assert parsed["entities"]["destination"] == "robotics lab"


def test_orchestrator_saves_place_and_persists(tmp_path):
    orch = make_orchestrator(tmp_path)
    state = orch.invoke("save this as AI lab entrance")
    assert state["parsed_intent"]["intent"] == "save_place"
    assert "Saved this location as" in state["speech_response"]
    places_path = Path(tmp_path) / "test_session" / "memory" / "places.jsonl"
    assert places_path.exists()
    assert "ai lab entrance" in places_path.read_text(encoding="utf-8")
    assert (Path(tmp_path) / "test_session" / "langgraph" / "checkpoints.sqlite").exists()


def test_orchestrator_routes_explore_through_subgraphs(tmp_path):
    live_context = {
        "frontier_candidates": {"candidates": [{"id": "f1", "pose": {"x": 2.0, "y": 0.0}, "information_gain": 0.9}]},
        "scan_summary": {"sector_clearance_m": {"front": 2.0}},
        "traversability": {"traversability_score": 0.8},
        "odom": {"pose": {"x": 0.0, "y": 0.0}},
    }
    orch = make_orchestrator(tmp_path, explore=True, live_context=live_context)
    state = orch.invoke("start explore mode")
    assert state["parsed_intent"]["intent"] == "explore"
    assert state["nav_command"]["action"] == "frontier_explore"
    assert state["nav_command"]["selected_goal"]["goal_id"] == "f1"
    graphs = [e["graph"] for e in state.get("events", [])]
    assert "exploration_graph" in graphs
    assert "navigation_graph" in graphs


def test_orchestrator_tour_speaks_from_memory(tmp_path):
    orch = make_orchestrator(tmp_path, tour=True)
    orch.invoke("save this as lobby")
    state = orch.invoke("start tour")
    assert state["parsed_intent"]["intent"] == "start_tour"
    assert "lobby" in state["speech_response"]
    assert state["nav_command"].get("action") in {"navigate_tour_route", None, "stop_robot"}
