from pathlib import Path

from go2_langgraph_agent.debate import run_debate
from go2_langgraph_agent.llm_debate import LLMVoteClient
from go2_langgraph_agent.persistence import NativeLangGraphStoreAdapter


class FakeLLMVoteClient(LLMVoteClient):
    def __init__(self):
        super().__init__(provider="openrouter", model="fake")

    def available(self):
        return True

    def _http_json(self, url, body, headers, timeout):
        # Every role approves navigate_to_pose except safety prefers ask_human.
        role = body["messages"][1]["content"].split("ROLE: ")[1].split("\n", 1)[0]
        vote = "ask_human" if role == "SafetyAgent" else "approve"
        return {
            "choices": [{"message": {"content": '{"vote":"%s","confidence":0.82,"risk_level":"medium","final_action":"navigate_to_pose","requires_human_interrupt":%s,"rationale":"fake role vote","fallback_plan":["stop_robot"]}' % (vote, "true" if vote == "ask_human" else "false")}}]
        }


def test_llm_debate_runs_and_applies_hard_safety():
    decision = run_debate(
        "go to the lab",
        {"localization_confidence": 0.8, "safety_blocked": False},
        ["navigate_to_pose", "stop_robot"],
        enable_llm=True,
        llm_client=FakeLLMVoteClient(),
    )
    assert decision.debate_mode == "llm_multi_agent"
    assert len(decision.council_votes) == 7
    assert decision.final_action == "navigate_to_pose"
    assert decision.requires_human_interrupt is True

    blocked = run_debate(
        "go to the lab",
        {"localization_confidence": 0.8, "safety_blocked": True},
        ["navigate_to_pose", "stop_robot"],
        enable_llm=True,
        llm_client=FakeLLMVoteClient(),
    )
    assert blocked.final_action == "stop_robot"
    assert blocked.safety_overrides


def test_store_adapter_mirror_fallback(tmp_path: Path):
    store = NativeLangGraphStoreAdapter(tmp_path / "store.sqlite", require_native=False)
    store.put(("threads", "demo", "facts"), "k1", {"fact": "robot has memory"})
    assert store.get(("threads", "demo", "facts"), "k1")["fact"] == "robot has memory"
    results = store.search(("threads", "demo"), query="robot")
    assert results
    store.close()
