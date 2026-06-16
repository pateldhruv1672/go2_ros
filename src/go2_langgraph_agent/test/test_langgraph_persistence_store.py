from pathlib import Path

from go2_langgraph_agent.persistence import SQLiteAgentStore


def test_sqlite_agent_store_put_get_search(tmp_path: Path):
    store = SQLiteAgentStore(tmp_path / "store.sqlite")
    store.put(("threads", "test", "facts"), "frontier", {"text": "frontier selected", "score": 0.9})
    assert store.get(("threads", "test", "facts"), "frontier")["score"] == 0.9
    rows = store.search(("threads", "test"), query="frontier", limit=5)
    assert rows
    assert rows[0]["key"] == "frontier"
    assert "threads/test/facts" in store.list_namespaces()
    store.close()
