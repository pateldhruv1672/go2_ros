from go2_memory_core.vlm_client import VLMClient


def test_offline_vlm_client_is_deterministic():
    result = VLMClient(provider='offline').summarize_image(None)
    assert result.success is True
    assert 'offline_vlm' in result.summary


def test_ollama_vlm_extracts_chat_message_content():
    assert VLMClient._extract_summary('ollama', {'message': {'content': 'robot sees a door'}}) == 'robot sees a door'
