from go2_memory_core.vlm_client import VLMClient


def test_offline_vlm_client_is_deterministic():
    result = VLMClient(provider='offline').summarize_image(None)
    assert result.success is True
    assert 'offline_vlm' in result.summary


def test_ollama_vlm_extracts_chat_message_content():
    assert VLMClient._extract_summary('ollama', {'message': {'content': 'robot sees a door'}}) == 'robot sees a door'


def test_ollama_vlm_disables_thinking_for_tts_safe_content():
    captured = {}

    class Client(VLMClient):
        def _post_json(self, url, payload, headers, provider):
            captured.update(payload)
            return super()._post_json(url, payload, headers, provider)

    client = Client(provider='ollama', model='fake')
    client._post_json = lambda url, payload, headers, provider: captured.update(payload) or None
    client._ollama(b'image', 'image/jpeg', 'prompt')

    assert captured['think'] is False
