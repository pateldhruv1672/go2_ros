from go2_memory_core.vlm_client import VLMClient


def test_offline_vlm_client_is_deterministic():
    result = VLMClient(provider='offline').summarize_image(None)
    assert result.success is True
    assert 'offline_vlm' in result.summary
