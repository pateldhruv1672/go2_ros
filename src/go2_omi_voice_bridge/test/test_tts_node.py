from go2_omi_voice_bridge.tts_node import _plain_spoken_text


def test_plain_spoken_text_strips_markdown_and_limits_sentences():
    text = """
    **Navigation Checkpoint Details:**
    * **Obstacles:** chairs and boxes.
    * **Cue:** exit sign visible.
    Third sentence should not be spoken.
    """

    clean = _plain_spoken_text(text, max_sentences=2)

    assert "*" not in clean
    assert "Navigation Checkpoint Details" in clean
    assert "Obstacles: chairs and boxes." in clean
    assert "Cue: exit sign visible." in clean
    assert "Third sentence" not in clean
