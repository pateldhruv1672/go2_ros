from go2_langgraph_agent.voice_io import OMI_AUDIO_CHAR_UUID, LocalSTTEngine, OmiBleAudioSource


def test_omi_ble_uses_standard_audio_uuid():
    assert OMI_AUDIO_CHAR_UUID.lower().startswith('19b10001')


def test_local_stt_reports_unavailable_without_optional_model():
    stt = LocalSTTEngine(backend='disabled')
    cmd = stt.transcribe_pcm16(b'\x00' * 1600)
    assert cmd.source == 'local_stt_error'


def test_omi_ble_packet_strips_three_byte_header():
    seen = []
    src = OmiBleAudioSource(lambda cmd: seen.append(cmd), stt_backend='disabled')
    src._codec = 0
    src._on_ble_audio(None, bytearray(b'\x01\x00\x00abcd'))
    assert src.audio_q.get_nowait() == b'abcd'
