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


def test_omi_ble_opus_packet_keeps_header_for_omi_decoder():
    class FakeDecoder:
        def __init__(self):
            self.seen = []

        def decode_packet(self, packet):
            self.seen.append(packet)
            return b'pcm'

    src = OmiBleAudioSource(lambda cmd: None, stt_backend='disabled')
    src._codec = 20
    src._opus_decoder = FakeDecoder()
    packet = b'\x01\x00\x00opus'

    src._on_ble_audio(None, bytearray(packet))

    assert src._opus_decoder.seen == [packet]
    assert src.audio_q.get_nowait() == b'pcm'
