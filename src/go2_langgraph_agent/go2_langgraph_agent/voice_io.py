from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional
import asyncio
import os
import queue
import subprocess
import tempfile
import threading
import time
import wave


OMI_AUDIO_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
OMI_CODEC_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"


@dataclass
class VoiceCommand:
    text: str
    source: str
    confidence: float = 0.0
    raw: Dict[str, Any] | None = None


class LocalSTTEngine:
    """Local lightweight STT adapter.

    Preferred order:
      1. faster-whisper tiny/base int8
      2. openai-whisper tiny/base
      3. vosk small model
      4. speech_recognition local Whisper method when used by LaptopMicSTTSource
    """

    def __init__(self, backend: str = "faster_whisper", model_name: str = "tiny.en", sample_rate: int = 16000, language: str = "en"):
        self.backend = backend.lower()
        self.model_name = model_name
        self.sample_rate = sample_rate
        self.language = language
        self._model = None
        self._load_error = ""
        self._load()

    def _load(self) -> None:
        if self.backend in {"off", "disabled"}:
            return
        try:
            if self.backend in {"faster_whisper", "whisper_cpp_like"}:
                from faster_whisper import WhisperModel  # type: ignore
                self._model = WhisperModel(self.model_name, device="cpu", compute_type="int8")
            elif self.backend in {"whisper", "openai_whisper"}:
                import whisper  # type: ignore
                self._model = whisper.load_model(self.model_name)
            elif self.backend == "vosk":
                from vosk import Model  # type: ignore
                self._model = Model(self.model_name)
        except Exception as exc:
            self._load_error = str(exc)
            self._model = None

    @property
    def available(self) -> bool:
        return self._model is not None

    @property
    def load_error(self) -> str:
        return self._load_error

    def transcribe_pcm16(self, pcm: bytes) -> VoiceCommand:
        if not pcm:
            return VoiceCommand("", "local_stt", 0.0, {"error": "empty audio"})
        if self.backend == "vosk" and self._model is not None:
            return self._transcribe_vosk(pcm)
        wav_path = self._write_temp_wav(pcm)
        try:
            return self.transcribe_file(wav_path)
        finally:
            try:
                wav_path.unlink(missing_ok=True)
            except Exception:
                pass

    def transcribe_file(self, wav_path: Path) -> VoiceCommand:
        if self._model is None:
            return VoiceCommand("", "local_stt_error", 0.0, {"backend": self.backend, "error": self._load_error or "model not loaded"})
        try:
            if self.backend in {"faster_whisper", "whisper_cpp_like"}:
                segments, info = self._model.transcribe(str(wav_path), language=self.language, vad_filter=True)
                text = " ".join(seg.text.strip() for seg in segments if getattr(seg, "text", "").strip()).strip()
                conf = max(0.0, min(1.0, 1.0 - float(getattr(info, "language_probability", 0.0) or 0.0) * 0.0 + 0.75))
                return VoiceCommand(text, "local_stt", conf, {"backend": self.backend, "model": self.model_name})
            if self.backend in {"whisper", "openai_whisper"}:
                result = self._model.transcribe(str(wav_path), language=self.language, fp16=False)
                text = str(result.get("text", "")).strip()
                return VoiceCommand(text, "local_stt", 0.75, {"backend": self.backend, "model": self.model_name})
        except Exception as exc:
            return VoiceCommand("", "local_stt_error", 0.0, {"backend": self.backend, "error": str(exc)})
        return VoiceCommand("", "local_stt_error", 0.0, {"backend": self.backend, "error": "unsupported backend"})

    def _transcribe_vosk(self, pcm: bytes) -> VoiceCommand:
        try:
            from vosk import KaldiRecognizer  # type: ignore
            import json
            rec = KaldiRecognizer(self._model, self.sample_rate)
            rec.AcceptWaveform(pcm)
            payload = json.loads(rec.FinalResult())
            return VoiceCommand(str(payload.get("text", "")).strip(), "local_stt", 0.65, {"backend": "vosk", "model": self.model_name})
        except Exception as exc:
            return VoiceCommand("", "local_stt_error", 0.0, {"backend": "vosk", "error": str(exc)})

    def _write_temp_wav(self, pcm: bytes) -> Path:
        fd, name = tempfile.mkstemp(prefix="go2_stt_", suffix=".wav")
        os.close(fd)
        path = Path(name)
        with wave.open(str(path), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(pcm)
        return path


class LaptopMicSTTSource:
    """Local microphone STT loop; no cloud/webspeech by default."""

    def __init__(self, callback: Callable[[VoiceCommand], None], backend: str = "faster_whisper", model_name: str = "tiny.en", phrase_time_limit: float = 6.0, energy_threshold: int = 300):
        self.callback = callback
        self.backend = backend
        self.model_name = model_name
        self.phrase_time_limit = phrase_time_limit
        self.energy_threshold = energy_threshold
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            import speech_recognition as sr  # type: ignore
        except Exception as exc:
            self.callback(VoiceCommand(text="", source="laptop_mic_error", confidence=0.0, raw={"error": f"speech_recognition unavailable: {exc}"}))
            return
        recognizer = sr.Recognizer()
        recognizer.energy_threshold = self.energy_threshold
        try:
            mic = sr.Microphone(sample_rate=16000)
        except Exception as exc:
            self.callback(VoiceCommand(text="", source="laptop_mic_error", confidence=0.0, raw={"error": f"microphone unavailable: {exc}"}))
            return
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=0.8)
        stt = LocalSTTEngine(self.backend, self.model_name, sample_rate=16000)
        if not stt.available and self.backend not in {"speech_recognition_whisper", "whisper_sr"}:
            self.callback(VoiceCommand(text="", source="laptop_mic_error", confidence=0.0, raw={"error": stt.load_error, "backend": self.backend}))
        while not self._stop.is_set():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=1.0, phrase_time_limit=self.phrase_time_limit)
                if self.backend in {"speech_recognition_whisper", "whisper_sr"} and hasattr(recognizer, "recognize_whisper"):
                    text = str(recognizer.recognize_whisper(audio, model=self.model_name)).strip()
                    command = VoiceCommand(text=text, source="laptop_mic", confidence=0.7, raw={"backend": self.backend})
                else:
                    command = stt.transcribe_pcm16(audio.get_raw_data(convert_rate=16000, convert_width=2))
                    command.source = "laptop_mic"
                if command.text:
                    self.callback(command)
            except sr.WaitTimeoutError:
                continue
            except Exception as exc:
                self.callback(VoiceCommand(text="", source="laptop_mic_error", confidence=0.0, raw={"error": str(exc)}))
                time.sleep(1.0)


class OmiBleAudioSource:
    """Omi BLE audio source with local STT on the laptop.

    It does not use Omi webhooks or cloud transcription. It connects over BLE to
    the standard Omi audio characteristic, decodes PCM/Opus when optional libs are
    available, chunks audio locally, and runs LocalSTTEngine on the laptop.
    """

    def __init__(
        self,
        callback: Callable[[VoiceCommand], None],
        device_name: str = "Omi",
        device_address: str = "",
        audio_char_uuid: str = OMI_AUDIO_CHAR_UUID,
        codec_char_uuid: str = OMI_CODEC_CHAR_UUID,
        stt_backend: str = "faster_whisper",
        stt_model: str = "tiny.en",
        chunk_sec: float = 4.0,
        sample_rate: int = 16000,
    ):
        self.callback = callback
        self.device_name = device_name
        self.device_address = device_address
        self.audio_char_uuid = audio_char_uuid
        self.codec_char_uuid = codec_char_uuid
        self.chunk_sec = chunk_sec
        self.sample_rate = sample_rate
        self.stt = LocalSTTEngine(stt_backend, stt_model, sample_rate=sample_rate)
        self.audio_q: queue.Queue[bytes] = queue.Queue(maxsize=100)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._consumer_thread: Optional[threading.Thread] = None
        self._opus_decoder = None
        self._codec = 20

    def start(self) -> None:
        self._consumer_thread = threading.Thread(target=self._consume_audio, daemon=True)
        self._consumer_thread.start()
        self._thread = threading.Thread(target=lambda: asyncio.run(self._run_ble()), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        for th in (self._thread, self._consumer_thread):
            if th:
                th.join(timeout=2.0)

    async def _run_ble(self) -> None:
        # Prefer the official Omi SDK BLE listener/Opus decoder when installed.
        try:
            import omi  # type: ignore
            if hasattr(omi, "OmiOpusDecoder"):
                self._opus_decoder = omi.OmiOpusDecoder()
        except Exception:
            pass
        try:
            from bleak import BleakClient, BleakScanner  # type: ignore
        except Exception as exc:
            self.callback(VoiceCommand("", "omi_ble_error", 0.0, {"error": f"bleak unavailable: {exc}"}))
            return
        while not self._stop.is_set():
            try:
                device = await self._resolve_device(BleakScanner)
                if not device:
                    self.callback(VoiceCommand("", "omi_ble_error", 0.0, {"error": "Omi BLE device not found", "address": self.device_address, "name": self.device_name}))
                    await asyncio.sleep(3.0)
                    continue
                async with BleakClient(device) as client:
                    try:
                        codec_raw = await client.read_gatt_char(self.codec_char_uuid)
                        self._codec = int(codec_raw[0]) if codec_raw else self._codec
                    except Exception:
                        pass
                    await client.start_notify(self.audio_char_uuid, self._on_ble_audio)
                    connected_address = str(getattr(client, "address", "") or self.device_address or device)
                    self.callback(VoiceCommand("", "omi_ble_status", 1.0, {"connected": True, "address": connected_address, "codec": self._codec}))
                    while not self._stop.is_set() and client.is_connected:
                        await asyncio.sleep(0.25)
                    await client.stop_notify(self.audio_char_uuid)
            except Exception as exc:
                self.callback(VoiceCommand("", "omi_ble_error", 0.0, {"error": str(exc)}))
                await asyncio.sleep(2.0)

    async def _resolve_device(self, scanner_cls: Any) -> Any:
        target_address = (self.device_address or "").strip().lower()
        if target_address:
            finder = getattr(scanner_cls, "find_device_by_address", None)
            if finder is not None:
                try:
                    device = await finder(self.device_address, timeout=5.0)
                    if device is not None:
                        return device
                except Exception:
                    pass
            devices = await scanner_cls.discover(timeout=5.0)
            for dev in devices:
                address = str(getattr(dev, "address", "") or "").lower()
                if address == target_address:
                    return dev
            try:
                from bleak.backends.device import BLEDevice  # type: ignore

                path = "/org/bluez/hci0/dev_" + self.device_address.replace(":", "_")
                return BLEDevice(
                    self.device_address,
                    self.device_name,
                    {
                        "path": path,
                        "props": {"Address": self.device_address, "Name": self.device_name},
                    },
                )
            except Exception:
                return self.device_address
        return await self._find_device(scanner_cls)

    async def _find_device(self, scanner_cls: Any) -> str:
        devices = await scanner_cls.discover(timeout=5.0)
        for dev in devices:
            name = getattr(dev, "name", "") or ""
            if self.device_name.lower() in name.lower():
                return dev
        return ""

    def _on_ble_audio(self, sender: Any, data: bytearray) -> None:
        payload = bytes(data)
        if len(payload) <= 3:
            return
        pcm = self._decode_audio(payload)
        if pcm:
            try:
                self.audio_q.put_nowait(pcm)
            except queue.Full:
                pass

    def _decode_audio(self, packet: bytes) -> bytes:
        if self._codec in {0, 1}:  # PCM 16-bit mono, 16 kHz or 8 kHz
            return packet[3:]  # 3-byte Omi packet header: packet number + fragment index
        if self._codec == 20 and self._opus_decoder is not None:
            try:
                decoded = self._opus_decoder.decode_packet(packet)
                return bytes(decoded or b"")
            except Exception:
                return b""
        # Without Opus decoder we cannot safely interpret codec 20.
        return b""

    def _consume_audio(self) -> None:
        target_bytes = int(self.sample_rate * 2 * max(1.0, self.chunk_sec))
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.audio_q.get(timeout=0.5)
                buf.extend(chunk)
                if len(buf) >= target_bytes:
                    command = self.stt.transcribe_pcm16(bytes(buf))
                    command.source = "omi_ble_local_stt"
                    if command.text:
                        self.callback(command)
                    buf.clear()
            except queue.Empty:
                continue


class LocalTTS:
    """Local TTS adapter: piper first, then pyttsx3/espeak fallback."""

    def __init__(self, backend: str = "piper", model_path: str = "", speaker_device: str = ""):
        self.backend = backend.lower()
        self.model_path = model_path
        self.speaker_device = speaker_device

    def speak(self, text: str) -> Dict[str, Any]:
        text = (text or "").strip()
        if not text:
            return {"success": False, "error": "empty_text"}
        if self.backend == "piper":
            ok = self._speak_piper(text)
            if ok:
                return {"success": True, "backend": "piper"}
        if self.backend in {"pyttsx3", "piper"}:
            ok = self._speak_pyttsx3(text)
            if ok:
                return {"success": True, "backend": "pyttsx3"}
        ok = self._speak_espeak(text)
        return {"success": ok, "backend": "espeak" if ok else self.backend, "error": None if ok else "no local TTS backend available"}

    def _speak_piper(self, text: str) -> bool:
        if not self.model_path:
            return False
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
            cmd = ["piper", "--model", self.model_path, "--output_file", wav_path]
            proc = subprocess.run(cmd, input=text.encode("utf-8"), stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            if proc.returncode != 0:
                return False
            subprocess.run(["aplay", wav_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            return True
        except Exception:
            return False
        finally:
            try:
                Path(wav_path).unlink(missing_ok=True)  # type: ignore[name-defined]
            except Exception:
                pass

    def _speak_pyttsx3(self, text: str) -> bool:
        try:
            import pyttsx3  # type: ignore
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
            return True
        except Exception:
            return False

    def _speak_espeak(self, text: str) -> bool:
        try:
            subprocess.run(["espeak", text], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            return True
        except Exception:
            return False
