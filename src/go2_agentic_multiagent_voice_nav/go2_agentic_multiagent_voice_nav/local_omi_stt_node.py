from __future__ import annotations

import asyncio
import io
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import wave

from bleak import BleakClient, BleakScanner
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from omi import OmiOpusDecoder
except Exception as exc:
    raise RuntimeError(
        "Failed to import OmiOpusDecoder from the Omi SDK. "
        "Install the real Omi SDK, not the unrelated 'omi' package."
    ) from exc


OMI_MAC = "EF:1C:34:C6:25:92"
OMI_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
SAMPLE_RATE = 16000


class LocalOmiSttNode(Node):
    def __init__(self) -> None:
        super().__init__("local_omi_stt_node")

        self.declare_parameter("omi_mac", OMI_MAC)
        self.declare_parameter("omi_char_uuid", OMI_CHAR_UUID)
        self.declare_parameter("input_backend", "auto")
        self.declare_parameter("sample_rate", SAMPLE_RATE)
        self.declare_parameter("chunk_seconds", 1.2)
        self.declare_parameter("whisper_model", "small.en")
        self.declare_parameter("compute_type", "int8")
        self.declare_parameter("language", "en")
        self.declare_parameter("vad_filter", True)
        self.declare_parameter("connection_retry_sec", 5.0)
        self.declare_parameter("scan_timeout_sec", 8.0)
        self.declare_parameter("speech_input_status_topic", "/agent/speech_input_status")

        self.transcript_pub = self.create_publisher(String, "/agent/transcript", 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("speech_input_status_topic").value), 10)

        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.chunk_seconds = float(self.get_parameter("chunk_seconds").value)
        self.chunk_size_bytes = int(self.sample_rate * 2 * self.chunk_seconds)

        self.omi_mac = str(self.get_parameter("omi_mac").value).upper()
        self.omi_char_uuid = str(self.get_parameter("omi_char_uuid").value)
        self.language = str(self.get_parameter("language").value)
        self.vad_filter = bool(self.get_parameter("vad_filter").value)
        self.connection_retry_sec = float(self.get_parameter("connection_retry_sec").value)
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)

        model_name = str(self.get_parameter("whisper_model").value)
        compute_type = str(self.get_parameter("compute_type").value)

        self.decoder = OmiOpusDecoder()
        self.audio_buffer = io.BytesIO()
        self.audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=32)
        self.transcript_queue: queue.Queue[str] = queue.Queue(maxsize=32)
        self.model = WhisperModel(model_name, compute_type=compute_type)
        self.stop_event = threading.Event()
        self.backend = str(self.get_parameter("input_backend").value).strip().lower() or "auto"
        self.omi_connected = False
        self.arecord_path = shutil.which("arecord")

        if self.backend in {"auto", "omi"}:
            threading.Thread(target=self._run_omi_listener, daemon=True).start()
        if self.backend in {"auto", "mic"}:
            threading.Thread(target=self._run_mic_listener, daemon=True).start()
        threading.Thread(target=self._run_transcriber, daemon=True).start()
        self.create_timer(0.1, self._publish_transcripts)

        self.get_logger().info(
            f"ready | backend={self.backend} | omi_mac={self.omi_mac} | chunk_seconds={self.chunk_seconds} | "
            f"whisper_model={model_name} | compute_type={compute_type} | arecord={self.arecord_path or 'missing'}"
        )
        self._publish_input_status(f"backend={self.backend} waiting")

    def _run_omi_listener(self) -> None:
        asyncio.run(self._omi_main())

    def _publish_input_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"speech_input_status: {text}")

    async def _find_omi_device(self):
        try:
            return await BleakScanner.find_device_by_address(self.omi_mac, timeout=self.scan_timeout_sec)
        except Exception as exc:
            self.get_logger().warn(f"Bluetooth discovery failed: {exc}")
            return None

    async def _omi_main(self) -> None:
        while not self.stop_event.is_set():
            try:
                device = await self._find_omi_device()
                if device is None:
                    self.omi_connected = False
                    self._publish_input_status("backend=omi unavailable; mic fallback active")
                    self.get_logger().warn(
                        f"Omi {self.omi_mac} not discoverable yet. Retrying in {self.connection_retry_sec:.1f} seconds..."
                    )
                    await asyncio.sleep(self.connection_retry_sec)
                    continue

                loop = asyncio.get_running_loop()
                disconnected_event = asyncio.Event()

                def _on_disconnect(_client):
                    loop.call_soon_threadsafe(disconnected_event.set)

                def handle_audio(_sender, data: bytearray):
                    pcm = self.decoder.decode_packet(bytes(data))
                    if not pcm:
                        return
                    self.audio_buffer.write(pcm)
                    if self.audio_buffer.tell() >= self.chunk_size_bytes:
                        chunk = self.audio_buffer.getvalue()
                        self.audio_buffer = io.BytesIO()
                        try:
                            self.audio_queue.put_nowait(chunk)
                        except queue.Full:
                            try:
                                _ = self.audio_queue.get_nowait()
                            except queue.Empty:
                                pass
                            try:
                                self.audio_queue.put_nowait(chunk)
                            except queue.Full:
                                pass

                self.get_logger().info(f"Connecting to Omi {self.omi_mac} ...")
                async with BleakClient(device, disconnected_callback=_on_disconnect, timeout=20.0) as client:
                    self.get_logger().info("Connected to Omi")
                    self.omi_connected = True
                    self._publish_input_status("backend=omi connected")
                    await client.start_notify(self.omi_char_uuid, handle_audio)
                    self.get_logger().info(f"Subscribed to Omi audio characteristic {self.omi_char_uuid}")
                    await disconnected_event.wait()
                    self.omi_connected = False
                    self._publish_input_status("backend=omi disconnected; mic fallback active")

            except Exception as exc:
                self.omi_connected = False
                self._publish_input_status("backend=omi error; mic fallback active")
                self.get_logger().warn(
                    f"Omi connection failed: {exc}. Retrying in {self.connection_retry_sec:.1f} seconds..."
                )
                await asyncio.sleep(self.connection_retry_sec)

    def _run_mic_listener(self) -> None:
        if not self.arecord_path:
            self._publish_input_status("backend=mic unavailable_arecord_missing")
            return
        while not self.stop_event.is_set():
            if self.backend == "auto" and self.omi_connected:
                time.sleep(0.25)
                continue
            if self.backend == "omi" and self.omi_connected:
                time.sleep(0.25)
                continue
            if self.backend == "auto":
                self._publish_input_status("backend=mic fallback recording")
            wav_path = ""
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                    wav_path = tmp.name
                cmd = [
                    self.arecord_path,
                    "-q",
                    "-f", "S16_LE",
                    "-c", "1",
                    "-r", str(self.sample_rate),
                    "-d", str(self.chunk_seconds),
                    "-t", "wav",
                    wav_path,
                ]
                subprocess.run(cmd, check=False, timeout=max(5.0, self.chunk_seconds + 3.0))
                text = self.transcribe_wav_file(wav_path)
                if text:
                    try:
                        self.transcript_queue.put_nowait(text)
                    except queue.Full:
                        self.get_logger().warn("transcript_queue full; dropping mic transcript")
            except Exception as exc:
                self._publish_input_status(f"backend=mic error {exc}")
                time.sleep(self.connection_retry_sec)
            finally:
                try:
                    if wav_path:
                        os.remove(wav_path)
                except Exception:
                    pass
            time.sleep(0.05)

    def _run_transcriber(self) -> None:
        while not self.stop_event.is_set():
            try:
                pcm_bytes = self.audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            text = self.transcribe_pcm_chunk(pcm_bytes)
            if text:
                try:
                    self.transcript_queue.put_nowait(text)
                except queue.Full:
                    self.get_logger().warn("transcript_queue full; dropping transcript")

    def _publish_transcripts(self) -> None:
        while True:
            try:
                text = self.transcript_queue.get_nowait()
            except queue.Empty:
                break
            msg = String()
            msg.data = text
            self.transcript_pub.publish(msg)
            self.get_logger().info(f"transcript: {text}")

    def transcribe_pcm_chunk(self, pcm_bytes: bytes) -> str:
        if not pcm_bytes:
            return ""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
        try:
            with wave.open(wav_path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(pcm_bytes)

            segments, _ = self.model.transcribe(
                wav_path,
                beam_size=1,
                language=self.language,
                vad_filter=self.vad_filter,
            )
            return " ".join(seg.text.strip() for seg in segments).strip()
        except Exception as exc:
            self.get_logger().error(f"Local STT failed: {exc}")
            return ""
        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def transcribe_wav_file(self, wav_path: str) -> str:
        try:
            segments, _ = self.model.transcribe(
                wav_path,
                beam_size=1,
                language=self.language,
                vad_filter=self.vad_filter,
            )
            return " ".join(seg.text.strip() for seg in segments).strip()
        except Exception as exc:
            self.get_logger().error(f"Local STT failed: {exc}")
            return ""

    def destroy_node(self):
        self.stop_event.set()
        time.sleep(0.2)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalOmiSttNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
