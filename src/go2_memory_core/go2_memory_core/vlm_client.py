from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional
import base64
import json
import os
import urllib.error
import urllib.request


@dataclass
class VLMResult:
    success: bool
    summary: str
    provider: str
    model: str
    raw: Dict[str, Any]
    error: str = ""


class VLMClient:
    """Small Ollama/OpenRouter/Gemini-compatible VLM client with offline fallback.

    It uses stdlib ``urllib`` so the ROS package does not require extra HTTP
    dependencies. In CI or offline robot runs, set ``provider=offline`` to still
    write deterministic checkpoint records without hallucinating visual details.
    """

    def __init__(self, provider: str = "offline", model: str = "", api_key: str = "", base_url: str = ""):
        self.provider = (provider or "offline").strip().lower()
        self.model = model or ("qwen3-vl:8b" if self.provider == "ollama" else "google/gemini-2.5-flash")
        self.api_key = api_key or self._default_key(self.provider)
        self.base_url = base_url or self._default_base_url(self.provider)

    def summarize_image(self, image_bytes: bytes | None, mime_type: str = "image/jpeg", prompt: str = "") -> VLMResult:
        prompt = prompt or (
            "Describe the robot's current navigation checkpoint. Mention durable place cues, "
            "doors, hallway geometry, signs, obstacles, hazards, and objects. Be concise and mark uncertainty."
        )
        if self.provider in {"offline", "none", "disabled"}:
            return VLMResult(True, "offline_vlm: image summary unavailable; checkpoint stores geometry/odom evidence only", "offline", self.model, {})
        if not image_bytes:
            return VLMResult(False, "", self.provider, self.model, {}, "no image bytes available")
        if self.provider == "openrouter":
            return self._openrouter(image_bytes, mime_type, prompt)
        if self.provider == "ollama":
            return self._ollama(image_bytes, mime_type, prompt)
        if self.provider == "gemini":
            return self._gemini(image_bytes, mime_type, prompt)
        return VLMResult(False, "", self.provider, self.model, {}, f"unsupported VLM provider: {self.provider}")

    def _openrouter(self, image_bytes: bytes, mime_type: str, prompt: str) -> VLMResult:
        if not self.api_key:
            return VLMResult(False, "", "openrouter", self.model, {}, "OPENROUTER_API_KEY is not set")
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:{mime_type};base64,{image_b64}"}},
                ],
            }],
            "temperature": 0.1,
            "max_tokens": 320,
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "HTTP-Referer": "https://github.com/pateldhruv1672/go2_ros",
            "X-Title": "Go2 Agentic Navigation",
        }
        return self._post_json(self.base_url, payload, headers, provider="openrouter")

    def _ollama(self, image_bytes: bytes, mime_type: str, prompt: str) -> VLMResult:
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": prompt,
                    "images": [image_b64],
                }
            ],
            "stream": False,
            "options": {
                "temperature": 0.1,
                "num_predict": 320,
            },
        }
        return self._post_json(self.base_url, payload, {"Content-Type": "application/json"}, provider="ollama")

    def _gemini(self, image_bytes: bytes, mime_type: str, prompt: str) -> VLMResult:
        if not self.api_key:
            return VLMResult(False, "", "gemini", self.model, {}, "GEMINI_API_KEY is not set")
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        model = self.model if self.model.startswith("models/") else f"models/{self.model}"
        url = self.base_url.rstrip("/") + f"/{model}:generateContent?key={self.api_key}"
        payload = {
            "contents": [{
                "parts": [
                    {"text": prompt},
                    {"inline_data": {"mime_type": mime_type, "data": image_b64}},
                ]
            }],
            "generationConfig": {"temperature": 0.1, "maxOutputTokens": 320},
        }
        return self._post_json(url, payload, {"Content-Type": "application/json"}, provider="gemini")

    def _post_json(self, url: str, payload: Dict[str, Any], headers: Dict[str, str], provider: str) -> VLMResult:
        request = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"), headers=headers, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=25) as response:
                raw = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            return VLMResult(False, "", provider, self.model, {}, f"HTTP {exc.code}: {body[:500]}")
        except Exception as exc:
            return VLMResult(False, "", provider, self.model, {}, str(exc))
        summary = self._extract_summary(provider, raw)
        return VLMResult(bool(summary), summary, provider, self.model, raw, "" if summary else "empty VLM response")

    @staticmethod
    def _extract_summary(provider: str, raw: Dict[str, Any]) -> str:
        if provider == "openrouter":
            try:
                return str(raw["choices"][0]["message"]["content"]).strip()
            except Exception:
                return ""
        if provider == "ollama":
            try:
                return str(raw["message"]["content"]).strip()
            except Exception:
                return str(raw.get("response", "")).strip()
        if provider == "gemini":
            try:
                return str(raw["candidates"][0]["content"]["parts"][0]["text"]).strip()
            except Exception:
                return ""
        return ""

    @staticmethod
    def _default_key(provider: str) -> str:
        if provider == "openrouter":
            return os.environ.get("OPENROUTER_API_KEY", "")
        if provider == "gemini":
            return os.environ.get("GEMINI_API_KEY", "")
        return ""

    @staticmethod
    def _default_base_url(provider: str) -> str:
        if provider == "openrouter":
            return "https://openrouter.ai/api/v1/chat/completions"
        if provider == "ollama":
            return os.environ.get("OLLAMA_CHAT_URL", "http://127.0.0.1:11434/api/chat")
        if provider == "gemini":
            return "https://generativelanguage.googleapis.com/v1beta"
        return ""
