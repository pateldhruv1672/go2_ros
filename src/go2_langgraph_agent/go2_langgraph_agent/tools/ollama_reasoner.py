from __future__ import annotations

import json
import os
import urllib.request
from typing import Any, Dict


class OllamaGroundedReasoner:
    """Optional local speech composer. It may rephrase evidence but may not invent facts."""

    def __init__(self, model: str = "", base_url: str = "", timeout_sec: float = 8.0):
        self.model = model or os.environ.get("GO2_OLLAMA_MODEL", "gemma4:e4b")
        self.base_url = base_url or os.environ.get("GO2_OLLAMA_URL", "http://127.0.0.1:11434/api/chat")
        self.timeout_sec = float(timeout_sec)

    def compose(self, user_request: str, evidence: Dict[str, Any], fallback: str = "") -> str:
        system = (
            "You are Sparky, a university robot tour guide. Use ONLY the supplied evidence. "
            "Never add names, counts, locations, capabilities, or current facts not present in evidence. "
            "Distinguish remembered facts from live observations and web research. Be concise and natural."
        )
        prompt = json.dumps({"request": user_request, "evidence": evidence}, sort_keys=True, default=str)
        body = {
            "model": self.model,
            "stream": False,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": prompt},
            ],
            "options": {"temperature": 0.1},
        }
        try:
            req = urllib.request.Request(
                self.base_url,
                data=json.dumps(body).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as response:
                payload = json.loads(response.read().decode("utf-8"))
            text = str(((payload.get("message") or {}).get("content") or "")).strip()
            return text or fallback
        except Exception:
            return fallback
