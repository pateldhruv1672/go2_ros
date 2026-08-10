from __future__ import annotations

import json
import os
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, List


class GeminiGroundedSearch:
    """Small dependency-free Gemini Google Search grounding client."""

    def __init__(self, model: str = "gemini-2.5-flash", api_key: str = "", timeout_sec: float = 20.0):
        self.model = model or os.environ.get("GO2_WEB_MODEL", "gemini-2.5-flash")
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY", "") or os.environ.get("GOOGLE_API_KEY", "")
        self.timeout_sec = float(timeout_sec)

    @property
    def available(self) -> bool:
        return bool(self.api_key)

    def search(self, query: str) -> Dict[str, Any]:
        query = (query or "").strip()
        if not query:
            return {"success": False, "error": "empty query", "answer": "", "sources": []}
        if not self.api_key:
            return {
                "success": False,
                "error": "GEMINI_API_KEY or GOOGLE_API_KEY is not set",
                "answer": "Web search is not configured on the DGX Spark.",
                "sources": [],
            }
        model = urllib.parse.quote(self.model, safe=".-_")
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
        body = {
            "contents": [{"role": "user", "parts": [{"text": query}]}],
            "tools": [{"google_search": {}}],
            "generationConfig": {"temperature": 0.2, "maxOutputTokens": 1024},
        }
        request = urllib.request.Request(
            url,
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json", "x-goog-api-key": self.api_key},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:1000]
            return {"success": False, "error": f"Gemini HTTP {exc.code}: {detail}", "answer": "", "sources": []}
        except Exception as exc:
            return {"success": False, "error": str(exc), "answer": "", "sources": []}

        candidates = payload.get("candidates") or []
        candidate = candidates[0] if candidates else {}
        parts = ((candidate.get("content") or {}).get("parts") or [])
        answer = "\n".join(str(p.get("text") or "") for p in parts if isinstance(p, dict) and p.get("text")).strip()
        metadata = candidate.get("groundingMetadata") or candidate.get("grounding_metadata") or {}
        chunks = metadata.get("groundingChunks") or metadata.get("grounding_chunks") or []
        sources: List[Dict[str, str]] = []
        seen = set()
        for chunk in chunks:
            if not isinstance(chunk, dict):
                continue
            web = chunk.get("web") or {}
            uri = str(web.get("uri") or "")
            title = str(web.get("title") or uri)
            if uri and uri not in seen:
                seen.add(uri)
                sources.append({"title": title, "uri": uri})
        return {
            "success": bool(answer),
            "answer": answer,
            "sources": sources,
            "model": self.model,
            "grounded": bool(sources),
            "raw_grounding_metadata": metadata,
        }
