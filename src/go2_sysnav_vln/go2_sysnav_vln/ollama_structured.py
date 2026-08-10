from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional


class StructuredOllamaError(RuntimeError):
    pass


def _type_ok(value: Any, expected: str) -> bool:
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "null":
        return value is None
    return True


def validate_schema(value: Any, schema: Dict[str, Any], path: str = "$") -> None:
    expected = schema.get("type")
    if isinstance(expected, str) and not _type_ok(value, expected):
        raise StructuredOllamaError(
            f"schema validation failed at {path}: expected {expected}, got {type(value).__name__}"
        )
    if "enum" in schema and value not in schema["enum"]:
        raise StructuredOllamaError(
            f"schema validation failed at {path}: {value!r} not in enum {schema['enum']!r}"
        )
    if isinstance(value, dict):
        properties = schema.get("properties", {})
        required = schema.get("required", [])
        for key in required:
            if key not in value:
                raise StructuredOllamaError(
                    f"schema validation failed at {path}: missing required key {key!r}"
                )
        if schema.get("additionalProperties") is False:
            extras = sorted(set(value) - set(properties))
            if extras:
                raise StructuredOllamaError(
                    f"schema validation failed at {path}: unexpected keys {extras!r}"
                )
        for key, item in value.items():
            child = properties.get(key)
            if isinstance(child, dict):
                validate_schema(item, child, f"{path}.{key}")
    elif isinstance(value, list) and isinstance(schema.get("items"), dict):
        for index, item in enumerate(value):
            validate_schema(item, schema["items"], f"{path}[{index}]")


class StructuredOllamaClient:
    """Dependency-free local Ollama client with schema validation and audit metadata."""

    def __init__(
        self,
        base_url: str,
        model: str,
        timeout_sec: float = 90.0,
        keep_alive: str = "10m",
        debug_jsonl_path: str = "",
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout_sec = float(timeout_sec)
        self.keep_alive = str(keep_alive)
        self.debug_jsonl_path = os.path.expanduser(str(debug_jsonl_path).strip())

    def _request(
        self,
        method: str,
        path: str,
        payload: Optional[Dict[str, Any]] = None,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers={"Content-Type": "application/json", "Accept": "application/json"},
            method=method,
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self.timeout_sec if timeout_sec is None else timeout_sec
            ) as response:
                body = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise StructuredOllamaError(f"HTTP {exc.code} {path}: {body[:1000]}") from exc
        except Exception as exc:
            raise StructuredOllamaError(
                f"Ollama request failed for {path}: {type(exc).__name__}: {exc}"
            ) from exc
        try:
            parsed = json.loads(body)
        except Exception as exc:
            raise StructuredOllamaError(
                f"Ollama returned non-JSON for {path}: {body[:1000]}"
            ) from exc
        if not isinstance(parsed, dict):
            raise StructuredOllamaError(f"Ollama response for {path} is not an object")
        return parsed

    def version(self) -> Dict[str, Any]:
        return self._request("GET", "/api/version", timeout_sec=min(self.timeout_sec, 5.0))

    def model_names(self) -> List[str]:
        payload = self._request("GET", "/api/tags", timeout_sec=min(self.timeout_sec, 10.0))
        names: List[str] = []
        for item in payload.get("models", []):
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                names.append(item["name"])
        return names

    def show_model(self) -> Dict[str, Any]:
        return self._request(
            "POST",
            "/api/show",
            payload={"model": self.model},
            timeout_sec=min(self.timeout_sec, 15.0),
        )

    def diagnose(self) -> Dict[str, Any]:
        version_payload = self.version()
        names = self.model_names()
        normalized = {name.removesuffix(":latest") for name in names}
        requested = self.model.removesuffix(":latest")
        available = requested in normalized
        details: Dict[str, Any] = {}
        show_error = ""
        if available:
            try:
                details = self.show_model()
            except Exception as exc:
                show_error = f"{type(exc).__name__}: {exc}"
        capabilities = details.get("capabilities", [])
        if not isinstance(capabilities, list):
            capabilities = []
        return {
            "ok": available,
            "base_url": self.base_url,
            "version": version_payload.get("version", "unknown"),
            "model": self.model,
            "model_available": available,
            "capabilities": capabilities,
            "vision_capable": "vision" in capabilities,
            "show_error": show_error,
            "installed_models": names,
        }

    def _append_debug(self, record: Dict[str, Any]) -> None:
        if not self.debug_jsonl_path:
            return
        path = Path(self.debug_jsonl_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")

    def chat(
        self,
        system: str,
        user: str,
        schema: Dict[str, Any],
        images: Optional[List[str]] = None,
        think: Any = False,
        request_id: str = "",
        kind: str = "",
    ) -> Dict[str, Any]:
        schema_text = json.dumps(schema, sort_keys=True, separators=(",", ":"))
        content = (
            user
            + "\n\nReturn only JSON matching this schema exactly. Do not add markdown.\n"
            + schema_text
        )
        message: Dict[str, Any] = {"role": "user", "content": content}
        if images:
            message["images"] = images
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                message,
            ],
            "stream": False,
            "format": schema,
            "options": {"temperature": 0.0, "seed": 7},
            "think": think,
            "keep_alive": self.keep_alive,
        }
        started_wall = time.time()
        started_mono = time.monotonic()
        request_record = {
            "event": "request",
            "stamp_sec": started_wall,
            "request_id": request_id,
            "kind": kind,
            "url": self.base_url + "/api/chat",
            "model": self.model,
            "think": think,
            "keep_alive": self.keep_alive,
            "timeout_sec": self.timeout_sec,
            "image_count": len(images or []),
            "image_base64_chars": sum(len(item) for item in (images or [])),
            "system": system,
            "user": user,
            "schema": schema,
        }
        self._append_debug(request_record)
        try:
            outer = self._request("POST", "/api/chat", payload=payload)
        except Exception as exc:
            self._append_debug(
                {
                    "event": "transport_error",
                    "stamp_sec": time.time(),
                    "request_id": request_id,
                    "kind": kind,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                }
            )
            raise

        wall_ms = round((time.monotonic() - started_mono) * 1000.0, 1)
        message_payload = outer.get("message", {})
        content_out = (
            message_payload.get("content", "")
            if isinstance(message_payload, dict)
            else ""
        )
        self._append_debug(
            {
                "event": "raw_response",
                "stamp_sec": time.time(),
                "request_id": request_id,
                "kind": kind,
                "wall_ms": wall_ms,
                "outer": outer,
                "message_content": content_out,
            }
        )
        try:
            answer = json.loads(content_out)
        except Exception as exc:
            self._append_debug(
                {
                    "event": "parse_error",
                    "stamp_sec": time.time(),
                    "request_id": request_id,
                    "kind": kind,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "message_content": content_out,
                }
            )
            raise StructuredOllamaError(
                f"Ollama returned invalid JSON content: {content_out[:2000]}"
            ) from exc
        if not isinstance(answer, dict):
            error = "Ollama structured response is not an object"
            self._append_debug(
                {
                    "event": "parse_error",
                    "stamp_sec": time.time(),
                    "request_id": request_id,
                    "kind": kind,
                    "error": error,
                    "answer": answer,
                }
            )
            raise StructuredOllamaError(error)
        try:
            validate_schema(answer, schema)
        except Exception as exc:
            self._append_debug(
                {
                    "event": "schema_error",
                    "stamp_sec": time.time(),
                    "request_id": request_id,
                    "kind": kind,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "answer": answer,
                    "schema": schema,
                }
            )
            raise
        meta = {
            "request_id": request_id,
            "kind": kind,
            "url": self.base_url + "/api/chat",
            "model_requested": self.model,
            "model_returned": outer.get("model", ""),
            "created_at": outer.get("created_at", ""),
            "done": bool(outer.get("done", False)),
            "done_reason": outer.get("done_reason", ""),
            "wall_ms": wall_ms,
            "total_ms": round(float(outer.get("total_duration", 0)) / 1e6, 1),
            "load_ms": round(float(outer.get("load_duration", 0)) / 1e6, 1),
            "prompt_eval_ms": round(float(outer.get("prompt_eval_duration", 0)) / 1e6, 1),
            "eval_ms": round(float(outer.get("eval_duration", 0)) / 1e6, 1),
            "prompt_tokens": int(outer.get("prompt_eval_count", 0) or 0),
            "eval_tokens": int(outer.get("eval_count", 0) or 0),
            "image_count": len(images or []),
            "image_base64_chars": sum(len(item) for item in (images or [])),
            "thinking_chars": len(str(message_payload.get("thinking", "")))
            if isinstance(message_payload, dict)
            else 0,
            "response_chars": len(content_out),
        }
        self._append_debug(
            {
                "event": "response",
                "stamp_sec": time.time(),
                "meta": meta,
                "answer": answer,
            }
        )
        return {"answer": answer, "meta": meta, "raw": outer}
