from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path
from typing import Any

from .ollama_structured import StructuredOllamaClient, StructuredOllamaError


def _think_value(value: str) -> Any:
    text = value.strip().lower()
    if text in {"false", "off", "none", "0"}:
        return False
    if text in {"true", "on", "1"}:
        return True
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe SysNav's local Ollama contract.")
    parser.add_argument("--url", default="http://127.0.0.1:11434")
    parser.add_argument("--model", default="gemma4:e4b")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--think", default="false")
    parser.add_argument("--keep-alive", default="10m")
    parser.add_argument("--image", default="")
    parser.add_argument(
        "--jsonl",
        default=str(Path.home() / ".ros/go2_sysnav_vln/ollama_probe.jsonl"),
    )
    args = parser.parse_args()
    client = StructuredOllamaClient(
        args.url, args.model, args.timeout, args.keep_alive, args.jsonl
    )
    try:
        health = client.diagnose()
        print("HEALTH")
        print(json.dumps(health, indent=2, sort_keys=True))
        if not health["model_available"]:
            print(f"ERROR: model {args.model!r} is not installed", file=sys.stderr)
            raise SystemExit(2)

        decompose_schema = {
            "type": "object",
            "properties": {
                "target_object": {"type": "string"},
                "room_condition": {"type": "string"},
                "spatial_condition": {"type": "string"},
                "attribute_condition": {"type": "string"},
                "anchor_object": {"type": "string"},
                "attribute_condition_anchor": {"type": "string"},
            },
            "required": [
                "target_object", "room_condition", "spatial_condition",
                "attribute_condition", "anchor_object", "attribute_condition_anchor",
            ],
            "additionalProperties": False,
        }
        result = client.chat(
            "Extract fields from an indoor object-navigation instruction. Use empty strings for absent fields.",
            "Find the blue trash can in the classroom next to the black desk.",
            decompose_schema,
            think=_think_value(args.think),
            request_id="probe-decompose",
            kind="decompose",
        )
        print("\nTEXT_STRUCTURED_TEST")
        print(json.dumps(result, indent=2, sort_keys=True))

        room_schema = {
            "type": "object",
            "properties": {
                "room_id": {"type": "string", "enum": ["R1", "R2"]},
                "reason": {"type": "string"},
            },
            "required": ["room_id", "reason"],
            "additionalProperties": False,
        }
        result = client.chat(
            "Select exactly one supplied room ID. Never invent an ID.",
            json.dumps(
                {
                    "instruction": "find a refrigerator",
                    "rooms": [
                        {"room_id": "R1", "label": "office", "objects": ["chair"]},
                        {"room_id": "R2", "label": "kitchen", "objects": ["refrigerator"]},
                    ],
                }
            ),
            room_schema,
            think=_think_value(args.think),
            request_id="probe-room",
            kind="select_room",
        )
        print("\nENUM_CONSTRAINT_TEST")
        print(json.dumps(result, indent=2, sort_keys=True))

        if args.image:
            image_path = Path(args.image).expanduser().resolve()
            image_b64 = base64.b64encode(image_path.read_bytes()).decode("ascii")
            vision_schema = {
                "type": "object",
                "properties": {
                    "setting": {"type": "string", "enum": ["indoor", "outdoor", "unknown"]},
                    "objects": {"type": "array", "items": {"type": "string"}},
                    "summary": {"type": "string"},
                },
                "required": ["setting", "objects", "summary"],
                "additionalProperties": False,
            }
            result = client.chat(
                "Describe the supplied robot camera image conservatively.",
                "Classify the setting and list only clearly visible objects.",
                vision_schema,
                images=[image_b64],
                think=_think_value(args.think),
                request_id="probe-vision",
                kind="vision",
            )
            print("\nVISION_STRUCTURED_TEST")
            print(json.dumps(result, indent=2, sort_keys=True))

        print(f"\nPASS: Ollama contract is valid. Audit log: {args.jsonl}")
    except StructuredOllamaError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
