from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

from .ollama_structured import StructuredOllamaClient, StructuredOllamaError


def think_value(value: str) -> Any:
    text = value.strip().lower()
    if text in {"false", "off", "none", "0"}:
        return False
    if text in {"true", "on", "1"}:
        return True
    return text


def schema() -> Dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "image_relevance": {
                "type": "string",
                "enum": [
                    "frontier_centered",
                    "frontier_partial",
                    "frontier_not_visible",
                    "uncertain",
                ],
            },
            "path_surface": {
                "type": "string",
                "enum": ["clear", "partially_blocked", "blocked", "uncertain"],
            },
            "dropoff_or_stairs": {"type": "boolean"},
            "low_overhead_hazard": {"type": "boolean"},
            "dynamic_obstacle": {"type": "boolean"},
            "doorway_or_passage": {"type": "boolean"},
            "hazards": {"type": "array", "items": {"type": "string"}},
            "decision": {
                "type": "string",
                "enum": ["approve", "reject", "unknown"],
            },
            "confidence": {"type": "number"},
            "reason": {"type": "string"},
        },
        "required": [
            "image_relevance",
            "path_surface",
            "dropoff_or_stairs",
            "low_overhead_hazard",
            "dynamic_obstacle",
            "doorway_or_passage",
            "hazards",
            "decision",
            "confidence",
            "reason",
        ],
        "additionalProperties": False,
    }


def gate(answer: Dict[str, Any], minimum: float) -> str:
    hard_hazard = any(
        bool(answer.get(key, False))
        for key in (
            "dropoff_or_stairs",
            "low_overhead_hazard",
            "dynamic_obstacle",
        )
    )
    relevant = str(answer.get("image_relevance", "uncertain")) in {
        "frontier_centered",
        "frontier_partial",
    }
    clear = str(answer.get("path_surface", "uncertain")) == "clear"
    confidence = float(answer.get("confidence", 0.0))
    if (
        str(answer.get("decision", "unknown")) == "approve"
        and relevant
        and clear
        and not hard_hazard
        and confidence >= minimum
    ):
        return "approve"
    if hard_hazard or str(answer.get("decision", "unknown")) == "reject":
        return "reject"
    return "unknown"


def discover_images(explicit: List[str], directory: str) -> List[Path]:
    paths = [Path(item).expanduser().resolve() for item in explicit]
    if directory:
        root = Path(directory).expanduser().resolve()
        for suffix in ("*.jpg", "*.jpeg", "*.png", "*.webp"):
            paths.extend(sorted(root.glob(suffix)))
    unique: List[Path] = []
    seen = set()
    for path in paths:
        if path in seen:
            continue
        seen.add(path)
        if path.is_file():
            unique.append(path)
    return unique


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Test SysNav Ollama text and frontier-image structured inference without ROS, "
            "Nav2, or a connected robot."
        )
    )
    parser.add_argument("--url", default="http://127.0.0.1:11434")
    parser.add_argument("--model", default="gemma4:e4b")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--think", default="false")
    parser.add_argument("--keep-alive", default="10m")
    parser.add_argument("--image", action="append", default=[])
    parser.add_argument("--image-dir", default="")
    parser.add_argument("--frontier-distance-m", type=float, default=2.0)
    parser.add_argument("--camera-alignment-deg", type=float, default=0.0)
    parser.add_argument("--min-confidence", type=float, default=0.65)
    parser.add_argument(
        "--jsonl",
        default=str(Path.home() / ".ros/go2_sysnav_vln/frontier_vision_probe.jsonl"),
    )
    parser.add_argument(
        "--output",
        default=str(Path.home() / ".ros/go2_sysnav_vln/frontier_vision_probe_results.json"),
    )
    args = parser.parse_args()

    client = StructuredOllamaClient(
        args.url,
        args.model,
        args.timeout,
        args.keep_alive,
        args.jsonl,
    )
    report: Dict[str, Any] = {
        "model": args.model,
        "url": args.url,
        "text_test": None,
        "images": [],
    }

    try:
        health = client.diagnose()
        report["health"] = health
        print("HEALTH")
        print(json.dumps(health, indent=2, sort_keys=True))
        if not health.get("model_available"):
            raise StructuredOllamaError(
                f"model {args.model!r} is not installed; installed={health.get('installed_models')}"
            )

        text_schema = {
            "type": "object",
            "properties": {
                "status": {"type": "string", "enum": ["ok"]},
                "value": {"type": "integer"},
            },
            "required": ["status", "value"],
            "additionalProperties": False,
        }
        text_result = client.chat(
            "Return the requested structured test object exactly.",
            "Return status ok and value 7.",
            text_schema,
            think=think_value(args.think),
            request_id="offline-text-contract",
            kind="offline_text_contract",
        )
        report["text_test"] = text_result
        print("\nTEXT CONTRACT")
        print(json.dumps(text_result, indent=2, sort_keys=True))

        images = discover_images(args.image, args.image_dir)
        if not images:
            print(
                "\nNo images supplied. Text transport and structured parsing were tested. "
                "Pass --image FILE or --image-dir DIR for vision testing."
            )
        elif not health.get("vision_capable"):
            print(
                "\nWARNING: /api/show did not advertise the vision capability for this model. "
                "The calls will still be attempted so the raw failure is captured."
            )

        system = (
            "You are an advisory visual safety checker for an indoor quadruped robot. "
            "The image is intended to show the route toward one geometric frontier. "
            "Approve only when that route is visibly relevant, clear, and traversable. "
            "Reject visible stairs, drop-offs, blocking people or objects, low overhead "
            "hazards, and blocked passages. Return unknown when the route is not visible "
            "or depth and relevance are unclear. Never output coordinates or commands."
        )
        for index, path in enumerate(images, 1):
            image_b64 = base64.b64encode(path.read_bytes()).decode("ascii")
            context = {
                "frontier_id": f"offline-F{index}",
                "frontier_distance_m": args.frontier_distance_m,
                "camera_alignment_deg": args.camera_alignment_deg,
                "image_filename": path.name,
                "instruction": (
                    "Assess only the visible route toward this frontier and be conservative."
                ),
            }
            try:
                result = client.chat(
                    system,
                    json.dumps(context, sort_keys=True),
                    schema(),
                    images=[image_b64],
                    think=think_value(args.think),
                    request_id=f"offline-frontier-{index}",
                    kind="offline_frontier_safety",
                )
                answer = result["answer"]
                gate_decision = gate(answer, args.min_confidence)
                item = {
                    "image": str(path),
                    "gate_decision": gate_decision,
                    "result": result,
                }
            except Exception as exc:
                item = {
                    "image": str(path),
                    "gate_decision": "error",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                }
            report["images"].append(item)
            print(f"\nIMAGE {index}: {path}")
            print(json.dumps(item, indent=2, sort_keys=True))

        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(f"\nRESULTS: {output}")
        print(f"AUDIT:   {Path(args.jsonl).expanduser()}")

        if report["text_test"] is None:
            raise SystemExit(1)
        if images and any(item.get("gate_decision") == "error" for item in report["images"]):
            raise SystemExit(2)
    except StructuredOllamaError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
