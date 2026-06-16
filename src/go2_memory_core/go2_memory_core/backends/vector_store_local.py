from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Tuple
import json
import math
import re

_TOKEN_RE = re.compile(r"[a-zA-Z0-9_]+")


def _tokens(text: str) -> Dict[str, float]:
    counts: Dict[str, float] = {}
    for token in _TOKEN_RE.findall(text.lower()):
        counts[token] = counts.get(token, 0.0) + 1.0
    return counts


def _cosine(a: Dict[str, float], b: Dict[str, float]) -> float:
    dot = sum(v * b.get(k, 0.0) for k, v in a.items())
    na = math.sqrt(sum(v * v for v in a.values()))
    nb = math.sqrt(sum(v * v for v in b.values()))
    if na == 0.0 or nb == 0.0:
        return 0.0
    return dot / (na * nb)


class VectorStoreLocal:
    """Dependency-free text retrieval fallback.

    It is not a replacement for CLIP/OpenAI/Gemini embeddings, but it gives the
    agent a working retrieval path until real embeddings are wired in.
    """

    def __init__(self, session_dir: Path):
        self.path = session_dir / 'vector_memory' / 'embeddings.jsonl'
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def add_text(self, record_id: str, text: str, metadata: Dict[str, Any] | None = None) -> None:
        record = {
            'id': record_id,
            'text': text,
            'metadata': metadata or {},
            'sparse_vector': _tokens(text),
        }
        with self.path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, sort_keys=True) + '\n')

    def search(self, text: str, limit: int = 5) -> List[Dict[str, Any]]:
        q = _tokens(text)
        scored: List[Tuple[float, Dict[str, Any]]] = []
        for record in self._read():
            score = _cosine(q, record.get('sparse_vector', {}))
            if score > 0.0:
                item = dict(record)
                item['score'] = score
                scored.append((score, item))
        scored.sort(key=lambda x: x[0], reverse=True)
        return [item for _, item in scored[:limit]]

    def _read(self) -> List[Dict[str, Any]]:
        if not self.path.exists():
            return []
        rows: List[Dict[str, Any]] = []
        with self.path.open('r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
        return rows
