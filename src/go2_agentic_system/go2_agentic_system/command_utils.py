from __future__ import annotations

import json
import re
from typing import Any, Dict


def make_event(event_type: str, **payload: Any) -> str:
    return json.dumps({'type': event_type, **payload}, ensure_ascii=False)



def parse_event(raw: str) -> Dict[str, Any]:
    return json.loads(raw)



def normalize_text(text: str) -> str:
    return re.sub(r'\s+', ' ', text.strip().lower())



def extract_destination(text: str) -> str | None:
    match = re.search(r'(?:go to|navigate to|move to|take me to|robot go to)\s+(.+)$', text)
    if match:
        return match.group(1).strip()
    return None



def extract_label(text: str) -> str | None:
    patterns = [
        r'(?:save this place as|remember this place as|mark this location as|name this location)\s+(.+)$',
        r'(?:save map as|save the map as|store map as)\s+(.+)$',
    ]
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return match.group(1).strip()
    return None
