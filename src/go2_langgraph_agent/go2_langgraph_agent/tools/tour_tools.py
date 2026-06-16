from __future__ import annotations

from typing import Any, Dict


def grounded_tour_response(context: Dict[str, Any], question: str = '') -> str:
    place = ((context.get('spawn') or {}).get('data') or {}).get('label') or 'this area'
    live = context.get('live_observation') or 'I do not have a live VLM observation yet.'
    if question:
        return f"Based on saved memory near {place}: {live} Question noted: {question}"
    return f"We are near {place}. Saved memory is loaded, and live observation says: {live}"
