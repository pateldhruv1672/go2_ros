from __future__ import annotations


def normalize_text_command(text: str) -> str:
    return ' '.join((text or '').strip().lower().split())
