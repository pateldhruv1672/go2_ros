from __future__ import annotations

from typing import Any, Dict


def should_promote(record: Dict[str, Any], min_confidence: float = 0.78) -> bool:
    data = record.get('data', {})
    if data.get('verified') is True or data.get('user_approved') is True:
        return True
    confidence = record.get('confidence') or data.get('confidence') or {}
    if isinstance(confidence, dict):
        score = max([float(v) for v in confidence.values() if isinstance(v, (int, float))] or [0.0])
    else:
        score = float(confidence or 0.0)
    return score >= min_confidence and not data.get('requires_verification', False)


def promote_record(record: Dict[str, Any], reason: str = '') -> Dict[str, Any]:
    promoted = dict(record)
    promoted['layer'] = 'permanent'
    promoted.setdefault('data', {})['promotion_reason'] = reason
    promoted.setdefault('data', {})['requires_verification'] = False
    return promoted
