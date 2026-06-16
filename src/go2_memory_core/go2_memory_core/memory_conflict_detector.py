from __future__ import annotations

from typing import Any, Dict, Iterable, List


def detect_name_conflicts(new_record: Dict[str, Any], existing_records: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    data = new_record.get('data', {})
    name = str(data.get('name') or data.get('label') or '').strip().lower()
    aliases = {str(a).strip().lower() for a in data.get('aliases', [])}
    candidates = {name} | aliases
    candidates.discard('')
    conflicts: List[Dict[str, Any]] = []
    for record in existing_records:
        rdata = record.get('data', {})
        rnames = {str(rdata.get('name') or rdata.get('label') or '').strip().lower()}
        rnames |= {str(a).strip().lower() for a in rdata.get('aliases', [])}
        rnames.discard('')
        overlap = candidates & rnames
        if overlap and record.get('id') != new_record.get('id'):
            conflicts.append({'record_id': record.get('id'), 'overlap': sorted(overlap)})
    return conflicts
