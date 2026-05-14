from __future__ import annotations

import re
from typing import Dict


def normalize_text(text: str) -> str:
    return re.sub(r'\s+', ' ', text.strip().lower())


def parse_command(raw_text: str) -> Dict[str, object]:
    text = normalize_text(raw_text)
    if not text:
        return {'type': 'noop'}
    if text in {'help', '?', 'commands'}:
        return {'type': 'help'}
    if text in {'status', 'robot status', 'where are you'}:
        return {'type': 'status'}
    if text in {'list places', 'show places'}:
        return {'type': 'list_places'}
    if text in {'memory status', 'memory stats', 'show memory stats'}:
        return {'type': 'memory_status'}
    if text in {'start survey', 'survey mode', 'toggle survey', 'enter survey mode'}:
        return {'type': 'start_survey'}
    if text in {'stop survey', 'stop survey mode', 'ai mode', 'exit survey mode'}:
        return {'type': 'stop_survey'}
    if text in {'stop', 'cancel', 'abort', 'stop now'}:
        return {'type': 'stop_all'}
    if text.startswith('set active map to '):
        return {'type': 'set_active_map', 'map_name': text.replace('set active map to ', '', 1).strip()}
    m = re.match(r'^(?:save map as|save the map as|store map as)\s+(.+)$', text)
    if m:
        return {'type': 'save_map', 'map_name': m.group(1).strip()}
    m = re.match(r'^(?:remember this place as|save this place as|mark this place as|name this location)\s+(.+)$', text)
    if m:
        return {'type': 'remember_current_pose', 'place': m.group(1).strip()}
    m = re.match(r'^(?:capture memory as|remember scene as)\s+(.+)$', text)
    if m:
        return {'type': 'capture_semantic', 'label': m.group(1).strip()}
    if text in {'capture memory', 'capture scene', 'remember scene'}:
        return {'type': 'capture_semantic', 'label': None}
    m = re.match(r'^(?:what do you remember about|recall|remember about|show memory for|what did you see at)\s+(.+)$', text)
    if m:
        return {'type': 'memory_recall', 'query': m.group(1).strip()}
    m = re.match(r'^(?:go to|navigate to|move to|take me to|robot go to)\s+(.+?)(?:\s+and\s+say\s+(.+))?$', text)
    if m:
        return {'type': 'navigate_named_place', 'place': m.group(1).strip(), 'say': m.group(2).strip() if m.group(2) else None}
    m = re.match(r'^(?:say|speak)\s+(.+)$', text)
    if m:
        return {'type': 'say', 'text': m.group(1).strip()}
    m = re.match(r'^(?:forward|ahead|w)(?:\s+(\d+(?:\.\d+)?))?$', text)
    if m:
        return {'type': 'manual_drive', 'linear': 1.0, 'angular': 0.0, 'duration': float(m.group(1)) if m.group(1) else None, 'label': 'forward'}
    m = re.match(r'^(?:back|backward|reverse|s)(?:\s+(\d+(?:\.\d+)?))?$', text)
    if m:
        return {'type': 'manual_drive', 'linear': -1.0, 'angular': 0.0, 'duration': float(m.group(1)) if m.group(1) else None, 'label': 'reverse'}
    m = re.match(r'^(?:left|turn left|a)(?:\s+(\d+(?:\.\d+)?))?$', text)
    if m:
        return {'type': 'manual_drive', 'linear': 0.0, 'angular': 1.0, 'duration': float(m.group(1)) if m.group(1) else None, 'label': 'turn_left'}
    m = re.match(r'^(?:right|turn right|d)(?:\s+(\d+(?:\.\d+)?))?$', text)
    if m:
        return {'type': 'manual_drive', 'linear': 0.0, 'angular': -1.0, 'duration': float(m.group(1)) if m.group(1) else None, 'label': 'turn_right'}
    if text in {'x', 'zero', 'stop motion'}:
        return {'type': 'stop_motion'}
    return {'type': 'unknown', 'text': text}
