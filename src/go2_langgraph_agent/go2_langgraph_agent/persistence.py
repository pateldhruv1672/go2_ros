from __future__ import annotations

from pathlib import Path
from typing import Any, Dict
import json
import os


class JsonCheckpointer:
    def __init__(self, session_root: str, session_name: str):
        self.path = Path(os.path.expanduser(session_root)).resolve() / session_name / 'langgraph' / 'checkpoints' / 'agent_state.json'
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def save(self, state: Dict[str, Any]) -> None:
        self.path.write_text(json.dumps(state, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    def load(self) -> Dict[str, Any]:
        if not self.path.exists():
            return {}
        return json.loads(self.path.read_text(encoding='utf-8'))
