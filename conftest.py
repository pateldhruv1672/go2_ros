from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
for rel in [
    'src/go2_agentic_system',
    'src/go2_semantic_nav_agent',
    'src/go2_agentic_multiagent_voice_nav',
    'src/go2_agentic_motion_skills',
    'src/council',
]:
    path = str((ROOT / rel).resolve())
    if path not in sys.path:
        sys.path.insert(0, path)
