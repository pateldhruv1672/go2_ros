from __future__ import annotations

import json
from typing import Any, Dict

from std_msgs.msg import String


def publish_nav_command(pub, action: str, payload: Dict[str, Any] | None = None) -> None:
    msg = {'action': action}
    if payload:
        msg.update(payload)
    pub.publish(String(data=json.dumps(msg, sort_keys=True)))
