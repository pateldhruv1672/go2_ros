from __future__ import annotations

import math
from pathlib import Path

from .common import bbox_iou_2d, clean_label, viewpoint_is_novel


def main() -> None:
    assert clean_label("Blue Trash Can") == "blue_trash_can"
    iou, overlap_a, overlap_b = bbox_iou_2d((0, 0, 1, 1), (0.5, 0, 1.5, 1))
    assert 0.32 < iou < 0.34
    assert overlap_a == 0.5 and overlap_b == 0.5

    obj = (2.0, 0.0)
    prior = [(0.0, 0.0, 0.0)]
    assert not viewpoint_is_novel(
        obj, (0.05, 0.0, 0.01), prior,
        math.radians(5.0), 0.30, 0.20, 0.15,
    )
    assert viewpoint_is_novel(
        obj, (0.0, 0.8, 0.0), prior,
        math.radians(5.0), 0.30, 0.20, 0.15,
    )

    root = Path(__file__).resolve().parent
    supervisor = (root / "vln_supervisor.py").read_text()
    assert "tool_calls" not in supervisor
    assert "cmd_vel" not in supervisor
    assert "ComputePathToPose" not in supervisor  # delegated to safe Nav2 server
    assert '"room_id"' in supervisor
    print("go2_sysnav_vln self-test: PASS")


if __name__ == "__main__":
    main()
