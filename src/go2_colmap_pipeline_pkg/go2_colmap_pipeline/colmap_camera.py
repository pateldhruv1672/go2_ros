from __future__ import annotations

from typing import Dict, Any, Tuple


def _flatten(data):
    if data is None:
        return []
    return [float(v) for v in data]


def camera_info_to_colmap(camera_info: Dict[str, Any]) -> Tuple[str, str]:
    k = _flatten(camera_info.get('k'))
    d = _flatten(camera_info.get('d'))
    if len(k) != 9:
        raise ValueError('Camera matrix K must contain 9 values.')

    fx = k[0]
    fy = k[4]
    cx = k[2]
    cy = k[5]

    if len(d) >= 4:
        model = 'OPENCV'
        k1 = d[0]
        k2 = d[1]
        p1 = d[2]
        p2 = d[3]
        params = f'{fx},{fy},{cx},{cy},{k1},{k2},{p1},{p2}'
    elif len(d) >= 2:
        model = 'RADIAL'
        f = (fx + fy) / 2.0
        params = f'{f},{cx},{cy},{d[0]},{d[1]}'
    else:
        model = 'PINHOLE'
        params = f'{fx},{fy},{cx},{cy}'

    return model, params
