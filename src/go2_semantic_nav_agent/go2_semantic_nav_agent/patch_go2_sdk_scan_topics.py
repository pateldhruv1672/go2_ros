from __future__ import annotations

import os
import re


def patch_file(path: str) -> None:
    if not os.path.isfile(path):
        return
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    text = re.sub(r'(/scan_fixed)+', '/scan_fixed', text)
    text = re.sub(r'(?m)^(\s*scan_topic\s*:\s*).*$' , r'\1/scan_fixed', text)
    text = re.sub(r'(?m)^(\s*topic\s*:\s*).*/scan.*$' , r'\1/scan_fixed', text)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'Patched {path}')


def main() -> None:
    ws = os.path.expanduser('~/ros2_ws/src/go2_robot_sdk/config')
    patch_file(os.path.join(ws, 'mapper_params_online_async.yaml'))
    patch_file(os.path.join(ws, 'nav2_params.yaml'))


if __name__ == '__main__':
    main()
