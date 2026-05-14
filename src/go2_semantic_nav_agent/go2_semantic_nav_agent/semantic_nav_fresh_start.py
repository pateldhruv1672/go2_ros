from __future__ import annotations

import glob
import os
import shutil


def main() -> None:
    root = os.path.expanduser('~/.ros/go2_semantic_nav_sessions')
    if os.path.isdir(root):
        for d in glob.glob(os.path.join(root, '*')):
            shutil.rmtree(d, ignore_errors=True)
    places = os.path.expanduser('~/.ros/go2_semantic_nav_places.yaml')
    if os.path.isfile(places):
        os.remove(places)
    print(f'Cleared semantic nav sessions under {root}')


if __name__ == '__main__':
    main()
