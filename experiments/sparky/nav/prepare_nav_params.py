#!/usr/bin/env python3
"""Turn the exported NavFn+DWB candidate config into a launchable params file. Records every change."""
import sys, json, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
src, map_yaml, scn_path, out = sys.argv[1:5]
scn = json.loads(Path(scn_path).read_text())
p = yaml.safe_load(Path(src).read_text())
changes = []
bt = p['bt_navigator']['ros__parameters']
base = share('nav2_bt_navigator')
for k, v in list(bt.items()):
    if isinstance(v, str) and '$(find-pkg-share nav2_bt_navigator)' in v:
        bt[k] = v.replace('$(find-pkg-share nav2_bt_navigator)', base); changes.append(f'bt_navigator.{k}: resolved find-pkg-share')
a = p['amcl']['ros__parameters']
a['set_initial_pose'] = True
a['initial_pose'] = {'x': scn['start']['x'], 'y': scn['start']['y'], 'z': 0.0, 'yaw': scn['start']['yaw']}
changes.append('amcl.set_initial_pose=true at scenario start')
p['map_server']['ros__parameters']['yaml_filename'] = map_yaml
changes.append('map_server.yaml_filename=' + map_yaml)
Path(out).write_text(yaml.safe_dump(p))
Path(out + '.changes.json').write_text(json.dumps(changes, indent=1))
print(changes)
