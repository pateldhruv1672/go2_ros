#!/usr/bin/env python3
"""Create a separate, auditable simulator copy. Never edit Sparky's source."""
import argparse
import difflib
import json
from pathlib import Path
import shutil
import subprocess

PIN = '55ff151632a0220bf3ae0c0772392615d86da7a8'
p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('workspace', type=Path)
p.add_argument('--patch-out', type=Path, default=Path('simulator-adapter.patch'))
a = p.parse_args()
sha = subprocess.check_output(['git', '-C', str(a.source), 'rev-parse', 'HEAD'], text=True).strip()
if sha != PIN:
    raise SystemExit(f'Wrong simulator revision: {sha}')
dest = a.workspace.resolve() / 'src' / 'unitree_go2_ros2'
if dest.exists():
    raise SystemExit(f'Refusing to overwrite existing workspace {dest}')
shutil.copytree(a.source, dest, ignore=shutil.ignore_patterns('.git'))
patches = []


def replace(rel, old, new):
    path = dest / rel
    before = path.read_text()
    if before.count(old) != 1:
        raise RuntimeError(f'Patch context must occur exactly once: {rel}: {old!r}')
    after = before.replace(old, new)
    path.write_text(after)
    patches.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                      fromfile='a/' + rel, tofile='b/' + rel))


launch = 'unitree_go2_sim/launch/unitree_go2_launch.py'
replace(launch, "'gz_args': [LaunchConfiguration('world'), ' -r']",
        "'gz_args': [LaunchConfiguration('world'), ' -r -s --headless-rendering']")
# AMCL must be the only map->odom owner once the navigation adapter is started.
replace(launch, '            map_to_odom_tf_node,',
        '            # map_to_odom_tf_node omitted: AMCL owns this transform.')
# Keep acquisition-time semantics; future-dating an old state is not prediction.
for file, offset in [('base_to_footprint.yaml', '0.045'), ('footprint_to_odom.yaml', '0.050')]:
    replace('champ_base/config/ekf/' + file, 'transform_time_offset: ' + offset, 'transform_time_offset: 0.0')
# Make the actuator-side angular cap match the repository baseline. Same for all arms.
replace('unitree_go2_sim/config/gait/gait.yaml', 'max_angular_velocity_z : 0.5', 'max_angular_velocity_z : 0.6')
# This plural dependency is not a package in the pinned simulator tree.
replace('unitree_go2_sim/package.xml', '<exec_depend>unitree_applications</exec_depend>',
        '<exec_depend>unitree_application</exec_depend>')
a.patch_out.parent.mkdir(parents=True, exist_ok=True)
a.patch_out.write_text(''.join(patches))
(a.patch_out.parent / 'simulator_provenance.json').write_text(json.dumps({
    'repository': 'https://github.com/khaledgabr77/unitree_go2_ros2',
    'source_commit': PIN, 'copy': str(dest), 'patch': a.patch_out.name,
    'status': 'PREPARED_NOT_RUNTIME_VALIDATED',
    'replaced': ['WebRTC transport', 'physical Go2 gait controller', 'physical sensor/odometry feeds'],
    'replacement': ['CHAMP gait + ros2_control', 'Gazebo articulated Go2', 'simulated sensors + CHAMP EKFs'],
    'not_claimed': ['Unitree onboard gait equivalence', 'real robot validation', 'Nav2 execution']
}, indent=2) + '\n')
print(dest)
