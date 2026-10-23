#!/usr/bin/env python3
"""Read-only runtime probe. Exit 2 if dependencies are absent; never fake success."""
import argparse
import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--out', type=Path, default=Path('logs/preflight.json'))
a = p.parse_args()
checks = {}
for cmd in ['ros2', 'gz', 'colcon', 'docker', 'podman', 'gh']:
    checks[cmd] = shutil.which(cmd)
checks['rclpy'] = importlib.util.find_spec('rclpy') is not None
checks['ros_setup'] = Path('/opt/ros/jazzy/setup.bash').exists()
packages = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\t${db:Status-Status}\n',
    'ros-jazzy-*'], text=True, capture_output=True)
result = {'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'os_release': Path('/etc/os-release').read_text(), 'checks': checks,
    'package_query_stdout': packages.stdout, 'package_query_stderr': packages.stderr,
    'github_token_environment_present': any(os.environ.get(k) for k in ['GH_TOKEN', 'GITHUB_TOKEN']),
    'navigation_runs': 0, 'controller_executed': None}
result['dependencies_discoverable'] = bool(checks['ros2'] and checks['gz'] and checks['rclpy'])
result['warning'] = 'Discoverable binaries do not prove package configuration, simulator operation, or message flow.'
a.out.parent.mkdir(parents=True, exist_ok=True)
a.out.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['dependencies_discoverable'] else 2)
