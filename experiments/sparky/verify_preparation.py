#!/usr/bin/env python3
"""Static/configuration checks only. Does not test ROS, gait or navigation."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml
from export_config import export, PIN

repo = Path(sys.argv[1]).resolve()
sim = Path(sys.argv[2]).resolve()
checks = []
with tempfile.TemporaryDirectory() as d:
    a, b = Path(d) / 'a', Path(d) / 'b'
    export(repo, a)
    old = os.environ.get('GO2_NAV_MAX_X')
    os.environ['GO2_NAV_MAX_X'] = '99'
    try:
        export(repo, b)
    finally:
        if old is None:
            os.environ.pop('GO2_NAV_MAX_X', None)
        else:
            os.environ['GO2_NAV_MAX_X'] = old
    assert (a/'resume_original_defaults.yaml').read_bytes() == (b/'resume_original_defaults.yaml').read_bytes()
    checks.append('source defaults export is isolated from GO2_* environment overrides')
    data = yaml.safe_load((a/'resume_original_defaults.yaml').read_text())
    ctrl = data['controller_server']['ros__parameters']
    assert ctrl['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert ctrl['FollowPath']['max_vel_x'] == 0.30
    assert ctrl['FollowPath']['max_vel_theta'] == 0.60
    assert ctrl['controller_frequency'] == 10.0
    assert data['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == 'odom'
    assert data['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    checks.append('effective source defaults are NavFn+DWB at 10 Hz, 0.30 m/s and 0.60 rad/s')
    candidate = yaml.safe_load((a/'navfn_dwb_sim_candidate.yaml').read_text())
    def walk(obj):
        if isinstance(obj, dict):
            for k,v in obj.items():
                if k == 'ros__parameters':
                    assert v['use_sim_time'] is True
                walk(v)
    walk(candidate)
    checks.append('candidate simulation config uses sim time throughout')
launch = sim/'src/unitree_go2_ros2/unitree_go2_sim/launch/unitree_go2_launch.py'
ast.parse(launch.read_text())
assert ' -r -s --headless-rendering' in launch.read_text()
assert '\n            map_to_odom_tf_node,' not in launch.read_text()
checks.append('candidate simulator launch parses, uses headless flags and omits static map->odom action')
assert subprocess.check_output(['git','-C',str(repo),'status','--porcelain']) == b''
assert subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip() == PIN
checks.append('original teach_repair checkout remains unchanged')
print(json.dumps({'type':'STATIC_PREPARATION_CHECKS_NOT_SIMULATION', 'checks':checks,
                  'passed':len(checks), 'navigation_runs':0},indent=2))
