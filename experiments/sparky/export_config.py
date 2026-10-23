#!/usr/bin/env python3
"""Execute the pinned repository's actual config builder without launching ROS.

This exports configuration, NOT simulation data. No trajectories are generated.
The source functions execute unchanged; only package discovery is supplied here.
"""
import argparse
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

PIN = 'ca39e50ba126a142f8ae7d806f8006ceea1f733a'
LAUNCH = 'src/go2_semantic_nav_agent/launch/semantic_nav_resume.launch.py'
CONFIG = 'src/go2_robot_sdk/config/nav2_params.yaml'
AMCL = 'src/go2_semantic_nav_agent/config/amcl_params.yaml'


def export(repo, out):
    commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != PIN:
        raise RuntimeError(f'Expected {PIN}, found {commit}')
    for rel in [LAUNCH, CONFIG, AMCL]:
        if subprocess.check_output(['git', '-C', str(repo), 'diff', 'HEAD', '--', rel]):
            raise RuntimeError(f'Source modified: {rel}')
    out.mkdir(parents=True, exist_ok=True)
    selected = {'_bool_flag', '_stvl_layer_params', '_resolve_stvl_enabled', '_build_semantic_nav2_params'}
    tree = ast.parse((repo / LAUNCH).read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in selected]
    assert len(functions) == len(selected)
    ns = {'os': os, 'yaml': yaml, 'tempfile': tempfile, '_package_available': lambda name: False}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(repo / LAUNCH), 'exec'), ns)
    previous = {k: v for k, v in os.environ.items() if k.startswith('GO2_')}
    for k in previous:
        del os.environ[k]
    try:
        generated = Path(ns['_build_semantic_nav2_params'](str(repo / CONFIG), '/scan_nav', '/point_cloud2', 'false'))
        original = yaml.safe_load(generated.read_text())
        generated.unlink()
    finally:
        os.environ.update(previous)
    (out / 'resume_original_defaults.yaml').write_text(yaml.safe_dump(original, sort_keys=False))
    sim = copy.deepcopy(original)
    # Resume launches AMCL from this separate file, not the generated Nav2 block.
    sim['amcl'] = yaml.safe_load((repo / AMCL).read_text())['amcl']
    def sim_time(obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k == 'ros__parameters':
                    v['use_sim_time'] = True
                sim_time(v)
    sim_time(sim)
    # Future simulation adapter consumes a direct Gazebo scan. AMCL keeps source stamps.
    sim['amcl']['ros__parameters']['scan_topic'] = '/scan'
    for key in ['local_costmap', 'global_costmap']:
        sim[key][key]['ros__parameters']['obstacle_layer']['scan']['topic'] = '/scan'
    cm = sim['collision_monitor']['ros__parameters']
    cm['observation_sources'] = ['scan']
    cm['scan']['enabled'] = True
    cm['scan']['topic'] = '/scan'
    cm['pointcloud']['enabled'] = False
    (out / 'navfn_dwb_sim_candidate.yaml').write_text(yaml.safe_dump(sim, sort_keys=False))
    manifest = {
        'status': 'CONFIGURATION_EXPORT_ONLY_NOT_A_NAVIGATION_RUN',
        'repository': 'https://github.com/pateldhruv1672/go2_ros',
        'source_commit': commit,
        'function': LAUNCH + ':_build_semantic_nav2_params',
        'source_sha256': {p: hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in [LAUNCH, CONFIG, AMCL]},
        'environment': 'GO2_* cleared for export; source defaults used',
        'package_discovery': 'STVL unavailable; stvl_enabled=false',
        'planner_configured': original['planner_server']['ros__parameters']['GridBased']['plugin'],
        'controller_configured': original['controller_server']['ros__parameters']['FollowPath']['plugin'],
        'controller_executed': None,
        'simulation_changes': ['AMCL taken from the separate amcl_params.yaml used by resume launch', 'use_sim_time=true on all parameter blocks', 'scan topics=/scan', 'collision monitor uses scan instead of pointcloud'],
        'runtime_validated': False,
    }
    (out / 'config_provenance.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('repo', type=Path)
    p.add_argument('--out', type=Path, default=Path('configs'))
    a = p.parse_args()
    print(json.dumps(export(a.repo.resolve(), a.out.resolve()), indent=2))
