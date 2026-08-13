#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
from datetime import datetime
import shutil
import yaml

WELCOME = (
    "Welcome to the San Jose State University Digital Twin Lab. I am Sparky, a Unitree Go2 robot guide. "
    "This lab brings together digital twins, robotics, simulation, perception, localization, navigation, embodied AI, "
    "and sim-to-real experimentation. I will guide you through three stops."
)
NOVA = (
    "This is NOVA, our Unitree G1 humanoid robot. NOVA is used to explore humanoid robotics, embodied AI, "
    "teleoperation, mobility, and sim-to-real workflows."
)
XARM = (
    "These are our xArm robotic arms. We use them for robotic manipulation, teleoperation, perception-guided "
    "interaction, and digital-twin and sim-to-real experiments."
)


def resolve_session(root: Path, name: str) -> Path:
    if name and name.lower() not in {'auto', 'latest'}:
        p = root / name
        if not p.is_dir():
            raise SystemExit(f'Session not found: {p}')
        return p
    candidates = [p for p in root.iterdir() if p.is_dir() and (p / 'route.yaml').is_file() and (p / 'map.yaml').is_file()]
    if not candidates:
        raise SystemExit(f'No session with route.yaml + map.yaml under {root}')
    return max(candidates, key=lambda p: p.stat().st_mtime)


def stop_text(stop):
    return (str(stop.get('name', '')) + ' ' + str(stop.get('place_name', ''))).lower()


def assign_roles(stops):
    roles = [None] * len(stops)
    used = set()
    for i, stop in enumerate(stops[:3]):
        txt = stop_text(stop)
        if ('nova' in txt or 'g1' in txt) and 'nova' not in used:
            roles[i] = 'nova'; used.add('nova')
        elif ('xarm' in txt or 'x arm' in txt or 'robotic_arm' in txt or 'robotic arm' in txt) and 'xarm' not in used:
            roles[i] = 'xarm'; used.add('xarm')
    for role in ('welcome', 'nova', 'xarm'):
        if role in used:
            continue
        for i in range(min(3, len(stops))):
            if roles[i] is None:
                roles[i] = role; used.add(role); break
    return roles


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--session-root', default=os.path.expanduser('~/.ros/go2_semantic_nav_sessions'))
    ap.add_argument('--session-name', default=os.environ.get('SESSION_NAME', 'latest'))
    ap.add_argument('--clear-tour-memory', action='store_true')
    args = ap.parse_args()

    root = Path(os.path.expanduser(args.session_root))
    session = resolve_session(root, args.session_name)
    route_path = session / 'route.yaml'
    payload = yaml.safe_load(route_path.read_text()) or {}
    route = payload.get('route') if isinstance(payload.get('route'), dict) else payload
    stops = list(route.get('stops') or [])
    if len(stops) < 3:
        raise SystemExit(f'Expected the existing 3 tour stops; found only {len(stops)} in {route_path}')

    backup = route_path.with_name(f'route.yaml.bak_guest_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    shutil.copy2(route_path, backup)

    roles = assign_roles(stops)
    scripts = {'welcome': WELCOME, 'nova': NOVA, 'xarm': XARM}
    facts = {
        'welcome': 'The Digital Twin Lab connects robotics, simulation, autonomy, perception, and sim-to-real research.',
        'nova': 'NOVA is a Unitree G1 humanoid robot.',
        'xarm': 'The xArm systems are robotic arms used for manipulation experiments.',
    }
    tags = {
        'welcome': ['digital_twin_lab', 'guest_tour'],
        'nova': ['nova', 'unitree_g1', 'humanoid'],
        'xarm': ['xarm', 'robotic_arm', 'manipulation'],
    }

    for i, stop in enumerate(stops):
        stop['status'] = 'pending'
        if i < 3:
            role = roles[i]
            stop['script'] = scripts[role]
            stop['fact'] = facts[role]
            stop['pause_seconds'] = 5.0
            existing_tags = [str(x) for x in (stop.get('tags') or [])]
            stop['tags'] = list(dict.fromkeys(existing_tags + tags[role]))

    route['mode'] = 'tour'
    route['state'] = 'idle'
    route['current_stop_index'] = 0
    route['last_failure'] = {}
    route['guest_prompt'] = WELCOME
    route['facts'] = [facts['welcome'], facts['nova'], facts['xarm']]
    payload = {'route': route}
    route_path.write_text(yaml.safe_dump(payload, sort_keys=False))

    if args.clear_tour_memory:
        events = session / 'route_events.jsonl'
        events.write_text('')
        print(f'Cleared tour event memory only: {events}')

    print(f'Configured guest tour: {route_path}')
    print(f'Backup: {backup}')
    for i, stop in enumerate(stops[:3], 1):
        print(f'  stop{i}: {stop.get("name")} place={stop.get("place_name")} role={roles[i-1]}')
    print('Preserved: map files, places.yaml coordinates, object_mapper.sqlite3, VLM checkpoints, object/spatial memory.')


if __name__ == '__main__':
    main()
