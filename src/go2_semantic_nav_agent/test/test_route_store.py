from __future__ import annotations

from pathlib import Path

from go2_agentic_system.patrol_events import PatrolEvent
from go2_semantic_nav_agent.route_store import RoutePlan, RouteStop, RouteStore, slugify
from go2_agentic_system.storage import MemoryStore


def test_route_store_persists_route_state(tmp_path):
    store = RouteStore(tmp_path)
    route = store.load_or_create(
        name='Sparky Tour Route',
        mode='tour',
        stop_candidates=[
            {
                'place_name': 'alpha_lab',
                'script': 'welcome to the alpha lab',
                'fact': 'Alpha lab is the demo area.',
                'navigation_hint': 'approach the lab entrance and pause',
                'resume_hook': 'resume from the entrance marker',
                'safety_notes': 'watch the doorway clearance',
                'scene_context': 'door, desk, and lab signage',
                'capture_kind': 'place',
                'sample_index': 3,
                'sample_group': 'alpha_sequence',
                'pause_seconds': 2.5,
                'safe_anchor': 'spawn',
                'kind': 'tour',
                'confidence': 0.91,
                'aliases': ['alpha'],
                'tags': ['lab', 'demo'],
            },
        ],
    )

    assert route.name == 'sparky_tour_route'
    assert route.mode == 'tour'
    assert route.stops[0].place_name == 'alpha_lab'
    assert route.stops[0].name == slugify('alpha_lab')
    assert route.stops[0].navigation_hint == 'approach the lab entrance and pause'
    assert route.stops[0].resume_hook == 'resume from the entrance marker'
    assert route.stops[0].sample_index == 3

    store.ensure_safe_anchor(route, 'spawn', 'alpha_lab')
    store.set_state(route, 'touring')
    store.set_last_failure(route, {'reason': 'blocked', 'attempts': 1})
    store.save(route)

    loaded = store.load()
    assert loaded is not None
    assert loaded.safe_anchors['spawn'] == 'alpha_lab'
    assert loaded.state == 'touring'
    assert loaded.last_failure['reason'] == 'blocked'
    assert loaded.stops[0].navigation_hint == 'approach the lab entrance and pause'
    assert loaded.stops[0].resume_hook == 'resume from the entrance marker'


def test_route_store_advances_and_completes(tmp_path):
    store = RouteStore(tmp_path)
    route = RoutePlan(
        name='demo',
        stops=[
            RouteStop(name='one', place_name='one'),
            RouteStop(name='two', place_name='two'),
        ],
    )
    store.save(route)

    assert store.current_stop(route).name == 'one'
    assert store.advance(route).name == 'two'
    assert route.current_stop_index == 1
    assert store.advance(route) is None
    assert route.state == 'complete'


def test_route_events_and_voxel_memory_round_trip(tmp_path):
    store = RouteStore(tmp_path)
    event = PatrolEvent(
        location={'x': 1.0, 'y': 2.0},
        confidence=0.83,
        request='tour_fact_request',
        status='tour_stop',
        completion=0.5,
        event_worthy=True,
        label='alpha_lab',
        replay_variations=['lighting', 'occlusion'],
        update_strength=0.4,
        route_name='sparky_tour_route',
        stop_name='alpha_lab',
        speech='This is the alpha lab.',
        details={'kind': 'tour'},
    )
    store.append_event(event)

    event_lines = Path(tmp_path) / 'route_events.jsonl'
    lines = [line for line in event_lines.read_text(encoding='utf-8').splitlines() if line.strip()]
    assert len(lines) == 1
    assert PatrolEvent.from_dict(event.to_dict()).label == 'alpha_lab'

    memory = MemoryStore(str(Path(tmp_path) / 'memory'))
    memory.add_voxel_snapshot({'map_name': 'sparky_tour_route', 'cells': 42, 'front_clear': True})
    summary = memory.voxel_summary('sparky_tour_route')
    assert summary['count'] == 1
    assert summary['recent'][0]['cells'] == 42


def test_memory_store_resolves_resume_fields(tmp_path):
    memory = MemoryStore(str(Path(tmp_path) / 'memory'))
    memory.remember_place(
        'sparky_tour_route',
        'alpha_lab',
        {
            'x': 1.0,
            'y': 2.0,
            'yaw': 0.5,
            'frame_id': 'map',
            'summary': 'alpha lab demo area',
            'tour_fact': 'This station explains the alpha lab instrumentation.',
            'navigation_hint': 'approach the doorway and stop at the marker',
            'resume_hook': 'resume after the doorway marker',
            'safety_notes': 'keep clear of the swinging door',
            'scene_context': 'doorway, desk, signage',
            'aliases': ['alpha', 'lab'],
            'tags': ['doorway', 'demo'],
        },
    )
    memory.add_observation(
        {
            'id': 'alpha_obs_1',
            'map_name': 'sparky_tour_route',
            'label': 'alpha_lab',
            'summary': 'alpha lab demo area',
            'tour_fact': 'This station explains the alpha lab instrumentation.',
            'navigation_hint': 'approach the doorway and stop at the marker',
            'resume_hook': 'resume after the doorway marker',
            'safety_notes': 'keep clear of the swinging door',
            'scene_context': 'doorway, desk, signage',
            'aliases': ['alpha', 'lab'],
            'objects': ['door', 'desk'],
            'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'frame_id': 'map'},
        }
    )

    resolved = memory.resolve_destination('sparky_tour_route', 'resume after the doorway marker')
    assert resolved is not None
    assert resolved['name'] == 'alpha_lab'
    search = memory.search_memories('doorway marker', 'sparky_tour_route', limit=3)
    assert search
    assert search[0]['name'] == 'alpha_lab'
