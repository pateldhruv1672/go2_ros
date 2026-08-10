from __future__ import annotations

from pathlib import Path
from typing import Any, Dict
import json

from .backends.artifact_store_files import ArtifactStoreFiles
from .backends.graph_store_kuzu import GraphStoreKuzu
from .backends.vector_store_local import VectorStoreLocal
from .backends.voxel_store_adapter import VoxelStoreAdapter
from .legacy_import_export import export_legacy_files, import_legacy_summary
from .memory_conflict_detector import detect_name_conflicts
from .memory_promotion import promote_record
from .world_memory import WorldMemoryStore
from .memory_schema import GraphEdge, GraphNode, MemoryRecord, new_id, normalize_json_payload, pose_from_dict, utc_now


class UnifiedMemoryAPI:
    def __init__(
        self,
        session_root: str | Path = '~/.ros/go2_semantic_nav_sessions',
        enable_graph_memory: bool = False,
        enable_voxel_memory: bool = False,
        enable_vector_memory: bool = False,
        enable_legacy_exports: bool = True,
    ):
        self.store = ArtifactStoreFiles(session_root)
        self.enable_graph_memory = enable_graph_memory
        self.enable_voxel_memory = enable_voxel_memory
        self.enable_vector_memory = enable_vector_memory
        self.enable_legacy_exports = enable_legacy_exports
        self._graph_cache: Dict[str, GraphStoreKuzu] = {}
        self._vector_cache: Dict[str, VectorStoreLocal] = {}
        self._voxel_cache: Dict[str, VoxelStoreAdapter] = {}
        self.world = WorldMemoryStore(self.store, self._graph, enable_graph_memory=self.enable_graph_memory)

    def write_spawn(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        spawn_id = data.get('spawn_id') or data.get('id') or new_id('spawn')
        data['map_pose'] = pose_from_dict(data.get('map_pose') or data.get('pose') or data, 'map')
        record = MemoryRecord(
            id=spawn_id,
            type='Spawn',
            session_name=session_name,
            layer='permanent',
            confidence=data.get('confidence', {}),
            source=data.get('source', ['user', 'teach']),
            data=data,
        ).to_dict()
        self.store.write_json(session_name, 'memory/spawn.json', record)
        self.store.write_yaml(session_name, 'spawn.yaml', record)
        self._graph(session_name).add_node(GraphNode(
            id=spawn_id,
            type='Spawn',
            session_name=session_name,
            label=data.get('label', 'spawn'),
            layer='permanent',
            map_pose=data['map_pose'],
            properties=data,
        ).to_dict())
        self._maybe_export(session_name)
        return {'success': True, 'id': spawn_id, 'record': record}

    def write_checkpoint(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        checkpoint_id = data.get('checkpoint_id') or data.get('id') or new_id('ckpt')
        layer = data.get('layer', 'temporary')
        if 'map_pose' in data:
            data['map_pose'] = pose_from_dict(data['map_pose'], 'map')
        if 'odom_pose' in data:
            data['odom_pose'] = pose_from_dict(data['odom_pose'], 'odom')
        record = MemoryRecord(
            id=checkpoint_id,
            type='Checkpoint',
            session_name=session_name,
            layer=layer,
            confidence=data.get('confidence', {}),
            source=data.get('source', ['teach']),
            data=data,
        ).to_dict()
        self.store.append_jsonl(session_name, 'memory/checkpoints.jsonl', record)
        graph = self._graph(session_name)
        graph.add_node(GraphNode(
            id=checkpoint_id,
            type='Checkpoint',
            session_name=session_name,
            label=data.get('label', checkpoint_id),
            layer=layer,
            map_pose=data.get('map_pose'),
            properties=data,
        ).to_dict())
        previous = self._previous_checkpoint(session_name, checkpoint_id)
        if previous:
            self.connect_locations(session_name, previous['id'], checkpoint_id, {
                'type': 'CONNECTED_TO',
                'distance_m': data.get('distance_from_previous_m', 0.0),
                'travel_time_sec': data.get('travel_time_from_previous_sec', 0.0),
                'traversability_score': data.get('traversability_score', 0.5),
                'nav2_success_count': 0,
                'nav2_failure_count': 0,
                'preferred': False,
                'blocked': False,
            })
        if self.enable_vector_memory and data.get('vlm_summary'):
            self._vector(session_name).add_text(checkpoint_id, data['vlm_summary'], {'type': 'Checkpoint'})
        if self.enable_voxel_memory and data.get('semantic_voxel_observation'):
            self._voxel(session_name).write_observation(data['semantic_voxel_observation'])
        self._maybe_export(session_name)
        return {'success': True, 'id': checkpoint_id, 'record': record}

    def write_place(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        place_id = data.get('place_id') or data.get('id') or new_id('place')
        layer = data.get('layer', 'permanent' if data.get('verified') else 'temporary')
        if 'map_pose' in data or 'pose' in data:
            data['map_pose'] = pose_from_dict(data.get('map_pose') or data.get('pose'), 'map')
        record = MemoryRecord(
            id=place_id,
            type='Place',
            session_name=session_name,
            layer=layer,
            confidence=data.get('confidence', {}),
            source=data.get('source', ['user']),
            data=data,
        ).to_dict()
        conflicts = detect_name_conflicts(record, self.store.read_jsonl(session_name, 'memory/places.jsonl'))
        record['data']['name_conflicts'] = conflicts
        self.store.append_jsonl(session_name, 'memory/places.jsonl', record)
        self._graph(session_name).add_node(GraphNode(
            id=place_id,
            type='Place',
            session_name=session_name,
            label=data.get('name', place_id),
            layer=layer,
            map_pose=data.get('map_pose'),
            properties=data,
        ).to_dict())
        if self.enable_vector_memory:
            text = ' '.join(str(x) for x in [data.get('name', ''), data.get('description', ''), ' '.join(data.get('aliases', []))])
            self._vector(session_name).add_text(place_id, text, {'type': 'Place'})
        self._maybe_export(session_name)
        return {'success': True, 'id': place_id, 'record': record, 'conflicts': conflicts}

    def connect_locations(self, session_name: str, from_id: str, to_id: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        edge_id = data.get('edge_id') or data.get('id') or new_id('edge')
        edge_type = data.get('type', 'CONNECTED_TO')
        edge = GraphEdge(
            id=edge_id,
            type=edge_type,
            session_name=session_name,
            from_id=from_id,
            to_id=to_id,
            properties=data,
        ).to_dict()
        self._graph(session_name).add_edge(edge)
        return {'success': True, 'id': edge_id, 'edge': edge}

    def query_memory(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        query = normalize_json_payload(payload)
        kind = query.get('type', 'all')
        text = query.get('text', '')
        result: Dict[str, Any] = {'session': session_name, 'query': query}
        if kind in ('all', 'checkpoints'):
            result['checkpoints'] = self.store.read_jsonl(session_name, 'memory/checkpoints.jsonl')[-int(query.get('limit', 20)):]
        if kind in ('all', 'places'):
            result['places'] = self.store.read_jsonl(session_name, 'memory/places.jsonl')[-int(query.get('limit', 20)):]
        if kind in ('all', 'spawn'):
            result['spawn'] = self.store.read_json(session_name, 'memory/spawn.json', default={})
        if text and self.enable_vector_memory:
            result['vector_hits'] = self._vector(session_name).search(text, int(query.get('limit', 5)))
        return result

    def query_graph(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        query = normalize_json_payload(payload)
        graph = self._graph(session_name)
        if query.get('validate_persistence'):
            return graph.validate_persistence()
        return graph.query(query)

    def promote_memory(self, session_name: str, memory_id: str, reason: str = '') -> Dict[str, Any]:
        promoted_any = False
        for rel in ['memory/checkpoints.jsonl', 'memory/places.jsonl']:
            records = self.store.read_jsonl(session_name, rel)
            updated = []
            for record in records:
                if record.get('id') == memory_id:
                    record = promote_record(record, reason)
                    promoted_any = True
                updated.append(record)
            if promoted_any:
                path = self.store.session_dir(session_name) / rel
                path.write_text(''.join(json.dumps(r, sort_keys=True) + '\n' for r in updated), encoding='utf-8')
        self._maybe_export(session_name)
        return {'success': promoted_any, 'memory_id': memory_id, 'reason': reason}

    def forget_memory(self, session_name: str, memory_id: str, reason: str = '') -> Dict[str, Any]:
        tombstone = {'id': memory_id, 'reason': reason, 'timestamp': utc_now(), 'action': 'forget'}
        self.store.append_jsonl(session_name, 'memory/forgotten.jsonl', tombstone)
        return {'success': True, 'memory_id': memory_id}

    def get_resume_context(self, session_name: str, payload: str | Dict[str, Any] | None = None) -> Dict[str, Any]:
        query = normalize_json_payload(payload)
        session_dir = self.store.session_dir(session_name)
        spawn = self.store.read_json(session_name, 'memory/spawn.json', default={}) or {}
        if not spawn and (session_dir / 'spawn.yaml').exists():
            spawn = {'legacy_spawn_yaml': str(session_dir / 'spawn.yaml')}
        graph = self.query_graph(session_name, {'node_type': query.get('node_type')})
        latest_checkpoint = self.store.latest_record(session_name, 'memory/checkpoints.jsonl')
        return {
            'session_name': session_name,
            'session_dir': str(session_dir),
            'map_yaml_path': str(session_dir / 'map.yaml') if (session_dir / 'map.yaml').exists() else '',
            'map_pgm_path': str(session_dir / 'map.pgm') if (session_dir / 'map.pgm').exists() else '',
            'spawn': spawn,
            'latest_checkpoint': latest_checkpoint,
            'legacy': import_legacy_summary(self.store, session_name),
            'graph_summary': {
                'node_count': len(graph.get('nodes', [])),
                'edge_count': len(graph.get('edges', [])),
            },
        }

    def write_room(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        return self.world.write_room(session_name, payload)

    def write_object_observation(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        return self.world.write_object_observation(session_name, payload)

    def upsert_object_instance(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        return self.world.upsert_object(session_name, payload)

    def query_objects(self, session_name: str, label: str = "", room: str = "", confirmed_only: bool = True, limit: int = 100) -> Dict[str, Any]:
        return self.world.query_objects(session_name, label=label, room=room, confirmed_only=confirmed_only, limit=limit)

    def count_objects(self, session_name: str, label: str = "", room: str = "", confirmed_only: bool = True) -> Dict[str, Any]:
        return self.world.count_objects(session_name, label=label, room=room, confirmed_only=confirmed_only)

    def write_fact(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        return self.world.write_fact(session_name, payload)

    def write_tour_stop(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        return self.world.write_tour_stop(session_name, payload)

    def world_snapshot(self, session_name: str, limit: int = 100) -> Dict[str, Any]:
        return self.world.snapshot(session_name, limit=limit)

    def cache_web_result(self, session_name: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.world.cache_web_result(session_name, payload)

    def _previous_checkpoint(self, session_name: str, current_id: str) -> Dict[str, Any] | None:
        checkpoints = [c for c in self.store.read_jsonl(session_name, 'memory/checkpoints.jsonl') if c.get('id') != current_id]
        return checkpoints[-1] if checkpoints else None

    def _graph(self, session_name: str) -> GraphStoreKuzu:
        if session_name not in self._graph_cache:
            self._graph_cache[session_name] = GraphStoreKuzu(self.store.session_dir(session_name), enable_kuzu=self.enable_graph_memory)
        return self._graph_cache[session_name]

    def _vector(self, session_name: str) -> VectorStoreLocal:
        if session_name not in self._vector_cache:
            self._vector_cache[session_name] = VectorStoreLocal(self.store.session_dir(session_name))
        return self._vector_cache[session_name]

    def _voxel(self, session_name: str) -> VoxelStoreAdapter:
        if session_name not in self._voxel_cache:
            self._voxel_cache[session_name] = VoxelStoreAdapter(self.store.session_dir(session_name))
        return self._voxel_cache[session_name]

    def _maybe_export(self, session_name: str) -> None:
        if self.enable_legacy_exports:
            export_legacy_files(self.store, session_name)
