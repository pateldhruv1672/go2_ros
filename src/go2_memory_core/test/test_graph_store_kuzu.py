from pathlib import Path

from go2_memory_core.backends.graph_store_kuzu import GraphStoreKuzu


def test_graph_store_jsonl_route(tmp_path: Path):
    store = GraphStoreKuzu(tmp_path / 'session', enable_kuzu=False)
    store.add_node({'id': 'spawn', 'type': 'Spawn', 'session_name': 's', 'label': 'spawn', 'layer': 'permanent', 'properties': {}})
    store.add_node({'id': 'lab', 'type': 'Place', 'session_name': 's', 'label': 'lab', 'layer': 'permanent', 'properties': {}})
    store.add_edge({'id': 'e1', 'type': 'CONNECTED_TO', 'session_name': 's', 'from_id': 'spawn', 'to_id': 'lab', 'properties': {'distance_m': 2.0}})
    route = store.route('spawn', 'lab')
    assert route['success'] is True
    assert route['node_ids'] == ['spawn', 'lab']
    assert store.query({'route': True, 'start_id': 'spawn', 'goal_id': 'lab'})['route']['success'] is True


def test_graph_store_kuzu_validation_is_structured(tmp_path: Path):
    store = GraphStoreKuzu(tmp_path / 'session', enable_kuzu=True)
    result = store.validate_persistence()
    assert 'success' in result
    assert 'kuzu_available' in result
