from go2_semantic_voxel_memory.voxel_store import SemanticVoxelStore


def test_voxel_store_aggregates_and_queries(tmp_path):
    store = SemanticVoxelStore(str(tmp_path), 's', voxel_size_m=0.5)
    a = store.write_observation({'x': 1.0, 'y': 2.0, 'z': 0.1}, ['doorway'], {'occupancy_probability': 0.2, 'traversability_score': 0.8})
    b = store.write_observation({'x': 1.1, 'y': 2.1, 'z': 0.1}, ['sign'], {'occupancy_probability': 0.4, 'traversability_score': 0.6})
    assert a['voxel_id'] == b['voxel_id']
    voxel = store.get_voxel(a['voxel_id'])
    assert voxel['seen_count'] == 2
    assert set(voxel['semantic_labels']) == {'doorway', 'sign'}
    assert store.query_landmarks(1.0, 2.0, radius_m=1.0)
    trav = store.get_traversability(1.0, 2.0)
    assert trav['known'] is True


def test_voxel_store_pointcloud_compare(tmp_path):
    store = SemanticVoxelStore(str(tmp_path), 's', voxel_size_m=1.0)
    store.write_pointcloud_observation([(0.1, 0.1, 0.0), (1.2, 0.0, 0.0)], labels=['wall'])
    result = store.compare_pointcloud([(0.2, 0.2, 0.0), (8.0, 8.0, 0.0)])
    assert result['matched_voxel_count'] == 1
    assert result['novel_voxel_count'] == 1
