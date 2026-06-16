from go2_memory_core.memory_api import UnifiedMemoryAPI


def test_memory_api_writes_spawn_checkpoint_place(tmp_path):
    api = UnifiedMemoryAPI(session_root=tmp_path, enable_graph_memory=True, enable_vector_memory=True, enable_voxel_memory=True)
    session = 'test_session'
    spawn = api.write_spawn(session, {'label': 'spawn', 'map_pose': {'x': 1, 'y': 2}})
    ckpt = api.write_checkpoint(session, {'label': 'hallway', 'map_pose': {'x': 1.2, 'y': 2.1}, 'vlm_summary': 'hallway with doors'})
    place = api.write_place(session, {'name': 'AI Lab Entrance', 'aliases': ['ai lab'], 'verified': True, 'map_pose': {'x': 1.5, 'y': 2.5}})
    context = api.get_resume_context(session, {})
    assert spawn['success']
    assert ckpt['success']
    assert place['success']
    assert context['spawn']['id'] == spawn['id']
    assert context['graph_summary']['node_count'] >= 3
