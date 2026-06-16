from __future__ import annotations

from collections import defaultdict, deque
from typing import Any, Dict, List


def plan_graph_route(nodes: List[Dict[str, Any]], edges: List[Dict[str, Any]], start_id: str, goal_id: str) -> Dict[str, Any]:
    adjacency = defaultdict(list)
    for edge in edges:
        if edge.get('properties', {}).get('blocked'):
            continue
        adjacency[edge.get('from_id')].append(edge.get('to_id'))
        if edge.get('properties', {}).get('bidirectional', True):
            adjacency[edge.get('to_id')].append(edge.get('from_id'))
    queue = deque([[start_id]])
    seen = {start_id}
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal_id:
            id_to_node = {n.get('id'): n for n in nodes}
            return {'success': True, 'path_ids': path, 'nodes': [id_to_node.get(i, {'id': i}) for i in path]}
        for nxt in adjacency.get(node, []):
            if nxt and nxt not in seen:
                seen.add(nxt)
                queue.append(path + [nxt])
    return {'success': False, 'message': 'no graph route found', 'path_ids': []}
