from go2_langgraph_agent.tools.goal_selection import select_exploration_goal


def test_select_exploration_goal_uses_frontier_and_safety_context():
    context = {
        'odom': {'pose': {'x': 0.0, 'y': 0.0}},
        'scan_summary': {'sector_clearance_m': {'front': 1.5}},
        'traversability': {'traversability_score': 0.9},
        'frontier_candidates': {'candidates': [
            {'id': 'bad', 'pose': {'x': 10.0, 'y': 0.0}, 'information_gain': 0.2},
            {'id': 'good', 'pose': {'x': 2.0, 'y': 0.0}, 'information_gain': 0.9},
        ]},
    }
    result = select_exploration_goal(context, 'frontier')
    assert result['best_goal']['goal_id'] == 'good'
