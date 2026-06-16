import pytest
nav_msgs = pytest.importorskip("nav_msgs.msg")
OccupancyGrid = nav_msgs.OccupancyGrid

from go2_nav_tools.frontier_explorer import frontier_candidates
from go2_nav_tools.coverage_explorer import coverage_waypoints_from_map


def _map():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = 6
    msg.info.height = 6
    msg.info.resolution = 1.0
    # free block next to unknown cells creates frontiers
    msg.data = [
        100, 100, 100, 100, 100, 100,
        100,   0,   0,  -1,  -1, 100,
        100,   0,   0,  -1,  -1, 100,
        100,   0,   0,   0,  -1, 100,
        100, 100,   0,   0,   0, 100,
        100, 100, 100, 100, 100, 100,
    ]
    return msg


def test_frontier_candidates_find_goal():
    result = frontier_candidates(_map(), (1.0, 1.0), max_candidates=5, min_cluster_cells=1)
    assert result['candidate_count'] >= 1
    assert result['candidates'][0]['pose']['frame_id'] == 'map'


def test_coverage_waypoints_from_map():
    result = coverage_waypoints_from_map(_map(), (1.0, 1.0), step_m=1.0, max_waypoints=10, inflation_cells=0)
    assert result['waypoints']
