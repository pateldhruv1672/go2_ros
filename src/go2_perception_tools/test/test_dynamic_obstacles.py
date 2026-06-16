import pytest
sensor_msgs = pytest.importorskip("sensor_msgs.msg")
LaserScan = sensor_msgs.LaserScan

from go2_perception_tools.dynamic_obstacle_tracker import scan_clusters


def test_scan_clusters_extract_centroid():
    msg = LaserScan()
    msg.angle_min = -0.5
    msg.angle_increment = 0.1
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [float('inf'), 1.0, 1.02, 1.01, float('inf'), 3.0, 3.05, 3.1]
    clusters = scan_clusters(msg, min_points=2)
    assert len(clusters) >= 2
