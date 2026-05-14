#!/usr/bin/env python3
"""
Test script for Council multi-agent system
Run with: python3 test_council.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import asyncio
import threading
import time
import numpy as np
import cv2

# Add to path
import sys
sys.path.insert(0, '/home/dkp/ros2_ws/install/council/lib/python3.10/site-packages')

from council.config import load_config
from council.ros_interfaces import SensorHub
from council.ros_interfaces.sensor_hub import create_birds_eye_view, filter_robot_body
from council.orchestrator import CouncilOrchestrator, VotingStrategy


def create_sample_bev():
    """Create a sample bird's-eye view with synthetic data to show the visualization"""
    print("[INFO] Creating sample BEV with synthetic obstacle data...")
    
    # Generate synthetic point cloud
    np.random.seed(42)
    points = []
    
    # Obstacles in front (1.5-2m)
    for _ in range(100):
        x = np.random.uniform(1.5, 2.5)
        y = np.random.uniform(-0.5, 0.5)
        z = np.random.uniform(-0.1, 0.3)
        points.append([x, y, z])
    
    # Obstacles on right (1-2m)
    for _ in range(80):
        x = np.random.uniform(0, 1.0)
        y = np.random.uniform(-2.0, -1.2)
        z = np.random.uniform(-0.1, 0.4)
        points.append([x, y, z])
    
    # Clear path on left
    for _ in range(30):
        x = np.random.uniform(1.0, 4.0)
        y = np.random.uniform(1.0, 2.5)
        z = np.random.uniform(-0.05, 0.2)
        points.append([x, y, z])
    
    # Some back obstacles
    for _ in range(50):
        x = np.random.uniform(-3.0, -1.0)
        y = np.random.uniform(-1.5, 1.5)
        z = np.random.uniform(-0.1, 0.3)
        points.append([x, y, z])
    
    points = np.array(points, dtype=np.float32)
    
    # Filter (as the real code would)
    filtered = filter_robot_body(points, min_distance=0.30)
    
    # Create BEV
    bev = create_birds_eye_view(filtered)
    
    # Save sample
    cv2.imwrite('/tmp/council_bev_sample.jpg', bev)
    print(f"[INFO] Sample BEV saved to /tmp/council_bev_sample.jpg")
    print(f"[INFO] Points: {len(points)} raw, {len(filtered)} after filtering")
    
    return bev


def main():
    print("=" * 60)
    print("COUNCIL TEST SCRIPT")
    print("=" * 60)
    
    rclpy.init()
    
    config = load_config()
    print(f"[CONFIG] Loaded config")
    print(f"  - Camera: {config.ros2.camera_topic}")
    print(f"  - LiDAR: {config.ros2.lidar_topic}")
    print(f"  - IMU: {config.ros2.imu_topic}")
    print(f"  - cmd_vel: {config.ros2.cmd_vel_topic}")
    
    # Create sensor hub
    sensor_hub = SensorHub(
        camera_topic=config.ros2.camera_topic,
        lidar_topic=config.ros2.lidar_topic,
        imu_topic=config.ros2.imu_topic,
        odom_topic=config.ros2.odom_topic,
        cmd_vel_topic=config.ros2.cmd_vel_topic,
        debug=True,
    )
    
    # Start spinner
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_hub)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print("[INFO] Waiting for sensor data... (5 seconds)")
    time.sleep(5)
    
    # Check sensor data
    data = sensor_hub.get_all_data()
    ages = sensor_hub.get_data_age()
    
    print("\n[SENSOR DATA STATUS]")
    points = data['lidar'].get('points') if data.get('lidar') else None
    point_count = len(points) if points is not None else 0
    has_camera = data.get('camera', {}).get('image_b64') is not None
    has_lidar = point_count > 0
    has_imu = bool(data.get('imu'))
    
    print(f"  Camera: age={ages['camera']:.2f}s, has_image={has_camera}")
    print(f"  LiDAR: age={ages['lidar']:.2f}s, points={point_count}")
    print(f"  IMU: age={ages['imu']:.2f}s, has_data={has_imu}")
    print(f"  Odom: age={ages['odometry']:.2f}s, has_data={bool(data.get('odometry'))}")
    
    if data.get('imu'):
        print(f"  IMU RPY: {data['imu'].get('rpy', 'N/A')}")
    
    # Check if BEV was created
    if data.get('lidar', {}).get('bev_b64'):
        import base64
        bev_data = base64.b64decode(data['lidar']['bev_b64'])
        with open('/tmp/council_bev_real.jpg', 'wb') as f:
            f.write(bev_data)
        print(f"\n[INFO] Real LiDAR BEV saved to /tmp/council_bev_real.jpg")
        print(f"  Filtered out {data['lidar'].get('stats', {}).get('filtered_out', 0)} points (robot body)")
    
    # If no sensor data, create sample and exit
    if not (has_camera or has_lidar):
        print("\n[WARNING] No sensor data available! Robot SDK may not be running.")
        print("[INFO] Creating sample bird's-eye view visualization...")
        create_sample_bev()
        print("\n[INFO] To run full test, ensure robot SDK is publishing sensor data:")
        print("  ros2 launch go2_robot_sdk robot.launch.py")
        sensor_hub.destroy_node()
        rclpy.shutdown()
        return
    
    # Test orchestrator
    print("\n[ORCHESTRATOR] Creating...")
    orchestrator = CouncilOrchestrator(
        config=config,
        voting_strategy=VotingStrategy.WEIGHTED,
        debug=True,
    )
    
    task = "find a water bottle go near to it and stop"
    print(f"\n[TASK] '{task}'")
    print("[INFO] Running deliberation... (this may take a few seconds)")
    
    # Run async deliberation
    async def run_test():
        try:
            decision = await orchestrator.deliberate(data, task)
            print("\n[DECISION]")
            print(f"  Action: {decision.action}")
            print(f"  Velocities: vx={decision.linear_x:.2f}, vy={decision.linear_y:.2f}, wz={decision.angular_z:.2f}")
            print(f"  Duration: {decision.duration:.2f}s")
            print(f"  Confidence: {decision.confidence:.2f}")
            print(f"  Risk: {decision.risk_level}")
            print(f"  Explanation: {decision.explanation[:100]}")
            print(f"  Agent votes: {decision.agent_votes}")
            return decision
        except Exception as e:
            print(f"\n[ERROR] Deliberation failed: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        decision = loop.run_until_complete(asyncio.wait_for(run_test(), timeout=30))
        
        if decision and decision.action != "stop":
            print("\n[CMD] Would publish velocity command:")
            print(f"  ros2 topic pub --once /cmd_vel_joy geometry_msgs/msg/Twist '{{linear: {{x: {decision.linear_x}, y: {decision.linear_y}}}, angular: {{z: {decision.angular_z}}}}}'")
        
    except asyncio.TimeoutError:
        print("[ERROR] Deliberation timed out after 30 seconds")
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    print("\n[CLEANUP]")
    sensor_hub.destroy_node()
    rclpy.shutdown()
    print("[DONE]")


if __name__ == "__main__":
    main()
