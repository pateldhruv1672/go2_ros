from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_out"),
            DeclareLaunchArgument("yolo_model", default_value="yolov8n.pt"),
            DeclareLaunchArgument("yolo_conf", default_value="0.35"),
            DeclareLaunchArgument("enable_sam2", default_value="false"),
            DeclareLaunchArgument("auto_navigate", default_value="true"),
            DeclareLaunchArgument("search_timeout_sec", default_value="180.0"),
            DeclareLaunchArgument("local_scan_duration_sec", default_value="8.0"),
            DeclareLaunchArgument("frontier_goal_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("frontier_max_distance_m", default_value="8.0"),
            DeclareLaunchArgument("camera_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("use_ollama_frontier_selector", default_value="false"),
            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434"),
            DeclareLaunchArgument("ollama_model", default_value="gemma3:4b"),
            DeclareLaunchArgument("ollama_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument("llm_top_k_frontiers", default_value="8"),
            DeclareLaunchArgument("frontier_camera_map_mode", default_value="fallback_and"),
            DeclareLaunchArgument("visual_frontier_fov_deg", default_value="80.0"),
            DeclareLaunchArgument("visual_frontier_max_range_m", default_value="9.0"),
            DeclareLaunchArgument("visual_sector_memory_sec", default_value="70.0"),
            DeclareLaunchArgument("visual_semantic_radius_m", default_value="2.0"),
            Node(
                package="go2_object_explorer",
                executable="frontier_object_explorer_node",
                name="frontier_object_explorer_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "yolo_conf": LaunchConfiguration("yolo_conf"),
                        "enable_sam2": LaunchConfiguration("enable_sam2"),
                        "auto_navigate": LaunchConfiguration("auto_navigate"),
                        "search_timeout_sec": LaunchConfiguration("search_timeout_sec"),
                        "local_scan_duration_sec": LaunchConfiguration("local_scan_duration_sec"),
                        "frontier_goal_timeout_sec": LaunchConfiguration("frontier_goal_timeout_sec"),
                        "frontier_max_distance_m": LaunchConfiguration("frontier_max_distance_m"),
                        "camera_yaw_offset_rad": LaunchConfiguration("camera_yaw_offset_rad"),
                        "use_ollama_frontier_selector": LaunchConfiguration("use_ollama_frontier_selector"),
                        "ollama_url": LaunchConfiguration("ollama_url"),
                        "ollama_model": LaunchConfiguration("ollama_model"),
                        "ollama_timeout_sec": LaunchConfiguration("ollama_timeout_sec"),
                        "llm_top_k_frontiers": LaunchConfiguration("llm_top_k_frontiers"),
                        "frontier_camera_map_mode": LaunchConfiguration("frontier_camera_map_mode"),
                        "visual_frontier_fov_deg": LaunchConfiguration("visual_frontier_fov_deg"),
                        "visual_frontier_max_range_m": LaunchConfiguration("visual_frontier_max_range_m"),
                        "visual_sector_memory_sec": LaunchConfiguration("visual_sector_memory_sec"),
                        "visual_semantic_radius_m": LaunchConfiguration("visual_semantic_radius_m"),
                    }
                ],
            ),
        ]
    )
