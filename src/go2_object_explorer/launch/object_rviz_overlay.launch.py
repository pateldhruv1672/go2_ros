from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("detections_topic", default_value="/object_explorer/detections"),
            DeclareLaunchArgument("memory_topic", default_value="/object_explorer/memory"),
            DeclareLaunchArgument("plan_topic", default_value="/plan"),
            DeclareLaunchArgument("annotated_image_topic", default_value="/object_explorer/annotated_image"),
            DeclareLaunchArgument("marker_topic", default_value="/object_explorer/detection_markers"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            Node(
                package="go2_object_explorer",
                executable="object_rviz_overlay_node",
                name="object_rviz_overlay_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "memory_topic": LaunchConfiguration("memory_topic"),
                        "plan_topic": LaunchConfiguration("plan_topic"),
                        "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                    }
                ],
            ),
        ]
    )
