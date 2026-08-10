from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("annotated_image_topic", default_value="/object_explorer/annotated_image"),
            DeclareLaunchArgument("sam2_detections_topic", default_value="/object_explorer/sam2_detections"),

            DeclareLaunchArgument("yolo_model", default_value="yolov8n.pt"),
            DeclareLaunchArgument("yolo_conf", default_value="0.35"),
            DeclareLaunchArgument("max_detections", default_value="8"),
            DeclareLaunchArgument("target_classes", default_value="person,chair,dining table,tv,laptop,backpack,bottle,cup,book"),

            DeclareLaunchArgument("enable_sam2", default_value="true"),
            DeclareLaunchArgument("sam2_model", default_value="sam2_t.pt"),
            DeclareLaunchArgument("sam2_imgsz", default_value="512"),

            DeclareLaunchArgument("inference_period_sec", default_value="0.70"),
            DeclareLaunchArgument("track_iou_threshold", default_value="0.25"),
            DeclareLaunchArgument("track_ttl_sec", default_value="3.0"),

            Node(
                package="go2_object_explorer",
                executable="sam2_tracker_overlay_node",
                name="sam2_tracker_overlay_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                        "sam2_detections_topic": LaunchConfiguration("sam2_detections_topic"),
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "yolo_conf": LaunchConfiguration("yolo_conf"),
                        "max_detections": LaunchConfiguration("max_detections"),
                        "target_classes": LaunchConfiguration("target_classes"),
                        "enable_sam2": LaunchConfiguration("enable_sam2"),
                        "sam2_model": LaunchConfiguration("sam2_model"),
                        "sam2_imgsz": LaunchConfiguration("sam2_imgsz"),
                        "inference_period_sec": LaunchConfiguration("inference_period_sec"),
                        "track_iou_threshold": LaunchConfiguration("track_iou_threshold"),
                        "track_ttl_sec": LaunchConfiguration("track_ttl_sec"),
                    }
                ],
            ),
        ]
    )
