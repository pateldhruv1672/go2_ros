from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("annotated_image_topic", default_value="/object_explorer/annotated_image"),
            DeclareLaunchArgument("detections_topic", default_value="/object_explorer/sam2_detections"),

            DeclareLaunchArgument("device", default_value="cuda:0"),
            DeclareLaunchArgument("half", default_value="true"),

            DeclareLaunchArgument("yolo_model", default_value="yolov8n.pt"),
            DeclareLaunchArgument("yolo_imgsz", default_value="640"),
            DeclareLaunchArgument("yolo_conf", default_value="0.18"),
            DeclareLaunchArgument("max_detections", default_value="20"),
            DeclareLaunchArgument("target_classes", default_value=""),

            DeclareLaunchArgument("enable_sam2", default_value="true"),
            DeclareLaunchArgument("sam2_model", default_value="sam2_t.pt"),
            DeclareLaunchArgument("sam2_imgsz", default_value="512"),
            DeclareLaunchArgument("sam2_every_n", default_value="1"),

            DeclareLaunchArgument("inference_period_sec", default_value="0.20"),
            DeclareLaunchArgument("track_iou_threshold", default_value="0.22"),
            DeclareLaunchArgument("track_ttl_sec", default_value="2.0"),

            Node(
                package="go2_object_explorer",
                executable="fast_sam2_tracker_overlay_node",
                name="fast_sam2_tracker_overlay_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "device": LaunchConfiguration("device"),
                        "half": LaunchConfiguration("half"),
                        "yolo_model": LaunchConfiguration("yolo_model"),
                        "yolo_imgsz": LaunchConfiguration("yolo_imgsz"),
                        "yolo_conf": LaunchConfiguration("yolo_conf"),
                        "max_detections": LaunchConfiguration("max_detections"),
                        "target_classes": LaunchConfiguration("target_classes"),
                        "enable_sam2": LaunchConfiguration("enable_sam2"),
                        "sam2_model": LaunchConfiguration("sam2_model"),
                        "sam2_imgsz": LaunchConfiguration("sam2_imgsz"),
                        "sam2_every_n": LaunchConfiguration("sam2_every_n"),
                        "inference_period_sec": LaunchConfiguration("inference_period_sec"),
                        "track_iou_threshold": LaunchConfiguration("track_iou_threshold"),
                        "track_ttl_sec": LaunchConfiguration("track_ttl_sec"),
                    }
                ],
            ),
        ]
    )
