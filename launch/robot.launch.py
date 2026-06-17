from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _load_robot_defaults():
    package_share = Path(get_package_share_directory("jmu_lunabotics"))
    config_path = package_share / "config" / "robot_defaults.yaml"
    with config_path.open("r", encoding="utf-8") as config_file:
        launch_defaults = yaml.safe_load(config_file)["launch"]

    camera_info_file = str(launch_defaults.get("camera_info_file", "")).strip()
    if camera_info_file and not Path(camera_info_file).is_absolute():
        launch_defaults["camera_info_file"] = str(package_share / camera_info_file)

    return launch_defaults


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("jmu_lunabotics"))
    defaults = _load_robot_defaults()
    sensors_launch = str(package_share / "launch" / "sensors.launch.py")
    rviz_launch = str(package_share / "launch" / "rviz.launch.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value=str(defaults["enable_camera"]).lower(),
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value=str(defaults["enable_apriltag"]).lower(),
            ),
            DeclareLaunchArgument(
                "enable_view_image_raw",
                default_value=str(defaults["enable_view_image_raw"]).lower(),
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value=str(defaults["enable_rviz"]).lower(),
            ),
            DeclareLaunchArgument(
                "webcam_driver",
                default_value=str(defaults["webcam_driver"]),
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value=str(defaults["video_device"]),
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value=str(defaults["camera_name"]),
            ),
            DeclareLaunchArgument("frame_id", default_value=str(defaults["frame_id"])),
            DeclareLaunchArgument(
                "camera_info_file",
                default_value=str(defaults["camera_info_file"]),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value=str(defaults["image_topic"]),
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value=str(defaults["camera_info_topic"]),
            ),
            DeclareLaunchArgument(
                "rectified_image_topic",
                default_value=str(defaults["rectified_image_topic"]),
            ),
            DeclareLaunchArgument("apriltag_ns", default_value=str(defaults["apriltag_ns"])),
            DeclareLaunchArgument(
                "apriltag_tag_id",
                default_value=str(defaults["apriltag_tag_id"]),
            ),
            DeclareLaunchArgument(
                "apriltag_tag_family",
                default_value=str(defaults["apriltag_tag_family"]),
            ),
            DeclareLaunchArgument(
                "apriltag_tag_size_meters",
                default_value=str(defaults["apriltag_tag_size_meters"]),
            ),
            DeclareLaunchArgument(
                "apriltag_pose_topic",
                default_value=str(defaults["apriltag_pose_topic"]),
            ),
            DeclareLaunchArgument(
                "apriltag_distance_topic",
                default_value=str(defaults["apriltag_distance_topic"]),
            ),
            DeclareLaunchArgument(
                "apriltag_pose_estimation_method",
                default_value=str(defaults["apriltag_pose_estimation_method"]),
            ),
            DeclareLaunchArgument(
                "apriltag_threads",
                default_value=str(defaults["apriltag_threads"]),
            ),
            DeclareLaunchArgument(
                "apriltag_decimate",
                default_value=str(defaults["apriltag_decimate"]),
            ),
            DeclareLaunchArgument(
                "apriltag_blur",
                default_value=str(defaults["apriltag_blur"]),
            ),
            DeclareLaunchArgument(
                "apriltag_refine",
                default_value=str(defaults["apriltag_refine"]),
            ),
            DeclareLaunchArgument(
                "apriltag_sharpening",
                default_value=str(defaults["apriltag_sharpening"]),
            ),
            DeclareLaunchArgument(
                "apriltag_debug",
                default_value=str(defaults["apriltag_debug"]),
            ),
            DeclareLaunchArgument(
                "apriltag_max_hamming",
                default_value=str(defaults["apriltag_max_hamming"]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    "enable_camera": LaunchConfiguration("enable_camera"),
                    "enable_apriltag": LaunchConfiguration("enable_apriltag"),
                    "enable_view_image_raw": LaunchConfiguration("enable_view_image_raw"),
                    "webcam_driver": LaunchConfiguration("webcam_driver"),
                    "video_device": LaunchConfiguration("video_device"),
                    "camera_name": LaunchConfiguration("camera_name"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "camera_info_file": LaunchConfiguration("camera_info_file"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "rectified_image_topic": LaunchConfiguration("rectified_image_topic"),
                    "apriltag_ns": LaunchConfiguration("apriltag_ns"),
                    "apriltag_tag_id": LaunchConfiguration("apriltag_tag_id"),
                    "apriltag_tag_family": LaunchConfiguration("apriltag_tag_family"),
                    "apriltag_tag_size_meters": LaunchConfiguration(
                        "apriltag_tag_size_meters"
                    ),
                    "apriltag_pose_topic": LaunchConfiguration("apriltag_pose_topic"),
                    "apriltag_distance_topic": LaunchConfiguration(
                        "apriltag_distance_topic"
                    ),
                    "apriltag_pose_estimation_method": LaunchConfiguration(
                        "apriltag_pose_estimation_method"
                    ),
                    "apriltag_threads": LaunchConfiguration("apriltag_threads"),
                    "apriltag_decimate": LaunchConfiguration("apriltag_decimate"),
                    "apriltag_blur": LaunchConfiguration("apriltag_blur"),
                    "apriltag_refine": LaunchConfiguration("apriltag_refine"),
                    "apriltag_sharpening": LaunchConfiguration("apriltag_sharpening"),
                    "apriltag_debug": LaunchConfiguration("apriltag_debug"),
                    "apriltag_max_hamming": LaunchConfiguration("apriltag_max_hamming"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                launch_arguments={
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_frame": LaunchConfiguration("frame_id"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
