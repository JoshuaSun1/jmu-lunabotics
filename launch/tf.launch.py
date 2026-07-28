from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_tf_defaults():
    package_share = Path(get_package_share_directory("jmu_lunabotics"))
    config_path = package_share / "config" / "robot_defaults.yaml"
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["frames"], config["mounts"]["camera"]


def generate_launch_description() -> LaunchDescription:
    frames, camera_mount = _load_tf_defaults()
    return LaunchDescription(
        [
            Node(
                package="jmu_lunabotics",
                executable="tf_backbone_node.py",
                name="tf_backbone",
                output="screen",
                parameters=[
                    {
                        "map_frame": frames["map"],
                        "odom_frame": frames["odom"],
                        "base_frame": frames["base_link"],
                        "camera_parent_frame": camera_mount["parent"],
                        "camera_frame": camera_mount["child"],
                        "camera_xyz": camera_mount["xyz"],
                        "camera_rpy": camera_mount["rpy"],
                    }
                ],
            )
        ]
    )
