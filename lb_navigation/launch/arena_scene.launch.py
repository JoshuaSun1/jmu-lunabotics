"""Launch the durable, visualization-only arena scene markers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('lb_navigation'))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=str(package_share / 'config' / 'arena_scene.yaml'),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='lb_navigation',
                executable='arena_scene_markers.py',
                name='arena_scene_markers',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
