"""Replay one explicitly selected bag with a freshness-gated tag path in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    bag_path = Path(LaunchConfiguration('bag_path').perform(context)).expanduser()
    if not bag_path.is_dir() or not (bag_path / 'metadata.yaml').is_file():
        raise RuntimeError(f'bag_path must contain metadata.yaml: {bag_path}')

    try:
        rate = float(LaunchConfiguration('playback_rate').perform(context))
    except ValueError as error:
        raise RuntimeError('playback_rate must be numeric') from error
    if rate <= 0.0:
        raise RuntimeError('playback_rate must be positive')

    command = ['ros2', 'bag', 'play', str(bag_path.resolve()), '--clock', '--rate', str(rate)]
    if LaunchConfiguration('loop').perform(context).lower() in ('1', 'true', 'yes'):
        command.append('--loop')

    return [
        ExecuteProcess(cmd=command, output='screen'),
        Node(
            package='lb_localization',
            executable='tag_replay_visualizer.py',
            name='tag_replay_visualizer',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'source_frame': LaunchConfiguration('source_frame'),
                    'observed_tag_frame': LaunchConfiguration('observed_tag_frame'),
                    'stale_timeout_s': LaunchConfiguration('stale_timeout_s'),
                    'max_path_poses': LaunchConfiguration('max_path_poses'),
                }
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    default_rviz = str(
        Path(get_package_share_directory('lb_navigation')) / 'rviz' / 'apriltag_replay.rviz'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bag_path', description='Explicit path to a rosbag2 directory'
            ),
            DeclareLaunchArgument('playback_rate', default_value='1.0'),
            DeclareLaunchArgument('loop', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz),
            DeclareLaunchArgument('source_frame', default_value='webcam_optical_frame'),
            DeclareLaunchArgument(
                'observed_tag_frame', default_value='webcam_observation_tag_1'
            ),
            DeclareLaunchArgument('stale_timeout_s', default_value='0.5'),
            DeclareLaunchArgument('max_path_poses', default_value='500'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
