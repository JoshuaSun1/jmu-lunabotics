"""Replay a tag bag as provisional camera and planar robot poses on the map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _required_file(value: str, label: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f'{label} was not found: {path}')
    return path.resolve()


def _float_argument(context, name: str) -> float:
    value = LaunchConfiguration(name).perform(context)
    try:
        return float(value)
    except ValueError as error:
        raise RuntimeError(f'{name} must be numeric, got {value!r}') from error


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    bag_path = Path(LaunchConfiguration('bag_path').perform(context)).expanduser()
    if not bag_path.is_dir() or not (bag_path / 'metadata.yaml').is_file():
        raise RuntimeError(f'bag_path must contain metadata.yaml: {bag_path}')

    rate = _float_argument(context, 'playback_rate')
    if rate <= 0.0:
        raise RuntimeError('playback_rate must be positive')

    rviz_config = _required_file(
        LaunchConfiguration('rviz_config').perform(context), 'rviz_config'
    )
    map_file = _required_file(LaunchConfiguration('map').perform(context), 'map')
    map_params = _required_file(
        LaunchConfiguration('map_params_file').perform(context), 'map_params_file'
    )

    command = ['ros2', 'bag', 'play', str(bag_path.resolve()), '--clock', '--rate', str(rate)]
    if LaunchConfiguration('loop').perform(context).lower() in ('1', 'true', 'yes'):
        command.append('--loop')

    map_launch = (
        Path(get_package_share_directory('lb_navigation')) / 'launch' / 'map.launch.py'
    )
    scene_launch = (
        Path(get_package_share_directory('lb_navigation'))
        / 'launch'
        / 'arena_scene.launch.py'
    )
    return [
        LogInfo(
            msg=(
                'Development-only replay: robot pose expires on tag loss and uses '
                'an estimated tag pose, camera mounts, and mock chassis geometry. '
                'No odometry, EKF, navigation, or control is started.'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(map_launch)),
            launch_arguments={
                'map': str(map_file),
                'params_file': str(map_params),
                'use_sim_time': 'true',
                'use_rviz': 'false',
                'use_arena_scene': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_map_server')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene_launch)),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        ExecuteProcess(cmd=command, output='screen'),
        Node(
            package='lb_localization',
            executable='tag_replay_robot_pose.py',
            name='tag_replay_robot_pose',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'map_frame': LaunchConfiguration('map_frame'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                    'observed_tag_frame': LaunchConfiguration('observed_tag_frame'),
                    'tag_position_m': [
                        _float_argument(context, 'tag_x_m'),
                        _float_argument(context, 'tag_y_m'),
                        _float_argument(context, 'tag_z_m'),
                    ],
                    'tag_rpy_rad': [
                        _float_argument(context, 'tag_roll_rad'),
                        _float_argument(context, 'tag_pitch_rad'),
                        _float_argument(context, 'tag_yaw_rad'),
                    ],
                    'base_to_camera_position_m': [
                        _float_argument(context, 'camera_x_m'),
                        _float_argument(context, 'camera_y_m'),
                        _float_argument(context, 'camera_z_m'),
                    ],
                    'base_to_camera_rpy_rad': [
                        _float_argument(context, 'camera_roll_rad'),
                        _float_argument(context, 'camera_pitch_rad'),
                        _float_argument(context, 'camera_yaw_rad'),
                    ],
                    'base_to_zed_position_m': [
                        _float_argument(context, 'zed_x_m'),
                        _float_argument(context, 'zed_y_m'),
                        _float_argument(context, 'zed_z_m'),
                    ],
                    'base_to_zed_rpy_rad': [
                        _float_argument(context, 'zed_roll_rad'),
                        _float_argument(context, 'zed_pitch_rad'),
                        _float_argument(context, 'zed_yaw_rad'),
                    ],
                    'chassis_dimensions_m': [
                        _float_argument(context, 'chassis_length_m'),
                        _float_argument(context, 'chassis_width_m'),
                        _float_argument(context, 'chassis_height_m'),
                    ],
                    'stale_timeout_s': _float_argument(context, 'stale_timeout_s'),
                    'max_path_poses': int(
                        _float_argument(context, 'max_path_poses')
                    ),
                }
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='arena_pose_replay_rviz',
            output='screen',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('lb_navigation'))
    half_pi = '1.5707963267948966'
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bag_path', description='Explicit path to a rosbag2 directory'
            ),
            DeclareLaunchArgument('playback_rate', default_value='0.25'),
            DeclareLaunchArgument('loop', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_map_server', default_value='true'),
            DeclareLaunchArgument(
                'map', default_value=str(navigation_share / 'maps' / 'arena.yaml')
            ),
            DeclareLaunchArgument(
                'map_params_file',
                default_value=str(navigation_share / 'config' / 'map_server.yaml'),
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=str(
                    navigation_share / 'rviz' / 'arena_pose_replay.rviz'
                ),
            ),
            DeclareLaunchArgument('map_frame', default_value='map'),
            DeclareLaunchArgument('camera_frame', default_value='webcam_optical_frame'),
            DeclareLaunchArgument(
                'observed_tag_frame', default_value='webcam_observation_tag_1'
            ),
            DeclareLaunchArgument('tag_x_m', default_value='0.0'),
            DeclareLaunchArgument('tag_y_m', default_value='1.0'),
            DeclareLaunchArgument('tag_z_m', default_value='0.75'),
            DeclareLaunchArgument('tag_roll_rad', default_value=half_pi),
            DeclareLaunchArgument('tag_pitch_rad', default_value='0.0'),
            DeclareLaunchArgument('tag_yaw_rad', default_value=half_pi),
            DeclareLaunchArgument('camera_x_m', default_value='0.0'),
            DeclareLaunchArgument('camera_y_m', default_value='0.0'),
            DeclareLaunchArgument('camera_z_m', default_value='0.40'),
            DeclareLaunchArgument('camera_roll_rad', default_value=f'-{half_pi}'),
            DeclareLaunchArgument('camera_pitch_rad', default_value='0.0'),
            DeclareLaunchArgument('camera_yaw_rad', default_value=half_pi),
            DeclareLaunchArgument('zed_x_m', default_value='0.0'),
            DeclareLaunchArgument('zed_y_m', default_value='0.0'),
            DeclareLaunchArgument('zed_z_m', default_value='0.40'),
            DeclareLaunchArgument('zed_roll_rad', default_value=f'-{half_pi}'),
            DeclareLaunchArgument('zed_pitch_rad', default_value='0.0'),
            DeclareLaunchArgument('zed_yaw_rad', default_value=f'-{half_pi}'),
            DeclareLaunchArgument('chassis_length_m', default_value='0.90'),
            DeclareLaunchArgument('chassis_width_m', default_value='0.60'),
            DeclareLaunchArgument('chassis_height_m', default_value='0.25'),
            DeclareLaunchArgument('stale_timeout_s', default_value='0.5'),
            DeclareLaunchArgument('max_path_poses', default_value='500'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
