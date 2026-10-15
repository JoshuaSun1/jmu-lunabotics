"""Launch only the static arena map, its lifecycle manager, and optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _required_file(value: str, label: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f'{label} was not found: {path}')
    return path.resolve()


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    map_yaml = _required_file(LaunchConfiguration('map').perform(context), 'map')
    params_file = _required_file(
        LaunchConfiguration('params_file').perform(context), 'params_file'
    )
    rviz_config = _required_file(
        LaunchConfiguration('rviz_config').perform(context), 'rviz_config'
    )
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    scene_params = _required_file(
        LaunchConfiguration('scene_params_file').perform(context),
        'scene_params_file',
    )

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                str(params_file),
                {'yaml_filename': str(map_yaml), 'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                str(params_file),
                {
                    'autostart': autostart,
                    'use_sim_time': use_sim_time,
                    'node_names': ['map_server'],
                },
            ],
        ),
        Node(
            package='lb_navigation',
            executable='arena_scene_markers.py',
            name='arena_scene_markers',
            output='screen',
            parameters=[str(scene_params), {'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_arena_scene')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='arena_map_rviz',
            output='screen',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory('lb_navigation'))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map', default_value=str(package_share / 'maps' / 'arena.yaml')
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=str(package_share / 'config' / 'map_server.yaml'),
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=str(package_share / 'rviz' / 'arena_map.rviz'),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('use_arena_scene', default_value='true'),
            DeclareLaunchArgument(
                'scene_params_file',
                default_value=str(package_share / 'config' / 'arena_scene.yaml'),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
