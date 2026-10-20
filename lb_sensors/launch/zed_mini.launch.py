"""Start the ZED Mini on the robot, with ROS topics available to other hosts."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    zed_launch = PathJoinSubstitution(
        [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("lb_sensors"), "config", "zed_mini.yaml"]
    )
    default_terrain_params = PathJoinSubstitution(
        [FindPackageShare("lb_sensors"), "config", "terrain_hazard.template.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="zed"),
            DeclareLaunchArgument("serial_number", default_value="0"),
            DeclareLaunchArgument("zed_params_file", default_value=default_params),
            DeclareLaunchArgument("enable_depth_perception", default_value="false"),
            DeclareLaunchArgument("terrain_params_file", default_value=default_terrain_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    "camera_model": "zedm",
                    "camera_name": LaunchConfiguration("camera_name"),
                    "serial_number": LaunchConfiguration("serial_number"),
                    "ros_params_override_path": LaunchConfiguration("zed_params_file"),
                    "publish_urdf": "true",
                    "publish_tf": "false",
                    "publish_map_tf": "false",
                    "publish_imu_tf": "false",
                    "enable_ipc": "false",
                }.items(),
            ),
            Node(
                package="lb_sensors",
                executable="terrain_hazard_node",
                name="terrain_hazard_node",
                output="screen",
                on_exit=[Shutdown(reason="terrain hazard detector exited")],
                condition=IfCondition(LaunchConfiguration("enable_depth_perception")),
                parameters=[
                    LaunchConfiguration("terrain_params_file"),
                    {"camera_name": LaunchConfiguration("camera_name")},
                ],
            ),
        ]
    )
