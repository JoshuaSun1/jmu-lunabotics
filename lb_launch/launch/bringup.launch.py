"""Phase 1 system entry point: mock description and mock ros2_control only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Expose the future bringup contract without allowing real motor control."""
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_motors = LaunchConfiguration("enable_motors")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    mock_launch = PathJoinSubstitution(
        [FindPackageShare("lb_sim"), "launch", "mock_robot.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("enable_motors", default_value="false"),
            DeclareLaunchArgument("enable_zed", default_value="false"),
            DeclareLaunchArgument("enable_lidar", default_value="false"),
            DeclareLaunchArgument("enable_apriltags", default_value="false"),
            DeclareLaunchArgument("enable_depth_perception", default_value="false"),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("enable_mission", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mock_launch),
                condition=IfCondition(use_mock_hardware),
                launch_arguments={"use_rviz": use_rviz, "use_sim_time": use_sim_time}.items(),
            ),
            LogInfo(
                condition=UnlessCondition(use_mock_hardware),
                msg="Phase 1 has no real hardware implementation; no motor hardware was launched.",
            ),
            LogInfo(
                condition=IfCondition(enable_motors),
                msg="enable_motors is intentionally inert in Phase 1; no command controller is loaded.",
            ),
        ]
    )
