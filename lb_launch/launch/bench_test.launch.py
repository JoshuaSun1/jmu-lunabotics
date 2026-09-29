"""Phase 2 drive bench entry point; it defaults to disabled in-memory mock output."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Expose one deliberately safe command/odometry test profile."""
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_motors = LaunchConfiguration("enable_motors")
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("lb_hardware"), "launch", "hardware.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("enable_motors", default_value="false"),
            DeclareLaunchArgument(
                "geometry_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_model"), "config", "mock_geometry.yaml"]
                ),
            ),
            DeclareLaunchArgument("mock_communication_loss_after_reads", default_value="-1"),
            DeclareLaunchArgument("mock_command_timeout_s", default_value="0.25"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_mock_hardware": use_mock_hardware,
                    "enable_motors": enable_motors,
                    "geometry_file": LaunchConfiguration("geometry_file"),
                    "mock_communication_loss_after_reads": LaunchConfiguration(
                        "mock_communication_loss_after_reads"
                    ),
                    "mock_command_timeout_s": LaunchConfiguration("mock_command_timeout_s"),
                }.items(),
            ),
            LogInfo(
                condition=IfCondition(enable_motors),
                msg=(
                    "Phase 2 enable_motors affects only MockDriveTransport. It does not authorize "
                    "physical propulsion."
                ),
            ),
            LogInfo(
                condition=UnlessCondition(use_mock_hardware),
                msg=(
                    "The Phase 2 real transport is intentionally unavailable and will fail closed; "
                    "no device or controller protocol is opened."
                ),
            ),
        ]
    )
