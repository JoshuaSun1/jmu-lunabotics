"""Safe public entry point for the mock bench and isolated camera profiles."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Keep real sensor launches separate from the synthetic robot model."""
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_motors = LaunchConfiguration("enable_motors")
    enable_apriltags = LaunchConfiguration("enable_apriltags")
    bench_launch = PathJoinSubstitution(
        [FindPackageShare("lb_launch"), "launch", "bench_test.launch.py"]
    )
    webcam_apriltag_launch = PathJoinSubstitution(
        [FindPackageShare("lb_sensors"), "launch", "webcam_apriltag.launch.py"]
    )
    zed_mini_launch = PathJoinSubstitution(
        [FindPackageShare("lb_sensors"), "launch", "zed_mini.launch.py"]
    )

    def validate_depth_profile(context, *args, **kwargs):
        depth_enabled = LaunchConfiguration("enable_depth_perception").perform(context)
        zed_enabled = LaunchConfiguration("enable_zed").perform(context)
        if depth_enabled.lower() == "true" and zed_enabled.lower() != "true":
            raise RuntimeError("enable_depth_perception requires enable_zed:=true")
        return []

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("enable_motors", default_value="false"),
            DeclareLaunchArgument("enable_zed", default_value="false"),
            DeclareLaunchArgument("zed_camera_name", default_value="zed"),
            DeclareLaunchArgument("zed_serial_number", default_value="0"),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_sensors"), "config", "zed_mini.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_lidar", default_value="false"),
            DeclareLaunchArgument("enable_apriltags", default_value="false"),
            DeclareLaunchArgument("enable_depth_perception", default_value="false"),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_sensors"), "config", "terrain_hazard.template.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("enable_mission", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "webcam_device",
                default_value="/dev/v4l/by-id/usb-046d_0825_961026E0-video-index0",
            ),
            DeclareLaunchArgument("webcam_namespace", default_value="sensors/webcam"),
            DeclareLaunchArgument("webcam_frame", default_value="webcam_optical_frame"),
            DeclareLaunchArgument("webcam_fps", default_value="15.0"),
            DeclareLaunchArgument(
                "webcam_calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_sensors"), "config", "webcam_calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "webcam_detector_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_sensors"), "config", "webcam_apriltag.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "geometry_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_model"), "config", "mock_geometry.yaml"]
                ),
            ),
            DeclareLaunchArgument("mock_communication_loss_after_reads", default_value="-1"),
            DeclareLaunchArgument("mock_command_timeout_s", default_value="0.25"),
            OpaqueFunction(function=validate_depth_profile),
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration("enable_zed")),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(bench_launch),
                        # The mock robot includes synthetic sensor mounts. Do not
                        # attach real camera data to that model.
                        condition=UnlessCondition(enable_apriltags),
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
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_mini_launch),
                condition=IfCondition(LaunchConfiguration("enable_zed")),
                launch_arguments={
                    "camera_name": LaunchConfiguration("zed_camera_name"),
                    "serial_number": LaunchConfiguration("zed_serial_number"),
                    "zed_params_file": LaunchConfiguration("zed_params_file"),
                    "enable_depth_perception": LaunchConfiguration("enable_depth_perception"),
                    "terrain_params_file": LaunchConfiguration("terrain_params_file"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(webcam_apriltag_launch),
                condition=IfCondition(enable_apriltags),
                launch_arguments={
                    "webcam_device": LaunchConfiguration("webcam_device"),
                    "camera_namespace": LaunchConfiguration("webcam_namespace"),
                    "camera_frame": LaunchConfiguration("webcam_frame"),
                    "camera_fps": LaunchConfiguration("webcam_fps"),
                    "calibration_file": LaunchConfiguration("webcam_calibration_file"),
                    "detector_config": LaunchConfiguration("webcam_detector_config"),
                }.items(),
            ),
            LogInfo(
                condition=IfCondition(enable_apriltags),
                msg=(
                    "Phase 4.1 webcam AprilTag detection is enabled. It publishes only "
                    "camera-scoped data and observation TF; the Phase 1/2 mock bench is "
                    "suppressed, so no localizer, EKF, synthetic base-camera TF, or "
                    "physical motor output is enabled."
                ),
            ),
            LogInfo(
                condition=IfCondition(enable_motors),
                msg=(
                    "Phase 2 uses enable_motors only for in-memory MockDriveTransport when "
                    "the mock bench is active. enable_apriltags:=true suppresses that bench; "
                    "this does not enable physical propulsion."
                ),
            ),
            LogInfo(
                condition=UnlessCondition(use_mock_hardware),
                msg=(
                    "When the Phase 2 bench is active, no real controller protocol exists: "
                    "the selected real skeleton fails closed before opening a device."
                ),
            ),
        ]
    )
