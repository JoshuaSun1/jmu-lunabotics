"""Start only the non-actuating Batch 1 localization boundary and command gate."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _required_file(value: str, label: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"{label} was not found: {path}")
    return path.resolve()


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    localization_params = _required_file(
        LaunchConfiguration("localization_params_file").perform(context),
        "localization_params_file",
    )
    gate_params = _required_file(
        LaunchConfiguration("gate_params_file").perform(context),
        "gate_params_file",
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return [
        LogInfo(
            msg=(
                "Nav2 Batch 1 development boundary only: AprilTag fused-pose "
                "pass-through and /cmd_vel_nav gate. No Nav2 server, /cmd_vel, "
                "TF broadcaster, hardware, or motor output is started."
            )
        ),
        Node(
            package="lb_localization",
            executable="fused_pose_passthrough.py",
            name="fused_pose_passthrough",
            output="screen",
            parameters=[str(localization_params), {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="lb_safety",
            executable="navigation_command_gate.py",
            name="navigation_command_gate",
            output="screen",
            parameters=[str(gate_params), {"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    localization_share = Path(get_package_share_directory("lb_localization"))
    safety_share = Path(get_package_share_directory("lb_safety"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=str(
                    localization_share / "config" / "fused_pose_passthrough_development.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "gate_params_file",
                default_value=str(
                    safety_share / "config" / "navigation_command_gate_development.yaml"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
