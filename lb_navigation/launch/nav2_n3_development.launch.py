"""Start the non-actuating N3 localization boundary and optional test TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _required_file(value: str, label: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"{label} was not found: {path}")
    return path.resolve()


def _bool_argument(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {"true", "false"}:
        raise RuntimeError(f"{name} must be exactly true or false, got {value!r}")
    return value == "true"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    enable_development_tf = _bool_argument(context, "enable_development_tf")
    allow_test_only_tf = _bool_argument(context, "allow_test_only_tf")
    production_ekfs_enabled = _bool_argument(context, "production_ekfs_enabled")
    localization_profile = LaunchConfiguration("localization_profile").perform(context)
    tf_params = _required_file(
        LaunchConfiguration("tf_params_file").perform(context), "tf_params_file"
    )
    if allow_test_only_tf and not enable_development_tf:
        raise RuntimeError("allow_test_only_tf:=true requires enable_development_tf:=true")
    if enable_development_tf and not allow_test_only_tf:
        raise RuntimeError("development TF requires explicit allow_test_only_tf:=true")
    if enable_development_tf and production_ekfs_enabled:
        raise RuntimeError("development TF is mutually exclusive with production EKF publishers")
    if enable_development_tf and localization_profile != "apriltag_development_only":
        raise RuntimeError(
            "development TF requires localization_profile:=apriltag_development_only"
        )

    navigation_share = Path(get_package_share_directory("lb_navigation"))
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(navigation_share / "launch" / "nav2_batch1.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "localization_params_file": LaunchConfiguration("localization_params_file"),
                "gate_params_file": LaunchConfiguration("gate_params_file"),
            }.items(),
        )
    ]
    if not enable_development_tf:
        actions.append(
            LogInfo(
                msg=(
                    "N3 development TF is disabled. Batch 1 pose and command gating "
                    "remain active, but map->odom->base_link is unavailable."
                )
            )
        )
        return actions

    actions.extend(
        [
            LogInfo(
                msg=(
                    "TEST ONLY N3 TF enabled: apriltag_development_tf_bridge solely owns "
                    "map->odom and odom->base_link; map=odom, no propagation, no motors."
                )
            ),
            Node(
                package="lb_localization",
                executable="apriltag_development_tf_bridge.py",
                name="apriltag_development_tf_bridge",
                output="screen",
                parameters=[
                    str(tf_params),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "allow_test_only_tf": True,
                        "production_ekfs_enabled": production_ekfs_enabled,
                        "localization_profile": localization_profile,
                    },
                ],
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    localization_share = Path(get_package_share_directory("lb_localization"))
    safety_share = Path(get_package_share_directory("lb_safety"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_development_tf", default_value="false"),
            DeclareLaunchArgument("allow_test_only_tf", default_value="false"),
            DeclareLaunchArgument("production_ekfs_enabled", default_value="false"),
            DeclareLaunchArgument(
                "localization_profile", default_value="apriltag_development_only"
            ),
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
            DeclareLaunchArgument(
                "tf_params_file",
                default_value=str(
                    localization_share / "config" / "apriltag_development_tf_bridge.yaml"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
