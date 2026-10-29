"""Launch explicitly authorized N4 static-only development costmaps."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
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
    enable_costmaps = _bool_argument(context, "enable_costmaps")
    enable_development_tf = _bool_argument(context, "enable_development_tf")
    allow_test_only_tf = _bool_argument(context, "allow_test_only_tf")
    production_ekfs_enabled = _bool_argument(context, "production_ekfs_enabled")
    localization_profile = LaunchConfiguration("localization_profile").perform(context)

    navigation_share = Path(get_package_share_directory("lb_navigation"))
    localization_share = Path(get_package_share_directory("lb_localization"))
    safety_share = Path(get_package_share_directory("lb_safety"))
    map_yaml = _required_file(LaunchConfiguration("map").perform(context), "map")
    params_file = _required_file(LaunchConfiguration("params_file").perform(context), "params_file")
    rviz_config = _required_file(LaunchConfiguration("rviz_config").perform(context), "rviz_config")
    scene_params = _required_file(
        LaunchConfiguration("scene_params_file").perform(context), "scene_params_file"
    )
    localization_params = _required_file(
        LaunchConfiguration("localization_params_file").perform(context),
        "localization_params_file",
    )
    gate_params = _required_file(
        LaunchConfiguration("gate_params_file").perform(context), "gate_params_file"
    )
    tf_params = _required_file(
        LaunchConfiguration("tf_params_file").perform(context), "tf_params_file"
    )

    if enable_costmaps and not enable_development_tf:
        raise RuntimeError("N4 costmaps require enable_development_tf:=true")
    if enable_costmaps and not allow_test_only_tf:
        raise RuntimeError("N4 costmaps require explicit allow_test_only_tf:=true")
    if enable_costmaps and production_ekfs_enabled:
        raise RuntimeError("N4 development costmaps prohibit production EKF coexistence")
    if enable_costmaps and localization_profile != "apriltag_development_only":
        raise RuntimeError(
            "N4 development costmaps require localization_profile:=apriltag_development_only"
        )

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(navigation_share / "launch" / "nav2_n3_development.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_development_tf": LaunchConfiguration("enable_development_tf"),
                "allow_test_only_tf": LaunchConfiguration("allow_test_only_tf"),
                "production_ekfs_enabled": LaunchConfiguration("production_ekfs_enabled"),
                "localization_profile": LaunchConfiguration("localization_profile"),
                "localization_params_file": str(localization_params),
                "gate_params_file": str(gate_params),
                "tf_params_file": str(tf_params),
            }.items(),
        )
    ]
    if not enable_costmaps:
        actions.append(
            LogInfo(
                msg=(
                    "N4 costmaps are disabled. The included localization boundary may run, "
                    "but no map server, costmap, RViz, or lifecycle manager is started."
                )
            )
        )
        return actions

    lifecycle_nodes = [
        "map_server",
        "global_costmap/global_costmap",
        "local_costmap/local_costmap",
    ]
    actions.extend(
        [
            LogInfo(
                msg=(
                    "TEST ONLY N4: static/inflation costmaps, provisional footprint, "
                    "no obstacle source, planner, controller, /cmd_vel, hardware, or motors."
                )
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    str(params_file),
                    {"yaml_filename": str(map_yaml), "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                namespace="global_costmap",
                name="global_costmap",
                output="screen",
                parameters=[str(params_file), {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                namespace="local_costmap",
                name="local_costmap",
                output="screen",
                parameters=[str(params_file), {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_n4",
                output="screen",
                parameters=[
                    str(params_file),
                    {
                        "autostart": autostart,
                        "use_sim_time": use_sim_time,
                        "node_names": lifecycle_nodes,
                    },
                ],
            ),
            Node(
                package="lb_navigation",
                executable="arena_scene_markers.py",
                name="arena_scene_markers",
                output="screen",
                parameters=[str(scene_params), {"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_arena_scene")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="nav2_n4_costmap_rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    navigation_share = Path(get_package_share_directory("lb_navigation"))
    localization_share = Path(get_package_share_directory("lb_localization"))
    safety_share = Path(get_package_share_directory("lb_safety"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_costmaps", default_value="false"),
            DeclareLaunchArgument("enable_development_tf", default_value="false"),
            DeclareLaunchArgument("allow_test_only_tf", default_value="false"),
            DeclareLaunchArgument("production_ekfs_enabled", default_value="false"),
            DeclareLaunchArgument(
                "localization_profile", default_value="apriltag_development_only"
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_arena_scene", default_value="true"),
            DeclareLaunchArgument(
                "map", default_value=str(navigation_share / "maps" / "arena.yaml")
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    navigation_share / "config" / "nav2_n4_costmaps_development.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(navigation_share / "rviz" / "nav2_n4_costmaps.rviz"),
            ),
            DeclareLaunchArgument(
                "scene_params_file",
                default_value=str(navigation_share / "config" / "arena_scene.yaml"),
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
