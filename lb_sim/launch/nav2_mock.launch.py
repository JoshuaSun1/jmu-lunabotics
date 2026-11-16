"""N5–N8 end-to-end mock profile with odometry-driven synthetic perception."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    def enabled(name):
        value = LaunchConfiguration(name).perform(context)
        if value not in {"true", "false"}:
            raise RuntimeError(f"Invalid boolean {name}")
        return value == "true"

    if not enabled("enable_nav2"):
        return [LogInfo(msg="N5–N8 mock profile disabled; starts no processes")]
    if not enabled("allow_mock_navigation") or not enabled("allow_test_only_tf"):
        raise RuntimeError("Mock navigation requires explicit mock-navigation and test-TF opt-ins")
    if not enabled("use_mock_hardware") or enabled("enable_motors"):
        raise RuntimeError("Only mock hardware is permitted; physical motor enable is prohibited")

    def include(package, path, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory(package)) / "launch" / path)
            ),
            launch_arguments=arguments.items(),
        )

    use_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        # enable_motors in Phase 2 means enable_on_activate for the explicitly
        # selected in-memory transport. No physical device exists in this path.
        include(
            "lb_hardware",
            "hardware.launch.py",
            {
                "use_mock_hardware": "true",
                "enable_motors": "true",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ),
        Node(
            package="lb_sim",
            executable="navigation_mock_world.py",
            name="navigation_mock_world",
            output="screen",
            parameters=[{"allow_test_only_world": True, "use_sim_time": use_time}],
        ),
        include(
            "lb_navigation",
            "nav2_n3_development.launch.py",
            {
                "enable_development_tf": "true",
                "allow_test_only_tf": "true",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ),
        include(
            "lb_navigation",
            "navigation_mock.launch.py",
            {
                "enable_nav2": "true",
                "allow_mock_navigation": "true",
                "use_mock_hardware": "true",
                "enable_motors": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
            },
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("lb_navigation"))
    defaults = {
        "enable_nav2": "false",
        "allow_mock_navigation": "false",
        "allow_test_only_tf": "false",
        "use_mock_hardware": "true",
        "enable_motors": "false",
        "use_sim_time": "false",
        "use_rviz": "false",
        "map": str(share / "maps/arena.yaml"),
        "params_file": str(share / "config/nav2_mock_development.yaml"),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=_launch_setup)]
    )
