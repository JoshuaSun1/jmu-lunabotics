"""Opt-in Nav2 servers and monitored mock command path; never physical bringup."""

import ast
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_setup(context):
    def boolean(name):
        value = LaunchConfiguration(name).perform(context)
        if value not in {"true", "false"}:
            raise RuntimeError(f"{name} requires true/false")
        return value == "true"

    if not boolean("enable_nav2"):
        return [LogInfo(msg="Mock navigation disabled; no Nav2 or command nodes started")]
    if not boolean("allow_mock_navigation") or not boolean("use_mock_hardware"):
        raise RuntimeError("Nav2 mock profile requires explicit opt-in and mock hardware")
    if boolean("enable_motors"):
        raise RuntimeError("Physical motor enable is prohibited in this profile")
    share = Path(get_package_share_directory("lb_navigation"))
    paths = {
        name: Path(LaunchConfiguration(name).perform(context)).expanduser().resolve()
        for name in ("map", "params_file")
    }
    for name, path in paths.items():
        if not path.is_file():
            raise RuntimeError(f"Missing {name}: {path}")
    params = yaml.safe_load(paths["params_file"].read_text())
    limits = yaml.safe_load((share / "config/robot_limits_development.yaml").read_text())
    for name in ("global_costmap", "local_costmap"):
        profile = params[name][name]["ros__parameters"]
        if ast.literal_eval(profile["footprint"]) != limits["footprint"]["polygon_xy_m"]:
            raise RuntimeError("Mock costmap footprint disagrees with development limits")
    use_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = [str(paths["params_file"]), {"use_sim_time": use_time}]
    actions = []
    for name, package, executable, overrides, remaps in (
        ("map_server", "nav2_map_server", "map_server", {"yaml_filename": str(paths["map"])}, []),
        ("planner_server", "nav2_planner", "planner_server", {}, []),
        (
            "controller_server",
            "nav2_controller",
            "controller_server",
            {"odom_topic": "/odometry/local"},
            [("cmd_vel", "/cmd_vel_nav_controller")],
        ),
        (
            "behavior_server",
            "nav2_behaviors",
            "behavior_server",
            {},
            [("cmd_vel", "/cmd_vel_nav_controller")],
        ),
        (
            "velocity_smoother",
            "nav2_velocity_smoother",
            "velocity_smoother",
            {},
            [("cmd_vel", "/cmd_vel_nav_controller"), ("cmd_vel_smoothed", "/cmd_vel_nav_raw")],
        ),
        (
            "bt_navigator",
            "nav2_bt_navigator",
            "bt_navigator",
            {"default_nav_to_pose_bt_xml": str(share / "behavior_trees/mock_navigate.xml")},
            [("navigate_to_pose", "/navigation/internal/navigate_to_pose")],
        ),
        ("lifecycle_manager_navigation", "nav2_lifecycle_manager", "lifecycle_manager", {}, []),
    ):
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=common + [overrides],
                remappings=remaps,
            )
        )
    actions.append(
        Node(
            package="lb_safety",
            executable="navigation_supervisor.py",
            name="navigation_supervisor",
            output="screen",
            parameters=[
                {
                    "allow_mock_navigation": True,
                    "use_sim_time": use_time,
                    "max_linear_velocity_mps": limits["motion_limits"]["max_linear_velocity_mps"],
                    "max_angular_velocity_radps": limits["motion_limits"][
                        "max_angular_velocity_radps"
                    ],
                }
            ],
        )
    )
    if boolean("use_rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="navigation_rviz",
                arguments=["-d", str(share / "rviz/navigation_mock.rviz")],
                parameters=[{"use_sim_time": use_time}],
            )
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("lb_navigation"))
    defaults = {
        "enable_nav2": "false",
        "allow_mock_navigation": "false",
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
