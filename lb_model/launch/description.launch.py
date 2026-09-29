"""Publish the Phase 1 mock robot description for model inspection only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start robot_state_publisher and an optional standalone joint-state publisher."""
    geometry_file = LaunchConfiguration("geometry_file")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_file = PathJoinSubstitution([FindPackageShare("lb_model"), "urdf", "lb_mock.urdf.xacro"])
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model_file,
                " ",
                "geometry_file:=",
                geometry_file,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_model"), "config", "mock_geometry.yaml"]
                ),
                description="Synthetic Phase 1 geometry profile; never a physical calibration file.",
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description="Run joint_state_publisher for standalone model inspection.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=IfCondition(publish_joint_states),
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
        ]
    )
