"""Launch the headless Phase 1 ros2_control mock robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start one mock controller manager, one joint broadcaster, and one RSP."""
    geometry_file = LaunchConfiguration("geometry_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_file = PathJoinSubstitution([FindPackageShare("lb_model"), "urdf", "lb_mock.urdf.xacro"])
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("lb_sim"), "config", "mock_controllers.yaml"]
    )
    rviz_file = PathJoinSubstitution([FindPackageShare("lb_model"), "rviz", "phase1_mock.rviz"])
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
                "use_rviz",
                default_value="false",
                description="Start RViz for a manual mock-model inspection on a development host.",
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
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time},
                    controllers_file,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="joint_state_broadcaster_spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "20",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
