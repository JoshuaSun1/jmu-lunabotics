"""Start the Phase 2 ros2_control hardware boundary with an explicit mock or fail-closed transport."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch controller manager, state publisher, and Phase 2 drive controllers."""
    geometry_file = LaunchConfiguration("geometry_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_motors = LaunchConfiguration("enable_motors")
    mock_communication_loss_after_reads = LaunchConfiguration("mock_communication_loss_after_reads")
    mock_command_timeout_s = LaunchConfiguration("mock_command_timeout_s")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_file = PathJoinSubstitution([FindPackageShare("lb_model"), "urdf", "lb_mock.urdf.xacro"])
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("lb_hardware"), "config", "mock_drive_controllers.yaml"]
    )

    # The real branch does not open a device: RealDriveTransport rejects activation
    # until DRIVE-03/SAFE-01 are reviewed. This keeps the public launch fail-closed.
    transport_type = PythonExpression(["'mock' if '", use_mock_hardware, "' == 'true' else 'real'"])
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model_file,
                " ",
                "geometry_file:=",
                geometry_file,
                " ",
                "hardware_plugin:=lb_hardware/LunabotDriveHardware",
                " ",
                "transport_type:=",
                transport_type,
                " ",
                "enable_on_activate:=",
                enable_motors,
                " ",
                "mock_communication_loss_after_reads:=",
                mock_communication_loss_after_reads,
                " ",
                "mock_command_timeout_s:=",
                mock_command_timeout_s,
            ]
        ),
        value_type=str,
    )

    joint_state_broadcaster_spawner = Node(
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
    )
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="drive_controller_spawner",
        output="screen",
        arguments=[
            "drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lb_model"), "config", "mock_geometry.yaml"]
                ),
                description="Synthetic mock geometry only; physical geometry remains GEOM-01/DRIVE-02.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Use the in-memory mock transport. false selects the intentionally unavailable real skeleton.",
            ),
            DeclareLaunchArgument(
                "enable_motors",
                default_value="false",
                description="Mock-output enable only. It never enables physical propulsion in Phase 2.",
            ),
            DeclareLaunchArgument(
                "mock_communication_loss_after_reads",
                default_value="-1",
                description="Mock fault injection: -1 disables it; 0 fails the first state read.",
            ),
            DeclareLaunchArgument(
                "mock_command_timeout_s",
                default_value="0.25",
                description="Mock transport heartbeat timeout in seconds; not a physical safety setting.",
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
                # These target the controller's native ~/cmd_vel and ~/odom names.
                # Runtime verification is retained because Humble loads controllers in-process.
                remappings=[
                    ("/drive_controller/cmd_vel", "/cmd_vel"),
                    ("/drive_controller/odom", "/odom/wheel"),
                ],
            ),
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[drive_controller_spawner],
                )
            ),
        ]
    )
