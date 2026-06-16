from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            ExecuteProcess(
                cmd=["ros2", "run", "rviz2", "rviz2"],
                output="screen",
            ),
        ]
    )
