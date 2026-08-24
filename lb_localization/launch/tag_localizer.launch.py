"""Fail-closed launch for the known-AprilTag localizer.

This launch deliberately has no usable default map or calibration.  A normal
run requires a surveyed tag map and measured base-to-camera extrinsics.  The
synthetic test fixtures are accepted only after an explicit test-only opt-in.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _required_file(value: str, label: str) -> str:
    """Return an explicit configuration path or stop before the node starts."""
    if not value.strip():
        raise RuntimeError(f"{label} must be an explicit file; no production default exists")
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"{label} was not found: {path}")
    return str(path.resolve())


def _launch_setup(context, *args, **kwargs):
    """Resolve files before constructing the non-actuating localization node."""
    del args, kwargs
    tag_map_file = _required_file(
        LaunchConfiguration("tag_map_file").perform(context), "tag_map_file"
    )
    localizer_config_file = _required_file(
        LaunchConfiguration("localizer_config_file").perform(context),
        "localizer_config_file",
    )
    return [
        LogInfo(
            msg=(
                "Starting tag_localizer only. It publishes an absolute map-frame "
                "measurement and diagnostics; it never broadcasts map->odom TF."
            )
        ),
        Node(
            package="lb_localization",
            executable="tag_localizer",
            name="tag_localizer",
            output="screen",
            parameters=[
                {
                    "tag_map_file": tag_map_file,
                    "localizer_config_file": localizer_config_file,
                    "allow_synthetic_test_data": ParameterValue(
                        LaunchConfiguration("allow_synthetic_test_data"), value_type=bool
                    ),
                    "pose_topic": LaunchConfiguration("pose_topic"),
                    "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare explicit configuration inputs for a surveyed or test-only localizer run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("tag_map_file", default_value=""),
            DeclareLaunchArgument("localizer_config_file", default_value=""),
            DeclareLaunchArgument("allow_synthetic_test_data", default_value="false"),
            DeclareLaunchArgument("pose_topic", default_value="/localization/apriltag_pose"),
            DeclareLaunchArgument("diagnostics_topic", default_value="/diagnostics"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
