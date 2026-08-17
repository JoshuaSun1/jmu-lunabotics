"""Calibrated webcam AprilTag detector for the non-actuating Phase 4.1 bench."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _required_file(value: str, label: str) -> Path:
    """Resolve an explicit local configuration file or fail before launching nodes."""
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"{label} was not found: {path}")
    return path.resolve()


def _resolved_device(value: str) -> str:
    """Resolve the stable public V4L2 path to the character device for a driver."""
    path = Path(value).expanduser()
    if not path.exists():
        raise RuntimeError(
            f"webcam_device was not found: {path}. Use an explicit "
            "V4L2 path, preferably /dev/v4l/by-id/."
        )
    return str(path.resolve(strict=True))


def _configure_v4l2_framerate(device: str, framerate: float) -> None:
    """Set and verify the UVC source rate before opening it in the ROS driver."""
    v4l2_control = shutil.which("v4l2-ctl")
    if v4l2_control is None:
        raise RuntimeError(
            "v4l2-ctl is required to configure the Phase 4.1 camera rate. "
            "Install the v4l-utils package in the runtime image."
        )

    set_result = subprocess.run(
        [v4l2_control, f"--device={device}", f"--set-parm={framerate:g}"],
        check=False,
        capture_output=True,
        text=True,
    )
    set_output = (set_result.stdout + set_result.stderr).strip()
    if set_result.returncode != 0:
        raise RuntimeError(
            f"Could not configure {device} for {framerate:g} FPS: {set_output}"
        )

    get_result = subprocess.run(
        [v4l2_control, f"--device={device}", "--get-parm"],
        check=False,
        capture_output=True,
        text=True,
    )
    get_output = (get_result.stdout + get_result.stderr).strip()
    rate_match = re.search(r"Frames per second:\s*([0-9.]+)", get_output)
    if get_result.returncode != 0 or rate_match is None:
        raise RuntimeError(f"Could not verify V4L2 frame rate for {device}: {get_output}")

    actual_framerate = float(rate_match.group(1))
    if abs(actual_framerate - framerate) > 0.1:
        raise RuntimeError(
            f"V4L2 frame rate for {device} is {actual_framerate:g} FPS, not "
            f"the requested {framerate:g} FPS."
        )


def _launch_setup(context, *args, **kwargs):
    """Construct one camera-scoped driver, rectifier, and detector pipeline."""
    del args, kwargs
    webcam_device = LaunchConfiguration("webcam_device").perform(context).strip()
    if not webcam_device:
        raise RuntimeError(
            "webcam_device must be an explicit V4L2 path; use /dev/v4l/by-id "
            "whenever it is available rather than an unstable /dev/videoN number."
        )
    driver_device = _resolved_device(webcam_device)
    requested_framerate = LaunchConfiguration("camera_fps").perform(context)
    try:
        framerate = float(requested_framerate)
    except ValueError as error:
        raise RuntimeError(f"camera_fps must be numeric, got {requested_framerate!r}") from error
    if not 10.0 <= framerate <= 15.0:
        raise RuntimeError("camera_fps must remain within the initial 10-15 FPS profile")
    _configure_v4l2_framerate(driver_device, framerate)

    calibration_file = _required_file(
        LaunchConfiguration("calibration_file").perform(context), "calibration_file"
    )
    detector_config = _required_file(
        LaunchConfiguration("detector_config").perform(context), "detector_config"
    )
    namespace = LaunchConfiguration("camera_namespace").perform(context).strip("/")
    if not namespace:
        raise RuntimeError("camera_namespace must not be empty")

    camera_name = LaunchConfiguration("camera_name").perform(context)
    camera_frame = LaunchConfiguration("camera_frame").perform(context)

    return [
        LogInfo(
            msg=(
                "Phase 4.1 starts webcam AprilTag detection only: no tag localizer, "
                "EKF, map/odom TF, or physical motor output is launched."
            )
        ),
        LogInfo(
            msg=(
                f"V4L2 source rate verified at {framerate:g} Hz for {webcam_device} "
                f"(resolved driver device: {driver_device})."
            )
        ),
        # Configure the source through V4L2 before launching this driver.  This
        # avoids a lossy post-rectification rate relay and keeps image and
        # calibration data paired for the detector.
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            namespace=namespace,
            name="camera",
            output="screen",
            parameters=[
                {
                    "camera_info_url": calibration_file.as_uri(),
                    "camera_name": camera_name,
                    "camera_frame_id": camera_frame,
                    "image_size": [640, 480],
                    "output_encoding": "rgb8",
                    "pixel_format": "YUYV",
                    "video_device": driver_device,
                }
            ],
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            namespace=namespace,
            name="rectify",
            output="screen",
            remappings=[
                ("image", "image_raw"),
                ("camera_info", "camera_info"),
                ("image_rect", "image_rect"),
            ],
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            namespace=namespace,
            name="apriltag_detector",
            output="screen",
            parameters=[str(detector_config)],
            remappings=[
                ("image_rect", "image_rect"),
                ("camera_info", "camera_info"),
                ("detections", "tag_detections"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the calibrated webcam profile and create the source-local pipeline."""
    package_share = Path(get_package_share_directory("lb_sensors"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "webcam_device",
                default_value=(
                    "/dev/v4l/by-id/usb-046d_0825_961026E0-video-index0"
                ),
            ),
            DeclareLaunchArgument("camera_namespace", default_value="sensors/webcam"),
            DeclareLaunchArgument(
                "camera_name", default_value="uvc_camera_(046d:0825)"
            ),
            DeclareLaunchArgument("camera_frame", default_value="webcam_optical_frame"),
            DeclareLaunchArgument("camera_fps", default_value="15.0"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(package_share / "config" / "webcam_calibration.yaml"),
            ),
            DeclareLaunchArgument(
                "detector_config",
                default_value=str(package_share / "config" / "webcam_apriltag.yaml"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
