"""Offline contract checks for the Phase 4.1 webcam AprilTag pipeline."""

from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
CALIBRATION_FILE = PACKAGE_ROOT / "config" / "webcam_calibration.yaml"
DETECTOR_FILE = PACKAGE_ROOT / "config" / "webcam_apriltag.yaml"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "webcam_apriltag.launch.py"


def test_calibration_matches_the_confirmed_capture_resolution() -> None:
    """The archived calibration is valid only for the explicit 640 x 480 profile."""
    calibration = CALIBRATION_FILE.read_text(encoding="utf-8")

    assert "image_width: 640" in calibration
    assert "image_height: 480" in calibration
    assert "camera_name: uvc_camera_(046d:0825)" in calibration
    assert "distortion_model: plumb_bob" in calibration
    assert "Archived calibration reused with user confirmation" in calibration


def test_detector_uses_the_confirmed_tag_and_camera_scoped_observation_frame() -> None:
    """Known tags have a consistent ID/frame/size mapping without world-frame claims."""
    detector = DETECTOR_FILE.read_text(encoding="utf-8")

    assert "family: 36h11" in detector
    assert "size: 0.250" in detector
    assert "max_hamming: 0" in detector
    assert "pose_estimation_method: pnp" in detector
    assert "ids: [0]" in detector
    assert "frames: [webcam_observation_tag_0]" in detector
    assert "sizes: [0.250]" in detector
    assert "map -> odom" not in detector
    assert "frames: [tag_0]" not in detector


def test_launch_keeps_camera_data_source_scoped_and_does_not_invent_robot_pose() -> None:
    """One camera owns only its driver, rectification, detection, and observation TF."""
    launch_file = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="v4l2_camera"' in launch_file
    assert 'package="image_proc"' in launch_file
    assert 'package="apriltag_ros"' in launch_file
    assert 'default_value="sensors/webcam"' in launch_file
    assert 'default_value="webcam_optical_frame"' in launch_file
    assert 'default_value="uvc_camera_(046d:0825)"' in launch_file
    assert "/dev/v4l/by-id/" in launch_file
    assert '"image_size": [640, 480]' in launch_file
    assert '"pixel_format": "YUYV"' in launch_file
    assert '"output_encoding": "rgb8"' in launch_file
    assert 'DeclareLaunchArgument("camera_fps", default_value="15.0")' in launch_file
    assert "_configure_v4l2_framerate" in launch_file
    assert "v4l2-ctl" in launch_file
    assert "resolve(strict=True)" in launch_file
    assert "10.0 <= framerate <= 15.0" in launch_file
    assert "V4L2 source rate verified" in launch_file
    assert '("detections", "tag_detections")' in launch_file
    assert "static_transform_publisher" not in launch_file
    assert "tag_localizer" not in launch_file
    assert "ekf_" not in launch_file
    assert "map -> odom" not in launch_file


def test_runtime_and_evidence_dependencies_are_declared() -> None:
    """The target image and recorder preserve the complete bench dependency path."""
    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    dockerfile = (REPOSITORY_ROOT / "docker" / "Dockerfile.jetson").read_text(
        encoding="utf-8"
    )
    recorder = (REPOSITORY_ROOT / "scripts" / "record_bag.sh").read_text(
        encoding="utf-8"
    )

    assert "<exec_depend>v4l-utils</exec_depend>" in manifest
    assert "<exec_depend>v4l2_camera</exec_depend>" in manifest
    assert "ros-${ROS_DISTRO}-apriltag-ros" in dockerfile
    assert "ros-${ROS_DISTRO}-image-proc" in dockerfile
    assert "ros-${ROS_DISTRO}-v4l2-camera" in dockerfile
    assert "v4l-utils" in dockerfile
    assert "webcam_apriltag)" in recorder
    assert "/sensors/webcam/tag_detections" in recorder
