"""Offline contract checks for the Phase 4.1 public bringup option."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bringup_connects_its_apriltag_switch_to_the_webcam_pipeline() -> None:
    """The existing public switch must not remain a silent no-op."""
    bringup = (PACKAGE_ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")

    assert "webcam_apriltag.launch.py" in bringup
    assert "condition=IfCondition(enable_apriltags)" in bringup
    assert "condition=UnlessCondition(enable_apriltags)" in bringup
    assert 'DeclareLaunchArgument("enable_apriltags", default_value="false")' in bringup
    assert 'DeclareLaunchArgument("webcam_namespace", default_value="sensors/webcam")' in bringup
    assert 'DeclareLaunchArgument("webcam_fps", default_value="15.0")' in bringup
    assert "webcam_detector_config" in bringup
    assert "no localizer, EKF" in bringup
    assert "physical motor output" in bringup
    assert "synthetic base-camera TF" in bringup
    assert "/dev/v4l/by-id/" in bringup
