"""Offline contract tests for the Phase 1 integration entry point."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bringup_defaults_to_mock_hardware_and_disabled_motors() -> None:
    """The public launch boundary cannot enable an unfinished real drivetrain."""
    launch_file = (PACKAGE_ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_mock_hardware", default_value="true")' in launch_file
    assert 'DeclareLaunchArgument("enable_motors", default_value="false")' in launch_file
    assert "enable_motors is intentionally inert in Phase 1" in launch_file
    assert "mock_robot.launch.py" in launch_file
