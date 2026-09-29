"""Offline contract tests for the Phase 1 integration entry point."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bringup_preserves_mock_and_disabled_defaults() -> None:
    """The Phase 2 public boundary remains mock-first and disabled by default."""
    launch_file = (PACKAGE_ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_mock_hardware", default_value="true")' in launch_file
    assert 'DeclareLaunchArgument("enable_motors", default_value="false")' in launch_file
    assert "bench_test.launch.py" in launch_file
    assert "does not enable physical propulsion" in launch_file
