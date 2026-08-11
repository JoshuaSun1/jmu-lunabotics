"""Offline contracts for the Phase 2 public bench launch."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bench_launch_delegates_to_the_drive_hardware_boundary() -> None:
    """The required Phase 2 bench entry point starts no vendor-specific driver."""
    bench_launch = (PACKAGE_ROOT / "launch" / "bench_test.launch.py").read_text(encoding="utf-8")

    assert "lb_hardware" in bench_launch
    assert "hardware.launch.py" in bench_launch
    assert 'DeclareLaunchArgument("use_mock_hardware", default_value="true")' in bench_launch
    assert 'DeclareLaunchArgument("enable_motors", default_value="false")' in bench_launch
    assert "MockDriveTransport" in bench_launch
    assert "physical propulsion" in bench_launch


def test_bringup_exposes_fault_injection_without_defaulting_to_it() -> None:
    """Communication-loss testing is explicit and the default mock run stays healthy."""
    bringup_launch = (PACKAGE_ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument("mock_communication_loss_after_reads", default_value="-1")'
        in bringup_launch
    )
    assert 'DeclareLaunchArgument("mock_command_timeout_s", default_value="0.25")' in bringup_launch
