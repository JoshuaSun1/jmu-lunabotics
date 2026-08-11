"""Offline contracts for the Phase 2 mock-first drive interface."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent

WHEEL_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
}


def test_transport_contract_is_explicit_and_protocol_free() -> None:
    """The real path must not silently select a physical communication design."""
    interface = (PACKAGE_ROOT / "include" / "lb_hardware" / "drive_transport.hpp").read_text(
        encoding="utf-8"
    )
    real_transport = (PACKAGE_ROOT / "src" / "real_drive_transport.cpp").read_text(encoding="utf-8")

    for method in (
        "connect()",
        "disconnect()",
        "read_state(",
        "write_command(",
        "set_enabled(",
        "stop()",
        "get_faults() const",
    ):
        assert method in interface
    assert "DriveState" in interface
    assert "DriveCommand" in interface
    assert "velocity_rad_s" in interface
    assert "position_rad" in interface
    assert "TODO(DRIVE-03)" in real_transport
    assert "TODO(SAFE-01)" in real_transport
    assert "return false;" in real_transport


def test_mock_controller_uses_all_four_wheels_and_never_owns_odom_tf() -> None:
    """The mock controller uses confirmed wheel groups and preserves Phase 3 TF authority."""
    controllers = (PACKAGE_ROOT / "config" / "mock_drive_controllers.yaml").read_text(
        encoding="utf-8"
    )

    for wheel_joint in WHEEL_JOINTS:
        assert wheel_joint in controllers
    assert "type: diff_drive_controller/DiffDriveController" in controllers
    assert "wheels_per_side: 2" in controllers
    assert "wheel_radius: 0.12  # m, synthetic_mock_only" in controllers
    assert "wheel_separation: 0.68  # m, synthetic_mock_only" in controllers
    assert "enable_odom_tf: false" in controllers
    assert "open_loop: false" in controllers
    assert "position_feedback: true" in controllers
    assert "use_stamped_vel: true" in controllers
    assert "cmd_vel_timeout: 0.25" in controllers
    assert "linear.x.has_jerk_limits: true" in controllers
    assert "angular.z.has_jerk_limits: true" in controllers


def test_phase_two_launch_is_mock_first_and_fail_closed() -> None:
    """The bench profile defaults to disabled mock output and selects the custom plugin."""
    model = (REPOSITORY_ROOT / "lb_model" / "urdf" / "lb_mock.urdf.xacro").read_text(
        encoding="utf-8"
    )
    hardware_launch = (PACKAGE_ROOT / "launch" / "hardware.launch.py").read_text(encoding="utf-8")
    bench_launch = (REPOSITORY_ROOT / "lb_launch" / "launch" / "bench_test.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'name="hardware_plugin" default="mock_components/GenericSystem"' in model
    assert "<plugin>$(arg hardware_plugin)</plugin>" in model
    assert "transport_type" in model
    assert "lb_hardware/LunabotDriveHardware" in hardware_launch
    assert "transport_type = PythonExpression" in hardware_launch
    assert 'DeclareLaunchArgument("enable_motors", default_value="false")' in bench_launch
    assert 'DeclareLaunchArgument("use_mock_hardware", default_value="true")' in bench_launch
    assert '"/drive_controller/cmd_vel", "/cmd_vel"' in hardware_launch
    assert '"/drive_controller/odom", "/odom/wheel"' in hardware_launch


def test_plugin_stops_and_disables_before_reporting_transport_errors() -> None:
    """Communication and write faults take the common fail-safe path."""
    plugin = (PACKAGE_ROOT / "src" / "lunabot_drive_hardware.cpp").read_text(encoding="utf-8")

    assert "bool LunabotDriveHardware::disable_and_stop()" in plugin
    assert "transport_->stop()" in plugin
    assert "transport_->set_enabled(false)" in plugin
    read_body = plugin.split("hardware_interface::return_type LunabotDriveHardware::read", 1)[1]
    write_body = plugin.split("hardware_interface::return_type LunabotDriveHardware::write", 1)[1]
    assert "disable_and_stop();" in read_body
    assert "disable_and_stop();" in write_body
