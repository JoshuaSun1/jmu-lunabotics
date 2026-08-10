"""Offline contract checks for the Phase 1 mock launch composition."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_mock_launch_has_one_joint_state_producer() -> None:
    """The controller broadcaster, not a second publisher, owns /joint_states."""
    launch_file = (PACKAGE_ROOT / "launch" / "mock_robot.launch.py").read_text(encoding="utf-8")
    assert 'package="joint_state_publisher"' not in launch_file
    assert '"joint_state_broadcaster"' in launch_file
    assert 'package="controller_manager"' in launch_file


def test_mock_launch_is_headless_and_uses_upstream_mock_hardware() -> None:
    """RViz stays opt-in and the model retains GenericSystem ownership."""
    launch_file = (PACKAGE_ROOT / "launch" / "mock_robot.launch.py").read_text(encoding="utf-8")
    controllers = (PACKAGE_ROOT / "config" / "mock_controllers.yaml").read_text(encoding="utf-8")
    model = (PACKAGE_ROOT.parent / "lb_model" / "urdf" / "lb_mock.urdf.xacro").read_text(
        encoding="utf-8"
    )

    assert 'default_value="false"' in launch_file
    assert 'package="rviz2"' in launch_file
    assert "joint_state_broadcaster/JointStateBroadcaster" in controllers
    assert "mock_components/GenericSystem" in model
