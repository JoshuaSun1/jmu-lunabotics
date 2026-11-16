"""Command-age, geometry and magnitude guards for the final mock command path."""

import importlib.util
from pathlib import Path
import sys
import time

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy


SPEC = importlib.util.spec_from_file_location(
    "navigation_supervisor", Path(__file__).parents[1] / "scripts/navigation_supervisor.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def command():
    message = TwistStamped()
    message.header.stamp.sec = 10
    message.header.frame_id = "base_link"
    message.twist.linear.x = 0.25
    return message


def test_current_planar_command_and_exact_limits_are_allowed():
    message = command()
    assert MODULE.bounded_command(message, 10_100_000_000, 0.25, 0.5, 1.0)
    message.twist.linear.x = -0.5
    message.twist.angular.z = 1.0
    assert MODULE.bounded_command(message, 10_100_000_000, 0.25, 0.5, 1.0)


@pytest.mark.parametrize("now", [9_999_999_999, 10_250_000_001])
def test_future_and_expired_commands_are_rejected(now):
    assert not MODULE.bounded_command(command(), now, 0.25, 0.5, 1.0)


@pytest.mark.parametrize(
    "attribute,value",
    [
        ("linear.x", 0.51),
        ("linear.x", float("nan")),
        ("angular.z", float("inf")),
        ("angular.z", -1.01),
        ("linear.y", 0.1),
        ("linear.z", 0.1),
        ("angular.x", 0.1),
        ("angular.y", 0.1),
    ],
)
def test_nonplanar_nonfinite_and_out_of_bounds_commands_are_rejected(attribute, value):
    message = command()
    vector, axis = attribute.split(".")
    setattr(getattr(message.twist, vector), axis, value)
    assert not MODULE.bounded_command(message, 10_100_000_000, 0.25, 0.5, 1.0)


def test_zero_stamp_and_wrong_frame_are_rejected():
    message = command()
    message.header.stamp.sec = 0
    assert not MODULE.bounded_command(message, 10_100_000_000, 0.25, 0.5, 1.0)

    message = command()
    message.header.frame_id = "map"
    assert not MODULE.bounded_command(message, 10_100_000_000, 0.25, 0.5, 1.0)


def test_watchdog_runs_when_simulation_clock_stops():
    rclpy.init(args=["--ros-args", "-p", "allow_mock_navigation:=true", "-p", "use_sim_time:=true"])
    node = MODULE.NavigationSupervisor()
    try:
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.last_clock == 0  # The steady watchdog actually executed.
        assert node.reason() == "CLOCK_STALLED"
    finally:
        node.destroy_node()
        rclpy.shutdown()
