"""Unit and configuration tests for the Batch 1 Nav2 command gate."""

import importlib.util
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "navigation_command_gate.py"
CONFIG = PACKAGE_ROOT / "config" / "navigation_command_gate_development.yaml"
SPEC = importlib.util.spec_from_file_location("navigation_command_gate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_gate_requires_valid_signal_and_fresh_fused_pose() -> None:
    tracker = MODULE.NavigationGateTracker(0.5, "map")

    assert not tracker.permitted(1_000_000_000)
    tracker.set_localization_valid(True)
    assert not tracker.permitted(1_000_000_000)
    assert tracker.observe_pose("map", 1_000_000_000, 1_000_000_000)
    assert tracker.permitted(1_500_000_000)
    assert not tracker.permitted(1_500_000_001)
    assert tracker.reason == "FUSED_POSE_TIMEOUT"


def test_invalid_signal_and_bad_pose_fail_closed() -> None:
    tracker = MODULE.NavigationGateTracker(0.5, "map")
    tracker.set_localization_valid(True)
    tracker.observe_pose("map", 1_000_000_000, 1_000_000_000)
    assert tracker.permitted(1_100_000_000)

    tracker.set_localization_valid(False)
    assert not tracker.permitted(1_100_000_000)
    tracker.set_localization_valid(True)
    assert not tracker.observe_pose("odom", 1_200_000_000, 1_200_000_000)
    assert not tracker.permitted(1_200_000_000)


def test_clock_rewind_fails_closed() -> None:
    tracker = MODULE.NavigationGateTracker(0.5, "map")
    tracker.set_localization_valid(True)
    tracker.observe_pose("map", 2_000_000_000, 2_000_000_000)
    assert tracker.permitted(2_000_000_000)

    assert not tracker.permitted(1_000_000_000)
    assert tracker.reason == "CLOCK_RESET"


def test_out_of_order_fused_pose_fails_closed() -> None:
    tracker = MODULE.NavigationGateTracker(0.5, "map")
    tracker.set_localization_valid(True)
    assert tracker.observe_pose("map", 2_000_000_000, 2_000_000_000)

    assert not tracker.observe_pose("map", 1_900_000_000, 2_000_000_000)
    assert not tracker.permitted(2_000_000_000)
    assert tracker.reason == "FUSED_POSE_OUT_OF_ORDER"


def test_pose_content_validator_is_fail_closed() -> None:
    message = PoseWithCovarianceStamped()
    message.pose.pose.orientation.w = 1.0
    assert MODULE.pose_is_valid(message)

    message.pose.covariance[0] = -1.0
    assert not MODULE.pose_is_valid(message)
    message.pose.covariance[0] = 0.0
    message.pose.pose.orientation.w = float("inf")
    assert not MODULE.pose_is_valid(message)


def test_zero_command_and_nonfinite_rejection_helpers() -> None:
    stamp = Time(sec=3, nanosec=4)
    command = MODULE.zero_twist(stamp, "base_link")
    assert command.header.stamp == stamp
    assert command.header.frame_id == "base_link"
    assert command.twist.linear.x == 0.0
    assert command.twist.angular.z == 0.0
    assert MODULE.twist_is_finite(command)

    invalid = TwistStamped()
    invalid.twist.linear.x = float("nan")
    assert not MODULE.twist_is_finite(invalid)


def test_configuration_stops_at_the_nav2_command_boundary() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["navigation_command_gate"][
        "ros__parameters"
    ]
    source = SCRIPT.read_text(encoding="utf-8")

    assert parameters["raw_cmd_topic"] == "/cmd_vel_nav_raw"
    assert parameters["gated_cmd_topic"] == "/cmd_vel_nav"
    assert parameters["fused_pose_topic"] == "/localization/fused_pose"
    assert parameters["valid_topic"] == "/localization/valid"
    assert parameters["freshness_timeout_s"] == 0.5
    assert parameters["check_period_s"] == 0.05
    assert 'declare_parameter("gated_cmd_topic", "/cmd_vel_nav")' in source
    assert 'declare_parameter("raw_cmd_topic", "/cmd_vel_nav_raw")' in source
