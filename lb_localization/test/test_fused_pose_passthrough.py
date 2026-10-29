"""Unit and configuration tests for the Batch 1 fused-pose boundary."""

import importlib.util
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "fused_pose_passthrough.py"
CONFIG = PACKAGE_ROOT / "config" / "fused_pose_passthrough_development.yaml"
SPEC = importlib.util.spec_from_file_location("fused_pose_passthrough", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_accepts_only_current_map_frame_pose() -> None:
    tracker = MODULE.PoseFreshnessTracker(0.5, "map")

    assert tracker.accept("map", 1_000_000_000, 1_100_000_000)
    assert tracker.state == MODULE.LocalizationState.VALID
    assert tracker.last_pose_stamp_ns == 1_000_000_000
    assert tracker.age_seconds(1_100_000_000) == pytest.approx(0.1)


def test_timeout_clears_pose_and_requires_reacquisition() -> None:
    tracker = MODULE.PoseFreshnessTracker(0.5, "map")
    tracker.accept("map", 1_000_000_000, 1_100_000_000)

    assert tracker.tick(1_500_000_000) == MODULE.LocalizationState.VALID
    assert tracker.tick(1_500_000_001) == MODULE.LocalizationState.STALE
    assert tracker.last_pose_stamp_ns is None
    assert tracker.reason == "APRILTAG_POSE_TIMEOUT"
    assert tracker.accept("map", 2_000_000_000, 2_000_000_000)


@pytest.mark.parametrize(
    ("frame_id", "stamp_ns", "now_ns", "expected_reason"),
    [
        ("odom", 1_000_000_000, 1_000_000_000, "FRAME_MISMATCH"),
        ("map", 0, 1_000_000_000, "ZERO_TIMESTAMP"),
        ("map", 1_100_000_000, 1_000_000_000, "FUTURE_TIMESTAMP"),
        ("map", 1_000_000_000, 1_500_000_001, "STALE_ON_ARRIVAL"),
    ],
)
def test_rejects_invalid_pose_without_retaining_old_pose(
    frame_id: str, stamp_ns: int, now_ns: int, expected_reason: str
) -> None:
    tracker = MODULE.PoseFreshnessTracker(0.5, "map")
    assert tracker.accept("map", 900_000_000, 1_000_000_000)

    assert not tracker.accept(frame_id, stamp_ns, now_ns)
    assert tracker.last_pose_stamp_ns is None
    assert tracker.reason == expected_reason


def test_backward_clock_jump_fails_closed() -> None:
    tracker = MODULE.PoseFreshnessTracker(0.5, "map")
    assert tracker.accept("map", 2_000_000_000, 2_000_000_000)

    assert tracker.tick(1_000_000_000) == MODULE.LocalizationState.UNAVAILABLE
    assert tracker.reason == "CLOCK_RESET"
    assert tracker.last_pose_stamp_ns is None


def test_out_of_order_rejection_retains_the_accepted_high_watermark() -> None:
    tracker = MODULE.PoseFreshnessTracker(0.5, "map")
    assert tracker.accept("map", 2_000_000_000, 2_000_000_000)

    assert not tracker.accept("map", 1_900_000_000, 2_000_000_000)
    assert not tracker.accept("map", 1_950_000_000, 2_000_000_000)
    assert tracker.reason == "OUT_OF_ORDER_TIMESTAMP"
    assert tracker.accept("map", 2_100_000_000, 2_100_000_000)


def test_rejects_malformed_pose_and_covariance_content() -> None:
    message = PoseWithCovarianceStamped()
    message.pose.pose.orientation.w = 1.0
    assert MODULE.pose_validation_error(message) is None

    message.pose.pose.position.x = float("nan")
    assert MODULE.pose_validation_error(message) == "NONFINITE_POSE_OR_COVARIANCE"
    message.pose.pose.position.x = 0.0
    message.pose.pose.orientation.w = 0.0
    assert MODULE.pose_validation_error(message) == "INVALID_ORIENTATION_QUATERNION"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[35] = -1.0
    assert MODULE.pose_validation_error(message) == "NEGATIVE_COVARIANCE_DIAGONAL"


def test_development_configuration_is_explicit_and_non_fusing() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["fused_pose_passthrough"][
        "ros__parameters"
    ]
    source = SCRIPT.read_text(encoding="utf-8")

    assert parameters["input_topic"] == "/localization/apriltag_pose"
    assert parameters["output_topic"] == "/localization/fused_pose"
    assert parameters["valid_topic"] == "/localization/valid"
    assert parameters["freshness_timeout_s"] == 0.5
    assert parameters["poll_period_s"] == 0.05
    assert "copy.deepcopy(message)" in source
    assert "TransformBroadcaster" not in source
    assert "sendTransform" not in source
