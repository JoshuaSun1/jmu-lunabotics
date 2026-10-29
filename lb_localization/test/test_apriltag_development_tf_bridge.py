"""Unit and configuration tests for the N3 development TF bridge."""

import importlib.util
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "apriltag_development_tf_bridge.py"
CONFIG = PACKAGE_ROOT / "config" / "apriltag_development_tf_bridge.yaml"
SPEC = importlib.util.spec_from_file_location("apriltag_development_tf_bridge", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _pose(stamp_ns: int = 1_000_000_000) -> PoseWithCovarianceStamped:
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp.sec = stamp_ns // MODULE.NANOSECONDS_PER_SECOND
    message.header.stamp.nanosec = stamp_ns % MODULE.NANOSECONDS_PER_SECOND
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.position.z = 0.1
    message.pose.pose.orientation.z = 0.25
    message.pose.pose.orientation.w = 0.9682458366
    message.pose.covariance[0] = 0.04
    return message


def test_gate_requires_both_current_pose_and_explicit_validity() -> None:
    gate = MODULE.DevelopmentTfGate(0.5, "map")
    pose = _pose()

    assert not gate.accept_pose(pose, 1_100_000_000).should_broadcast
    assert gate.reason == "WAITING_FOR_VALID_LOCALIZATION"
    assert gate.set_source_valid(True, 1_100_000_000).should_broadcast
    assert not gate.evaluate(1_200_000_000).should_broadcast
    assert gate.reason == "CURRENT_SAMPLE_ALREADY_BROADCAST"


def test_validity_loss_and_timeout_stop_new_transform_samples() -> None:
    gate = MODULE.DevelopmentTfGate(0.5, "map")
    gate.accept_pose(_pose(), 1_000_000_000)
    assert gate.set_source_valid(True, 1_000_000_000).should_broadcast

    assert not gate.set_source_valid(False, 1_100_000_000).should_broadcast
    assert gate.cached_pose is None
    assert gate.reason == "LOCALIZATION_INVALID"

    assert not gate.accept_pose(_pose(2_000_000_000), 2_000_000_000).should_broadcast
    assert gate.set_source_valid(True, 2_000_000_000).should_broadcast
    assert not gate.evaluate(2_500_000_001).should_broadcast
    assert gate.reason == "FUSED_POSE_TIMEOUT"
    assert not gate.source_valid


def test_rejects_wrong_frame_time_order_and_clock_reset() -> None:
    gate = MODULE.DevelopmentTfGate(0.5, "map")
    wrong_frame = _pose()
    wrong_frame.header.frame_id = "odom"
    assert gate.accept_pose(wrong_frame, 1_000_000_000).reason == "FRAME_MISMATCH"
    assert not gate.source_valid

    assert gate.accept_pose(_pose(2_000_000_000), 2_000_000_000).reason in {
        "WAITING_FOR_VALID_LOCALIZATION",
        "WAITING_FOR_FUSED_POSE",
    }
    assert gate.accept_pose(_pose(1_900_000_000), 2_000_000_000).reason == (
        "OUT_OF_ORDER_TIMESTAMP"
    )
    assert not gate.source_valid
    gate.set_source_valid(True, 2_100_000_000)
    assert gate.evaluate(1_000_000_000).reason == "CLOCK_RESET"
    assert gate.cached_pose is None
    assert not gate.source_valid


def test_transform_split_preserves_source_stamp_and_pose() -> None:
    pose = _pose(3_250_000_000)
    transforms = MODULE.development_transforms(pose, "map", "odom", "base_link")

    assert len(transforms) == 2
    map_to_odom, odom_to_base = transforms
    assert map_to_odom.header.stamp == pose.header.stamp
    assert map_to_odom.header.frame_id == "map"
    assert map_to_odom.child_frame_id == "odom"
    assert map_to_odom.transform.translation.x == 0.0
    assert map_to_odom.transform.rotation.w == 1.0
    assert odom_to_base.header.stamp == pose.header.stamp
    assert odom_to_base.header.frame_id == "odom"
    assert odom_to_base.child_frame_id == "base_link"
    assert odom_to_base.transform.translation.x == pose.pose.pose.position.x
    assert odom_to_base.transform.translation.y == pose.pose.pose.position.y
    assert odom_to_base.transform.translation.z == pose.pose.pose.position.z
    assert odom_to_base.transform.rotation == pose.pose.pose.orientation


def test_configuration_is_disabled_and_prohibited_by_default() -> None:
    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "apriltag_development_tf_bridge"
    ]["ros__parameters"]
    source = SCRIPT.read_text(encoding="utf-8")

    assert parameters["allow_test_only_tf"] is False
    assert parameters["production_ekfs_enabled"] is False
    assert parameters["localization_profile"] == "apriltag_development_only"
    assert parameters["physical_robot_use"] == "prohibited"
    assert parameters["freshness_timeout_s"] == 0.5
    assert parameters["map_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_frame"] == "base_link"
    assert source.count("TransformBroadcaster(") == 1
    assert "StaticTransformBroadcaster" not in source
