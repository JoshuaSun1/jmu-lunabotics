"""In-process ROS freshness and sole-authority test for the N3 TF bridge."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def _load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


LOCALIZATION = _load_module(
    "n3_fused_pose_passthrough",
    REPOSITORY_ROOT / "lb_localization" / "scripts" / "fused_pose_passthrough.py",
)
BRIDGE = _load_module(
    "n3_apriltag_development_tf_bridge",
    REPOSITORY_ROOT / "lb_localization" / "scripts" / "apriltag_development_tf_bridge.py",
)


def _spin_until(executor, condition, timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return
    raise AssertionError("timed out waiting for ROS integration condition")


def test_tf_is_sole_owned_timestamped_and_stops_on_localization_loss() -> None:
    rclpy.init()
    localization = LOCALIZATION.FusedPosePassthrough()
    bridge = BRIDGE.AprilTagDevelopmentTfBridge(
        parameter_overrides=[Parameter("allow_test_only_tf", value=True)]
    )
    probe = Node("n3_tf_runtime_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(localization)
    executor.add_node(bridge)
    executor.add_node(probe)

    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received_tf = []
    probe.create_subscription(TFMessage, "/tf", received_tf.append, qos)
    pose_publisher = probe.create_publisher(
        PoseWithCovarianceStamped,
        "/localization/apriltag_pose",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )

    try:
        _spin_until(executor, lambda: pose_publisher.get_subscription_count() == 1)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = probe.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.25
        pose.pose.pose.position.y = 0.75
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.04
        pose_publisher.publish(pose)

        _spin_until(
            executor,
            lambda: any(len(message.transforms) == 2 for message in received_tf),
        )
        message = next(message for message in received_tf if len(message.transforms) == 2)
        assert [
            (transform.header.frame_id, transform.child_frame_id)
            for transform in message.transforms
        ] == [("map", "odom"), ("odom", "base_link")]
        assert all(transform.header.stamp == pose.header.stamp for transform in message.transforms)
        assert message.transforms[0].transform.rotation.w == 1.0
        assert message.transforms[1].transform.translation.x == 1.25
        assert message.transforms[1].transform.translation.y == 0.75

        publishers = {
            information.node_name for information in probe.get_publishers_info_by_topic("/tf")
        }
        assert publishers == {"apriltag_development_tf_bridge"}

        tf_count = len(received_tf)
        deadline = time.monotonic() + 0.75
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(received_tf) == tf_count
        assert not bridge.gate.source_valid
        assert bridge.gate.reason in {
            "WAITING_FOR_VALID_LOCALIZATION",
            "FUSED_POSE_TIMEOUT",
        }

        reacquired_pose = PoseWithCovarianceStamped()
        reacquired_pose.header.frame_id = "map"
        reacquired_pose.header.stamp = probe.get_clock().now().to_msg()
        reacquired_pose.pose.pose.position.x = 1.5
        reacquired_pose.pose.pose.orientation.w = 1.0
        pose_publisher.publish(reacquired_pose)
        _spin_until(executor, lambda: len(received_tf) > tf_count)
        assert received_tf[-1].transforms[1].transform.translation.x == 1.5
    finally:
        executor.remove_node(probe)
        executor.remove_node(bridge)
        executor.remove_node(localization)
        probe.destroy_node()
        bridge.destroy_node()
        localization.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
