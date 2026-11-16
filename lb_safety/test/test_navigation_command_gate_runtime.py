"""In-process ROS integration test for the Batch 1 localization command gate."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def _load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


LOCALIZATION = _load_module(
    "batch1_fused_pose_passthrough",
    REPOSITORY_ROOT / "lb_localization" / "scripts" / "fused_pose_passthrough.py",
)
GATE = _load_module(
    "batch1_navigation_command_gate",
    REPOSITORY_ROOT / "lb_safety" / "scripts" / "navigation_command_gate.py",
)


def _spin_until(executor, condition, timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return
    raise AssertionError("timed out waiting for ROS integration condition")


def test_fresh_pose_permits_command_and_timeout_publishes_zero() -> None:
    rclpy.init()
    localization = LOCALIZATION.FusedPosePassthrough()
    gate = GATE.NavigationCommandGate()
    probe = Node("batch1_runtime_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(localization)
    executor.add_node(gate)
    executor.add_node(probe)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    valid_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received_fused = []
    received_valid = []
    received_commands = []
    probe.create_subscription(
        PoseWithCovarianceStamped,
        "/localization/fused_pose",
        received_fused.append,
        qos,
    )
    probe.create_subscription(
        Bool,
        "/localization/valid",
        received_valid.append,
        valid_qos,
    )
    probe.create_subscription(
        TwistStamped,
        "/cmd_vel_nav",
        received_commands.append,
        qos,
    )
    pose_publisher = probe.create_publisher(
        PoseWithCovarianceStamped,
        "/localization/apriltag_pose",
        qos,
    )
    command_publisher = probe.create_publisher(TwistStamped, "/cmd_vel_nav_raw", qos)

    try:
        _spin_until(
            executor,
            lambda: pose_publisher.get_subscription_count() == 1
            and command_publisher.get_subscription_count() == 1,
        )

        blocked_command = TwistStamped()
        blocked_command.header.frame_id = "base_link"
        blocked_command.twist.linear.x = 0.2
        command_publisher.publish(blocked_command)
        _spin_until(executor, lambda: len(received_commands) >= 1)
        assert received_commands[-1].twist.linear.x == 0.0

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = probe.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.25
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.04
        pose_publisher.publish(pose)
        _spin_until(
            executor,
            lambda: received_fused
            and received_valid
            and received_valid[-1].data
            and gate.tracker.permitted(gate.get_clock().now().nanoseconds),
        )
        assert received_fused[-1] == pose

        permitted_command = TwistStamped()
        permitted_command.header.frame_id = "base_link"
        permitted_command.twist.linear.x = 0.2
        command_publisher.publish(permitted_command)
        _spin_until(
            executor,
            lambda: any(command.twist.linear.x == 0.2 for command in received_commands),
        )

        nonzero_index = max(
            index
            for index, command in enumerate(received_commands)
            if command.twist.linear.x == 0.2
        )
        # Force the upstream-validity-loss path to own the transition. This
        # reproduces the race where the pass-through reports stale before the
        # gate's independent freshness timer gets its next callback.
        gate.timer.cancel()
        _spin_until(
            executor,
            lambda: any(
                command.twist.linear.x == 0.0 for command in received_commands[nonzero_index + 1 :]
            ),
            timeout_s=1.5,
        )
        assert not gate.tracker.permitted(gate.get_clock().now().nanoseconds)
        assert gate.tracker.reason == "LOCALIZATION_INVALID"
    finally:
        executor.remove_node(probe)
        executor.remove_node(gate)
        executor.remove_node(localization)
        probe.destroy_node()
        gate.destroy_node()
        localization.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
