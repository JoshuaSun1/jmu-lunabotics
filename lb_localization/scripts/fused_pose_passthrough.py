#!/usr/bin/env python3
"""Publish a development fused pose only while AprilTag localization is fresh."""

from __future__ import annotations

import copy
from enum import Enum
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


NANOSECONDS_PER_SECOND = 1_000_000_000


class LocalizationState(str, Enum):
    """Development localization states exposed to the command gate."""

    UNAVAILABLE = "UNAVAILABLE"
    VALID = "VALID"
    STALE = "STALE"
    INVALID = "INVALID"


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a ROS builtin_interfaces/Time value to integer nanoseconds."""
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def pose_validation_error(message: PoseWithCovarianceStamped) -> str | None:
    """Return a fail-closed reason for malformed pose or covariance data."""
    pose = message.pose.pose
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        *message.pose.covariance,
    )
    if not all(math.isfinite(value) for value in values):
        return "NONFINITE_POSE_OR_COVARIANCE"

    quaternion_norm_squared = (
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    if quaternion_norm_squared <= 1.0e-12:
        return "INVALID_ORIENTATION_QUATERNION"

    if any(message.pose.covariance[index] < 0.0 for index in (0, 7, 14, 21, 28, 35)):
        return "NEGATIVE_COVARIANCE_DIAGONAL"
    return None


class PoseFreshnessTracker:
    """Validate an event-driven absolute pose without propagating it."""

    def __init__(self, freshness_timeout_s: float, expected_frame: str) -> None:
        if not math.isfinite(freshness_timeout_s) or freshness_timeout_s <= 0.0:
            raise ValueError("freshness_timeout_s must be finite and positive")
        if not expected_frame:
            raise ValueError("expected_frame must not be empty")
        self.timeout_ns = int(freshness_timeout_s * NANOSECONDS_PER_SECOND)
        self.expected_frame = expected_frame
        self.state = LocalizationState.UNAVAILABLE
        self.reason = "NO_ACCEPTED_POSE"
        self.last_pose_stamp_ns: int | None = None
        self.latest_accepted_stamp_ns: int | None = None
        self.last_clock_ns: int | None = None

    def _invalidate(self, state: LocalizationState, reason: str) -> None:
        self.state = state
        self.reason = reason
        self.last_pose_stamp_ns = None

    def reject(self, reason: str) -> None:
        """Reject a malformed sample and discard previously accepted freshness."""
        self._invalidate(LocalizationState.INVALID, reason)

    def update_clock(self, now_ns: int) -> bool:
        """Reset on a backward clock jump and return whether one occurred."""
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self._invalidate(LocalizationState.UNAVAILABLE, "CLOCK_RESET")
            self.latest_accepted_stamp_ns = None
            self.last_clock_ns = now_ns
            return True
        self.last_clock_ns = now_ns
        return False

    def accept(self, frame_id: str, stamp_ns: int, now_ns: int) -> bool:
        """Accept one pose only when its frame and timestamp are valid."""
        self.update_clock(now_ns)
        if frame_id != self.expected_frame:
            self._invalidate(LocalizationState.INVALID, "FRAME_MISMATCH")
            return False
        if stamp_ns <= 0:
            self._invalidate(LocalizationState.INVALID, "ZERO_TIMESTAMP")
            return False
        if stamp_ns > now_ns:
            self._invalidate(LocalizationState.INVALID, "FUTURE_TIMESTAMP")
            return False
        if self.latest_accepted_stamp_ns is not None and stamp_ns < self.latest_accepted_stamp_ns:
            self._invalidate(LocalizationState.INVALID, "OUT_OF_ORDER_TIMESTAMP")
            return False
        if now_ns - stamp_ns > self.timeout_ns:
            self._invalidate(LocalizationState.STALE, "STALE_ON_ARRIVAL")
            return False

        self.latest_accepted_stamp_ns = stamp_ns
        self.last_pose_stamp_ns = stamp_ns
        self.state = LocalizationState.VALID
        self.reason = "ACCEPTED_APRILTAG_POSE"
        return True

    def tick(self, now_ns: int) -> LocalizationState:
        """Update freshness without creating a new pose sample."""
        if self.update_clock(now_ns):
            return self.state
        if self.last_pose_stamp_ns is None:
            return self.state
        if now_ns - self.last_pose_stamp_ns > self.timeout_ns:
            self._invalidate(LocalizationState.STALE, "APRILTAG_POSE_TIMEOUT")
        return self.state

    def age_seconds(self, now_ns: int) -> float | None:
        """Return current pose age, or None when no accepted pose remains."""
        if self.last_pose_stamp_ns is None:
            return None
        return (now_ns - self.last_pose_stamp_ns) / NANOSECONDS_PER_SECOND


class FusedPosePassthrough(Node):
    """Expose a stable fused-pose topic without claiming real fusion."""

    def __init__(self) -> None:
        super().__init__("fused_pose_passthrough")
        self.declare_parameter("input_topic", "/localization/apriltag_pose")
        self.declare_parameter("output_topic", "/localization/fused_pose")
        self.declare_parameter("valid_topic", "/localization/valid")
        self.declare_parameter("status_topic", "/localization/status")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("freshness_timeout_s", 0.5)
        self.declare_parameter("poll_period_s", 0.05)

        freshness_timeout_s = float(self.get_parameter("freshness_timeout_s").value)
        poll_period_s = float(self.get_parameter("poll_period_s").value)
        if not math.isfinite(poll_period_s) or poll_period_s <= 0.0:
            raise ValueError("poll_period_s must be finite and positive")

        self.tracker = PoseFreshnessTracker(
            freshness_timeout_s,
            str(self.get_parameter("expected_frame").value),
        )
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        valid_qos = QoSProfile(depth=1)
        valid_qos.reliability = ReliabilityPolicy.RELIABLE
        valid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_topic").value),
            reliable_qos,
        )
        self.valid_publisher = self.create_publisher(
            Bool, str(self.get_parameter("valid_topic").value), valid_qos
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            reliable_qos,
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("input_topic").value),
            self._pose_callback,
            reliable_qos,
        )
        self.last_reported_state: LocalizationState | None = None
        self.last_report_ns: int | None = None
        self.report_interval_ns = NANOSECONDS_PER_SECOND
        self.timer = self.create_timer(poll_period_s, self._poll)
        self._report(self.get_clock().now().nanoseconds, force=True)

        self.get_logger().warn(
            "Development-only fused pose: direct AprilTag pass-through, "
            f"{freshness_timeout_s:.2f} s timeout, no TF or propagation."
        )

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        validation_error = pose_validation_error(message)
        if validation_error is not None:
            self.tracker.reject(validation_error)
        elif self.tracker.accept(message.header.frame_id, stamp_ns, now_ns):
            self.pose_publisher.publish(copy.deepcopy(message))
        self._report(now_ns, force=True)

    def _poll(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        previous = self.tracker.state
        self.tracker.tick(now_ns)
        interval_elapsed = (
            self.last_report_ns is None or now_ns - self.last_report_ns >= self.report_interval_ns
        )
        self._report(
            now_ns,
            force=self.tracker.state != previous or interval_elapsed,
        )

    def _report(self, now_ns: int, force: bool = False) -> None:
        state_changed = self.tracker.state != self.last_reported_state
        if not force and not state_changed:
            return

        valid = self.tracker.state == LocalizationState.VALID
        valid_message = Bool()
        valid_message.data = valid
        self.valid_publisher.publish(valid_message)

        level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        if self.tracker.state == LocalizationState.INVALID:
            level = DiagnosticStatus.ERROR
        age = self.tracker.age_seconds(now_ns)
        status = DiagnosticStatus()
        status.level = level
        status.name = "lb_localization/fused_pose_passthrough"
        status.hardware_id = "software_only"
        status.message = self.tracker.state.value
        status.values = [
            KeyValue(key="source", value="apriltag_pose_direct_passthrough"),
            KeyValue(key="reason", value=self.tracker.reason),
            KeyValue(
                key="freshness_timeout_s",
                value=f"{self.tracker.timeout_ns / NANOSECONDS_PER_SECOND:.3f}",
            ),
            KeyValue(
                key="pose_age_s",
                value="unavailable" if age is None else f"{age:.6f}",
            ),
            KeyValue(key="publishes_tf", value="false"),
            KeyValue(key="continuous_localization", value="false"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_publisher.publish(array)
        self.last_reported_state = self.tracker.state
        self.last_report_ns = now_ns


def main(args=None) -> None:
    """Run the development-only fused-pose pass-through."""
    rclpy.init(args=args)
    node = FusedPosePassthrough()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
