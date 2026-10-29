#!/usr/bin/env python3
"""Publish test-only Nav2 TF from a current AprilTag-derived fused pose."""

from __future__ import annotations

import copy
import math
from typing import NamedTuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a ROS timestamp to integer nanoseconds."""
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def pose_validation_error(message: PoseWithCovarianceStamped) -> str | None:
    """Return a fail-closed reason for malformed development pose content."""
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


class DevelopmentTfDecision(NamedTuple):
    """Result of checking whether one cached pose may create TF samples."""

    should_broadcast: bool
    reason: str


class DevelopmentTfGate:
    """Require a fresh pose and explicit upstream validity for each TF sample."""

    def __init__(self, freshness_timeout_s: float, expected_frame: str) -> None:
        if not math.isfinite(freshness_timeout_s) or freshness_timeout_s <= 0.0:
            raise ValueError("freshness_timeout_s must be finite and positive")
        if not expected_frame:
            raise ValueError("expected_frame must not be empty")
        self.timeout_ns = int(freshness_timeout_s * NANOSECONDS_PER_SECOND)
        self.expected_frame = expected_frame
        self.source_valid = False
        self.cached_pose: PoseWithCovarianceStamped | None = None
        self.latest_pose_stamp_ns: int | None = None
        self.last_broadcast_stamp_ns: int | None = None
        self.last_clock_ns: int | None = None
        self.reason = "WAITING_FOR_VALID_LOCALIZATION"

    def _update_clock(self, now_ns: int) -> bool:
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.source_valid = False
            self.cached_pose = None
            self.latest_pose_stamp_ns = None
            self.last_broadcast_stamp_ns = None
            self.last_clock_ns = now_ns
            self.reason = "CLOCK_RESET"
            return False
        self.last_clock_ns = now_ns
        return True

    def set_source_valid(self, valid: bool, now_ns: int) -> DevelopmentTfDecision:
        """Apply the upstream transient-local localization-valid signal."""
        if not self._update_clock(now_ns):
            return DevelopmentTfDecision(False, self.reason)
        self.source_valid = valid
        if not valid:
            self.cached_pose = None
            self.reason = "LOCALIZATION_INVALID"
            return DevelopmentTfDecision(False, self.reason)
        return self.evaluate(now_ns)

    def accept_pose(self, message: PoseWithCovarianceStamped, now_ns: int) -> DevelopmentTfDecision:
        """Cache one valid fused pose without altering its source timestamp."""
        if not self._update_clock(now_ns):
            return DevelopmentTfDecision(False, self.reason)
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        error = pose_validation_error(message)
        if error is None and message.header.frame_id != self.expected_frame:
            error = "FRAME_MISMATCH"
        if error is None and stamp_ns <= 0:
            error = "ZERO_TIMESTAMP"
        if error is None and stamp_ns > now_ns:
            error = "FUTURE_TIMESTAMP"
        if error is None and now_ns - stamp_ns > self.timeout_ns:
            error = "STALE_ON_ARRIVAL"
        if (
            error is None
            and self.latest_pose_stamp_ns is not None
            and stamp_ns < self.latest_pose_stamp_ns
        ):
            error = "OUT_OF_ORDER_TIMESTAMP"
        if error is not None:
            self.cached_pose = None
            self.source_valid = False
            self.reason = error
            return DevelopmentTfDecision(False, self.reason)

        self.cached_pose = copy.deepcopy(message)
        self.latest_pose_stamp_ns = stamp_ns
        self.reason = "WAITING_FOR_VALID_LOCALIZATION"
        return self.evaluate(now_ns)

    def evaluate(self, now_ns: int) -> DevelopmentTfDecision:
        """Return true exactly once for a current, validity-approved sample."""
        if not self._update_clock(now_ns):
            return DevelopmentTfDecision(False, self.reason)
        if not self.source_valid:
            self.reason = "WAITING_FOR_VALID_LOCALIZATION"
            return DevelopmentTfDecision(False, self.reason)
        if self.cached_pose is None:
            self.reason = "WAITING_FOR_FUSED_POSE"
            return DevelopmentTfDecision(False, self.reason)

        stamp_ns = stamp_to_nanoseconds(self.cached_pose.header.stamp)
        if now_ns - stamp_ns > self.timeout_ns:
            self.cached_pose = None
            self.source_valid = False
            self.reason = "FUSED_POSE_TIMEOUT"
            return DevelopmentTfDecision(False, self.reason)
        if self.last_broadcast_stamp_ns == stamp_ns:
            self.reason = "CURRENT_SAMPLE_ALREADY_BROADCAST"
            return DevelopmentTfDecision(False, self.reason)

        self.last_broadcast_stamp_ns = stamp_ns
        self.reason = "BROADCAST_CURRENT_FUSED_POSE"
        return DevelopmentTfDecision(True, self.reason)

    def age_seconds(self, now_ns: int) -> float | None:
        """Return cached pose age for diagnostics."""
        if self.cached_pose is None:
            return None
        return (now_ns - stamp_to_nanoseconds(self.cached_pose.header.stamp)) / (
            NANOSECONDS_PER_SECOND
        )


def development_transforms(
    pose: PoseWithCovarianceStamped,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
) -> list[TransformStamped]:
    """Create the temporary map=odom split without refreshing the pose stamp."""
    map_to_odom = TransformStamped()
    map_to_odom.header.stamp = copy.deepcopy(pose.header.stamp)
    map_to_odom.header.frame_id = map_frame
    map_to_odom.child_frame_id = odom_frame
    map_to_odom.transform.rotation.w = 1.0

    odom_to_base = TransformStamped()
    odom_to_base.header.stamp = copy.deepcopy(pose.header.stamp)
    odom_to_base.header.frame_id = odom_frame
    odom_to_base.child_frame_id = base_frame
    odom_to_base.transform.translation.x = pose.pose.pose.position.x
    odom_to_base.transform.translation.y = pose.pose.pose.position.y
    odom_to_base.transform.translation.z = pose.pose.pose.position.z
    odom_to_base.transform.rotation = copy.deepcopy(pose.pose.pose.orientation)
    return [map_to_odom, odom_to_base]


class AprilTagDevelopmentTfBridge(Node):
    """Own both mock-only TF edges while explicitly authorized and fresh."""

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("apriltag_development_tf_bridge", parameter_overrides=parameter_overrides)
        self.declare_parameter("allow_test_only_tf", False)
        self.declare_parameter("production_ekfs_enabled", False)
        self.declare_parameter("localization_profile", "apriltag_development_only")
        self.declare_parameter("physical_robot_use", "prohibited")
        self.declare_parameter("fused_pose_topic", "/localization/fused_pose")
        self.declare_parameter("valid_topic", "/localization/valid")
        self.declare_parameter("status_topic", "/diagnostics")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("freshness_timeout_s", 0.5)
        self.declare_parameter("poll_period_s", 0.05)

        if not bool(self.get_parameter("allow_test_only_tf").value):
            raise RuntimeError("test-only TF requires allow_test_only_tf:=true")
        if bool(self.get_parameter("production_ekfs_enabled").value):
            raise RuntimeError("development TF and production EKF TF publishers are exclusive")
        if self.get_parameter("localization_profile").value != "apriltag_development_only":
            raise RuntimeError(
                "development TF requires localization_profile=apriltag_development_only"
            )
        if self.get_parameter("physical_robot_use").value != "prohibited":
            raise RuntimeError("development TF is prohibited from physical robot use")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        if len({self.map_frame, self.odom_frame, self.base_frame}) != 3:
            raise ValueError("map_frame, odom_frame, and base_frame must be distinct")
        poll_period_s = float(self.get_parameter("poll_period_s").value)
        if not math.isfinite(poll_period_s) or poll_period_s <= 0.0:
            raise ValueError("poll_period_s must be finite and positive")

        self.gate = DevelopmentTfGate(
            float(self.get_parameter("freshness_timeout_s").value), self.map_frame
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        valid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.broadcaster = TransformBroadcaster(self)
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            reliable_qos,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("fused_pose_topic").value),
            self._pose_callback,
            reliable_qos,
        )
        self.valid_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("valid_topic").value),
            self._valid_callback,
            valid_qos,
        )
        self.timer = self.create_timer(poll_period_s, self._poll)
        self.last_report_reason = ""
        self.last_report_ns: int | None = None
        self.report_interval_ns = NANOSECONDS_PER_SECOND
        self._report(self.get_clock().now().nanoseconds, force=True)
        self.get_logger().warn(
            "TEST ONLY: owns map->odom and odom->base_link from intermittent "
            "AprilTag pose; map=odom, no propagation, physical use prohibited."
        )

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        decision = self.gate.accept_pose(message, now_ns)
        self._apply(decision, now_ns)

    def _valid_callback(self, message: Bool) -> None:
        now_ns = self.get_clock().now().nanoseconds
        decision = self.gate.set_source_valid(message.data, now_ns)
        self._apply(decision, now_ns)

    def _poll(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        decision = self.gate.evaluate(now_ns)
        interval_elapsed = (
            self.last_report_ns is None or now_ns - self.last_report_ns >= self.report_interval_ns
        )
        self._apply(decision, now_ns, force_report=interval_elapsed)

    def _apply(
        self,
        decision: DevelopmentTfDecision,
        now_ns: int,
        force_report: bool = False,
    ) -> None:
        if decision.should_broadcast:
            if self.gate.cached_pose is None:
                raise RuntimeError("TF gate approved a missing cached pose")
            transforms = development_transforms(
                self.gate.cached_pose,
                self.map_frame,
                self.odom_frame,
                self.base_frame,
            )
            self.broadcaster.sendTransform(transforms)
        self._report(now_ns, force=force_report or decision.should_broadcast)

    def _report(self, now_ns: int, force: bool = False) -> None:
        reason_changed = self.gate.reason != self.last_report_reason
        if not force and not reason_changed:
            return
        age = self.gate.age_seconds(now_ns)
        broadcasting_current = self.gate.reason in {
            "BROADCAST_CURRENT_FUSED_POSE",
            "CURRENT_SAMPLE_ALREADY_BROADCAST",
        }
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if broadcasting_current else DiagnosticStatus.WARN
        status.name = "lb_localization/apriltag_development_tf_bridge"
        status.hardware_id = "test_only_software"
        status.message = "CURRENT_TEST_TF" if broadcasting_current else "TF_NOT_CURRENT"
        status.values = [
            KeyValue(key="reason", value=self.gate.reason),
            KeyValue(key="profile", value="apriltag_development_only"),
            KeyValue(key="physical_robot_use", value="prohibited"),
            KeyValue(key="map_to_odom_owner", value=self.get_name()),
            KeyValue(key="odom_to_base_link_owner", value=self.get_name()),
            KeyValue(key="map_odom_relationship", value="temporary_identity"),
            KeyValue(key="propagates_between_detections", value="false"),
            KeyValue(
                key="freshness_timeout_s",
                value=f"{self.gate.timeout_ns / NANOSECONDS_PER_SECOND:.3f}",
            ),
            KeyValue(key="pose_age_s", value="unavailable" if age is None else f"{age:.6f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_publisher.publish(array)
        self.last_report_reason = self.gate.reason
        self.last_report_ns = now_ns


def main(args=None) -> None:
    """Run the explicitly authorized development-only TF bridge."""
    rclpy.init(args=args)
    node = AprilTagDevelopmentTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
