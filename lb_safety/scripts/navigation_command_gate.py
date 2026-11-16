#!/usr/bin/env python3
"""Fail-closed gate from raw Nav2 velocity commands to the canonical topic."""

from __future__ import annotations

import copy
from enum import Enum
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a ROS timestamp to integer nanoseconds."""
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def twist_is_finite(message: TwistStamped) -> bool:
    """Reject NaN and infinite command components."""
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def pose_is_valid(message: PoseWithCovarianceStamped) -> bool:
    """Reject malformed localization data independently at the safety boundary."""
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
        return False
    quaternion_norm_squared = (
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    if quaternion_norm_squared <= 1.0e-12:
        return False
    return not any(message.pose.covariance[index] < 0.0 for index in (0, 7, 14, 21, 28, 35))


class GateState(str, Enum):
    """Externally diagnosable command-gate states."""

    BLOCKED = "BLOCKED"
    PERMITTED = "PERMITTED"


class NavigationGateTracker:
    """Require both a valid signal and a fresh fused pose."""

    def __init__(self, freshness_timeout_s: float, expected_frame: str) -> None:
        if not math.isfinite(freshness_timeout_s) or freshness_timeout_s <= 0.0:
            raise ValueError("freshness_timeout_s must be finite and positive")
        if not expected_frame:
            raise ValueError("expected_frame must not be empty")
        self.timeout_ns = int(freshness_timeout_s * NANOSECONDS_PER_SECOND)
        self.expected_frame = expected_frame
        self.localization_valid = False
        self.last_pose_stamp_ns: int | None = None
        self.latest_observed_stamp_ns: int | None = None
        self.last_clock_ns: int | None = None
        self.reason = "NO_LOCALIZATION_VALIDITY"

    def update_clock(self, now_ns: int) -> bool:
        """Fail closed after simulated or system time moves backward."""
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.latest_observed_stamp_ns = None
            self.reason = "CLOCK_RESET"
            self.last_clock_ns = now_ns
            return True
        self.last_clock_ns = now_ns
        return False

    def set_localization_valid(self, valid: bool) -> None:
        """Apply the explicit upstream localization validity state."""
        self.localization_valid = valid
        if not valid:
            self.last_pose_stamp_ns = None
            self.reason = "LOCALIZATION_INVALID"

    def observe_pose(self, frame_id: str, stamp_ns: int, now_ns: int) -> bool:
        """Record a fused pose only when it is independently fresh and valid."""
        self.update_clock(now_ns)
        if frame_id != self.expected_frame:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_FRAME_MISMATCH"
            return False
        if stamp_ns <= 0 or stamp_ns > now_ns:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_TIMESTAMP_INVALID"
            return False
        if now_ns - stamp_ns > self.timeout_ns:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_STALE_ON_ARRIVAL"
            return False
        if self.latest_observed_stamp_ns is not None and stamp_ns < self.latest_observed_stamp_ns:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_OUT_OF_ORDER"
            return False
        self.latest_observed_stamp_ns = stamp_ns
        self.last_pose_stamp_ns = stamp_ns
        return True

    def permitted(self, now_ns: int) -> bool:
        """Return true only while both validity and pose freshness hold."""
        if self.update_clock(now_ns):
            return False
        if not self.localization_valid:
            return False
        if self.last_pose_stamp_ns is None:
            self.reason = "NO_FUSED_POSE"
            return False
        age_ns = now_ns - self.last_pose_stamp_ns
        if age_ns < 0:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_FROM_FUTURE"
            return False
        if age_ns > self.timeout_ns:
            self.localization_valid = False
            self.last_pose_stamp_ns = None
            self.reason = "FUSED_POSE_TIMEOUT"
            return False
        self.reason = "LOCALIZATION_CURRENT"
        return True


def zero_twist(stamp, frame_id: str = "base_link") -> TwistStamped:
    """Build an explicit all-zero stamped command."""
    message = TwistStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    return message


class NavigationCommandGate(Node):
    """Prevent raw Nav2 commands from leaving the navigation boundary."""

    def __init__(self) -> None:
        super().__init__("navigation_command_gate")
        self.declare_parameter("raw_cmd_topic", "/cmd_vel_nav_raw")
        self.declare_parameter("gated_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("fused_pose_topic", "/localization/fused_pose")
        self.declare_parameter("valid_topic", "/localization/valid")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("freshness_timeout_s", 0.5)
        self.declare_parameter("check_period_s", 0.05)

        freshness_timeout_s = float(self.get_parameter("freshness_timeout_s").value)
        check_period_s = float(self.get_parameter("check_period_s").value)
        if not math.isfinite(check_period_s) or check_period_s <= 0.0:
            raise ValueError("check_period_s must be finite and positive")
        self.raw_cmd_topic = str(self.get_parameter("raw_cmd_topic").value)
        self.gated_cmd_topic = str(self.get_parameter("gated_cmd_topic").value)
        self.tracker = NavigationGateTracker(
            freshness_timeout_s,
            str(self.get_parameter("expected_frame").value),
        )

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        valid_qos = QoSProfile(depth=1)
        valid_qos.reliability = ReliabilityPolicy.RELIABLE
        valid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.command_publisher = self.create_publisher(
            TwistStamped,
            self.gated_cmd_topic,
            reliable_qos,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            reliable_qos,
        )
        self.command_subscription = self.create_subscription(
            TwistStamped,
            self.raw_cmd_topic,
            self._command_callback,
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

        self.last_command_frame = "base_link"
        self.previous_gate_state = GateState.BLOCKED
        self.last_report_ns: int | None = None
        self.report_interval_ns = NANOSECONDS_PER_SECOND
        self.timer = self.create_timer(check_period_s, self._check)
        self._publish_zero()
        self._publish_diagnostics(self.get_clock().now().nanoseconds, force=True)
        self.get_logger().warn(
            "Development Nav2 command gate starts BLOCKED and publishes only "
            f"{self.gated_cmd_topic}; it has no /cmd_vel or motor connection."
        )

    def _valid_callback(self, message: Bool) -> None:
        was_permitted = self.previous_gate_state == GateState.PERMITTED
        self.tracker.set_localization_valid(bool(message.data))
        now_ns = self.get_clock().now().nanoseconds
        is_permitted = self.tracker.permitted(now_ns)
        if was_permitted and not is_permitted:
            self._publish_zero()
        self._publish_diagnostics(now_ns, force=True)

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        was_permitted = self.previous_gate_state == GateState.PERMITTED
        if not pose_is_valid(message):
            self.tracker.localization_valid = False
            self.tracker.last_pose_stamp_ns = None
            self.tracker.reason = "FUSED_POSE_CONTENT_INVALID"
        else:
            self.tracker.observe_pose(
                message.header.frame_id,
                stamp_to_nanoseconds(message.header.stamp),
                now_ns,
            )
        if was_permitted and not self.tracker.permitted(now_ns):
            self._publish_zero()
        self._publish_diagnostics(now_ns, force=True)

    def _command_callback(self, message: TwistStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if message.header.frame_id:
            self.last_command_frame = message.header.frame_id
        if not twist_is_finite(message):
            self._publish_zero()
            self._publish_diagnostics(
                now_ns,
                force=True,
                reason_override="NONFINITE_NAV2_COMMAND",
            )
            return
        if self.tracker.permitted(now_ns):
            self.command_publisher.publish(copy.deepcopy(message))
        else:
            self._publish_zero()
        self._publish_diagnostics(now_ns, force=True)

    def _check(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        is_permitted = self.tracker.permitted(now_ns)
        state = GateState.PERMITTED if is_permitted else GateState.BLOCKED
        if self.previous_gate_state == GateState.PERMITTED and state == GateState.BLOCKED:
            self._publish_zero()
        interval_elapsed = (
            self.last_report_ns is None or now_ns - self.last_report_ns >= self.report_interval_ns
        )
        self._publish_diagnostics(
            now_ns,
            force=state != self.previous_gate_state or interval_elapsed,
        )
        self.previous_gate_state = state

    def _publish_zero(self) -> None:
        self.command_publisher.publish(
            zero_twist(self.get_clock().now().to_msg(), self.last_command_frame)
        )

    def _publish_diagnostics(
        self,
        now_ns: int,
        force: bool = False,
        reason_override: str | None = None,
    ) -> None:
        permitted = self.tracker.permitted(now_ns)
        state = GateState.PERMITTED if permitted else GateState.BLOCKED
        if not force and state == self.previous_gate_state:
            return
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if permitted else DiagnosticStatus.WARN
        status.name = "lb_safety/navigation_command_gate"
        status.hardware_id = "software_only"
        status.message = state.value
        status.values = [
            KeyValue(
                key="reason",
                value=reason_override or self.tracker.reason,
            ),
            KeyValue(key="raw_topic", value=self.raw_cmd_topic),
            KeyValue(key="gated_topic", value=self.gated_cmd_topic),
            KeyValue(key="physical_motor_output", value="false"),
            KeyValue(
                key="freshness_timeout_s",
                value=f"{self.tracker.timeout_ns / NANOSECONDS_PER_SECOND:.3f}",
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)
        self.previous_gate_state = state
        self.last_report_ns = now_ns


def main(args=None) -> None:
    """Run the development-only navigation command gate."""
    rclpy.init(args=args)
    node = NavigationCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
