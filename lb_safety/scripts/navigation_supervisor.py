#!/usr/bin/env python3
"""Mock-only goal admission, cancellation, lifecycle supervision and final command gate."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def bounded_command(message, now_ns, timeout_s, linear_limit, angular_limit):
    """Reject stale, nonplanar, nonfinite or out-of-envelope commands."""
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    age = (now_ns - stamp_ns(message.header.stamp)) / 1.0e9
    return (
        stamp_ns(message.header.stamp) > 0
        and 0 <= age <= timeout_s
        and message.header.frame_id in {"", "base_link"}
        and all(math.isfinite(value) for value in values)
        and all(value == 0.0 for value in (values[1], values[2], values[3], values[4]))
        and abs(values[0]) <= linear_limit + 1.0e-6
        and abs(values[5]) <= angular_limit + 1.0e-6
    )


@dataclass
class Session:
    public: object
    completion: Future
    started: float
    command_epoch_ns: int = 0
    backend: object = None
    reason: str = ""
    backend_finished: bool = False


class NavigationSupervisor(Node):
    """No physical enable path. Replace this profile only after hardware validation."""

    def __init__(self):
        super().__init__("navigation_supervisor")
        self.declare_parameter("allow_mock_navigation", False)
        if self.get_parameter("allow_mock_navigation").value is not True:
            raise ValueError("Supervisor requires allow_mock_navigation:=true")
        for name, value in {
            "freshness_timeout_s": 0.5,
            "command_timeout_s": 0.25,
            "lifecycle_timeout_s": 1.0,
            "goal_timeout_s": 120.0,
            "max_linear_velocity_mps": 0.5,
            "max_angular_velocity_radps": 1.0,
        }.items():
            self.declare_parameter(name, value)
            configured = self.get_parameter(name).value
            if not math.isfinite(configured) or configured <= 0:
                raise ValueError(f"Invalid {name}")
        self.group = ReentrantCallbackGroup()
        # A single executor thread serializes state updates; reentrant action
        # callbacks permit async results without blocking service/timer work.
        self.active = self.draining = None
        self.reserved = False
        self.pose = self.odom = self.command = None
        self.valid = False
        self.sensor_stamps = {}
        self.states = {}
        self.last_clock = None
        self.last_clock_progress = time.monotonic()
        self.command_received = 0.0
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/localization/valid", self.validity, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/localization/fused_pose", self.pose_input, 10
        )
        self.create_subscription(Odometry, "/odometry/local", self.odom_input, 10)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self.command_input, 10)
        self.create_subscription(
            LaserScan,
            "/sim/scan",
            lambda msg: self.sensor_input("scan", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/sim/hazards",
            lambda msg: self.sensor_input("hazards", msg),
            qos_profile_sensor_data,
        )
        self.output = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.ready_output = self.create_publisher(Bool, "/navigation/ready", qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.backend = ActionClient(
            self, NavigateToPose, "/navigation/internal/navigate_to_pose", callback_group=self.group
        )
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            self.execute,
            goal_callback=self.admit,
            cancel_callback=self.cancel,
            callback_group=self.group,
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in (
                "map_server",
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
                "velocity_smoother",
            )
        }
        self.pending_states = {}
        self.create_timer(0.25, self.poll_states, clock=self.watchdog_clock)
        self.create_timer(0.05, self.tick, clock=self.watchdog_clock)
        self.get_logger().warn("MOCK ONLY supervisor owns /cmd_vel; physical use prohibited")

    def parameter(self, name):
        return self.get_parameter(name).value

    def validity(self, message):
        self.valid = message.data

    def pose_input(self, message):
        self.pose = message

    def odom_input(self, message):
        self.odom = message

    def command_input(self, message):
        if self.active is None or self.active.backend_finished or self.active.completion.done():
            self.command = None
            return
        self.command = message
        self.command_received = time.monotonic()

    def sensor_input(self, name, message):
        if message.header.frame_id == "base_link":
            self.sensor_stamps[name] = stamp_ns(message.header.stamp)

    def fresh(self, stamp, now):
        return stamp > 0 and 0 <= (now - stamp) / 1.0e9 <= self.parameter("freshness_timeout_s")

    def reason(self):
        now = self.get_clock().now().nanoseconds
        if time.monotonic() - self.last_clock_progress > self.parameter("freshness_timeout_s"):
            return "CLOCK_STALLED"
        if not self.valid or self.pose is None or self.pose.header.frame_id != "map":
            return "LOCALIZATION_INVALID"
        if not self.fresh(stamp_ns(self.pose.header.stamp), now):
            return "LOCALIZATION_STALE"
        if self.odom is None or not self.fresh(stamp_ns(self.odom.header.stamp), now):
            return "ODOMETRY_STALE"
        for name in ("scan", "hazards"):
            if not self.fresh(self.sensor_stamps.get(name, 0), now):
                return f"{name.upper()}_STALE"
        for parent, child in (("map", "odom"), ("odom", "base_link")):
            try:
                transform = self.tf.lookup_transform(parent, child, Time())
                if not self.fresh(stamp_ns(transform.header.stamp), now):
                    return "TF_STALE"
            except TransformException:
                return "TF_UNAVAILABLE"
        for name in self.lifecycle_clients:
            state, observed = self.states.get(name, (0, 0.0))
            if state != 3 or time.monotonic() - observed > self.parameter("lifecycle_timeout_s"):
                return f"{name.upper()}_UNAVAILABLE"
        if not self.backend.server_is_ready():
            return "NAVIGATOR_UNAVAILABLE"
        return ""

    def poll_states(self):
        for name, client in self.lifecycle_clients.items():
            future = self.pending_states.get(name)
            if future is not None and not future.done():
                continue
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                self.pending_states[name] = future
                future.add_done_callback(lambda done, server=name: self.record_state(server, done))

    def record_state(self, name, future):
        try:
            self.states[name] = (future.result().current_state.id, time.monotonic())
        except Exception:
            self.states[name] = (0, time.monotonic())

    def admit(self, request):
        pose = request.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        norm = sum(value * value for value in values[3:])
        valid_goal = (
            request.pose.header.frame_id == "map"
            and not request.behavior_tree
            and all(math.isfinite(value) for value in values)
            and abs(norm - 1.0) < 0.01
        )
        if self.reserved or self.draining is not None or self.reason() or not valid_goal:
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        if self.active is not None and self.active.public == handle:
            self.halt(self.active, "USER_CANCELLED")
        return CancelResponse.ACCEPT

    def halt(self, session, reason):
        if session.reason:
            return
        session.reason = reason
        if session.backend is not None:
            session.backend.cancel_goal_async()
        if not session.completion.done():
            session.completion.set_result(None)
        self.publish_zero()

    async def execute(self, handle):
        session = Session(handle, Future(), time.monotonic(), self.get_clock().now().nanoseconds)
        self.active = session
        try:
            reason = self.reason()
            if handle.is_cancel_requested:
                session.backend_finished = True
                self.halt(session, "USER_CANCELLED")
            elif reason:
                session.backend_finished = True
                self.halt(session, reason)
            else:
                future = self.backend.send_goal_async(
                    copy.deepcopy(handle.request),
                    feedback_callback=lambda msg: (
                        handle.publish_feedback(msg.feedback) if handle.is_active else None
                    ),
                )
                future.add_done_callback(lambda done: self.backend_accepted(session, done))
            response = await session.completion
            if session.reason:
                if session.reason == "USER_CANCELLED" and handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                return NavigateToPose.Result(error_code=1, error_msg=session.reason)
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                handle.succeed()
            elif response.status == GoalStatus.STATUS_CANCELED:
                handle.canceled()
            else:
                handle.abort()
                if not response.result.error_msg:
                    response.result.error_msg = "NAVIGATOR_ABORTED"
                if response.result.error_code == 0:
                    response.result.error_code = 1
            return response.result
        finally:
            self.publish_zero()
            self.command = None
            self.active = None
            self.reserved = False
            if not session.backend_finished:
                self.draining = session

    def backend_accepted(self, session, future):
        try:
            session.backend = future.result()
            if not session.backend.accepted:
                session.backend_finished = True
                self.halt(session, "NAVIGATOR_REJECTED")
                return
            result = session.backend.get_result_async()
            result.add_done_callback(lambda done: self.backend_finished(session, done))
            if session.reason:
                session.backend.cancel_goal_async()
        except Exception:
            session.backend_finished = True
            self.halt(session, "NAVIGATOR_ERROR")

    def backend_finished(self, session, future):
        session.backend_finished = True
        self.command = None
        self.publish_zero()
        if self.draining is session:
            self.draining = None
        if not session.completion.done():
            try:
                session.completion.set_result(future.result())
            except Exception:
                self.halt(session, "NAVIGATOR_ERROR")

    def publish_zero(self):
        message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = self.get_clock().now().to_msg()
        self.output.publish(message)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock != now:
            self.last_clock_progress = time.monotonic()
        reason = self.reason()
        if self.last_clock is not None and now < self.last_clock:
            reason = "CLOCK_REWIND"
            self.command = None
            self.valid = False
            self.pose = self.odom = None
            self.sensor_stamps.clear()
            self.states.clear()
        self.last_clock = now
        session = self.active
        if session is not None:
            if session.public.is_cancel_requested:
                self.halt(session, "USER_CANCELLED")
            elif reason:
                self.halt(session, reason)
            elif time.monotonic() - session.started > self.parameter("goal_timeout_s"):
                self.halt(session, "GOAL_TIMEOUT")
        permitted = (
            session is not None
            and not session.reason
            and not reason
            and not session.backend_finished
            and not session.completion.done()
        )
        valid_command = (
            self.command is not None
            and session is not None
            and stamp_ns(self.command.header.stamp) >= session.command_epoch_ns
            and time.monotonic() - self.command_received <= self.parameter("command_timeout_s")
            and bounded_command(
                self.command,
                now,
                self.parameter("command_timeout_s"),
                self.parameter("max_linear_velocity_mps"),
                self.parameter("max_angular_velocity_radps"),
            )
        )
        if permitted and valid_command:
            self.output.publish(copy.deepcopy(self.command))
        else:
            self.publish_zero()
        self.ready_output.publish(Bool(data=not reason and self.draining is None))
        status = DiagnosticStatus(
            name="lb_safety/navigation_supervisor",
            hardware_id="mock_only",
            level=DiagnosticStatus.OK if not reason else DiagnosticStatus.WARN,
            message=reason or ("GOAL_ACTIVE" if permitted else "READY_IDLE"),
            values=[
                KeyValue(key="physical_use", value="prohibited"),
                KeyValue(key="command_permitted", value=str(permitted and valid_command)),
            ],
        )
        diagnostic = DiagnosticArray(status=[status])
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationSupervisor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
