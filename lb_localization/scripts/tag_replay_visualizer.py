#!/usr/bin/env python3
"""Publish a bounded, freshness-gated path from recorded AprilTag observation TF."""

from collections import deque
from enum import Enum

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TrackingState(Enum):
    LOST = 'LOST'
    TRACKING = 'TRACKING'


class ReplayPathState:
    """ROS-independent state used to bound paths and detect replay resets."""

    def __init__(self, max_poses: int):
        if max_poses <= 0:
            raise ValueError('max_poses must be positive')
        self.poses = deque(maxlen=max_poses)
        self.last_clock_ns = None
        self.last_transform_ns = None
        self.state = TrackingState.LOST

    def update_clock(self, now_ns: int) -> bool:
        reset = self.last_clock_ns is not None and now_ns < self.last_clock_ns
        self.last_clock_ns = now_ns
        if reset:
            self.clear()
        return reset

    def clear(self) -> None:
        self.poses.clear()
        self.last_transform_ns = None
        self.state = TrackingState.LOST

    def mark_lost(self) -> bool:
        transitioned = self.state != TrackingState.LOST
        if transitioned:
            self.poses.clear()
        self.state = TrackingState.LOST
        return transitioned

    def accept(self, stamp_ns: int, pose: PoseStamped) -> str | None:
        if stamp_ns == self.last_transform_ns:
            return None
        event = 'REACQUIRED' if self.state == TrackingState.LOST else 'TRACKING'
        self.state = TrackingState.TRACKING
        self.last_transform_ns = stamp_ns
        self.poses.append(pose)
        return event


class TagReplayVisualizer(Node):
    """Convert a dynamic camera-to-tag observation TF into pose and path topics."""

    def __init__(self) -> None:
        super().__init__('tag_replay_visualizer')
        self.declare_parameter('source_frame', 'webcam_optical_frame')
        self.declare_parameter('observed_tag_frame', 'webcam_observation_tag_1')
        self.declare_parameter('pose_topic', '/visualization/webcam/tag_1_pose')
        self.declare_parameter('path_topic', '/visualization/webcam/tag_1_path')
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('max_path_poses', 500)
        self.declare_parameter('poll_period_s', 0.05)

        self.source_frame = str(self.get_parameter('source_frame').value)
        self.observed_tag_frame = str(self.get_parameter('observed_tag_frame').value)
        stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        max_path_poses = int(self.get_parameter('max_path_poses').value)
        poll_period_s = float(self.get_parameter('poll_period_s').value)
        if stale_timeout_s <= 0.0 or poll_period_s <= 0.0:
            raise ValueError('stale_timeout_s and poll_period_s must be positive')

        self.stale_timeout = Duration(seconds=stale_timeout_s)
        self.path_state = ReplayPathState(max_path_poses)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('pose_topic').value), 10
        )
        self.path_publisher = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.timer = self.create_timer(poll_period_s, self._update)

    def _publish_path(self, stamp) -> None:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.source_frame
        path.poses = list(self.path_state.poses)
        self.path_publisher.publish(path)

    def _publish_status(self, state: str, detail: str, level: int) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'lb_localization/tag_replay_visualizer'
        status.hardware_id = 'recorded_webcam'
        status.message = state
        status.values = [
            KeyValue(key='detail', value=detail),
            KeyValue(key='source_frame', value=self.source_frame),
            KeyValue(key='observed_tag_frame', value=self.observed_tag_frame),
            KeyValue(key='path_pose_count', value=str(len(self.path_state.poses))),
        ]
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def _update(self) -> None:
        now = self.get_clock().now()
        if self.path_state.update_clock(now.nanoseconds):
            self._publish_path(now.to_msg())
            self._publish_status(
                'CLOCK_RESET', 'simulated time moved backward; path cleared', DiagnosticStatus.WARN
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.observed_tag_frame, Time()
            )
        except TransformException as error:
            if self.path_state.mark_lost():
                self._publish_path(now.to_msg())
                self._publish_status('LOST', str(error), DiagnosticStatus.WARN)
            return

        transform_time = Time.from_msg(transform.header.stamp)
        age = now - transform_time
        if age.nanoseconds < 0 or age > self.stale_timeout:
            if self.path_state.mark_lost():
                self._publish_path(now.to_msg())
                self._publish_status(
                    'LOST',
                    f'observation age is {age.nanoseconds / 1e9:.3f} s',
                    DiagnosticStatus.WARN,
                )
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        event = self.path_state.accept(transform_time.nanoseconds, pose)
        if event is None:
            return
        self.pose_publisher.publish(pose)
        self._publish_path(transform.header.stamp)
        if event == 'REACQUIRED':
            self._publish_status(
                'REACQUIRED', 'fresh observation started a new path segment', DiagnosticStatus.OK
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagReplayVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
