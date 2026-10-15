#!/usr/bin/env python3
"""Visualize provisional map-frame camera and robot poses from tag replay."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class RigidTransform:
    """Parent-to-child rigid transform using ROS quaternion ordering."""

    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


class TrackingState(Enum):
    LOST = 'LOST'
    TRACKING = 'TRACKING'


class ReplayPoseState:
    """Bound one contiguous planar path and reset it on loss or clock rewind."""

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


def normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return a unit quaternion or reject invalid geometry."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError('quaternion must be finite and nonzero')
    return tuple(value / norm for value in quaternion)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Create a ROS-order quaternion from fixed-axis roll, pitch, and yaw."""
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def multiply_quaternions(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Compose rotations as first followed by second."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return normalize_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a vector without constructing a temporary non-unit quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose(first: RigidTransform, second: RigidTransform) -> RigidTransform:
    """Compose T_a_b and T_b_c into T_a_c."""
    rotated = rotate_vector(first.rotation, second.translation)
    return RigidTransform(
        tuple(first.translation[index] + rotated[index] for index in range(3)),
        multiply_quaternions(first.rotation, second.rotation),
    )


def inverse(transform: RigidTransform) -> RigidTransform:
    """Invert a rigid transform."""
    x, y, z, w = normalize_quaternion(transform.rotation)
    inverse_rotation = (-x, -y, -z, w)
    inverse_translation = rotate_vector(
        inverse_rotation, tuple(-value for value in transform.translation)
    )
    return RigidTransform(inverse_translation, inverse_rotation)


def yaw_of(quaternion: tuple[float, float, float, float]) -> float:
    """Extract REP-103 yaw from a quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def planarized(transform: RigidTransform) -> RigidTransform:
    """Project a full pose onto the map XY plane."""
    return RigidTransform(
        (transform.translation[0], transform.translation[1], 0.0),
        quaternion_from_rpy(0.0, 0.0, yaw_of(transform.rotation)),
    )


def rectangle_corners(
    transform: RigidTransform, length_m: float, width_m: float, z_m: float = 0.01
) -> list[tuple[float, float, float]]:
    """Return a closed rectangular outline transformed into its parent frame."""
    if length_m <= 0.0 or width_m <= 0.0:
        raise ValueError('rectangle dimensions must be positive')
    half_length = 0.5 * length_m
    half_width = 0.5 * width_m
    local_corners = [
        (half_length, half_width, z_m),
        (half_length, -half_width, z_m),
        (-half_length, -half_width, z_m),
        (-half_length, half_width, z_m),
        (half_length, half_width, z_m),
    ]
    return [
        compose(
            transform, RigidTransform(corner, (0.0, 0.0, 0.0, 1.0))
        ).translation
        for corner in local_corners
    ]


class TagReplayRobotPose(Node):
    """Compute non-production map poses from one recorded observation TF."""

    _DYNAMIC_MARKERS = (
        ('raw_webcam_axes', 10),
        ('raw_webcam_axes', 11),
        ('raw_webcam_axes', 12),
        ('planar_base_link_proxy', 20),
        ('synthetic_robot_chassis', 30),
        ('synthetic_robot_footprint', 31),
        ('provisional_sensor_bodies', 40),
        ('provisional_sensor_bodies', 41),
        ('provisional_sensor_directions', 42),
        ('provisional_sensor_directions', 43),
    )

    def __init__(self) -> None:
        super().__init__('tag_replay_robot_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'webcam_optical_frame')
        self.declare_parameter('observed_tag_frame', 'webcam_observation_tag_1')
        self.declare_parameter('tag_position_m', [0.0, 1.0, 0.75])
        self.declare_parameter(
            'tag_rpy_rad', [math.pi / 2.0, 0.0, math.pi / 2.0]
        )
        self.declare_parameter('base_to_camera_position_m', [0.0, 0.0, 0.4])
        self.declare_parameter(
            'base_to_camera_rpy_rad', [-math.pi / 2.0, 0.0, math.pi / 2.0]
        )
        self.declare_parameter('base_to_zed_position_m', [0.0, 0.0, 0.4])
        self.declare_parameter(
            'base_to_zed_rpy_rad', [-math.pi / 2.0, 0.0, -math.pi / 2.0]
        )
        self.declare_parameter('chassis_dimensions_m', [0.90, 0.60, 0.25])
        self.declare_parameter('webcam_visual_dimensions_m', [0.06, 0.03, 0.03])
        self.declare_parameter('zed_visual_dimensions_m', [0.08, 0.04, 0.03])
        self.declare_parameter(
            'raw_camera_pose_topic', '/visualization/replay/webcam_pose_raw'
        )
        self.declare_parameter(
            'planar_base_pose_topic', '/visualization/replay/base_pose_planar'
        )
        self.declare_parameter(
            'planar_path_topic', '/visualization/replay/base_path_planar'
        )
        self.declare_parameter('marker_topic', '/visualization/replay/markers')
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('max_path_poses', 500)
        self.declare_parameter('poll_period_s', 0.05)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.observed_tag_frame = str(
            self.get_parameter('observed_tag_frame').value
        )
        tag_position = self._three_values('tag_position_m')
        tag_rpy = self._three_values('tag_rpy_rad')
        camera_position = self._three_values('base_to_camera_position_m')
        camera_rpy = self._three_values('base_to_camera_rpy_rad')
        zed_position = self._three_values('base_to_zed_position_m')
        zed_rpy = self._three_values('base_to_zed_rpy_rad')
        self.chassis_dimensions = self._positive_three_values(
            'chassis_dimensions_m'
        )
        self.webcam_visual_dimensions = self._positive_three_values(
            'webcam_visual_dimensions_m'
        )
        self.zed_visual_dimensions = self._positive_three_values(
            'zed_visual_dimensions_m'
        )
        self.map_to_tag = RigidTransform(
            tag_position, quaternion_from_rpy(*tag_rpy)
        )
        self.base_to_camera = RigidTransform(
            camera_position, quaternion_from_rpy(*camera_rpy)
        )
        self.base_to_zed = RigidTransform(
            zed_position, quaternion_from_rpy(*zed_rpy)
        )

        stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        poll_period_s = float(self.get_parameter('poll_period_s').value)
        max_path_poses = int(self.get_parameter('max_path_poses').value)
        if stale_timeout_s <= 0.0 or poll_period_s <= 0.0:
            raise ValueError('stale_timeout_s and poll_period_s must be positive')

        self.stale_timeout = Duration(seconds=stale_timeout_s)
        self.marker_lifetime = Duration(seconds=stale_timeout_s).to_msg()
        self.path_state = ReplayPoseState(max_path_poses)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.raw_pose_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('raw_camera_pose_topic').value),
            10,
        )
        self.planar_pose_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('planar_base_pose_topic').value),
            10,
        )
        self.path_publisher = self.create_publisher(
            Path, str(self.get_parameter('planar_path_topic').value), 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.timer = self.create_timer(poll_period_s, self._update)

        self.get_logger().warning(
            'Development assumptions active: tag pose, 0.40 m camera mounts, '
            'and 0.90 x 0.60 x 0.25 m chassis are not surveyed; output is '
            'visualization-only.'
        )

    def _three_values(self, parameter_name: str) -> tuple[float, float, float]:
        values = tuple(float(value) for value in self.get_parameter(parameter_name).value)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{parameter_name} must contain three finite values')
        return values

    def _positive_three_values(
        self, parameter_name: str
    ) -> tuple[float, float, float]:
        values = self._three_values(parameter_name)
        if any(value <= 0.0 for value in values):
            raise ValueError(f'{parameter_name} values must be positive')
        return values

    @staticmethod
    def _from_transform_message(message) -> RigidTransform:
        return RigidTransform(
            (
                message.translation.x,
                message.translation.y,
                message.translation.z,
            ),
            normalize_quaternion(
                (
                    message.rotation.x,
                    message.rotation.y,
                    message.rotation.z,
                    message.rotation.w,
                )
            ),
        )

    def _pose_message(self, transform: RigidTransform, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            transform.translation
        )
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = transform.rotation
        return pose

    def _publish_path(self, stamp) -> None:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp
        path.poses = list(self.path_state.poses)
        self.path_publisher.publish(path)

    def _axis_marker(
        self,
        marker_id: int,
        transform: RigidTransform,
        local_axis: tuple[float, float, float],
        color: tuple[float, float, float],
        stamp,
    ) -> Marker:
        direction = rotate_vector(transform.rotation, local_axis)
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'raw_webcam_axes'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(
                x=transform.translation[0],
                y=transform.translation[1],
                z=transform.translation[2],
            ),
            Point(
                x=transform.translation[0] + 0.25 * direction[0],
                y=transform.translation[1] + 0.25 * direction[1],
                z=transform.translation[2] + 0.25 * direction[2],
            ),
        ]
        marker.scale.x = 0.018
        marker.scale.y = 0.035
        marker.scale.z = 0.05
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.lifetime = self.marker_lifetime
        return marker

    def _base_marker(self, transform: RigidTransform, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'planar_base_link_proxy'
        marker.id = 20
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = self._pose_message(transform, stamp).pose
        marker.scale.x = 0.45
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = self.marker_lifetime
        return marker

    def _chassis_marker(self, transform: RigidTransform, stamp) -> Marker:
        length_m, width_m, height_m = self.chassis_dimensions
        chassis_transform = compose(
            transform,
            RigidTransform((0.0, 0.0, 0.5 * height_m), (0.0, 0.0, 0.0, 1.0)),
        )
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'synthetic_robot_chassis'
        marker.id = 30
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._pose_message(chassis_transform, stamp).pose
        marker.scale.x = length_m
        marker.scale.y = width_m
        marker.scale.z = height_m
        marker.color.r = 0.20
        marker.color.g = 0.35
        marker.color.b = 0.55
        marker.color.a = 0.55
        marker.lifetime = self.marker_lifetime
        return marker

    def _footprint_marker(self, transform: RigidTransform, stamp) -> Marker:
        length_m, width_m, _ = self.chassis_dimensions
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'synthetic_robot_footprint'
        marker.id = 31
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [
            Point(x=x, y=y, z=z)
            for x, y, z in rectangle_corners(transform, length_m, width_m)
        ]
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime = self.marker_lifetime
        return marker

    def _sensor_body_marker(
        self,
        marker_id: int,
        name: str,
        map_to_sensor: RigidTransform,
        dimensions: tuple[float, float, float],
        color: tuple[float, float, float],
        stamp,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'provisional_sensor_bodies'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._pose_message(map_to_sensor, stamp).pose
        marker.scale.x, marker.scale.y, marker.scale.z = dimensions
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.90
        marker.text = name
        marker.lifetime = self.marker_lifetime
        return marker

    def _sensor_direction_marker(
        self,
        marker_id: int,
        map_to_sensor: RigidTransform,
        color: tuple[float, float, float],
        stamp,
    ) -> Marker:
        direction = rotate_vector(map_to_sensor.rotation, (0.0, 0.0, 1.0))
        origin = map_to_sensor.translation
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'provisional_sensor_directions'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=origin[0], y=origin[1], z=origin[2]),
            Point(
                x=origin[0] + 0.30 * direction[0],
                y=origin[1] + 0.30 * direction[1],
                z=origin[2] + 0.30 * direction[2],
            ),
        ]
        marker.scale.x = 0.025
        marker.scale.y = 0.055
        marker.scale.z = 0.075
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.lifetime = self.marker_lifetime
        return marker

    def _publish_tracking_markers(
        self, map_to_camera: RigidTransform, map_to_base_planar: RigidTransform, stamp
    ) -> None:
        map_to_webcam_proxy = compose(map_to_base_planar, self.base_to_camera)
        map_to_zed_proxy = compose(map_to_base_planar, self.base_to_zed)
        map_to_webcam_body = RigidTransform(
            map_to_webcam_proxy.translation, map_to_base_planar.rotation
        )
        map_to_zed_body = RigidTransform(
            map_to_zed_proxy.translation, map_to_base_planar.rotation
        )
        markers = [
            self._axis_marker(10, map_to_camera, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), stamp),
            self._axis_marker(11, map_to_camera, (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), stamp),
            self._axis_marker(12, map_to_camera, (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), stamp),
            self._base_marker(map_to_base_planar, stamp),
            self._chassis_marker(map_to_base_planar, stamp),
            self._footprint_marker(map_to_base_planar, stamp),
            self._sensor_body_marker(
                40,
                'webcam_rear',
                map_to_webcam_body,
                self.webcam_visual_dimensions,
                (0.95, 0.25, 0.75),
                stamp,
            ),
            self._sensor_body_marker(
                41,
                'zed_forward',
                map_to_zed_body,
                self.zed_visual_dimensions,
                (0.10, 0.90, 0.40),
                stamp,
            ),
            self._sensor_direction_marker(
                42, map_to_webcam_proxy, (0.95, 0.25, 0.75), stamp
            ),
            self._sensor_direction_marker(
                43, map_to_zed_proxy, (0.10, 0.90, 0.40), stamp
            ),
        ]
        self.marker_publisher.publish(MarkerArray(markers=markers))

    def _publish_lost_markers(self, stamp) -> None:
        markers = []
        for namespace, marker_id in self._DYNAMIC_MARKERS:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.append(marker)
        self.marker_publisher.publish(MarkerArray(markers=markers))

    def _publish_status(self, state: str, detail: str, level: int) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'lb_localization/tag_replay_robot_pose'
        status.hardware_id = 'recorded_webcam_development_assumption'
        status.message = state
        status.values = [
            KeyValue(key='detail', value=detail),
            KeyValue(key='map_frame', value=self.map_frame),
            KeyValue(key='camera_frame', value=self.camera_frame),
            KeyValue(key='observed_tag_frame', value=self.observed_tag_frame),
            KeyValue(key='path_pose_count', value=str(len(self.path_state.poses))),
            KeyValue(key='tag_pose_status', value='estimated/development-only'),
            KeyValue(key='camera_extrinsic_status', value='estimated/development-only'),
            KeyValue(key='odometry_available', value='false'),
        ]
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def _mark_lost(self, stamp, detail: str) -> None:
        if self.path_state.mark_lost():
            self._publish_path(stamp)
            self._publish_lost_markers(stamp)
            self._publish_status('LOST', detail, DiagnosticStatus.WARN)

    def _update(self) -> None:
        now = self.get_clock().now()
        now_message = now.to_msg()
        if self.path_state.update_clock(now.nanoseconds):
            self._publish_path(now_message)
            self._publish_lost_markers(now_message)
            self._publish_status(
                'CLOCK_RESET',
                'simulated time moved backward; provisional path cleared',
                DiagnosticStatus.WARN,
            )
            return

        try:
            observation = self.tf_buffer.lookup_transform(
                self.camera_frame, self.observed_tag_frame, Time()
            )
        except TransformException as error:
            self._mark_lost(now_message, str(error))
            return

        observation_time = Time.from_msg(observation.header.stamp)
        age = now - observation_time
        if age.nanoseconds < 0 or age > self.stale_timeout:
            self._mark_lost(
                now_message,
                f'observation age is {age.nanoseconds / 1e9:.3f} s',
            )
            return

        try:
            camera_to_tag = self._from_transform_message(observation.transform)
            map_to_camera = compose(self.map_to_tag, inverse(camera_to_tag))
            map_to_base = compose(map_to_camera, inverse(self.base_to_camera))
            map_to_base_planar = planarized(map_to_base)
        except ValueError as error:
            self._mark_lost(now_message, f'invalid transform: {error}')
            return

        raw_pose = self._pose_message(map_to_camera, observation.header.stamp)
        planar_pose = self._pose_message(
            map_to_base_planar, observation.header.stamp
        )
        event = self.path_state.accept(observation_time.nanoseconds, planar_pose)
        if event is None:
            return

        self.raw_pose_publisher.publish(raw_pose)
        self.planar_pose_publisher.publish(planar_pose)
        self._publish_path(observation.header.stamp)
        self._publish_tracking_markers(
            map_to_camera, map_to_base_planar, observation.header.stamp
        )
        if event == 'REACQUIRED':
            self._publish_status(
                'REACQUIRED',
                'fresh tag observation started a new provisional path segment',
                DiagnosticStatus.WARN,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagReplayRobotPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
