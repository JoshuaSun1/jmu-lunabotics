#!/usr/bin/env python3
"""Publish non-collision RViz markers for the nominal Lunabotics arena."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class ArenaGeometry:
    """Dimensions for the guidebook-derived arena scene."""

    length_m: float
    width_m: float
    excavation_boundary_x_m: float
    starting_zone_length_m: float
    starting_zone_depth_m: float
    berm_center_x_m: float
    berm_center_y_m: float
    berm_length_m: float
    berm_width_m: float

    def validate(self) -> None:
        """Reject invalid or out-of-arena scene geometry."""
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('arena scene dimensions must be finite and positive')
        if self.excavation_boundary_x_m >= self.length_m:
            raise ValueError('excavation boundary must be inside the arena')
        if self.starting_zone_length_m > self.length_m:
            raise ValueError('starting zone length exceeds arena length')
        if self.starting_zone_depth_m > self.width_m:
            raise ValueError('starting zone depth exceeds arena width')
        half_berm_length = 0.5 * self.berm_length_m
        half_berm_width = 0.5 * self.berm_width_m
        if not half_berm_length <= self.berm_center_x_m <= (
            self.length_m - half_berm_length
        ):
            raise ValueError('berm target exceeds arena length')
        if not half_berm_width <= self.berm_center_y_m <= (
            self.width_m - half_berm_width
        ):
            raise ValueError('berm target exceeds arena width')


def rectangle_points(
    minimum_x: float,
    minimum_y: float,
    length_m: float,
    width_m: float,
    z_m: float,
) -> list[Point]:
    """Return the five points of a closed axis-aligned rectangle."""
    return [
        Point(x=minimum_x, y=minimum_y, z=z_m),
        Point(x=minimum_x + length_m, y=minimum_y, z=z_m),
        Point(x=minimum_x + length_m, y=minimum_y + width_m, z=z_m),
        Point(x=minimum_x, y=minimum_y + width_m, z=z_m),
        Point(x=minimum_x, y=minimum_y, z=z_m),
    ]


def dashed_rectangle_points(
    minimum_x: float,
    minimum_y: float,
    length_m: float,
    width_m: float,
    z_m: float,
    dash_m: float = 0.15,
) -> list[Point]:
    """Return LINE_LIST points for a dashed rectangular outline."""
    if dash_m <= 0.0:
        raise ValueError('dash length must be positive')
    corners = rectangle_points(minimum_x, minimum_y, length_m, width_m, z_m)
    points = []
    for start, end in zip(corners, corners[1:]):
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        edge_length = math.hypot(delta_x, delta_y)
        distance = 0.0
        while distance < edge_length:
            segment_end = min(distance + dash_m, edge_length)
            start_fraction = distance / edge_length
            end_fraction = segment_end / edge_length
            points.extend(
                [
                    Point(
                        x=start.x + start_fraction * delta_x,
                        y=start.y + start_fraction * delta_y,
                        z=z_m,
                    ),
                    Point(
                        x=start.x + end_fraction * delta_x,
                        y=start.y + end_fraction * delta_y,
                        z=z_m,
                    ),
                ]
            )
            distance += 2.0 * dash_m
    return points


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Return a ROS-order unit quaternion from fixed-axis RPY."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_vector(quaternion, vector):
    """Rotate a vector by a unit quaternion."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class ArenaSceneMarkers(Node):
    """Publish durable guidebook and provisional arena visualization markers."""

    def __init__(self) -> None:
        super().__init__('arena_scene_markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_topic', '/visualization/arena/markers')
        self.declare_parameter('republish_period_s', 1.0)
        self.declare_parameter('arena_length_m', 6.88)
        self.declare_parameter('arena_width_m', 5.0)
        self.declare_parameter('excavation_boundary_x_m', 2.5)
        self.declare_parameter('starting_zone_length_m', 2.0)
        self.declare_parameter('starting_zone_depth_m', 2.0)
        self.declare_parameter('berm_center_x_m', 5.38)
        self.declare_parameter('berm_center_y_m', 0.9)
        self.declare_parameter('berm_length_m', 2.2)
        self.declare_parameter('berm_width_m', 0.9)
        self.declare_parameter('tag_ids', [1])
        self.declare_parameter('tag_sizes_m', [0.30])
        self.declare_parameter('tag_positions_m', [0.0, 1.0, 0.75])
        self.declare_parameter(
            'tag_rpy_rad', [math.pi / 2.0, 0.0, math.pi / 2.0]
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.geometry = ArenaGeometry(
            length_m=self._float_parameter('arena_length_m'),
            width_m=self._float_parameter('arena_width_m'),
            excavation_boundary_x_m=self._float_parameter(
                'excavation_boundary_x_m'
            ),
            starting_zone_length_m=self._float_parameter(
                'starting_zone_length_m'
            ),
            starting_zone_depth_m=self._float_parameter('starting_zone_depth_m'),
            berm_center_x_m=self._float_parameter('berm_center_x_m'),
            berm_center_y_m=self._float_parameter('berm_center_y_m'),
            berm_length_m=self._float_parameter('berm_length_m'),
            berm_width_m=self._float_parameter('berm_width_m'),
        )
        self.geometry.validate()
        self.tags = self._tag_parameters()

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), qos
        )
        self.publish_scene()
        republish_period_s = self._float_parameter('republish_period_s')
        if republish_period_s <= 0.0:
            raise ValueError('republish_period_s must be positive')
        self.timer = self.create_timer(republish_period_s, self.publish_scene)
        self.get_logger().warning(
            'Arena scene is visualization-only: the 2.0 m starting-zone depth '
            'and tag poses are provisional; markers are not Nav2 obstacles.'
        )

    def _float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _tag_parameters(self):
        tag_ids = [int(value) for value in self.get_parameter('tag_ids').value]
        sizes = [float(value) for value in self.get_parameter('tag_sizes_m').value]
        positions = [
            float(value) for value in self.get_parameter('tag_positions_m').value
        ]
        rotations = [
            float(value) for value in self.get_parameter('tag_rpy_rad').value
        ]
        if len(set(tag_ids)) != len(tag_ids):
            raise ValueError('tag_ids must be unique')
        if len(sizes) != len(tag_ids):
            raise ValueError('tag_sizes_m must contain one value per tag')
        if len(positions) != 3 * len(tag_ids):
            raise ValueError('tag_positions_m must contain XYZ triples')
        if len(rotations) != 3 * len(tag_ids):
            raise ValueError('tag_rpy_rad must contain RPY triples')
        if not all(math.isfinite(value) for value in positions + rotations):
            raise ValueError('tag poses must be finite')
        if not all(math.isfinite(value) and value > 0.0 for value in sizes):
            raise ValueError('tag sizes must be finite and positive')
        return [
            (
                tag_id,
                sizes[index],
                tuple(positions[3 * index: 3 * index + 3]),
                tuple(rotations[3 * index: 3 * index + 3]),
            )
            for index, tag_id in enumerate(tag_ids)
        ]

    def _marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        return marker

    def _outline_marker(
        self,
        namespace: str,
        marker_id: int,
        points: list[Point],
        color,
        width_m: float,
        dashed: bool = False,
    ) -> Marker:
        marker_type = Marker.LINE_LIST if dashed else Marker.LINE_STRIP
        marker = self._marker(namespace, marker_id, marker_type)
        marker.points = points
        marker.scale.x = width_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _scene_geometry_markers(self) -> list[Marker]:
        geometry = self.geometry
        floor = self._marker('arena_floor', 1, Marker.CUBE)
        floor.pose.position.x = 0.5 * geometry.length_m
        floor.pose.position.y = 0.5 * geometry.width_m
        floor.pose.position.z = -0.015
        floor.scale.x = geometry.length_m
        floor.scale.y = geometry.width_m
        floor.scale.z = 0.01
        floor.color.r = 1.0
        floor.color.g = 0.88
        floor.color.b = 0.45
        floor.color.a = 0.10

        arena_outline = self._outline_marker(
            'arena_outline',
            2,
            rectangle_points(0.0, 0.0, geometry.length_m, geometry.width_m, 0.02),
            (1.0, 0.88, 0.45, 1.0),
            0.045,
        )

        excavation = self._outline_marker(
            'zone_boundaries',
            3,
            [
                Point(x=geometry.excavation_boundary_x_m, y=0.0, z=0.025),
                Point(
                    x=geometry.excavation_boundary_x_m,
                    y=geometry.width_m,
                    z=0.025,
                ),
            ],
            (0.25, 0.75, 1.0, 1.0),
            0.04,
        )

        starting = self._outline_marker(
            'provisional_starting_zone',
            4,
            dashed_rectangle_points(
                0.0,
                0.0,
                geometry.starting_zone_length_m,
                geometry.starting_zone_depth_m,
                0.035,
            ),
            (0.35, 1.0, 0.25, 1.0),
            0.04,
            dashed=True,
        )

        berm_minimum_x = geometry.berm_center_x_m - 0.5 * geometry.berm_length_m
        berm_minimum_y = geometry.berm_center_y_m - 0.5 * geometry.berm_width_m
        berm = self._outline_marker(
            'target_berm',
            5,
            rectangle_points(
                berm_minimum_x,
                berm_minimum_y,
                geometry.berm_length_m,
                geometry.berm_width_m,
                0.04,
            ),
            (1.0, 0.15, 0.10, 1.0),
            0.055,
        )
        berm_center = self._marker('target_berm', 6, Marker.SPHERE)
        berm_center.pose.position.x = geometry.berm_center_x_m
        berm_center.pose.position.y = geometry.berm_center_y_m
        berm_center.pose.position.z = 0.045
        berm_center.scale.x = 0.12
        berm_center.scale.y = 0.12
        berm_center.scale.z = 0.03
        berm_center.color.r = 1.0
        berm_center.color.g = 0.15
        berm_center.color.b = 0.10
        berm_center.color.a = 1.0
        return [floor, arena_outline, excavation, starting, berm, berm_center]

    def _tag_markers(self) -> list[Marker]:
        markers = []
        for index, (tag_id, size_m, position, rpy) in enumerate(self.tags):
            quaternion = quaternion_from_rpy(*rpy)
            tag = self._marker('assumed_apriltags', 100 + 3 * index, Marker.CUBE)
            tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = position
            (
                tag.pose.orientation.x,
                tag.pose.orientation.y,
                tag.pose.orientation.z,
                tag.pose.orientation.w,
            ) = quaternion
            tag.scale.x = size_m
            tag.scale.y = size_m
            tag.scale.z = 0.01
            tag.color.r = 1.0
            tag.color.g = 0.55
            tag.color.b = 0.05
            tag.color.a = 0.9

            normal = rotate_vector(quaternion, (0.0, 0.0, 1.0))
            arrow = self._marker(
                'assumed_apriltag_normals', 101 + 3 * index, Marker.ARROW
            )
            arrow.points = [
                Point(x=position[0], y=position[1], z=position[2]),
                Point(
                    x=position[0] + 0.35 * normal[0],
                    y=position[1] + 0.35 * normal[1],
                    z=position[2] + 0.35 * normal[2],
                ),
            ]
            arrow.scale.x = 0.025
            arrow.scale.y = 0.055
            arrow.scale.z = 0.075
            arrow.color.r = 1.0
            arrow.color.g = 0.55
            arrow.color.b = 0.05
            arrow.color.a = 1.0

            label = self._marker(
                'assumed_apriltag_labels', 102 + 3 * index, Marker.TEXT_VIEW_FACING
            )
            label.pose.position.x = position[0]
            label.pose.position.y = position[1]
            label.pose.position.z = position[2] + 0.25
            label.scale.z = 0.14
            label.color.r = 1.0
            label.color.g = 0.75
            label.color.b = 0.15
            label.color.a = 1.0
            label.text = f'AprilTag {tag_id} (assumed)'
            markers.extend([tag, arrow, label])
        return markers

    def publish_scene(self) -> None:
        """Publish one durable snapshot of the complete arena scene."""
        markers = self._scene_geometry_markers() + self._tag_markers()
        self.publisher.publish(MarkerArray(markers=markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArenaSceneMarkers()
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
