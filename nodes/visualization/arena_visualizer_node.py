#!/usr/bin/env python3

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class ArenaVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("arena_visualizer")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("arena.width_m", 5.0)
        self.declare_parameter("arena.height_m", 6.88)
        self.declare_parameter("arena.resolution_m_per_cell", 0.10)
        self.declare_parameter("arena.origin_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("robot_marker.size_xyz", [0.8, 0.6, 0.25])
        self.declare_parameter("robot_marker.color_rgba", [0.15, 0.6, 0.95, 0.75])

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._arena_width_m = float(self.get_parameter("arena.width_m").value)
        self._arena_height_m = float(self.get_parameter("arena.height_m").value)
        self._resolution_m = float(self.get_parameter("arena.resolution_m_per_cell").value)
        self._origin_xyz = [float(v) for v in self.get_parameter("arena.origin_xyz").value]
        self._robot_size_xyz = [
            float(v) for v in self.get_parameter("robot_marker.size_xyz").value
        ]
        self._robot_color_rgba = [
            float(v) for v in self.get_parameter("robot_marker.color_rgba").value
        ]

        if self._arena_width_m <= 0.0 or self._arena_height_m <= 0.0:
            raise ValueError("Arena width and height must be positive.")
        if self._resolution_m <= 0.0:
            raise ValueError("Arena resolution must be positive.")

        self._grid_width_cells = math.ceil(self._arena_width_m / self._resolution_m)
        self._grid_height_cells = math.ceil(self._arena_height_m / self._resolution_m)
        self._grid_span_width_m = self._grid_width_cells * self._resolution_m
        self._grid_span_height_m = self._grid_height_cells * self._resolution_m

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._grid_publisher = self.create_publisher(
            OccupancyGrid,
            "/visualization/arena_grid",
            qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "/visualization/markers",
            qos,
        )

        self.create_timer(1.0, self._publish_visuals)
        self._publish_visuals()

        if (
            not math.isclose(self._grid_span_width_m, self._arena_width_m)
            or not math.isclose(self._grid_span_height_m, self._arena_height_m)
        ):
            self.get_logger().warning(
                "Arena dimensions do not divide evenly by the resolution. "
                f"Occupancy grid spans {self._grid_span_width_m:.3f} x "
                f"{self._grid_span_height_m:.3f} m while line markers show the exact "
                f"configured {self._arena_width_m:.3f} x {self._arena_height_m:.3f} m arena."
            )

    def _publish_visuals(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._grid_publisher.publish(self._build_grid(stamp))
        self._marker_publisher.publish(self._build_marker_array(stamp))

    def _build_grid(self, stamp) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._map_frame
        grid.info.resolution = self._resolution_m
        grid.info.width = self._grid_width_cells
        grid.info.height = self._grid_height_cells
        grid.info.origin.position.x = self._origin_xyz[0]
        grid.info.origin.position.y = self._origin_xyz[1]
        grid.info.origin.position.z = self._origin_xyz[2]
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self._grid_width_cells * self._grid_height_cells)
        return grid

    def _build_marker_array(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self._build_arena_outline_marker(stamp))
        markers.markers.append(self._build_arena_cell_marker(stamp))
        markers.markers.append(self._build_robot_body_marker(stamp))
        return markers

    def _build_arena_outline_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._map_frame
        marker.ns = "arena_outline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.98
        marker.color.g = 0.81
        marker.color.b = 0.24
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0)

        ox = self._origin_xyz[0]
        oy = self._origin_xyz[1]
        oz = self._origin_xyz[2]
        points = [
            (ox, oy, oz),
            (ox + self._arena_width_m, oy, oz),
            (ox + self._arena_width_m, oy + self._arena_height_m, oz),
            (ox, oy + self._arena_height_m, oz),
            (ox, oy, oz),
        ]
        marker.points = [self._point(*point) for point in points]
        return marker

    def _build_arena_cell_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._map_frame
        marker.ns = "arena_cells"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 0.83
        marker.color.g = 0.83
        marker.color.b = 0.83
        marker.color.a = 0.85
        marker.lifetime = Duration(sec=0)

        ox = self._origin_xyz[0]
        oy = self._origin_xyz[1]
        oz = self._origin_xyz[2] + 0.01
        points = []

        x = 0.0
        while x <= self._arena_width_m + 1e-9:
            points.append(self._point(ox + x, oy, oz))
            points.append(self._point(ox + x, oy + self._arena_height_m, oz))
            x += self._resolution_m

        if not math.isclose((x - self._resolution_m), self._arena_width_m, abs_tol=1e-9):
            points.append(self._point(ox + self._arena_width_m, oy, oz))
            points.append(self._point(ox + self._arena_width_m, oy + self._arena_height_m, oz))

        y = 0.0
        while y <= self._arena_height_m + 1e-9:
            points.append(self._point(ox, oy + y, oz))
            points.append(self._point(ox + self._arena_width_m, oy + y, oz))
            y += self._resolution_m

        if not math.isclose((y - self._resolution_m), self._arena_height_m, abs_tol=1e-9):
            points.append(self._point(ox, oy + self._arena_height_m, oz))
            points.append(self._point(ox + self._arena_width_m, oy + self._arena_height_m, oz))

        marker.points = points
        return marker

    def _build_robot_body_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._base_frame
        marker.ns = "robot"
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.z = self._robot_size_xyz[2] * 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._robot_size_xyz[0]
        marker.scale.y = self._robot_size_xyz[1]
        marker.scale.z = self._robot_size_xyz[2]
        marker.color.r = self._robot_color_rgba[0]
        marker.color.g = self._robot_color_rgba[1]
        marker.color.b = self._robot_color_rgba[2]
        marker.color.a = self._robot_color_rgba[3]
        marker.lifetime = Duration(sec=0)
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main() -> None:
    rclpy.init()
    node = ArenaVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
