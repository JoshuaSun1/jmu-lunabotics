#!/usr/bin/env python3
"""Opt-in synthetic localization and hazards driven by mock wheel odometry."""

from __future__ import annotations

import copy
import math
import struct

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult


def ray_rectangle(x, y, angle, rectangle, maximum):
    """Return the first positive intersection with an axis-aligned rectangle."""
    lower, upper = 0.0, maximum
    for origin, direction, minimum, limit in zip(
        (x, y), (math.cos(angle), math.sin(angle)), rectangle[:2], rectangle[2:]
    ):
        if abs(direction) < 1.0e-12:
            if not minimum <= origin <= limit:
                return math.inf
        else:
            a, b = sorted(((minimum - origin) / direction, (limit - origin) / direction))
            lower, upper = max(lower, a), min(upper, b)
            if upper < lower:
                return math.inf
    return lower if 0.0 < lower <= maximum else math.inf


def yaw_of(q):
    """Extract planar yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class NavigationMockWorld(Node):
    """Publish no physical data; all outputs are deterministic synthetic fixtures."""

    def __init__(self):
        super().__init__("navigation_mock_world")
        self.declare_parameter("allow_test_only_world", False)
        if self.get_parameter("allow_test_only_world").value is not True:
            raise ValueError("Synthetic world requires allow_test_only_world:=true")
        self.declare_parameter("initial_x_m", 1.5)
        self.declare_parameter("initial_y_m", 1.5)
        self.declare_parameter("scenario", "clear")
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.pose_enabled = self.scan_enabled = self.hazards_enabled = True
        self.freeze_pose = False
        self.wheel = None
        self.offset_x = self.get_parameter("initial_x_m").value
        self.offset_y = self.get_parameter("initial_y_m").value
        self.frozen = None
        self.last_stamp = None
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/apriltag_pose", 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/local", 10)
        self.scan_publisher = self.create_publisher(LaserScan, "/sim/scan", qos_profile_sensor_data)
        self.hazard_publisher = self.create_publisher(
            PointCloud2, "/sim/hazards", qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom/wheel", self.observe_wheel, 10)
        for name, attribute in (
            ("localization_enabled", "pose_enabled"),
            ("scans_enabled", "scan_enabled"),
            ("hazards_enabled", "hazards_enabled"),
            ("freeze_pose", "freeze_pose"),
        ):
            self.create_service(SetBool, f"~/scenario/{name}", self.switch(attribute))
        self.create_timer(0.05, self.publish)
        self.create_service(SetBool, "~/scenario/reset_pose", self.reset_pose)
        self.get_logger().warn(
            "TEST ONLY world: synthetic AprilTag pose, scan and hazard points; no physical use"
        )

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "scenario" and parameter.value not in {
                "clear",
                "block",
                "wall",
                "hazard",
            }:
                return SetParametersResult(successful=False, reason="Unknown synthetic scenario")
            if parameter.name == "allow_test_only_world" and parameter.value is not True:
                return SetParametersResult(
                    successful=False, reason="Cannot change test-only identity"
                )
        return SetParametersResult(successful=True)

    def switch(self, attribute):
        def callback(request, response):
            setattr(self, attribute, request.data)
            if attribute == "freeze_pose":
                self.frozen = None
            response.success = True
            response.message = "Synthetic fault injection only"
            return response

        return callback

    def observe_wheel(self, message):
        self.wheel = message

    def reset_pose(self, request, response):
        if self.wheel is None or not request.data:
            response.success = False
            return response
        self.offset_x = 1.5 - self.wheel.pose.pose.position.x
        self.offset_y = 1.5 - self.wheel.pose.pose.position.y
        self.frozen = None
        response.success = True
        response.message = "Synthetic teleport to (1.5, 1.5); no physical reset"
        return response

    def publish(self):
        if self.wheel is None:
            return
        stamp = self.wheel.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
        if stamp_ns <= 0 or not 0.0 <= age <= 0.25 or stamp_ns == self.last_stamp:
            return
        self.last_stamp = stamp_ns
        odom = copy.deepcopy(self.wheel)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x += self.offset_x
        odom.pose.pose.position.y += self.offset_y
        if self.freeze_pose:
            if self.frozen is None:
                self.frozen = copy.deepcopy(odom.pose)
            odom.pose = copy.deepcopy(self.frozen)
            odom.twist.twist.linear.x = odom.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(odom)
        pose = PoseWithCovarianceStamped()
        pose.header = copy.deepcopy(odom.header)
        pose.header.frame_id = "map"
        pose.pose = copy.deepcopy(odom.pose)
        pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = 0.001
        if self.pose_enabled:
            self.pose_publisher.publish(pose)

        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        yaw = yaw_of(odom.pose.pose.orientation)
        scenario = self.get_parameter("scenario").value
        rectangles = {
            "clear": [],
            "hazard": [],
            "block": [(2.3, 1.1, 2.7, 1.9)],
            "wall": [(2.4, 0.0, 2.6, 5.0)],
        }[scenario]
        if self.scan_enabled:
            scan = LaserScan()
            scan.header = copy.deepcopy(odom.header)
            scan.header.frame_id = "base_link"
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = 2.0 * math.pi / 720
            scan.range_min, scan.range_max, scan.scan_time = 0.05, 8.0, 0.05
            scan.ranges = [
                min(
                    (
                        ray_rectangle(
                            x, y, yaw + scan.angle_min + i * scan.angle_increment, rect, 8.0
                        )
                        for rect in rectangles
                    ),
                    default=math.inf,
                )
                for i in range(721)
            ]
            self.scan_publisher.publish(scan)
        if self.hazards_enabled:
            points = []
            if scenario == "hazard":
                for ix in range(9):
                    for iy in range(9):
                        dx, dy = 2.3 + ix * 0.05 - x, 1.3 + iy * 0.05 - y
                        points.append(
                            (
                                math.cos(yaw) * dx + math.sin(yaw) * dy,
                                -math.sin(yaw) * dx + math.cos(yaw) * dy,
                                0.1,
                            )
                        )
            cloud = PointCloud2()
            cloud.header = copy.deepcopy(odom.header)
            cloud.header.frame_id = "base_link"
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [
                PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(("x", "y", "z"))
            ]
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.is_dense = True
            cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
            self.hazard_publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMockWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
