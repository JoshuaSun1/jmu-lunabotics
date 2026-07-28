#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class TfBackboneNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_backbone")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("backbone_publish_rate_hz", 10.0)
        self.declare_parameter("camera_parent_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("camera_xyz", [0.0, 0.0, 0.5])
        self.declare_parameter("camera_rpy", [0.0, 0.0, 0.0])

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._camera_parent_frame = self.get_parameter("camera_parent_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._camera_xyz = [float(value) for value in self.get_parameter("camera_xyz").value]
        self._camera_rpy = [float(value) for value in self.get_parameter("camera_rpy").value]

        publish_rate_hz = float(self.get_parameter("backbone_publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("backbone_publish_rate_hz must be positive.")

        self._dynamic_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_camera_mount()

        self.create_timer(1.0 / publish_rate_hz, self._publish_backbone)
        self.get_logger().info(
            "Publishing TF backbone "
            f"{self._map_frame} -> {self._odom_frame} -> {self._base_frame} "
            f"with static camera mount {self._camera_parent_frame} -> {self._camera_frame}"
        )

    def _publish_camera_mount(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._camera_parent_frame
        transform.child_frame_id = self._camera_frame
        transform.transform.translation.x = self._camera_xyz[0]
        transform.transform.translation.y = self._camera_xyz[1]
        transform.transform.translation.z = self._camera_xyz[2]

        qx, qy, qz, qw = _quaternion_from_rpy(
            self._camera_rpy[0],
            self._camera_rpy[1],
            self._camera_rpy[2],
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._static_broadcaster.sendTransform(transform)

    def _publish_backbone(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = [
            self._identity_transform(stamp, self._map_frame, self._odom_frame),
            self._identity_transform(stamp, self._odom_frame, self._base_frame),
        ]
        self._dynamic_broadcaster.sendTransform(transforms)

    @staticmethod
    def _identity_transform(stamp, parent_frame: str, child_frame: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.rotation.w = 1.0
        return transform


def main() -> None:
    rclpy.init()
    node = TfBackboneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
