#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class AprilTagPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_pose_publisher")

        self.declare_parameter("tag_frame", "apriltag_0")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("pose_topic", "/apriltag/camera_pose")
        self.declare_parameter("distance_topic", "/apriltag/camera_distance")

        self._tag_frame = self.get_parameter("tag_frame").get_parameter_value().string_value
        self._camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
            or "camera_link"
        )
        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self._distance_topic = (
            self.get_parameter("distance_topic").get_parameter_value().string_value
        )
        self._camera_alias_broadcaster = StaticTransformBroadcaster(self)
        self._camera_alias_published = False

        self._pose_publisher = self.create_publisher(PoseStamped, self._pose_topic, 10)
        self._distance_publisher = self.create_publisher(Float64, self._distance_topic, 10)
        self.create_subscription(TFMessage, "/tf", self._handle_tf, 10)

        self.get_logger().info(
            f"Republishing pose for {self._tag_frame} relative to {self._camera_frame}"
        )
        self.get_logger().info(f"  pose topic: {self._pose_topic}")
        self.get_logger().info(f"  distance topic: {self._distance_topic}")

    def _handle_tf(self, message: TFMessage) -> None:
        for transform_stamped in message.transforms:
            if transform_stamped.child_frame_id != self._tag_frame:
                continue

            source_camera_frame = transform_stamped.header.frame_id or self._camera_frame
            self._ensure_camera_alias(source_camera_frame, transform_stamped.header.stamp)

            pose = PoseStamped()
            pose.header.stamp = transform_stamped.header.stamp
            pose.header.frame_id = self._camera_frame
            pose.pose.position.x = transform_stamped.transform.translation.x
            pose.pose.position.y = transform_stamped.transform.translation.y
            pose.pose.position.z = transform_stamped.transform.translation.z
            pose.pose.orientation = transform_stamped.transform.rotation
            self._pose_publisher.publish(pose)

            distance = Float64()
            distance.data = math.sqrt(
                pose.pose.position.x**2 + pose.pose.position.y**2 + pose.pose.position.z**2
            )
            self._distance_publisher.publish(distance)

    def _ensure_camera_alias(self, source_camera_frame: str, stamp) -> None:
        if not source_camera_frame:
            return

        if source_camera_frame == self._camera_frame:
            return

        if self._camera_alias_published:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._camera_frame
        transform.child_frame_id = source_camera_frame
        transform.transform.rotation.w = 1.0
        self._camera_alias_broadcaster.sendTransform(transform)
        self._camera_alias_published = True
        self.get_logger().warn(
            f"Aliasing camera frame {source_camera_frame} under {self._camera_frame} "
            "with an identity transform"
        )


def main() -> None:
    rclpy.init()
    node = AprilTagPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
