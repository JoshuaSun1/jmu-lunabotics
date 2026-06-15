#!/usr/bin/env python3

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


class AprilTagPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_pose_publisher")

        self._tag_frame = os.environ.get("APRILTAG_FRAME_NAME", "apriltag_0")
        self._camera_frame = os.environ.get("FRAME_ID", "camera_link")
        self._pose_topic = os.environ.get("APRILTAG_POSE_TOPIC", "/apriltag/camera_pose")
        self._distance_topic = os.environ.get(
            "APRILTAG_DISTANCE_TOPIC", "/apriltag/camera_distance"
        )

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

            if transform_stamped.header.frame_id != self._camera_frame:
                continue

            pose = PoseStamped()
            pose.header = transform_stamped.header
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
