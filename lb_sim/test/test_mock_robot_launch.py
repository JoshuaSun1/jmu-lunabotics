"""Headless integration test for the Phase 1 mock TF and joint-state contract."""

from __future__ import annotations

import time
import unittest
from collections.abc import Callable
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


REQUIRED_CHILD_FRAMES = {
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
    "scoop_link",
    "zed_camera_mount_link",
    "zed_camera_link",
    "zed_camera_optical_frame",
    "imu_link",
    "webcam_mount_link",
    "webcam_link",
    "webcam_optical_frame",
    "lidar_mount_link",
}
WHEEL_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
}


@pytest.mark.launch_test
def generate_test_description() -> tuple[launch.LaunchDescription, dict[str, object]]:
    """Start the installed mock launch with RViz disabled for CI."""
    mock_launch = Path(get_package_share_directory("lb_sim")) / "launch" / "mock_robot.launch.py"
    return (
        launch.LaunchDescription(
            [
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        str(mock_launch)
                    ),
                    launch_arguments={"use_rviz": "false"}.items(),
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestMockRobotRuntime(unittest.TestCase):
    """Check controller state, TF ownership, and absence of localization frames."""

    @classmethod
    def setUpClass(cls) -> None:
        """Create a node that observes the already-launched mock stack."""
        if not rclpy.ok():
            rclpy.init()
        cls.node = rclpy.create_node("phase1_mock_robot_test")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.joint_states: list[JointState] = []
        cls.node.create_subscription(JointState, "/joint_states", cls.joint_states.append, 10)

    @classmethod
    def tearDownClass(cls) -> None:
        """Release the observer node after launch_testing shuts the stack down."""
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def wait_until(
        self, predicate: Callable[[], bool], description: str, timeout_s: float = 20.0
    ) -> None:
        """Spin the observer until a condition becomes true or report a useful failure."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f"timed out waiting for {description}")

    def test_joint_state_broadcaster_is_active_and_unique(self) -> None:
        """GenericSystem exposes four wheel states through the sole JSB publisher."""
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.wait_until(client.service_is_ready, "controller manager list service")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result())
        states = {controller.name: controller.state for controller in future.result().controller}
        self.assertEqual(states.get("joint_state_broadcaster"), "active")

        self.wait_until(lambda: bool(self.joint_states), "/joint_states")
        self.assertTrue(WHEEL_JOINTS <= set(self.joint_states[-1].name))
        self.wait_until(
            lambda: bool(self.node.get_publishers_info_by_topic("/joint_states")),
            "/joint_states publisher discovery",
        )
        publishers = self.node.get_publishers_info_by_topic("/joint_states")
        self.assertEqual(
            {publisher.node_name for publisher in publishers}, {"joint_state_broadcaster"}
        )

    def test_robot_state_publisher_owns_the_mock_tf_tree(self) -> None:
        """Every required Phase 1 frame resolves from base_link through RSP."""
        for child_frame in REQUIRED_CHILD_FRAMES:
            self.wait_until(
                lambda child_frame=child_frame: self._has_transform(child_frame),
                f"base_link -> {child_frame}",
            )

        for topic in ("/tf", "/tf_static"):
            self.wait_until(
                lambda topic=topic: bool(self.node.get_publishers_info_by_topic(topic)),
                f"{topic} publisher discovery",
            )
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(
                {publisher.node_name for publisher in publishers}, {"robot_state_publisher"}
            )

    def test_mock_tree_has_no_localization_frames(self) -> None:
        """Phase 1 reserves map and odom for the later EKF implementations."""
        self.wait_until(lambda: bool(self.tf_buffer.all_frames_as_yaml()), "TF frame graph")
        frame_graph = self.tf_buffer.all_frames_as_yaml()
        self.assertNotIn("map:", frame_graph)
        self.assertNotIn("odom:", frame_graph)

    def _has_transform(self, child_frame: str) -> bool:
        """Return whether the expected base-relative transform is currently available."""
        try:
            self.tf_buffer.lookup_transform(
                "base_link", child_frame, Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException:
            return False
        return True
