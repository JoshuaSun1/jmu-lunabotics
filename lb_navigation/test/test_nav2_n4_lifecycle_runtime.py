"""Live N4 lifecycle/costmap test, skipped when declared Nav2 packages are absent."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import time

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


REQUIRED_PACKAGES = (
    "nav2_costmap_2d",
    "nav2_lifecycle_manager",
    "nav2_map_server",
)
ACTIVE_STATE = 3


def _missing_packages() -> list[str]:
    missing = []
    for package in REQUIRED_PACKAGES:
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            missing.append(package)
    return missing


def _spin_until(node: Node, condition, pose_publisher, timeout_s: float = 15.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 1.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.04
        pose.pose.covariance[7] = 0.04
        pose_publisher.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise AssertionError("timed out waiting for N4 lifecycle condition")


def _wait_for_active(node: Node, client, pose_publisher, timeout_s: float = 15.0) -> None:
    deadline = time.monotonic() + timeout_s
    future = None
    while time.monotonic() < deadline:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 1.0
        pose.pose.pose.orientation.w = 1.0
        pose_publisher.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.05)
        if not client.service_is_ready():
            continue
        if future is None:
            future = client.call_async(GetState.Request())
        elif future.done():
            if future.result().current_state.id == ACTIVE_STATE:
                return
            future = client.call_async(GetState.Request())
    raise AssertionError(f"lifecycle node did not become active: {client.srv_name}")


def test_n4_nodes_activate_and_publish_expected_costmap_geometry() -> None:
    missing = _missing_packages()
    if missing:
        pytest.skip(f"N4 runtime packages not installed: {', '.join(missing)}")

    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", "61")
    environment.setdefault("ROS_LOG_DIR", "/tmp/lunabotics_n4_runtime_logs")
    Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    previous_log_directory = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = environment["ROS_LOG_DIR"]
    rclpy.init(domain_id=int(environment["ROS_DOMAIN_ID"]))
    probe = Node("n4_costmap_runtime_probe")
    pose_publisher = probe.create_publisher(
        PoseWithCovarianceStamped,
        "/localization/apriltag_pose",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    global_costmaps = []
    local_costmaps = []
    probe.create_subscription(
        OccupancyGrid, "/global_costmap/costmap", global_costmaps.append, map_qos
    )
    probe.create_subscription(
        OccupancyGrid, "/local_costmap/costmap", local_costmaps.append, map_qos
    )
    lifecycle_clients = [
        probe.create_client(GetState, "/map_server/get_state"),
        probe.create_client(GetState, "/global_costmap/global_costmap/get_state"),
        probe.create_client(GetState, "/local_costmap/local_costmap/get_state"),
    ]

    command = [
        "ros2",
        "launch",
        "lb_navigation",
        "nav2_n4_costmaps.launch.py",
        "enable_costmaps:=true",
        "enable_development_tf:=true",
        "allow_test_only_tf:=true",
        "use_rviz:=false",
        "use_arena_scene:=false",
    ]
    with tempfile.TemporaryFile(mode="w+") as output:
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            _spin_until(
                probe,
                lambda: all(client.service_is_ready() for client in lifecycle_clients),
                pose_publisher,
            )
            for client in lifecycle_clients:
                _wait_for_active(probe, client, pose_publisher)
            _spin_until(
                probe,
                lambda: bool(global_costmaps) and bool(local_costmaps),
                pose_publisher,
            )

            global_map = global_costmaps[-1]
            local_map = local_costmaps[-1]
            assert global_map.header.frame_id == "map"
            assert global_map.info.resolution == pytest.approx(0.05)
            assert (global_map.info.width, global_map.info.height) == (139, 102)
            assert max(global_map.data) == 100
            assert any(0 < cost < 100 for cost in global_map.data)
            assert local_map.header.frame_id == "odom"
            assert local_map.info.resolution == pytest.approx(0.05)
            assert (local_map.info.width, local_map.info.height) == (80, 80)
        except Exception:
            output.seek(0)
            pytest.fail(output.read())
        finally:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
            probe.destroy_node()
            rclpy.shutdown()
            if previous_log_directory is None:
                del os.environ["ROS_LOG_DIR"]
            else:
                os.environ["ROS_LOG_DIR"] = previous_log_directory
