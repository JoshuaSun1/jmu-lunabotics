"""Live N4 lifecycle/costmap test, skipped when declared Nav2 packages are absent."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


REQUIRED_PACKAGES = (
    "nav2_costmap_2d",
    "nav2_lifecycle_manager",
    "nav2_map_server",
    "nav2_msgs",
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


def _spin_until(node: Node, condition, timeout_s: float = 20.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise AssertionError("timed out waiting for N4 runtime condition")


def _call(node: Node, client, request):
    _spin_until(node, client.service_is_ready)
    future = client.call_async(request)
    _spin_until(node, future.done)
    return future.result()


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

    # A timer bounds the synthetic input to 20 Hz even when service replies
    # and costmap callbacks are immediately ready.
    def publish_pose():
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = probe.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 1.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.04
        pose.pose.covariance[7] = 0.04
        pose_publisher.publish(pose)

    pose_timer = probe.create_timer(0.05, publish_pose)
    transforms, validity, commands = [], [], []
    footprints = {name: [] for name in ("global_costmap", "local_costmap")}
    probe.create_subscription(TFMessage, "/tf", transforms.append, 10)
    probe.create_subscription(Bool, "/localization/valid", validity.append, map_qos)
    probe.create_subscription(TwistStamped, "/cmd_vel_nav", commands.append, 10)
    raw_publisher = probe.create_publisher(TwistStamped, "/cmd_vel_nav_raw", 10)
    for name, samples in footprints.items():
        probe.create_subscription(
            PolygonStamped, f"/{name}/published_footprint", samples.append, 10
        )
    manager = probe.create_client(ManageLifecycleNodes, "/lifecycle_manager_n4/manage_nodes")

    command = [
        "ros2",
        "launch",
        "lb_navigation",
        "nav2_n4_costmaps.launch.py",
        "enable_costmaps:=true",
        "enable_development_tf:=true",
        "allow_test_only_tf:=true",
        "autostart:=false",
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
            start_new_session=True,
        )
        try:
            # Configure/activate only after the opted-in test TF is available.
            _spin_until(probe, lambda: bool(transforms))
            request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
            assert _call(probe, manager, request).success
            for client in lifecycle_clients:
                assert _call(probe, client, GetState.Request()).current_state.id == ACTIVE_STATE
            _spin_until(
                probe,
                lambda: bool(global_costmaps)
                and bool(local_costmaps)
                and all(footprints.values()),
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
            for samples in footprints.values():
                points = samples[-1].polygon.points
                assert len(points) == 4
                assert sorted(point.x for point in points) == pytest.approx([0.45, 0.45, 1.55, 1.55])
                assert sorted(point.y for point in points) == pytest.approx([0.70, 0.70, 1.30, 1.30])

            # Query the live raw grid: perimeter is lethal, adjacent cells are
            # inflated, and the central arena is free.
            client = probe.create_client(GetCostmap, "/global_costmap/get_costmap")
            grid = _call(probe, client, GetCostmap.Request()).map
            width = grid.metadata.size_x
            assert grid.data[50 * width] == 254
            assert 0 < grid.data[50 * width + 1] < 254
            assert grid.data[50 * width + 60] == 0

            def publish_command():
                command = TwistStamped()
                command.header.stamp = probe.get_clock().now().to_msg()
                command.twist.linear.x = 0.2
                raw_publisher.publish(command)

            command_timer = probe.create_timer(0.05, publish_command)
            _spin_until(probe, lambda: any(msg.twist.linear.x == 0.2 for msg in commands))
            pose_timer.cancel()
            # Drain queued input, then observe the 0.50 s validity timeout.
            _spin_until(probe, lambda: validity and not validity[-1].data)
            commands.clear()
            _spin_until(probe, lambda: len(commands) >= 5)
            assert all(msg.twist.linear.x == 0.0 for msg in commands)
            command_timer.cancel()
            # N4 has no final drive publisher, even while costmaps are active.
            assert not probe.get_publishers_info_by_topic("/cmd_vel")
        except Exception:
            output.seek(0)
            pytest.fail(
                f"TF samples: {len(transforms)}, validity: {[v.data for v in validity[-5:]]}\n"
                f"TF subscribers: {probe.get_subscriptions_info_by_topic('/tf')}\n"
                + output.read()
            )
        finally:
            if manager.service_is_ready():
                _call(
                    probe,
                    manager,
                    ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN),
                )
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
            probe.destroy_node()
            rclpy.shutdown()
            if previous_log_directory is None:
                del os.environ["ROS_LOG_DIR"]
            else:
                os.environ["ROS_LOG_DIR"] = previous_log_directory
