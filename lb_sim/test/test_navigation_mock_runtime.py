"""N5–N8 acceptance against real Nav2 servers and the in-memory drive transport."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from nav_msgs.msg import Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Probe(Node):
    def __init__(self):
        super().__init__("navigation_acceptance_probe")
        self.ready = False
        self.pose = None
        self.commands = []
        self.paths = []
        self.status = []
        self.feedback = []
        self.records = []
        self.command_ages = []
        self.max_linear = self.max_angular = 0.0
        self.resources = []
        self.root_pid = None
        self.process_fault = False
        self.goal_started = time.monotonic()
        self.create_timer(1.0, self.sample_resources)
        self.create_subscription(
            Bool,
            "/navigation/ready",
            self.observe_ready,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/localization/fused_pose", self.observe_pose, 10
        )
        self.create_subscription(TwistStamped, "/cmd_vel", self.observe_command, 10)
        self.create_subscription(NavPath, "/plan", self.paths.append, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.observe_status, 10)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.manager = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )

    def observe_ready(self, message):
        self.ready = message.data

    def observe_pose(self, message):
        self.pose = message

    def observe_command(self, message):
        self.commands.append((time.monotonic(), message))
        self.max_linear = max(self.max_linear, abs(message.twist.linear.x))
        self.max_angular = max(self.max_angular, abs(message.twist.angular.z))
        age = (
            self.get_clock().now().nanoseconds
            - message.header.stamp.sec * 1_000_000_000
            - message.header.stamp.nanosec
        ) / 1.0e9
        self.command_ages.append(age)

    def processes(self):
        """Inspect only the process group created for this test's launch."""
        result = []
        for path in Path("/proc").iterdir():
            if not path.name.isdigit():
                continue
            try:
                fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) != self.root_pid:
                    continue
                executable = (path / "cmdline").read_bytes().split(b"\0")[0].decode()
                result.append(
                    (
                        int(path.name),
                        executable,
                        (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK"),
                        int(fields[21]) * os.sysconf("SC_PAGE_SIZE"),
                    )
                )
            except (OSError, ValueError, IndexError):
                continue
        return result

    def sample_resources(self):
        if self.root_pid is None:
            return
        processes = self.processes()
        self.resources.append(
            {
                "time": time.monotonic(),
                "processes": len(processes),
                "cpu_s": sum(item[2] for item in processes),
                "rss_mib": sum(item[3] for item in processes) / 1024**2,
            }
        )

    def observe_status(self, message):
        self.status.extend(
            status.message
            for status in message.status
            if status.name.endswith("navigation_supervisor")
        )

    def wait(self, condition, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise AssertionError(f"Timed out; recent supervisor status: {self.status[-10:]}")

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, timeout=30.0):
        self.wait(client.service_is_ready, timeout)
        future = client.call_async(request)
        self.wait(future.done, timeout)
        return future.result()

    def switch(self, name, value):
        client = self.create_client(SetBool, f"/navigation_mock_world/scenario/{name}")
        assert self.call(client, SetBool.Request(data=value)).success
        self.destroy_client(client)

    def scenario(self, name):
        client = self.create_client(SetParameters, "/navigation_mock_world/set_parameters")
        response = self.call(
            client,
            SetParameters.Request(
                parameters=[
                    Parameter(name="scenario", value=ParameterValue(type=4, string_value=name))
                ]
            ),
        )
        assert all(result.successful for result in response.results)
        self.destroy_client(client)

    def goal(self, x, y):
        self.feedback.clear()
        self.goal_started = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.w = 1.0
        self.wait(self.action.server_is_ready)
        future = self.action.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback)
        )
        self.wait(future.done)
        return future.result()

    def result(self, handle, timeout=60):
        future = handle.get_result_async()
        self.wait(future.done, timeout)
        response = future.result()
        self.records.append(
            {
                "status": response.status,
                "error": response.result.error_msg,
                "duration_s": time.monotonic() - self.goal_started,
                "recoveries": max((item.number_of_recoveries for item in self.feedback), default=0),
            }
        )
        return response

    def assert_stopped(self, seconds=0.5):
        self.commands.clear()
        # Different action/command topics can deliver already queued samples
        # in either order. Observe the first zero, then require only zeroes.
        before = time.monotonic()
        self.wait(
            lambda: any(
                msg.twist.linear.x == 0 and msg.twist.angular.z == 0 for _, msg in self.commands
            ),
            timeout=0.25,
        )
        assert time.monotonic() - before < 0.3
        self.commands.clear()
        self.spin(seconds)
        assert self.commands, "No explicit stop commands"
        assert all(
            item.twist.linear.x == 0 and item.twist.angular.z == 0 for _, item in self.commands
        )

    def cost(self, x, y):
        client = self.create_client(GetCostmap, "/global_costmap/get_costmap")
        grid = self.call(client, GetCostmap.Request()).map
        self.destroy_client(client)
        column = int((x - grid.metadata.origin.position.x) / grid.metadata.resolution)
        row = int((y - grid.metadata.origin.position.y) / grid.metadata.resolution)
        return grid.data[row * grid.metadata.size_x + column]


def test_n5_n8_mock_navigation_acceptance():
    required = (
        "nav2_bt_navigator",
        "nav2_controller",
        "nav2_navfn_planner",
        "nav2_velocity_smoother",
        "controller_manager",
        "diff_drive_controller",
        "xacro",
    )
    for package in required:
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            pytest.skip(f"Mock acceptance requires {package}; install declared dependencies")
    directory = Path(os.environ.get("LB_MOCK_EVIDENCE_DIR", "/tmp/lb_mock_acceptance"))
    directory.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", "85")
    environment.setdefault("ROS_LOG_DIR", str(directory / "ros_logs"))
    rclpy.init(domain_id=int(environment["ROS_DOMAIN_ID"]))
    probe = Probe()
    command = [
        "ros2",
        "launch",
        "lb_sim",
        "nav2_mock.launch.py",
        "enable_nav2:=true",
        "allow_mock_navigation:=true",
        "allow_test_only_tf:=true",
        "use_rviz:=false",
    ]
    started = time.monotonic()
    with (directory / "launch.log").open("w+") as output:
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        probe.root_pid = process.pid
        try:
            probe.wait(lambda: probe.pose is not None)
            assert not probe.goal(2.0, 1.5).accepted, "Goal accepted before lifecycle readiness"
            assert probe.call(probe.manager, ManageLifecycleNodes.Request(command=0), 45).success
            probe.wait(lambda: probe.ready)
            assert [info.node_name for info in probe.get_publishers_info_by_topic("/cmd_vel")] == [
                "navigation_supervisor"
            ]
            # Fresh commands alone cannot authorize motion without a public goal.
            injection = probe.create_publisher(TwistStamped, "/cmd_vel_nav", 10)
            probe.wait(lambda: injection.get_subscription_count() > 0)
            message = TwistStamped()
            message.header.frame_id = "base_link"
            message.header.stamp = probe.get_clock().now().to_msg()
            message.twist.linear.x = 0.2
            injection.publish(message)
            probe.assert_stopped()
            probe.destroy_publisher(injection)
            for x, y in ((2.0, 1.5), (2.0, 2.0)):
                handle = probe.goal(x, y)
                assert handle.accepted
                response = probe.result(handle)
                assert response.status == GoalStatus.STATUS_SUCCEEDED, response
                assert (
                    math.hypot(
                        probe.pose.pose.pose.position.x - x, probe.pose.pose.pose.position.y - y
                    )
                    < 0.2
                )
            assert probe.paths and any(
                abs(message.twist.linear.x) > 0 for _, message in probe.commands
            )
            assert all(
                abs(message.twist.linear.x) <= 0.5 + 1e-6
                and abs(message.twist.angular.z) <= 1.0 + 1e-6
                for _, message in probe.commands
            )
            probe.assert_stopped()
            if os.environ.get("LB_MOCK_BASELINE_ONLY") != "1":
                exercise_faults(probe)
                exercise_endurance_and_node_loss(probe)
            cpu_percent = [
                max(0.0, (b["cpu_s"] - a["cpu_s"]) / (b["time"] - a["time"]) * 100)
                for a, b in zip(probe.resources, probe.resources[1:])
            ]
            assert probe.max_linear <= 0.5 + 1.0e-6
            assert probe.max_angular <= 1.0 + 1.0e-6
            (directory / "plans.json").write_text(
                json.dumps(
                    [
                        {
                            "frame": path.header.frame_id,
                            "xy": [
                                [pose.pose.position.x, pose.pose.position.y] for pose in path.poses
                            ],
                        }
                        for path in probe.paths
                    ],
                    indent=2,
                )
                + "\n"
            )
            (directory / "results.json").write_text(
                json.dumps(
                    {
                        "physical_use": "prohibited",
                        "elapsed_s": time.monotonic() - started,
                        "goals": probe.records,
                        "last_status": probe.status[-10:],
                        "resource_samples": probe.resources,
                        "max_aggregate_rss_mib": max(
                            (item["rss_mib"] for item in probe.resources), default=0
                        ),
                        "cpu_core_percent_mean": statistics.mean(cpu_percent) if cpu_percent else 0,
                        "cpu_core_percent_peak": max(cpu_percent, default=0),
                        "command_age_s_max": max(probe.command_ages, default=0),
                        "max_linear_mps": probe.max_linear,
                        "max_angular_radps": probe.max_angular,
                        "command_age_s_median": (
                            statistics.median(probe.command_ages) if probe.command_ages else 0
                        ),
                    },
                    indent=2,
                )
                + "\n"
            )
        except Exception:
            output.flush()
            output.seek(0)
            print(output.read())
            raise
        finally:
            try:
                if probe.manager.service_is_ready() and not probe.process_fault:
                    probe.call(probe.manager, ManageLifecycleNodes.Request(command=4), 15)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                probe.destroy_node()
                rclpy.shutdown()


def exercise_faults(probe):
    """Extend acceptance with N7/N8 faults once the N5/N6 baseline succeeds."""
    handle = probe.goal(4.0, 2.0)
    assert handle.accepted
    future = handle.cancel_goal_async()
    probe.wait(future.done)
    assert future.result().goals_canceling
    assert probe.result(handle, 3).status == GoalStatus.STATUS_CANCELED
    probe.assert_stopped()
    probe.spin(0.5)
    for source in ("localization_enabled", "scans_enabled", "hazards_enabled"):
        probe.wait(lambda: probe.ready)
        handle = probe.goal(4.0, 2.0)
        assert handle.accepted
        probe.commands.clear()
        probe.wait(lambda: any(abs(msg.twist.linear.x) > 0 for _, msg in probe.commands))
        before = time.monotonic()
        probe.switch(source, False)
        response = probe.result(handle, 3.0)
        assert response.status == GoalStatus.STATUS_ABORTED
        latency = time.monotonic() - before
        assert latency < 1.5
        probe.records[-1]["fault_to_result_s"] = latency
        probe.assert_stopped()
        probe.switch(source, True)
        probe.spin(0.5)
        probe.wait(lambda: probe.ready)
        probe.assert_stopped()  # Reacquisition never resumes the cancelled goal.

    probe.switch("reset_pose", True)
    probe.spin(1.0)
    probe.paths.clear()
    handle = probe.goal(4.0, 1.5)
    assert handle.accepted
    probe.wait(lambda: bool(probe.paths))
    probe.scenario("block")
    probe.wait(lambda: any(abs(pose.pose.position.y - 1.5) > 0.4 for pose in probe.paths[-1].poses))
    # A new obstacle invalidates the straight route; the live planner detours.
    client = probe.create_client(GetCostmap, "/global_costmap/get_costmap")
    grid = probe.call(client, GetCostmap.Request()).map
    probe.destroy_client(client)
    for pose in probe.paths[-1].poses:
        column = int(
            (pose.pose.position.x - grid.metadata.origin.position.x) / grid.metadata.resolution
        )
        row = int(
            (pose.pose.position.y - grid.metadata.origin.position.y) / grid.metadata.resolution
        )
        assert grid.data[row * grid.metadata.size_x + column] < 253
    cancellation = handle.cancel_goal_async()
    probe.wait(cancellation.done)
    assert probe.result(handle, 3).status == GoalStatus.STATUS_CANCELED
    probe.assert_stopped()
    assert probe.cost(2.3, 1.5) >= 253
    probe.scenario("clear")
    probe.spin(1.5)
    assert probe.cost(2.3, 1.5) == 0
    probe.scenario("hazard")
    probe.spin(1.5)
    assert probe.cost(2.3, 1.3) >= 253
    probe.scenario("clear")
    probe.spin(0.5)
    # Hazard points deliberately persist until explicitly cleared; a stale
    # source stops motion independently of retained costmap data.
    from nav2_msgs.srv import ClearEntireCostmap

    for name in ("global_costmap", "local_costmap"):
        client = probe.create_client(ClearEntireCostmap, f"/{name}/clear_entirely_{name}")
        probe.call(client, ClearEntireCostmap.Request())
        probe.destroy_client(client)
    probe.spin(1.0)
    for goal in ((10.0, 10.0),):
        handle = probe.goal(*goal)
        assert handle.accepted
        assert probe.result(handle, 20).status == GoalStatus.STATUS_ABORTED
        probe.assert_stopped()
    probe.feedback.clear()
    probe.switch("freeze_pose", True)
    handle = probe.goal(4.0, 1.5)
    assert handle.accepted
    assert probe.result(handle, 25).status == GoalStatus.STATUS_ABORTED
    assert max((item.number_of_recoveries for item in probe.feedback), default=0) >= 1
    probe.assert_stopped()

    probe.switch("freeze_pose", False)
    probe.switch("reset_pose", True)
    probe.spin(1.0)
    probe.scenario("wall")
    probe.spin(1.5)
    handle = probe.goal(4.0, 1.5)
    assert handle.accepted
    assert probe.result(handle, 25).status == GoalStatus.STATUS_ABORTED
    probe.assert_stopped()


def exercise_endurance_and_node_loss(probe):
    from nav2_msgs.srv import ClearEntireCostmap

    probe.scenario("clear")
    probe.switch("reset_pose", True)
    for name in ("global_costmap", "local_costmap"):
        client = probe.create_client(ClearEntireCostmap, f"/{name}/clear_entirely_{name}")
        probe.call(client, ClearEntireCostmap.Request())
        probe.destroy_client(client)
    probe.spin(1.0)
    # Freeze the controller process (including its local costmap and lifecycle
    # service). Command age first stops output; lifecycle TTL aborts the action.
    controllers = [
        pid
        for pid, executable, _, _ in probe.processes()
        if executable.endswith("/nav2_controller/controller_server")
    ]
    assert len(controllers) == 1
    handle = probe.goal(4.0, 1.5)
    assert handle.accepted
    probe.commands.clear()
    probe.wait(lambda: any(abs(msg.twist.linear.x) > 0 for _, msg in probe.commands))
    before = time.monotonic()
    os.kill(controllers[0], signal.SIGSTOP)
    try:
        assert probe.result(handle, 3).status == GoalStatus.STATUS_ABORTED
        assert time.monotonic() - before < 1.5
        probe.records[-1]["controller_stall_to_result_s"] = time.monotonic() - before
        probe.assert_stopped()
    finally:
        os.kill(controllers[0], signal.SIGCONT)
    probe.spin(1.0)
    probe.wait(lambda: probe.ready)
    probe.assert_stopped()
    probe.switch("reset_pose", True)
    probe.spin(1.0)
    duration = float(os.environ.get("LB_MOCK_SOAK_SECONDS", "0"))
    assert math.isfinite(duration) and 0 <= duration <= 180
    deadline = time.monotonic() + duration
    count = 0
    safety_aborts = 0
    while time.monotonic() < deadline:
        probe.wait(lambda: probe.ready)
        handle = probe.goal(2.0 if count % 2 == 0 else 1.5, 1.5)
        assert handle.accepted
        response = probe.result(handle)
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            count += 1
        else:
            assert response.status == GoalStatus.STATUS_ABORTED
            assert response.result.error_msg in {
                "TF_STALE",
                "ODOMETRY_STALE",
                "LOCALIZATION_STALE",
                "LOCALIZATION_INVALID",
                "SCAN_STALE",
                "HAZARDS_STALE",
            }, response
            safety_aborts += 1
            probe.assert_stopped()
            probe.spin(0.5)
            probe.wait(lambda: probe.ready)
            probe.assert_stopped()  # Only the next explicit goal may resume motion.
    if duration > 0:
        assert count >= 2, "The soak must demonstrate repeated successful navigation"
        assert safety_aborts <= count, "Freshness stops must not dominate navigation"
    probe.records.append(
        {"soak_requested_s": duration, "soak_goals": count, "soak_safety_aborts": safety_aborts}
    )
    probe.commands.clear()
    handle = probe.goal(4.0, 1.5)
    assert handle.accepted
    probe.wait(lambda: any(abs(msg.twist.linear.x) > 0 for _, msg in probe.commands))
    planners = [
        pid
        for pid, executable, _, _ in probe.processes()
        if executable.endswith("/nav2_planner/planner_server")
    ]
    assert len(planners) == 1
    before = time.monotonic()
    probe.process_fault = True
    os.kill(planners[0], signal.SIGKILL)
    response = probe.result(handle, 3)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert time.monotonic() - before < 1.5
    probe.records[-1]["node_loss_to_result_s"] = time.monotonic() - before
    probe.assert_stopped()
