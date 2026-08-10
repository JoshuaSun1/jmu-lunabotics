#!/usr/bin/env python3
"""Check the Phase 1 TF authority contract statically or against a running mock."""

from __future__ import annotations

import argparse
from pathlib import Path


MOCK_REQUIRED_FRAMES = {
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


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def check_contract() -> int:
    """Ensure the documented and source-level sole-authority rules are present."""
    architecture = (REPOSITORY_ROOT / "docs" / "architecture.md").read_text(encoding="utf-8")
    expected_phrases = (
        "`map -> odom` | Global `robot_localization` EKF",
        "`odom -> base_link` | Local `robot_localization` EKF",
        "`base_link -> fixed sensor and mount frames` | `robot_state_publisher`",
        "`base_link -> wheel and mechanism joint frames` | `robot_state_publisher`",
    )
    missing = [phrase for phrase in expected_phrases if phrase not in architecture]
    if missing:
        for phrase in missing:
            print(f"missing TF authority contract: {phrase}")
        return 1
    model = (REPOSITORY_ROOT / "lb_model" / "urdf" / "lb_mock.urdf.xacro").read_text(
        encoding="utf-8"
    )
    missing_frames = []
    for frame in MOCK_REQUIRED_FRAMES:
        if frame.endswith("_wheel_link"):
            wheel_name = frame.removesuffix("_link")
            declaration = f'<xacro:mock_wheel name="{wheel_name}"'
        else:
            declaration = f'name="{frame}"'
        if declaration not in model:
            missing_frames.append(frame)
    if missing_frames:
        for frame in sorted(missing_frames):
            print(f"missing Phase 1 mock frame: {frame}")
        return 1
    if 'name="map"' in model or 'name="odom"' in model:
        print("Phase 1 model must not create map or odom frames")
        return 1
    print("Phase 1 TF authority contract and mock frame declarations are present.")
    return 0


def check_runtime(timeout_s: float) -> int:
    """Check a running Phase 1 mock graph without starting or commanding hardware."""
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as error:
        print(f"runtime check requires ROS 2 Python dependencies: {error}")
        return 2

    rclpy.init()
    node = rclpy.create_node("phase1_tf_authority_check")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        for frame in sorted(MOCK_REQUIRED_FRAMES):
            deadline_ns = node.get_clock().now().nanoseconds + int(timeout_s * 1_000_000_000)
            while node.get_clock().now().nanoseconds < deadline_ns:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    buffer.lookup_transform(
                        "base_link", frame, Time(), timeout=Duration(seconds=0.1)
                    )
                except TransformException:
                    continue
                break
            else:
                print(f"missing runtime transform: base_link -> {frame}")
                return 1

        frame_graph = buffer.all_frames_as_yaml()
        if "map:" in frame_graph or "odom:" in frame_graph:
            print("Phase 1 mock graph unexpectedly contains map or odom")
            return 1

        expected_publishers = {
            "/tf": {"robot_state_publisher"},
            "/tf_static": {"robot_state_publisher"},
            "/joint_states": {"joint_state_broadcaster"},
        }
        for topic, expected in expected_publishers.items():
            publishers = {entry.node_name for entry in node.get_publishers_info_by_topic(topic)}
            if publishers != expected:
                print(
                    f"unexpected {topic} publishers: {sorted(publishers)} (expected {sorted(expected)})"
                )
                return 1
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()

    print("Phase 1 runtime TF authority check passed.")
    return 0


def main() -> int:
    """Parse command-line options and run static or live Phase 1 checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runtime",
        action="store_true",
        help="verify a separately running `lb_sim mock_robot.launch.py` graph",
    )
    parser.add_argument(
        "--timeout", type=float, default=15.0, help="runtime TF wait timeout in seconds"
    )
    arguments = parser.parse_args()
    if arguments.runtime:
        return check_runtime(arguments.timeout)
    return check_contract()


if __name__ == "__main__":
    raise SystemExit(main())
