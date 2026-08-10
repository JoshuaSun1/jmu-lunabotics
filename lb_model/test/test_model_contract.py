"""Structural checks for the Phase 1 Xacro description."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as element_tree
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_FILE = PACKAGE_ROOT / "urdf" / "lb_mock.urdf.xacro"
GEOMETRY_FILE = PACKAGE_ROOT / "config" / "mock_geometry.yaml"

WHEEL_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
}
REQUIRED_LINKS = {
    "base_link",
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


def test_mock_profile_is_explicitly_nonphysical() -> None:
    """Synthetic geometry cannot be mistaken for a calibration source."""
    geometry = GEOMETRY_FILE.read_text(encoding="utf-8")
    assert "source: synthetic_mock_only" in geometry
    assert "physical_robot_use: prohibited" in geometry
    assert "replacement_condition: reviewed CAD or measured calibration record" in geometry


def test_xacro_declares_phase_one_scope() -> None:
    """Raw Xacro keeps the required mock interfaces and excludes future frames."""
    model = MODEL_FILE.read_text(encoding="utf-8")
    assert "mock_components/GenericSystem" in model
    assert model.count('<command_interface name="velocity"/>') == 4
    assert "diff_drive_controller" not in model
    assert 'name="map"' not in model
    assert 'name="odom"' not in model
    assert 'name="base_footprint"' not in model


def test_xacro_expands_to_the_expected_tf_tree() -> None:
    """Expand the installed Xacro profile and validate its links and joints."""
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro is not installed on this development host")

    result = subprocess.run(
        [xacro, str(MODEL_FILE), f"geometry_file:={GEOMETRY_FILE}"],
        check=True,
        capture_output=True,
        text=True,
    )
    root = element_tree.fromstring(result.stdout)
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"] for joint in root.findall("joint")}

    assert REQUIRED_LINKS <= links
    assert WHEEL_JOINTS <= joints
    assert {"map", "odom", "base_footprint"}.isdisjoint(links)
