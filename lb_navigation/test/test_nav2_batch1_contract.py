"""Offline contracts for the non-actuating Nav2 Batch 1 scaffold."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
LIMITS = PACKAGE_ROOT / "config" / "robot_limits_development.yaml"
TEMPLATE = PACKAGE_ROOT / "config" / "nav2_development.template.yaml"
LAUNCH = PACKAGE_ROOT / "launch" / "nav2_batch1.launch.py"
LOCALIZATION_CONFIG = (
    REPOSITORY_ROOT / "lb_localization" / "config" / "fused_pose_passthrough_development.yaml"
)
GATE_CONFIG = REPOSITORY_ROOT / "lb_safety" / "config" / "navigation_command_gate_development.yaml"


def test_estimated_centered_footprint_is_explicitly_nonphysical() -> None:
    limits = yaml.safe_load(LIMITS.read_text(encoding="utf-8"))

    assert limits["profile"] == "nav2_batch1_development_only"
    assert limits["physical_robot_use"] == "prohibited"
    assert limits["footprint"]["source"] == "team_estimate_2026-10-26"
    assert limits["footprint"]["base_link_origin_status"] == "assumed_rectangle_center"
    assert limits["footprint"]["length_m"] == 1.1
    assert limits["footprint"]["width_m"] == 0.6
    assert limits["footprint"]["padding_m"] == 0.0
    assert limits["footprint"]["polygon_xy_m"] == [
        [0.55, 0.30],
        [0.55, -0.30],
        [-0.55, -0.30],
        [-0.55, 0.30],
    ]
    assert limits["motion_limits"]["physical_robot_use"] == "prohibited"


def test_nav2_template_is_nonrunnable_and_uses_the_same_footprint() -> None:
    limits = yaml.safe_load(LIMITS.read_text(encoding="utf-8"))
    template = yaml.safe_load(TEMPLATE.read_text(encoding="utf-8"))

    assert template["configuration_status"] == "batch1_non_runnable_template"
    assert template["physical_robot_use"] == "prohibited"
    assert template["footprint"]["polygon_xy_m"] == limits["footprint"]["polygon_xy_m"]
    assert template["footprint"]["padding_m"] == 0.0
    assert template["global_costmap"]["plugins"] == "TBD"
    assert template["local_costmap"]["plugins"] == "TBD"
    assert template["planner"]["plugin"] == "TBD"
    assert template["controller"]["plugin"] == "TBD"
    assert template["command_boundary"]["final_drive_connection_enabled"] is False


def test_localization_and_gate_share_the_approved_timeout() -> None:
    localization = yaml.safe_load(LOCALIZATION_CONFIG.read_text(encoding="utf-8"))[
        "fused_pose_passthrough"
    ]["ros__parameters"]
    gate = yaml.safe_load(GATE_CONFIG.read_text(encoding="utf-8"))["navigation_command_gate"][
        "ros__parameters"
    ]

    assert localization["freshness_timeout_s"] == 0.5
    assert gate["freshness_timeout_s"] == 0.5
    assert localization["output_topic"] == gate["fused_pose_topic"]
    assert localization["valid_topic"] == gate["valid_topic"]


def test_batch1_launch_starts_no_nav2_tf_hardware_or_final_command_path() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'package="lb_localization"' in source
    assert 'executable="fused_pose_passthrough.py"' in source
    assert 'package="lb_safety"' in source
    assert 'executable="navigation_command_gate.py"' in source
    assert "planner_server" not in source
    assert "controller_server" not in source
    assert "bt_navigator" not in source
    assert "robot_localization" not in source
    assert "static_transform_publisher" not in source
    assert "diff_drive_controller" not in source
    assert 'package="lb_hardware"' not in source
