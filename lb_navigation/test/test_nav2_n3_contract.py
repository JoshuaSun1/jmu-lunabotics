"""Offline safety and launch contracts for the Nav2 N3 development TF profile."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
LAUNCH = PACKAGE_ROOT / "launch" / "nav2_n3_development.launch.py"
TF_CONFIG = REPOSITORY_ROOT / "lb_localization" / "config" / "apriltag_development_tf_bridge.yaml"
TF_SCRIPT = REPOSITORY_ROOT / "lb_localization" / "scripts" / "apriltag_development_tf_bridge.py"


def test_n3_tf_requires_two_opt_ins_and_rejects_ekf_coexistence() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_development_tf", default_value="false")' in source
    assert 'DeclareLaunchArgument("allow_test_only_tf", default_value="false")' in source
    assert 'DeclareLaunchArgument("production_ekfs_enabled", default_value="false")' in source
    assert "if enable_development_tf and not allow_test_only_tf:" in source
    assert "if enable_development_tf and production_ekfs_enabled:" in source
    assert 'localization_profile != "apriltag_development_only"' in source


def test_n3_launch_has_one_tf_owner_and_no_actuating_components() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    bridge_source = TF_SCRIPT.read_text(encoding="utf-8")

    assert source.count('executable="apriltag_development_tf_bridge.py"') == 1
    assert bridge_source.count("TransformBroadcaster(") == 1
    assert "StaticTransformBroadcaster" not in bridge_source
    assert "robot_localization" not in source
    assert "ekf_node" not in source
    assert "planner_server" not in source
    assert "controller_server" not in source
    assert "bt_navigator" not in source
    assert "diff_drive_controller" not in source
    assert 'package="lb_hardware"' not in source
    assert 'remappings=[("cmd_vel"' not in source


def test_n3_config_matches_batch1_freshness_and_prohibits_physical_use() -> None:
    parameters = yaml.safe_load(TF_CONFIG.read_text(encoding="utf-8"))[
        "apriltag_development_tf_bridge"
    ]["ros__parameters"]
    passthrough = yaml.safe_load(
        (
            REPOSITORY_ROOT
            / "lb_localization"
            / "config"
            / "fused_pose_passthrough_development.yaml"
        ).read_text(encoding="utf-8")
    )["fused_pose_passthrough"]["ros__parameters"]

    assert parameters["allow_test_only_tf"] is False
    assert parameters["production_ekfs_enabled"] is False
    assert parameters["physical_robot_use"] == "prohibited"
    assert parameters["fused_pose_topic"] == passthrough["output_topic"]
    assert parameters["valid_topic"] == passthrough["valid_topic"]
    assert parameters["freshness_timeout_s"] == passthrough["freshness_timeout_s"]
    assert [parameters[key] for key in ("map_frame", "odom_frame", "base_frame")] == [
        "map",
        "odom",
        "base_link",
    ]
