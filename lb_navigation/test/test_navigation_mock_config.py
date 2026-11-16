"""Mock Nav2 configuration preserves shared limits and finite safe recoveries."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]


def test_mock_servers_and_costmaps_share_frames_limits_and_observation_contracts():
    params = yaml.safe_load((ROOT / "config/nav2_mock_development.yaml").read_text())
    limits = yaml.safe_load((ROOT / "config/robot_limits_development.yaml").read_text())
    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["enable_stamped_cmd_vel"] is True
    assert smoother["max_velocity"] == [
        limits["motion_limits"]["max_linear_velocity_mps"],
        0.0,
        limits["motion_limits"]["max_angular_velocity_radps"],
    ]
    for name, frame in (("global_costmap", "map"), ("local_costmap", "odom")):
        profile = params[name][name]["ros__parameters"]
        assert profile["global_frame"] == frame
        assert profile["obstacle_layer"]["scan"]["topic"] == "/sim/scan"
        assert profile["hazard_layer"]["hazards"]["topic"] == "/sim/hazards"
    assert params["behavior_server"]["ros__parameters"]["behavior_plugins"] == ["wait"]
    lifecycle = params["lifecycle_manager_navigation"]["ros__parameters"]
    assert lifecycle["autostart"] is False
    assert lifecycle["bond_timeout"] > 0
    assert "bt_navigator" in lifecycle["node_names"]


def test_behavior_tree_has_bounded_wait_clear_recovery_and_no_motion_recoveries():
    tree = ET.parse(ROOT / "behavior_trees/mock_navigate.xml")
    assert tree.find(".//RecoveryNode").attrib["number_of_retries"] == "2"
    assert tree.find(".//RateController").attrib["hz"] == "1.0"
    assert tree.find(".//Wait") is not None
    assert tree.find(".//ClearEntireCostmap") is not None
    for forbidden in ("Spin", "BackUp", "DriveOnHeading"):
        assert tree.find(f".//{forbidden}") is None
