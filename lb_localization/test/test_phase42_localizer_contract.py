"""Offline contracts for the Phase 4.2 fail-closed tag-localizer boundary."""

from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
CORE_FILE = PACKAGE_ROOT / "src" / "tag_localizer_core.cpp"
NODE_FILE = PACKAGE_ROOT / "src" / "tag_localizer_node.cpp"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "tag_localizer.launch.py"
TAG_MAP_TEMPLATE = PACKAGE_ROOT / "config" / "tag_map.template.yaml"
LOCALIZER_TEMPLATE = PACKAGE_ROOT / "config" / "tag_localizer.template.yaml"
EKF_TEMPLATE = PACKAGE_ROOT / "config" / "ekf_global.template.yaml"


def test_field_inventory_is_separate_from_the_phase41_bench_tag() -> None:
    """The planned 0.300 m field inventory cannot overwrite the ID-0 bench record."""
    tag_map = TAG_MAP_TEMPLATE.read_text(encoding="utf-8")

    assert "survey_status: unverified" in tag_map
    assert "id: 1" in tag_map
    assert "id: 2" in tag_map
    assert "id: 3" in tag_map
    assert tag_map.count("size_m: 0.300") == 3
    assert "id: 0" not in tag_map
    assert tag_map.count("pose_map: null") == 3
    assert "not a physical world map" in tag_map


def test_localizer_configuration_has_no_hidden_operational_defaults() -> None:
    """A surveyed map, extrinsics, gates, and covariance must be explicit."""
    template = LOCALIZER_TEMPLATE.read_text(encoding="utf-8")
    launch = LAUNCH_FILE.read_text(encoding="utf-8")

    for parameter in (
        "max_tag_distance_m",
        "max_view_angle_deg",
        "min_decision_margin",
        "max_hamming",
        "max_pose_jump_m",
        "max_yaw_jump_rad",
        "max_detection_age_s",
        "minimum_valid_tags",
        "transform_timestamp_tolerance_s",
        "max_intertag_position_spread_m",
        "unobserved_roll_pitch_variance_rad2",
    ):
        assert f"{parameter}: TBD" in template
    assert "observation_frame_prefix: webcam_observation_tag_" in template
    assert "observation_frame_prefix: zed_observation_tag_" in template
    assert 'DeclareLaunchArgument("tag_map_file", default_value="")' in launch
    assert 'DeclareLaunchArgument("localizer_config_file", default_value="")' in launch
    assert "allow_synthetic_test_data" in launch
    assert "no production default exists" in launch


def test_localizer_uses_scoped_observations_and_never_broadcasts_global_tf() -> None:
    """Metric pose comes from exact-time TF, but TF authority remains with the EKFs."""
    core = CORE_FILE.read_text(encoding="utf-8")
    node = NODE_FILE.read_text(encoding="utf-8")

    assert "known_tag.map_to_tag * normalized_camera_to_tag.inverse()" in core
    assert "NormalizedTransform(base_to_camera).inverse()" in core
    assert "minimum_valid_tags" in core
    assert "max_intertag_position_spread_m" in core
    assert "map_to_base" in core
    assert "lookupTransform(" in node
    assert "detection_stamp" in node
    assert "IsFreshObservationTransform" in node
    assert "TransformListener>(*tf_buffer_, this, true)" in node
    assert "webcam_observation_tag_" not in node
    assert "TransformBroadcaster" not in node
    assert "StaticTransformBroadcaster" not in node
    assert "sendTransform" not in node
    assert '"/localization/apriltag_pose"' in node
    assert '"/diagnostics"' in node


def test_future_global_ekf_is_only_a_nonlaunchable_template() -> None:
    """Phase 4.2 does not claim Phase 3 inputs or map-to-odom runtime ownership."""
    ekf_template = EKF_TEMPLATE.read_text(encoding="utf-8")
    bringup = (REPOSITORY_ROOT / "lb_launch" / "launch" / "bringup.launch.py").read_text(
        encoding="utf-8"
    )

    assert "world_frame: map" in ekf_template
    assert "odom_frame: odom" in ekf_template
    assert "base_link_frame: base_link" in ekf_template
    assert "two_d_mode: true" in ekf_template
    assert "publish_tf: TBD" in ekf_template
    assert "not launched" in ekf_template
    assert "tag_localizer.launch.py" not in bringup
