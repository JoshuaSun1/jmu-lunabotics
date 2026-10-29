"""Contracts for the N4 static-only development costmaps."""

import ast
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
PARAMS = PACKAGE_ROOT / "config" / "nav2_n4_costmaps_development.yaml"
LIMITS = PACKAGE_ROOT / "config" / "robot_limits_development.yaml"
LAUNCH = PACKAGE_ROOT / "launch" / "nav2_n4_costmaps.launch.py"
RVIZ = PACKAGE_ROOT / "rviz" / "nav2_n4_costmaps.rviz"


def _parameters():
    configuration = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    return (
        configuration,
        configuration["global_costmap"]["global_costmap"]["ros__parameters"],
        configuration["local_costmap"]["local_costmap"]["ros__parameters"],
    )


def test_costmaps_share_the_approved_footprint_and_static_only_layers() -> None:
    _, global_costmap, local_costmap = _parameters()
    limits = yaml.safe_load(LIMITS.read_text(encoding="utf-8"))
    footprint = limits["footprint"]["polygon_xy_m"]

    assert ast.literal_eval(global_costmap["footprint"]) == footprint
    assert ast.literal_eval(local_costmap["footprint"]) == footprint
    assert global_costmap["footprint_padding"] == 0.0
    assert local_costmap["footprint_padding"] == 0.0
    assert global_costmap["plugins"] == ["static_layer", "inflation_layer"]
    assert local_costmap["plugins"] == ["static_layer", "inflation_layer"]
    parameter_text = PARAMS.read_text(encoding="utf-8")
    assert "obstacle_layer" not in parameter_text
    assert "voxel_layer" not in parameter_text
    assert "LaserScan" not in parameter_text
    assert "PointCloud2" not in parameter_text


def test_global_and_local_geometry_matches_the_n4_decision() -> None:
    _, global_costmap, local_costmap = _parameters()

    assert global_costmap["global_frame"] == "map"
    assert global_costmap["rolling_window"] is False
    assert local_costmap["global_frame"] == "odom"
    assert local_costmap["rolling_window"] is True
    assert local_costmap["width"] == 4.0
    assert local_costmap["height"] == 4.0
    assert global_costmap["resolution"] == 0.05
    assert local_costmap["resolution"] == 0.05
    assert global_costmap["robot_base_frame"] == "base_link"
    assert local_costmap["robot_base_frame"] == "base_link"


@pytest.mark.parametrize("costmap_name", ["global_costmap", "local_costmap"])
def test_inflation_is_explicit_provisional_and_nine_cells(costmap_name: str) -> None:
    _, global_costmap, local_costmap = _parameters()
    costmap = {"global_costmap": global_costmap, "local_costmap": local_costmap}[costmap_name]

    inflation = costmap["inflation_layer"]
    assert inflation["inflation_radius"] == 0.45
    assert inflation["cost_scaling_factor"] == 3.0
    assert inflation["inflation_radius"] / costmap["resolution"] == pytest.approx(9.0)
    assert costmap["static_layer"]["map_topic"] == "/map"
    assert costmap["static_layer"]["map_subscribe_transient_local"] is True


def test_lifecycle_order_and_launch_are_fail_closed_and_non_actuating() -> None:
    configuration, _, _ = _parameters()
    manager = configuration["lifecycle_manager_n4"]["ros__parameters"]
    source = LAUNCH.read_text(encoding="utf-8")

    assert manager["node_names"] == [
        "map_server",
        "global_costmap/global_costmap",
        "local_costmap/local_costmap",
    ]
    assert 'DeclareLaunchArgument("enable_costmaps", default_value="false")' in source
    assert 'DeclareLaunchArgument("enable_development_tf", default_value="false")' in source
    assert 'DeclareLaunchArgument("allow_test_only_tf", default_value="false")' in source
    assert "if enable_costmaps and not enable_development_tf:" in source
    assert "if enable_costmaps and not allow_test_only_tf:" in source
    assert "if enable_costmaps and production_ekfs_enabled:" in source
    assert 'package="nav2_map_server"' in source
    assert source.count('package="nav2_costmap_2d"') == 2
    assert 'package="nav2_lifecycle_manager"' in source
    for forbidden in (
        "planner_server",
        "controller_server",
        "bt_navigator",
        "velocity_smoother",
        "diff_drive_controller",
        'package="lb_hardware"',
        'remappings=[("cmd_vel"',
    ):
        assert forbidden not in source


def test_rviz_has_map_costmaps_footprints_pose_and_tf() -> None:
    source = RVIZ.read_text(encoding="utf-8")

    for topic in (
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/global_costmap/published_footprint",
        "/local_costmap/published_footprint",
        "/localization/fused_pose",
    ):
        assert f"Value: {topic}" in source
    assert "Class: rviz_default_plugins/TF" in source
    assert "Fixed Frame: map" in source
    assert "static + inflation only" in source


def test_static_arena_boundary_remains_lethal_and_interior_free() -> None:
    image = PACKAGE_ROOT / "maps" / "arena.pgm"
    with image.open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        dimensions = stream.readline()
        while dimensions.startswith(b"#"):
            dimensions = stream.readline()
        columns, rows = (int(value) for value in dimensions.split())
        assert int(stream.readline()) == 255
        pixels = stream.read()

    grid = [pixels[row * columns : (row + 1) * columns] for row in range(rows)]
    assert all(pixel == 0 for pixel in grid[0] + grid[-1])
    assert all(row[0] == 0 and row[-1] == 0 for row in grid)
    assert set(pixel for row in grid[1:-1] for pixel in row[1:-1]) == {254}
