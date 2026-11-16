"""Offline contracts for the lifecycle-managed static arena map launch."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
PARAMS = PACKAGE_ROOT / "config" / "map_server.yaml"
LAUNCH = PACKAGE_ROOT / "launch" / "map.launch.py"
RVIZ = PACKAGE_ROOT / "rviz" / "arena_map.rviz"


def test_map_server_parameters_are_static_map_only():
    parameters = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    map_parameters = parameters["map_server"]["ros__parameters"]
    lifecycle_parameters = parameters["lifecycle_manager_map"]["ros__parameters"]

    assert map_parameters == {
        "use_sim_time": False,
        "yaml_filename": "",
        "topic_name": "map",
        "frame_id": "map",
    }
    assert lifecycle_parameters["autostart"] is True
    assert lifecycle_parameters["node_names"] == ["map_server"]


def test_launch_uses_installed_defaults_and_has_no_localization_or_navigation():
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    node_keywords = [
        {keyword.arg: keyword.value for keyword in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == "Node"
    ]
    packages = {ast.literal_eval(keywords["package"]) for keywords in node_keywords}
    assert packages == {"nav2_map_server", "nav2_lifecycle_manager", "rviz2", "lb_navigation"}
    assert any(
        ast.literal_eval(keywords["executable"]) == "arena_scene_markers.py"
        for keywords in node_keywords
    )
    settings = [
        (key.value, value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, ast.Constant)
    ]
    assert any(
        key == "yaml_filename"
        and ast.dump(value) == ast.dump(ast.parse("str(map_yaml)", mode="eval").body)
        for key, value in settings
    )
    assert any(
        key == "node_names" and ast.literal_eval(value) == ["map_server"] for key, value in settings
    )
    assert "controller_server" not in source
    assert "planner_server" not in source
    assert "robot_localization" not in source
    assert "static_transform_publisher" not in source
    assert "map -> odom" not in source


def test_rviz_uses_map_frame_and_transient_local_map_qos():
    rviz = RVIZ.read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "Class: rviz_default_plugins/Map" in rviz
    assert "Value: /map" in rviz
    assert "Durability Policy: Transient Local" in rviz
    assert "Class: rviz_default_plugins/TopDownOrtho" in rviz
    assert "Value: /visualization/arena/markers" in rviz
    assert "Name: Guidebook Arena Scene (visual only)" in rviz
