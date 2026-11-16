"""Offline contract checks for development-only map-frame pose replay."""

import ast
from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "arena_pose_replay.launch.py"


def _launch_arguments(tree):
    return {
        ast.literal_eval(call.args[0]): {keyword.arg: keyword.value for keyword in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
    }


def test_replay_requires_explicit_bag_and_uses_simulated_time():
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    arguments = _launch_arguments(tree)
    assert "bag_path" in arguments
    assert "default_value" not in arguments["bag_path"]
    assert ast.literal_eval(arguments["bag_path"]["description"]).startswith("Explicit path")
    assert ast.literal_eval(arguments["use_map_server"]["default_value"]) == "true"
    strings = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert {"--clock", "arena_scene.launch.py"} <= strings
    assert any(
        isinstance(key, ast.Constant)
        and key.value == "use_sim_time"
        and isinstance(value, ast.Constant)
        and value.value is True
        for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
    )
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and any(
            keyword.arg == "executable"
            and ast.literal_eval(keyword.value) == "tag_replay_robot_pose.py"
            for keyword in call.keywords
        )
        for call in ast.walk(tree)
    )


def test_assumptions_are_explicit_and_non_actuating():
    source = LAUNCH.read_text(encoding="utf-8")
    arguments = _launch_arguments(ast.parse(source))
    for name, expected in {
        "tag_y_m": "1.0",
        "tag_z_m": "0.75",
        "camera_z_m": "0.40",
        "zed_z_m": "0.40",
        "chassis_length_m": "0.90",
        "chassis_width_m": "0.60",
        "chassis_height_m": "0.25",
    }.items():
        assert ast.literal_eval(arguments[name]["default_value"]) == expected
    for name, expression in {"tag_roll_rad": "half_pi", "zed_yaw_rad": "f'-{half_pi}'"}.items():
        assert ast.dump(arguments[name]["default_value"]) == ast.dump(
            ast.parse(expression, mode="eval").body
        )
    assert "mock chassis geometry" in source
    assert "controller_manager" not in source
    assert "cmd_vel" not in source
    assert "robot_localization" not in source
    assert "static_transform_publisher" not in source
