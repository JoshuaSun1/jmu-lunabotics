"""Offline checks for non-actuating, explicit bag replay."""

import ast
from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "bag_replay.launch.py"


def test_replay_is_explicit_clocked_and_non_actuating():
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    bag_arguments = [
        call
        for call in calls
        if isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and ast.literal_eval(call.args[0]) == "bag_path"
    ]
    assert len(bag_arguments) == 1
    assert not any(keyword.arg == "default_value" for keyword in bag_arguments[0].keywords)
    strings = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert {"--clock", "--loop"} <= strings
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
        isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and any(
            keyword.arg == "executable"
            and ast.literal_eval(keyword.value) == "tag_replay_visualizer.py"
            for keyword in call.keywords
        )
        for call in calls
    )
    assert "controller_manager" not in source
    assert "webcam_apriltag.launch.py" not in source
    assert "map_server" not in source
    assert "nav2" not in source.lower()
