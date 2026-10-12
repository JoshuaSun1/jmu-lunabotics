"""Offline checks for non-actuating, explicit bag replay."""

from pathlib import Path


LAUNCH = Path(__file__).parents[1] / 'launch' / 'bag_replay.launch.py'


def test_replay_is_explicit_clocked_and_non_actuating():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "DeclareLaunchArgument(\n                'bag_path'" in source
    assert "'--clock'" in source
    assert "'--loop'" in source
    assert "'use_sim_time': True" in source
    assert "executable='tag_replay_visualizer.py'" in source
    assert 'controller_manager' not in source
    assert 'webcam_apriltag.launch.py' not in source
    assert 'map_server' not in source
    assert 'nav2' not in source.lower()
