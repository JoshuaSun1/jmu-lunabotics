"""Offline contract checks for development-only map-frame pose replay."""

from pathlib import Path


LAUNCH = Path(__file__).parents[1] / 'launch' / 'arena_pose_replay.launch.py'


def test_replay_requires_explicit_bag_and_uses_simulated_time():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "'bag_path', description='Explicit path" in source
    assert "'--clock'" in source
    assert "'use_sim_time': True" in source
    assert "executable='tag_replay_robot_pose.py'" in source
    assert "'arena_scene.launch.py'" in source
    assert "'use_map_server', default_value='true'" in source


def test_assumptions_are_explicit_and_non_actuating():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "'tag_y_m', default_value='1.0'" in source
    assert "'tag_z_m', default_value='0.75'" in source
    assert "'tag_roll_rad', default_value=half_pi" in source
    assert "'camera_z_m', default_value='0.40'" in source
    assert "'zed_z_m', default_value='0.40'" in source
    assert "'zed_yaw_rad', default_value=f'-{half_pi}'" in source
    assert "'chassis_length_m', default_value='0.90'" in source
    assert "'chassis_width_m', default_value='0.60'" in source
    assert "'chassis_height_m', default_value='0.25'" in source
    assert 'mock chassis geometry' in source
    assert 'controller_manager' not in source
    assert 'cmd_vel' not in source
    assert 'robot_localization' not in source
    assert 'static_transform_publisher' not in source
