"""Offline contracts for the lifecycle-managed static arena map launch."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
PARAMS = PACKAGE_ROOT / 'config' / 'map_server.yaml'
LAUNCH = PACKAGE_ROOT / 'launch' / 'map.launch.py'
RVIZ = PACKAGE_ROOT / 'rviz' / 'arena_map.rviz'


def test_map_server_parameters_are_static_map_only():
    parameters = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    map_parameters = parameters['map_server']['ros__parameters']
    lifecycle_parameters = parameters['lifecycle_manager_map']['ros__parameters']

    assert map_parameters == {
        'use_sim_time': False,
        'yaml_filename': '',
        'topic_name': 'map',
        'frame_id': 'map',
    }
    assert lifecycle_parameters['autostart'] is True
    assert lifecycle_parameters['node_names'] == ['map_server']


def test_launch_uses_installed_defaults_and_has_no_localization_or_navigation():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "package='nav2_map_server'" in source
    assert "package='nav2_lifecycle_manager'" in source
    assert "'yaml_filename': str(map_yaml)" in source
    assert "'node_names': ['map_server']" in source
    assert "package='rviz2'" in source
    assert "executable='arena_scene_markers.py'" in source
    assert 'controller_server' not in source
    assert 'planner_server' not in source
    assert 'robot_localization' not in source
    assert 'static_transform_publisher' not in source
    assert 'map -> odom' not in source


def test_rviz_uses_map_frame_and_transient_local_map_qos():
    rviz = RVIZ.read_text(encoding='utf-8')
    assert 'Fixed Frame: map' in rviz
    assert 'Class: rviz_default_plugins/Map' in rviz
    assert 'Value: /map' in rviz
    assert 'Durability Policy: Transient Local' in rviz
    assert 'Class: rviz_default_plugins/TopDownOrtho' in rviz
    assert 'Value: /visualization/arena/markers' in rviz
    assert 'Name: Guidebook Arena Scene (visual only)' in rviz
