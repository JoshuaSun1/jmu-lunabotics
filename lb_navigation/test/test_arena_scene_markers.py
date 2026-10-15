"""Unit and configuration tests for static arena scene markers."""

import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT = PACKAGE_ROOT / 'scripts' / 'arena_scene_markers.py'
CONFIG = PACKAGE_ROOT / 'config' / 'arena_scene.yaml'
SPEC = importlib.util.spec_from_file_location('arena_scene_markers', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_guidebook_geometry_and_provisional_starting_depth_are_explicit():
    parameters = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'arena_scene_markers'
    ]['ros__parameters']

    assert parameters['arena_length_m'] == 6.88
    assert parameters['republish_period_s'] == 1.0
    assert parameters['arena_width_m'] == 5.0
    assert parameters['excavation_boundary_x_m'] == 2.5
    assert parameters['starting_zone_length_m'] == 2.0
    assert parameters['starting_zone_depth_m'] == 2.0
    assert parameters['berm_center_x_m'] == 5.38
    assert parameters['berm_center_y_m'] == 0.9
    assert parameters['berm_length_m'] == 2.2
    assert parameters['berm_width_m'] == 0.9
    assert parameters['tag_ids'] == [1]


def test_geometry_validation_rejects_out_of_arena_values():
    geometry = MODULE.ArenaGeometry(6.88, 5.0, 2.5, 2.0, 2.0, 5.38, 0.9, 2.2, 0.9)
    geometry.validate()

    with pytest.raises(ValueError):
        MODULE.ArenaGeometry(
            6.88, 5.0, 7.0, 2.0, 2.0, 5.38, 0.9, 2.2, 0.9
        ).validate()


def test_rectangles_are_closed_and_dashed_boundaries_keep_known_extents():
    outline = MODULE.rectangle_points(0.0, 0.0, 6.88, 5.0, 0.02)
    assert len(outline) == 5
    assert (outline[0].x, outline[0].y) == (outline[-1].x, outline[-1].y)
    assert max(point.x for point in outline) == 6.88
    assert max(point.y for point in outline) == 5.0

    dashed = MODULE.dashed_rectangle_points(0.0, 0.0, 2.0, 2.0, 0.03)
    assert len(dashed) % 2 == 0
    assert min(point.x for point in dashed) == 0.0
    assert max(point.x for point in dashed) == 2.0
    assert min(point.y for point in dashed) == 0.0
    assert max(point.y for point in dashed) == 2.0


def test_assumed_tag_normal_points_toward_positive_arena_x():
    rotation = MODULE.quaternion_from_rpy(
        1.5707963267948966, 0.0, 1.5707963267948966
    )
    normal = MODULE.rotate_vector(rotation, (0.0, 0.0, 1.0))
    assert normal == pytest.approx((1.0, 0.0, 0.0), abs=1.0e-9)


def test_scene_is_durable_visualization_only_and_omits_column():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert "'/visualization/arena/markers'" in source
    assert 'TransformBroadcaster' not in source
    assert 'static_transform_publisher' not in source
    assert 'column' not in source.lower()
