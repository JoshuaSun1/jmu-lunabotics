"""Deterministic synthetic scan geometry, independent of any physical sensor."""

import importlib.util
import math
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    "navigation_mock_world", Path(__file__).parents[1] / "scripts/navigation_mock_world.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(
    "angle,rectangle,expected",
    [
        (0, (2, -1, 3, 1), 2),
        (math.pi, (2, -1, 3, 1), math.inf),
        (math.pi / 2, (-1, 2, 1, 3), 2),
        (0, (2, 1, 3, 2), math.inf),
        (0, (9, -1, 10, 1), math.inf),
    ],
)
def test_raycast_marks_only_visible_in_range_rectangles(angle, rectangle, expected):
    assert MODULE.ray_rectangle(0, 0, angle, rectangle, 8) == pytest.approx(expected)
