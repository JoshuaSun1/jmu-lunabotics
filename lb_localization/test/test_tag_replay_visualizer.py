"""Unit tests for replay path segmentation independent of a live ROS graph."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'tag_replay_visualizer.py'
SPEC = importlib.util.spec_from_file_location('tag_replay_visualizer', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(stamp):
    value = MODULE.PoseStamped()
    value.header.stamp.sec = stamp
    return value


def test_path_is_bounded_and_duplicate_transforms_are_ignored():
    state = MODULE.ReplayPathState(max_poses=2)
    assert state.accept(1, pose(1)) == 'REACQUIRED'
    assert state.accept(1, pose(1)) is None
    assert state.accept(2, pose(2)) == 'TRACKING'
    assert state.accept(3, pose(3)) == 'TRACKING'
    assert [item.header.stamp.sec for item in state.poses] == [2, 3]


def test_loss_clears_segment_and_reacquisition_starts_another():
    state = MODULE.ReplayPathState(max_poses=10)
    state.accept(1, pose(1))
    assert state.mark_lost()
    assert not state.poses
    assert not state.mark_lost()
    assert state.accept(2, pose(2)) == 'REACQUIRED'


def test_backward_clock_clears_path():
    state = MODULE.ReplayPathState(max_poses=10)
    assert not state.update_clock(100)
    state.accept(1, pose(1))
    assert state.update_clock(99)
    assert not state.poses
    assert state.state == MODULE.TrackingState.LOST


def test_invalid_bound_is_rejected():
    with pytest.raises(ValueError):
        MODULE.ReplayPathState(max_poses=0)
