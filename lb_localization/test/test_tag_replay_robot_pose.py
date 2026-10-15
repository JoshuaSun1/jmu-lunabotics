"""Unit tests for provisional replay pose math and segmentation."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'tag_replay_robot_pose.py'
SPEC = importlib.util.spec_from_file_location('tag_replay_robot_pose', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def assert_vector_close(actual, expected, tolerance=1.0e-9):
    assert actual == pytest.approx(expected, abs=tolerance)


def pose(stamp):
    value = MODULE.PoseStamped()
    value.header.stamp.sec = stamp
    return value


def test_recovers_camera_and_base_pose_from_known_tag():
    tag_orientation = MODULE.quaternion_from_rpy(
        math.pi / 2.0, 0.0, math.pi / 2.0
    )
    camera_orientation = MODULE.quaternion_from_rpy(
        -math.pi / 2.0, 0.0, math.pi / 2.0
    )
    map_to_tag = MODULE.RigidTransform((0.0, 1.0, 0.75), tag_orientation)
    base_to_camera = MODULE.RigidTransform(
        (0.0, 0.0, 0.4), camera_orientation
    )
    desired_map_to_base = MODULE.RigidTransform(
        (1.25, 2.0, 0.0), MODULE.quaternion_from_rpy(0.0, 0.0, 0.0)
    )
    desired_map_to_camera = MODULE.compose(desired_map_to_base, base_to_camera)
    camera_to_tag = MODULE.compose(MODULE.inverse(desired_map_to_camera), map_to_tag)

    recovered_camera = MODULE.compose(map_to_tag, MODULE.inverse(camera_to_tag))
    recovered_base = MODULE.compose(recovered_camera, MODULE.inverse(base_to_camera))

    assert_vector_close(recovered_camera.translation, (1.25, 2.0, 0.4))
    assert_vector_close(recovered_base.translation, desired_map_to_base.translation)
    assert MODULE.yaw_of(recovered_base.rotation) == pytest.approx(0.0, abs=1.0e-9)


def test_full_six_degree_composition_and_planar_projection():
    tag = MODULE.RigidTransform(
        (0.0, 1.0, 0.75),
        MODULE.quaternion_from_rpy(math.pi / 2.0, 0.0, math.pi / 2.0),
    )
    base_to_camera = MODULE.RigidTransform(
        (0.0, 0.0, 0.4),
        MODULE.quaternion_from_rpy(-math.pi / 2.0, 0.0, math.pi / 2.0),
    )
    expected_base = MODULE.RigidTransform(
        (2.2, 1.6, 0.12), MODULE.quaternion_from_rpy(0.18, -0.11, 0.63)
    )
    camera = MODULE.compose(expected_base, base_to_camera)
    observation = MODULE.compose(MODULE.inverse(camera), tag)

    recovered_camera = MODULE.compose(tag, MODULE.inverse(observation))
    recovered_base = MODULE.compose(recovered_camera, MODULE.inverse(base_to_camera))
    planar = MODULE.planarized(recovered_base)

    assert_vector_close(recovered_base.translation, expected_base.translation)
    assert MODULE.yaw_of(recovered_base.rotation) == pytest.approx(0.63, abs=1.0e-9)
    assert_vector_close(planar.translation, (2.2, 1.6, 0.0))
    assert MODULE.yaw_of(planar.rotation) == pytest.approx(0.63, abs=1.0e-9)


def test_default_rotations_match_rear_optical_and_detector_tag_axes():
    camera_rotation = MODULE.quaternion_from_rpy(
        -math.pi / 2.0, 0.0, math.pi / 2.0
    )
    assert_vector_close(
        MODULE.rotate_vector(camera_rotation, (1.0, 0.0, 0.0)),
        (0.0, 1.0, 0.0),
    )
    assert_vector_close(
        MODULE.rotate_vector(camera_rotation, (0.0, 1.0, 0.0)),
        (0.0, 0.0, -1.0),
    )
    assert_vector_close(
        MODULE.rotate_vector(camera_rotation, (0.0, 0.0, 1.0)),
        (-1.0, 0.0, 0.0),
    )

    tag_rotation = MODULE.quaternion_from_rpy(
        math.pi / 2.0, 0.0, math.pi / 2.0
    )
    assert_vector_close(
        MODULE.rotate_vector(tag_rotation, (1.0, 0.0, 0.0)),
        (0.0, 1.0, 0.0),
    )
    assert_vector_close(
        MODULE.rotate_vector(tag_rotation, (0.0, 1.0, 0.0)),
        (0.0, 0.0, 1.0),
    )
    assert_vector_close(
        MODULE.rotate_vector(tag_rotation, (0.0, 0.0, 1.0)),
        (1.0, 0.0, 0.0),
    )

    zed_rotation = MODULE.quaternion_from_rpy(
        -math.pi / 2.0, 0.0, -math.pi / 2.0
    )
    assert_vector_close(
        MODULE.rotate_vector(zed_rotation, (0.0, 0.0, 1.0)),
        (1.0, 0.0, 0.0),
    )


def test_rectangular_footprint_is_closed_and_tracks_planar_pose():
    transform = MODULE.RigidTransform(
        (2.0, 3.0, 0.0), MODULE.quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
    )
    corners = MODULE.rectangle_corners(transform, 0.90, 0.60)

    assert len(corners) == 5
    assert_vector_close(corners[0], (1.70, 3.45, 0.01))
    assert_vector_close(corners[-1], corners[0])

    with pytest.raises(ValueError):
        MODULE.rectangle_corners(transform, 0.0, 0.60)


def test_path_is_bounded_and_restarts_after_loss_or_clock_reset():
    state = MODULE.ReplayPoseState(max_poses=2)
    assert state.accept(1, pose(1)) == 'REACQUIRED'
    assert state.accept(1, pose(1)) is None
    assert state.accept(2, pose(2)) == 'TRACKING'
    assert state.accept(3, pose(3)) == 'TRACKING'
    assert [item.header.stamp.sec for item in state.poses] == [2, 3]

    assert state.mark_lost()
    assert not state.poses
    assert state.accept(4, pose(4)) == 'REACQUIRED'
    assert not state.update_clock(100)
    assert state.update_clock(99)
    assert not state.poses


def test_invalid_quaternion_and_path_bound_are_rejected():
    with pytest.raises(ValueError):
        MODULE.normalize_quaternion((0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        MODULE.ReplayPoseState(max_poses=0)
