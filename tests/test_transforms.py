import numpy as np
import pytest

from tpsprojector.transforms import Pose, look_at, rot_x, rot_y, rot_z


def test_look_at_origin_is_eye():
    pose = look_at(eye=[-5.0, 0.0, 3.0], target=[0.0, 0.0, 0.0])
    np.testing.assert_allclose(pose.t, [-5.0, 0.0, 3.0])


def test_look_at_forward_points_to_target():
    pose = look_at(eye=[-5.0, 0.0, 0.0], target=[0.0, 0.0, 0.0])
    fwd = pose.R @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(fwd, [1.0, 0.0, 0.0], atol=1e-9)


def test_look_at_straight_down_is_well_defined():
    pose = look_at(eye=[0.0, 0.0, 5.0], target=[0.0, 0.0, 0.0])
    fwd = pose.R @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(fwd, [0.0, 0.0, -1.0], atol=1e-9)
    # rotation must stay orthonormal even in the degenerate look-down case
    np.testing.assert_allclose(pose.R @ pose.R.T, np.eye(3), atol=1e-9)


def test_look_at_image_down_has_downward_component():
    pose = look_at(eye=[-5.0, 0.0, 3.0], target=[0.0, 0.0, 0.0])
    down = pose.R @ np.array([0.0, 1.0, 0.0])
    assert down[2] < 0  # +Y in the image points generally downward in the world


def test_identity_transforms_points_unchanged():
    pose = Pose.identity()
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    out = pose.transform_points(pts)
    np.testing.assert_allclose(out, pts)


def test_translation_only():
    pose = Pose(R=np.eye(3), t=np.array([1.0, 0.0, -2.0]))
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
    out = pose.transform_points(pts)
    np.testing.assert_allclose(out, [[1.0, 0.0, -2.0], [2.0, 1.0, -1.0]])


def test_rot_z_90_degrees():
    # rotating +X by +90deg about Z gives +Y
    R = rot_z(np.pi / 2)
    v = np.array([[1.0, 0.0, 0.0]])
    out = (R @ v.T).T
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_inverse_round_trips():
    pose = Pose(R=rot_x(0.3) @ rot_y(-0.7), t=np.array([2.0, -1.0, 5.0]))
    pts = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 2.0], [0.0, 0.0, 0.0]])
    back = pose.inverse().transform_points(pose.transform_points(pts))
    np.testing.assert_allclose(back, pts, atol=1e-9)


def test_compose_matches_sequential_application():
    a = Pose(R=rot_z(0.5), t=np.array([1.0, 0.0, 0.0]))
    b = Pose(R=rot_y(0.2), t=np.array([0.0, 2.0, 0.0]))
    pts = np.array([[1.0, 1.0, 1.0], [3.0, -2.0, 0.5]])
    composed = a.compose(b).transform_points(pts)
    sequential = a.transform_points(b.transform_points(pts))
    np.testing.assert_allclose(composed, sequential, atol=1e-9)


def test_matrix_is_4x4_homogeneous():
    pose = Pose(R=rot_x(0.1), t=np.array([1.0, 2.0, 3.0]))
    M = pose.matrix()
    assert M.shape == (4, 4)
    np.testing.assert_allclose(M[3], [0.0, 0.0, 0.0, 1.0])
