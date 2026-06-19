import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.transforms import Pose
from tpsprojector.world.rasterizer import rasterize


def make_cam():
    return PinholeCamera.from_fov(100, 100, 90.0, Pose.identity())


def test_triangle_fills_center_pixel():
    cam = make_cam()
    verts = np.array([[-2.0, -2.0, 5.0], [2.0, -2.0, 5.0], [0.0, 3.0, 5.0]])
    faces = np.array([[0, 1, 2]])
    colors = np.array([[1.0, 0.0, 0.0]])
    img, depth = rasterize(cam, verts, faces, colors, bg_color=(0.0, 0.0, 0.0))
    np.testing.assert_allclose(img[50, 50], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(depth[50, 50], 5.0, atol=1e-4)


def test_background_outside_triangle():
    cam = make_cam()
    verts = np.array([[-2.0, -2.0, 5.0], [2.0, -2.0, 5.0], [0.0, 3.0, 5.0]])
    faces = np.array([[0, 1, 2]])
    colors = np.array([[1.0, 0.0, 0.0]])
    img, depth = rasterize(cam, verts, faces, colors, bg_color=(0.1, 0.1, 0.1))
    np.testing.assert_allclose(img[0, 0], [0.1, 0.1, 0.1])
    assert np.isinf(depth[0, 0])


def test_zbuffer_nearer_triangle_wins():
    cam = make_cam()
    verts = np.array([
        [-3.0, -3.0, 5.0], [3.0, -3.0, 5.0], [0.0, 4.0, 5.0],   # far (red)
        [-3.0, -3.0, 3.0], [3.0, -3.0, 3.0], [0.0, 4.0, 3.0],   # near (blue)
    ])
    faces = np.array([[0, 1, 2], [3, 4, 5]])
    colors = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    img, depth = rasterize(cam, verts, faces, colors)
    np.testing.assert_allclose(img[50, 50], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(depth[50, 50], 3.0, atol=1e-4)


def test_triangle_behind_camera_skipped():
    cam = make_cam()
    verts = np.array([[-2.0, -2.0, -5.0], [2.0, -2.0, -5.0], [0.0, 3.0, -5.0]])
    faces = np.array([[0, 1, 2]])
    colors = np.array([[1.0, 0.0, 0.0]])
    img, depth = rasterize(cam, verts, faces, colors, bg_color=(0.0, 0.0, 0.0))
    assert np.all(np.isinf(depth))
