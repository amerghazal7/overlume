import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.transforms import Pose, rot_y


def make_cam(width=640, height=480, fov_deg=90.0, pose=None):
    return PinholeCamera.from_fov(width, height, fov_deg, pose or Pose.identity())


def test_principal_point_at_image_center():
    cam = make_cam()
    cx, cy = cam.K[0, 2], cam.K[1, 2]
    assert cx == 320.0
    assert cy == 240.0


def test_point_on_optical_axis_projects_to_center():
    cam = make_cam()
    P = np.array([[0.0, 0.0, 5.0]])  # straight ahead (+Z)
    uv, valid = cam.project(P)
    assert valid[0]
    np.testing.assert_allclose(uv[0], [320.0, 240.0], atol=1e-9)


def test_point_behind_camera_is_invalid():
    cam = make_cam()
    P = np.array([[0.0, 0.0, -5.0]])  # behind
    uv, valid = cam.project(P)
    assert not valid[0]


def test_90_deg_fov_edge_maps_to_image_edge():
    # With 90deg horizontal FOV, a point at 45deg from axis hits the horizontal edge.
    cam = make_cam(width=640, height=480, fov_deg=90.0)
    P = np.array([[5.0, 0.0, 5.0]])  # x == z -> 45 deg
    uv, valid = cam.project(P)
    assert valid[0]
    np.testing.assert_allclose(uv[0, 0], 640.0, atol=1e-6)


def test_project_unproject_round_trip_direction():
    cam = make_cam(pose=Pose(R=rot_y(0.4), t=np.array([1.0, -2.0, 3.0])))
    uv = np.array([[100.0, 150.0], [500.0, 300.0]])
    origins, dirs = cam.unproject(uv)
    # marching along the ray and re-projecting returns the same pixels
    P = origins + 7.5 * dirs
    uv2, valid = cam.project(P)
    assert valid.all()
    np.testing.assert_allclose(uv2, uv, atol=1e-6)


def test_unproject_origin_is_camera_center():
    pose = Pose(R=rot_y(0.4), t=np.array([1.0, -2.0, 3.0]))
    cam = make_cam(pose=pose)
    uv = np.array([[320.0, 240.0]])
    origins, dirs = cam.unproject(uv)
    np.testing.assert_allclose(origins[0], pose.t, atol=1e-9)
    # center pixel ray points along the camera's +Z axis in world frame
    np.testing.assert_allclose(dirs[0], pose.R @ np.array([0, 0, 1.0]), atol=1e-9)


def test_vfov_from_square_pixels():
    cam = make_cam(width=320, height=240, fov_deg=90.0)
    # f = 160; vfov = 2*atan(120/160) = 73.74 deg
    np.testing.assert_allclose(cam.vfov_deg(), 73.7398, atol=1e-2)


def test_backproject_center_pixel():
    cam = make_cam()
    pts = cam.backproject(np.array([[320.0, 240.0]]), np.array([5.0]))
    np.testing.assert_allclose(pts[0], [0.0, 0.0, 5.0], atol=1e-9)


def test_backproject_inverts_project():
    cam = make_cam(pose=Pose(R=rot_y(0.4), t=np.array([1.0, -2.0, 3.0])))
    uv = np.array([[100.0, 150.0], [500.0, 300.0]])
    depth = np.array([4.0, 9.0])  # camera-space z
    pts = cam.backproject(uv, depth)
    uv2, valid = cam.project(pts)
    assert valid.all()
    np.testing.assert_allclose(uv2, uv, atol=1e-6)


def test_in_bounds():
    cam = make_cam(width=640, height=480)
    uv = np.array([[0.0, 0.0], [639.0, 479.0], [-1.0, 5.0], [640.0, 5.0]])
    np.testing.assert_array_equal(cam.in_bounds(uv), [True, True, False, False])
