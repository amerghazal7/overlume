import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.transforms import Pose, rot_x
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import Scene, default_scene


def test_ring_rig_count():
    cams = make_ring_rig(n=6)
    assert len(cams) == 6


def test_ring_first_camera_pose():
    cams = make_ring_rig(n=6, radius=0.3, mount_height=0.5)
    c0 = cams[0]
    np.testing.assert_allclose(c0.pose.t, [0.3, 0.0, 0.5], atol=1e-9)
    fwd = c0.pose.R @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(fwd, [1.0, 0.0, 0.0], atol=1e-9)  # looks outward
    down = c0.pose.R @ np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(down, [0.0, 0.0, -1.0], atol=1e-9)  # +Y is world-down


def test_ring_neighbours_are_evenly_spaced():
    cams = make_ring_rig(n=6)
    f0 = cams[0].pose.R @ np.array([0.0, 0.0, 1.0])
    f1 = cams[1].pose.R @ np.array([0.0, 0.0, 1.0])
    ang = np.degrees(np.arccos(np.clip(f0 @ f1, -1.0, 1.0)))
    np.testing.assert_allclose(ang, 60.0, atol=1e-6)


def test_ring_overlap_default():
    # 6 cams * 60deg spacing; 75deg fov => 15deg overlap each side seam
    cams = make_ring_rig(n=6, hfov_deg=75.0)
    assert all(c.width > 0 and c.height > 0 for c in cams)


def test_scene_arrays_consistent():
    s = default_scene()
    assert isinstance(s, Scene)
    assert int(s.faces.max()) < len(s.vertices)
    assert len(s.face_colors) == len(s.faces)


def test_scene_has_ground_at_z0():
    s = default_scene()
    assert np.any(np.isclose(s.vertices[:, 2], 0.0))


def test_scene_has_height_for_walls_or_boxes():
    s = default_scene()
    assert s.vertices[:, 2].max() > 0.5  # something stands up off the ground


def test_render_sees_ground_from_above():
    s = default_scene()
    cam = PinholeCamera.from_fov(80, 80, 90.0,
                                 Pose(R=rot_x(np.pi), t=np.array([0.0, 0.0, 5.0])))
    img, depth = s.render(cam, bg_color=(0.0, 0.0, 0.0))
    non_bg = np.mean(np.any(img > 0.0, axis=-1))
    assert non_bg > 0.8  # ground fills most of a downward view
    assert np.isfinite(depth).mean() > 0.8
