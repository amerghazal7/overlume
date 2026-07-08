"""upload_points: GPU colorization of explicit (lidar) points + hybrid splat."""
import sys

import numpy as np
import pytest

sys.path.insert(0, "cuda/install/libs/rendering_reprojector/python")
tpscuda = pytest.importorskip("tpscuda")


def _cam(t, color_img_size=64):
    # Camera at t looking along +x (rig frame), up +z:
    # optical columns right=(0,-1,0), down=(0,0,-1), fwd=(1,0,0).
    R = np.array([[0, 0, 1],
                  [-1, 0, 0],
                  [0, -1, 0]], dtype=np.float32)
    s = color_img_size
    K = np.array([[s / 2, 0, s / 2], [0, s / 2, s / 2], [0, 0, 1]], dtype=np.float32)
    return {"K": K, "R": R, "t": np.array(t, dtype=np.float32), "width": s, "height": s}


def _setup(img_rgb=(1.0, 0.0, 0.0)):
    r = tpscuda.Reprojector(64, 64)
    cam = _cam([0, 0, 1])
    r.set_cameras([cam])
    img = np.zeros((1, 64, 64, 3), dtype=np.float32)
    img[..., 0], img[..., 1], img[..., 2] = img_rgb
    r.upload_images(img)
    return r, cam


def test_point_in_view_gets_camera_color_and_splats():
    r, cam = _setup(img_rgb=(1.0, 0.0, 0.0))
    # Point 2 m ahead of the camera -> projects near image center, colorized red.
    r.upload_points(np.array([[2.0, 0.0, 1.0]], dtype=np.float32))
    out = r.render_depth(cam, 1)
    ys, xs = np.where(out[..., 3] == 1.0)
    assert len(ys) > 0, "point did not splat"
    assert abs(ys.mean() - 32) < 3 and abs(xs.mean() - 32) < 3
    px = out[ys[0], xs[0]]
    assert px[0] > 0.9 and px[1] < 0.1 and px[2] < 0.1


def test_point_behind_all_cameras_is_skipped():
    r, cam = _setup()
    r.upload_points(np.array([[-2.0, 0.0, 1.0]], dtype=np.float32))
    out = r.render_depth(cam, 1)
    assert not np.any(out[..., 3] == 1.0), "uncovered point must not splat"


def test_hybrid_point_wins_over_bowl():
    r, cam = _setup(img_rgb=(0.0, 1.0, 0.0))
    r.upload_points(np.array([[2.0, 0.0, 1.0]], dtype=np.float32))
    out = r.render_hybrid(cam, R0=6.0, k=0.08, Rmax=20.0, radius=1)
    # Center pixel: splatted point (green, from the camera image).
    px = out[32, 32]
    assert px[3] == 1.0 and px[1] > 0.9
