"""exposure_match: per-camera GPU gain matching converges the sector brightness."""
import sys

import numpy as np
import pytest

sys.path.insert(0, "cuda/install/libs/rendering_reprojector/python")
tpscuda = pytest.importorskip("tpscuda")


def _down_cam(ty, s=64):
    # Camera at (0, ty, 2) looking straight down (fwd=-z, right=-y, down=-x):
    # 90 deg fov -> floor footprint y in [ty-2, ty+2].
    R = np.stack([np.array([0, -1, 0], np.float32), np.array([-1, 0, 0], np.float32),
                  np.array([0, 0, -1], np.float32)], axis=1)
    K = np.array([[s / 2, 0, s / 2], [0, s / 2, s / 2], [0, 0, 1]], dtype=np.float32)
    return {"K": K, "R": R, "t": np.array([0, ty, 2], np.float32), "width": s, "height": s}


def _ratio(exposure_match):
    r = tpscuda.Reprojector(64, 64)
    cam_a, cam_b = _down_cam(1.0), _down_cam(-1.0)  # overlap band y in [-1, 1]
    r.set_cameras([cam_a, cam_b])
    imgs = np.zeros((2, 64, 64, 3), np.float32)
    imgs[0] = 0.8  # bright camera
    imgs[1] = 0.4  # dark camera
    r.upload_images(imgs)
    vcam = _down_cam(0.0)
    vcam["t"] = np.array([0, 0, 4], np.float32)
    out = None
    for _ in range(12):  # gains apply next frame; EMA needs a few frames
        out = r.render_bowl(vcam, R0=10.0, k=0.05, Rmax=20.0,
                            exposure_match=exposure_match)
    # pixel (16,32) = world y=+2 (cam A only); (48,32) = y=-2 (cam B only)
    a = out[32, 16, :3].mean()
    b = out[32, 48, :3].mean()
    assert out[32, 16, 3] == 1.0 and out[32, 48, 3] == 1.0
    return a / b


def test_gains_converge_sector_brightness():
    off = _ratio(False)
    on = _ratio(True)
    assert off == pytest.approx(2.0, abs=0.05), "without matching ratio stays 2:1"
    assert on < 1.25, f"matched ratio should approach 1, got {on:.2f}"


def test_default_off_is_bit_identical():
    r = tpscuda.Reprojector(64, 64)
    r.set_cameras([_down_cam(1.0), _down_cam(-1.0)])
    imgs = np.full((2, 64, 64, 3), 0.6, np.float32)
    r.upload_images(imgs)
    vcam = _down_cam(0.0); vcam["t"] = np.array([0, 0, 4], np.float32)
    a = r.render_bowl(vcam, R0=10.0, k=0.05, Rmax=20.0)
    b = r.render_bowl(vcam, R0=10.0, k=0.05, Rmax=20.0, exposure_match=False)
    assert np.array_equal(a, b)
