import os
import sys

import numpy as np
import pytest

# tpscuda is installed under cuda/install/libs/rendering_reprojector/python
_CUDA_PY = os.path.join(os.path.dirname(__file__), "..", "cuda", "install",
                        "libs", "rendering_reprojector", "python")
if os.path.isdir(_CUDA_PY):
    sys.path.insert(0, _CUDA_PY)


def deps_available():
    try:
        import tpscuda  # noqa
        import cv2  # noqa
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not deps_available(), reason="tpscuda/cv2 not available")

W, H, FX = 256, 192, 200.0
DIST = np.array([-0.08, 0.14, -0.001, 0.002, 0.0], "f4")  # plumb_bob, real-lens scale
R_DOWN = np.array([[0, -1, 0],
                   [-1, 0, 0],
                   [0, 0, -1]], "f4")  # columns right/down/fwd: straight down
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]], "f4")
T = np.array([0, 0, 2.0], "f4")


def _setup(with_dist):
    import tpscuda
    cam = dict(K=K.ravel(), R=R_DOWN.ravel(), t=T, width=W, height=H)
    if with_dist:
        cam["dist"] = DIST
    # gradient image encodes source pixel coords: R=u/W, G=v/H
    img = np.zeros((1, H, W, 3), "f4")
    img[0, :, :, 0] = np.arange(W)[None, :] / W
    img[0, :, :, 1] = np.arange(H)[:, None] / H
    r = tpscuda.Reprojector(W, H)
    r.set_cameras([cam])
    r.upload_images(img)
    # pinhole virtual camera at the SAME pose: without distortion the sampled
    # gradient equals the vcam pixel itself; with distortion it must equal the
    # OpenCV-projected (distorted) source pixel.
    return r, dict(K=K.ravel(), R=R_DOWN.ravel(), t=T, width=W, height=H)


def _expected_uv(u, v):
    """OpenCV ground truth: vcam pixel -> ground point -> distorted source pixel."""
    import cv2
    xn, yn = (u - W / 2) / FX, (v - H / 2) / FX
    d_world = R_DOWN @ np.array([xn, yn, 1.0])   # dir = right*xn + down*yn + fwd
    tt = (0.0 - T[2]) / d_world[2]
    Pw = T + tt * d_world
    Pc = R_DOWN.T @ (Pw - T)                      # world -> camera frame
    uv, _ = cv2.projectPoints(Pc.reshape(1, 1, 3).astype("f8"),
                              np.zeros(3), np.zeros(3),
                              K.astype("f8"), DIST.astype("f8"))
    return uv.ravel()


def test_distorted_sampling_matches_opencv():
    r, vc = _setup(with_dist=True)
    out = r.render_bowl(vc, 6.0, 0.08, 20.0)
    for (u, v) in [(200, 40), (60, 150), (128, 96), (30, 30)]:
        ue, ve = _expected_uv(u, v)
        px = out[v, u]
        assert px[3] == 1.0, f"({u},{v}) not covered"
        assert abs(px[0] - ue / W) < 0.012 and abs(px[1] - ve / H) < 0.012, \
            f"({u},{v}): sampled {px[:2]}, opencv expects ({ue/W:.3f},{ve/H:.3f})"


def test_zero_dist_is_identity_pinhole():
    r, vc = _setup(with_dist=False)
    out = r.render_bowl(vc, 6.0, 0.08, 20.0)
    for (u, v) in [(200, 40), (60, 150)]:
        px = out[v, u]
        # same pose, no distortion: gradient sample == the pixel itself
        assert px[3] == 1.0
        assert abs(px[0] - u / W) < 0.01 and abs(px[1] - v / H) < 0.01
