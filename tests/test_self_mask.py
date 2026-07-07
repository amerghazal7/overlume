import os
import sys

import numpy as np
import pytest

# tpscuda is installed under cuda/install/libs/rendering_reprojector/python
_CUDA_PY = os.path.join(os.path.dirname(__file__), "..", "cuda", "install",
                        "libs", "rendering_reprojector", "python")
if os.path.isdir(_CUDA_PY):
    sys.path.insert(0, _CUDA_PY)


def cuda_available():
    try:
        import tpscuda  # noqa
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not cuda_available(), reason="tpscuda module not built")

W, H, FX = 128, 96, 60.0


def _down_cam(z=2.0):
    R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], "f4")
    K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]], "f4")
    return dict(K=K.ravel(), R=R.ravel(), t=np.array([0, 0, z], "f4"), width=W, height=H)


def _setup():
    """One down camera whose image is green except a red blob in the center —
    standing in for the robot body the camera sees. Fill enabled."""
    import tpscuda
    r = tpscuda.Reprojector(W, H)
    r.set_cameras([_down_cam()])
    img = np.zeros((1, H, W, 3), "f4")
    img[..., 1] = 1.0
    img[0, 38:58, 54:74, :] = [1.0, 0.0, 0.0]   # "robot body" pixels
    r.upload_images(img)
    return r


def test_self_mask_excludes_body_pixels_and_fill_covers():
    r = _setup()
    vc = _down_cam()
    # without masks: the red body pixels smear into the scene
    out = r.render_bowl(vc, 6.0, 0.08, 20.0, fill_blind_zone=True)
    assert out[48, 64, 0] > 0.9, "sanity: body pixels visible without mask"
    # mask exactly the red blob -> those source pixels are not-scene;
    # the fill must paint that region from the surrounding green
    mask = np.zeros((1, H, W), np.uint8)
    mask[0, 38:58, 54:74] = 255
    r.upload_self_masks(mask)
    out2 = r.render_bowl(vc, 6.0, 0.08, 20.0, fill_blind_zone=True)
    px = out2[48, 64]
    assert px[3] == 1.0, "masked region must be filled, not left invalid"
    assert px[1] > 0.5 and px[0] < 0.2, f"masked region must take surrounding color, got {px}"
    # clearing the masks restores the old behavior
    r.upload_self_masks(np.zeros((0, H, W), np.uint8))
    out3 = r.render_bowl(vc, 6.0, 0.08, 20.0, fill_blind_zone=True)
    assert out3[48, 64, 0] > 0.9


def test_self_mask_without_fill_leaves_invalid():
    r = _setup()
    vc = _down_cam()
    mask = np.zeros((1, H, W), np.uint8)
    mask[0, 38:58, 54:74] = 255
    r.upload_self_masks(mask)
    out = r.render_bowl(vc, 6.0, 0.08, 20.0)   # fill off (legacy contract)
    assert out[48, 64, 3] == 0.0, "masked+unfilled pixels are invalid (sky for the node)"
