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


def _down_cam(z, fx, W=64, H=48):
    """Straight-down camera at (0,0,z): R columns = right/down/fwd (CV frame)."""
    R = np.array([[0, -1, 0],
                  [-1, 0, 0],
                  [0, 0, -1]], dtype="f4")
    K = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]], dtype="f4")
    return dict(K=K.ravel(), R=R.ravel(), t=np.array([0, 0, z], "f4"),
                width=W, height=H)


def _partial_coverage_setup():
    """One low camera covering only a central ground patch; vcam sees far wider.

    cam at z=2, fx=40 covers |x|<=1.6m of ground; the vcam at z=5 maps that to a
    ~26x19 px central patch of its 64x48 frame — everything outside is ground the
    bowl ray hits but no camera sees (the blind zone).
    """
    import tpscuda
    W, H = 64, 48
    r = tpscuda.Reprojector(W, H)
    cam = _down_cam(z=2.0, fx=40.0, W=W, H=H)
    r.set_cameras([cam])
    img = np.zeros((1, H, W, 3), "f4")
    img[..., 1] = 1.0  # solid green
    r.upload_images(img)
    return r, _down_cam(z=5.0, fx=40.0, W=W, H=H)


def test_blind_zone_filled_from_surroundings():
    r, vcam = _partial_coverage_setup()
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0, fill_blind_zone=True)
    # covered center stays green
    assert out[24, 32, 3] == 1.0 and out[24, 32, 1] > 0.9
    # blind-zone corner: bowl ray hits ground, no camera sees it -> filled from
    # the surrounding (green) scene instead of being left for the sky fill
    px = out[2, 2]
    assert px[3] == 1.0, "blind-zone pixel not filled"
    assert px[1] > 0.5 and px[0] < 0.2 and px[2] < 0.2, \
        f"blind-zone fill should propagate surrounding colors, got {px}"


def test_blind_zone_untouched_when_disabled():
    r, vcam = _partial_coverage_setup()
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0)  # default: no fill (legacy)
    assert out[24, 32, 3] == 1.0 and out[24, 32, 1] > 0.9
    assert out[2, 2, 3] == 0.0, "uncovered pixel must stay invalid with fill off"
    # legacy alpha contract: strictly 0 or 1
    assert np.all((out[..., 3] == 0.0) | (out[..., 3] == 1.0))


def test_blind_zone_fill_in_hybrid():
    r, vcam = _partial_coverage_setup()
    # one splat point so the depth pass has a non-empty cloud; everywhere else
    # hybrid falls back to the (filled) bowl layer
    depth = np.full((1, 48, 64), np.inf, "f4")
    depth[0, 24, 32] = 2.0
    r.upload_depth(depth)
    out = r.render_hybrid(vcam, 6.0, 0.08, 20.0, 1, fill_blind_zone=True)
    px = out[2, 2]
    assert px[3] == 1.0 and px[1] > 0.5, \
        f"hybrid must inherit the filled bowl layer, got {px}"
