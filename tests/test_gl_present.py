import numpy as np
import pytest

from tpsprojector.gl_context import gl_available, get_fbo, get_context

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _read(fbo, W, H):
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(H, W, 4)
    return raw


def test_present_array_roundtrips_upright():
    from tpsprojector.gl_present import present_array
    W, H = 16, 12
    # distinct per-row values so an accidental flip is caught
    arr = np.zeros((H, W, 3), dtype="f4")
    arr[:, :, 0] = (np.arange(H)[:, None] / H)      # red ramps top->bottom
    arr[:, :, 1] = 0.5
    target = get_fbo(W, H)
    target.use(); target.clear(0, 0, 0, 1)
    present_array(arr, target=target)
    out = np.flipud(_read(target, W, H)).copy()[..., :3]   # flipud -> row 0 = top
    assert np.allclose(out, arr, atol=1e-3)


def test_present_fbo_copies_1to1():
    from tpsprojector.gl_present import present_array, present_fbo
    W, H = 16, 12
    src = get_fbo(W, H)
    src.use(); src.clear(0, 0, 0, 1)
    arr = np.zeros((H, W, 3), dtype="f4"); arr[:, :, 2] = 0.7
    present_array(arr, target=src)                  # put a known image into src
    src_raw = _read(src, W, H).copy()

    # NOTE: get_fbo caches by (W, H) so we create dst directly to avoid
    # the framebuffer feedback loop that results when src is dst.
    ctx = get_context()
    color = ctx.texture((W, H), 4, dtype="f4")
    dst = ctx.framebuffer(color_attachments=[color])
    dst.use(); dst.clear(0, 0, 0, 1)
    present_fbo(src, target=dst)
    assert np.allclose(_read(dst, W, H), src_raw, atol=1e-3)


def test_present_array_alpha_blends_over_target():
    from tpsprojector.gl_present import present_array
    W, H = 8, 8
    target = get_fbo(W, H)
    target.use(); target.clear(0.0, 0.0, 0.0, 1.0)  # black background
    rgba = np.zeros((H, W, 4), dtype="f4")
    rgba[:, :, 0] = 1.0                              # red
    rgba[:, :, 3] = 0.5                              # half alpha
    present_array(rgba, target=target, blend=True)
    out = _read(target, W, H)
    assert np.allclose(out[..., 0], 0.5, atol=2e-2)  # 0.5*red over black
