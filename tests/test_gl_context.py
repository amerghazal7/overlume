import numpy as np
import pytest

from tpsprojector.gl_context import gl_available, get_context, get_fbo

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def test_context_is_singleton():
    assert get_context() is get_context()


def test_fbo_has_requested_size_and_is_cached():
    fbo = get_fbo(64, 48)
    assert fbo.size == (64, 48)
    assert get_fbo(64, 48) is fbo  # cached by size


def test_fbo_clear_and_read_roundtrips_rgba32f():
    fbo = get_fbo(8, 8)
    fbo.use()
    fbo.clear(0.25, 0.5, 0.75, 1.0)
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(8, 8, 4)
    assert np.allclose(raw[..., :3], [0.25, 0.5, 0.75], atol=1e-3)


def test_depth_fbo_has_depth_attachment():
    fbo = get_fbo(16, 16, depth=True)
    assert fbo.depth_attachment is not None
