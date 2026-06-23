import pytest

from tpsprojector.gl_context import gl_available

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


@pytest.mark.slow
def test_gl_bowl_is_realtime_at_720p():
    from tpsprojector.gl_benchmark import benchmark
    stats = benchmark(width=1280, height=720, mode="bowl", iters=30)
    assert stats["frames"] == 30
    # generous floor to catch gross regressions, not to pin exact hardware speed
    assert stats["fps"] > 30.0
