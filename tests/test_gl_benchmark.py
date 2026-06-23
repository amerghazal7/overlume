import pytest

from tpsprojector.gl_context import gl_available

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


@pytest.mark.slow
def test_gl_bowl_no_readback_is_realtime_at_720p():
    from tpsprojector.gl_benchmark import benchmark
    stats = benchmark(width=1280, height=720, mode="bowl", iters=30)
    assert stats["frames"] == 30
    assert "fps_readback" in stats
    assert stats["fps"] > 60.0          # no-readback GPU render clears the spec target
