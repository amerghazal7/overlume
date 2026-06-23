import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.app import Engine, RENDER_MODES
from tpsprojector.presets import get_preset, PRESET_NAMES

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


@pytest.mark.parametrize("mode", RENDER_MODES)
def test_gl_engine_matches_numpy_engine(mode):
    shot = get_preset(PRESET_NAMES[0])
    np_eng = Engine.from_defaults(width=96, height=72, mode=mode, backend="numpy")
    gl_eng = Engine.from_defaults(width=96, height=72, mode=mode, backend="gl")
    np_res = np_eng.synthesize(shot)
    gl_res = gl_eng.synthesize(shot)
    assert (gl_res.valid == np_res.valid).mean() > 0.90
    both = gl_res.valid & np_res.valid
    assert _psnr(gl_res.synth[both], np_res.synth[both]) > 28.0
