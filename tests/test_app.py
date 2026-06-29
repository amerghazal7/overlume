import os
import sys

import numpy as np
import pytest

# Engine's reprojection core is the C++ CUDA library (tpscuda); skip if unbuilt.
_CUDA_PY = os.path.join(os.path.dirname(__file__), "..", "cuda", "install",
                        "libs", "rendering_reprojector", "python")
if os.path.isdir(_CUDA_PY):
    sys.path.insert(0, _CUDA_PY)


def _cuda_available():
    try:
        import tpscuda  # noqa: F401
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _cuda_available(), reason="tpscuda module not built")

from tpsprojector.app import Engine, RenderResult  # noqa: E402
from tpsprojector.presets import get_preset  # noqa: E402


def test_engine_render_modes_all_produce_valid_frames():
    eng = Engine.from_defaults(width=120, height=120)
    for mode in ("bowl", "depth", "hybrid"):
        eng.mode = mode
        res = eng.synthesize(get_preset("behind"))
        assert res.synth.shape == (120, 120, 3)
        assert res.valid.any()


def test_hybrid_never_loses_coverage_to_depth():
    # By construction hybrid = depth where valid, else bowl, so it can only add
    # coverage. (How much it adds depends on the mounting/scene.)
    eng = Engine.from_defaults(width=160, height=120)
    shot = get_preset("behind")
    eng.mode = "depth"
    depth_cover = eng.synthesize(shot).valid.mean()
    eng.mode = "hybrid"
    hybrid_cover = eng.synthesize(shot).valid.mean()
    assert hybrid_cover >= depth_cover
    assert hybrid_cover > 0.75


def test_modes_produce_distinct_results():
    eng = Engine.from_defaults(width=160, height=120)
    shot = get_preset("behind")
    frames = {}
    for mode in ("bowl", "depth", "hybrid"):
        eng.mode = mode
        frames[mode] = eng.synthesize(shot).synth
    assert not np.allclose(frames["bowl"], frames["depth"])
    assert not np.allclose(frames["bowl"], frames["hybrid"])


def test_engine_synthesize_returns_aligned_buffers():
    eng = Engine.from_defaults(width=120, height=120)
    res = eng.synthesize(get_preset("top_down"))
    assert isinstance(res, RenderResult)
    assert res.synth.shape == (120, 120, 3)
    assert res.truth.shape == (120, 120, 3)
    assert res.diff.shape == (120, 120, 3)
    assert 0.0 <= res.synth.min() and res.synth.max() <= 1.0


def test_engine_reports_reasonable_metrics():
    eng = Engine.from_defaults(width=120, height=120)
    res = eng.synthesize(get_preset("top_down"))
    assert 5.0 < res.psnr < 99.0
    assert 0.0 < res.ssim <= 1.0


def test_engine_composites_robot_into_view():
    # A behind shot must show robot-proxy pixels that are absent from the
    # environment-only synthesis.
    eng = Engine.from_defaults(width=120, height=120)
    shot = get_preset("behind")
    res = eng.synthesize(shot)
    assert res.robot_mask.any()
    # robot pixels in the composited frame match the rendered robot, not env
    rgb, depth = eng.robot.render(eng.virtual_camera(shot))
    mask = np.isfinite(depth)
    np.testing.assert_allclose(res.synth[mask], rgb[mask])
