import numpy as np

from tpsprojector.app import Engine, RenderResult
from tpsprojector.presets import get_preset


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
