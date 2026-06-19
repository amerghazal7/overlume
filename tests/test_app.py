import numpy as np

from tpsprojector.app import Engine, RenderResult
from tpsprojector.presets import get_preset


def test_engine_render_modes_all_produce_valid_frames():
    eng = Engine.from_defaults(width=120, height=120)
    for mode in ("bowl", "depth", "hybrid"):
        eng.mode = mode
        res = eng.synthesize(get_preset("behind"))
        assert res.synth.shape == (120, 120, 3)
        assert res.valid.any()


def test_hybrid_fills_depth_disocclusion_holes():
    # Hybrid (depth + bowl fallback) must cover at least as much as pure depth.
    eng = Engine.from_defaults(width=160, height=120)
    shot = get_preset("behind")
    eng.mode = "depth"
    depth_cover = eng.synthesize(shot).valid.mean()
    eng.mode = "hybrid"
    hybrid_cover = eng.synthesize(shot).valid.mean()
    # hybrid fills depth's disocclusion holes with bowl fallback (remaining gap
    # is sky above the horizon, which has no geometry in either renderer)
    assert hybrid_cover >= depth_cover + 0.1
    assert hybrid_cover > 0.8


def test_depth_mode_beats_bowl_against_truth_on_oblique():
    eng = Engine.from_defaults(width=160, height=120)
    shot = get_preset("behind")
    eng.mode = "bowl"
    bowl_psnr = eng.synthesize(shot).psnr
    eng.mode = "depth"
    depth_psnr = eng.synthesize(shot).psnr
    assert depth_psnr > bowl_psnr + 3.0


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
