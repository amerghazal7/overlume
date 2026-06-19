import numpy as np

from tpsprojector.presets import PRESET_NAMES, Shot, get_preset, tween


def test_expected_presets_exist():
    for name in ["behind", "top_down", "three_quarter_left", "three_quarter_right"]:
        assert name in PRESET_NAMES


def test_get_preset_returns_shot_with_pose():
    shot = get_preset("behind")
    assert isinstance(shot, Shot)
    pose = shot.pose()
    np.testing.assert_allclose(pose.t, shot.eye)


def test_tween_endpoints():
    a = get_preset("behind")
    b = get_preset("top_down")
    np.testing.assert_allclose(tween(a, b, 0.0).eye, a.eye)
    np.testing.assert_allclose(tween(a, b, 0.0).target, a.target)
    np.testing.assert_allclose(tween(a, b, 1.0).eye, b.eye)
    np.testing.assert_allclose(tween(a, b, 1.0).target, b.target)


def test_tween_midpoint_is_average_with_smoothstep():
    a = Shot(eye=[0.0, 0.0, 0.0], target=[1.0, 0.0, 0.0])
    b = Shot(eye=[4.0, 0.0, 0.0], target=[1.0, 4.0, 0.0])
    mid = tween(a, b, 0.5)  # smoothstep(0.5) == 0.5
    np.testing.assert_allclose(mid.eye, [2.0, 0.0, 0.0])
    np.testing.assert_allclose(mid.target, [1.0, 2.0, 0.0])


def test_tween_is_eased_not_linear():
    a = Shot(eye=[0.0, 0.0, 0.0], target=[0.0, 0.0, 0.0])
    b = Shot(eye=[1.0, 0.0, 0.0], target=[0.0, 0.0, 0.0])
    # smoothstep(0.25)=0.15625 < linear 0.25
    np.testing.assert_allclose(tween(a, b, 0.25).eye[0], 0.15625, atol=1e-6)


def test_tween_clamps_out_of_range():
    a = get_preset("behind")
    b = get_preset("top_down")
    np.testing.assert_allclose(tween(a, b, -1.0).eye, a.eye)
    np.testing.assert_allclose(tween(a, b, 2.0).eye, b.eye)
