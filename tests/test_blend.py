import numpy as np

from tpsprojector.blend import blend, border_feather


def test_single_candidate_passes_through():
    colors = np.array([[[0.2, 0.4, 0.6]]])  # (1 pixel, 1 cand, rgb)
    weights = np.array([[1.0]])
    out, valid = blend(colors, weights)
    assert valid[0]
    np.testing.assert_allclose(out[0], [0.2, 0.4, 0.6])


def test_equal_weights_average():
    colors = np.array([[[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]])
    weights = np.array([[1.0, 1.0]])
    out, _ = blend(colors, weights)
    np.testing.assert_allclose(out[0], [0.5, 0.0, 0.5])


def test_weights_are_normalized():
    colors = np.array([[[1.0, 1.0, 1.0], [0.0, 0.0, 0.0]]])
    # unnormalized weights should behave like their normalized ratio
    out_a, _ = blend(colors, np.array([[3.0, 1.0]]))
    out_b, _ = blend(colors, np.array([[0.75, 0.25]]))
    np.testing.assert_allclose(out_a, out_b)
    np.testing.assert_allclose(out_a[0], [0.75, 0.75, 0.75])


def test_zero_weight_candidate_ignored():
    colors = np.array([[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    weights = np.array([[1.0, 0.0]])
    out, valid = blend(colors, weights)
    assert valid[0]
    np.testing.assert_allclose(out[0], [1.0, 0.0, 0.0])


def test_all_zero_weights_marks_invalid():
    colors = np.array([[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    weights = np.array([[0.0, 0.0]])
    out, valid = blend(colors, weights)
    assert not valid[0]


def test_border_feather_center_is_one():
    uv = np.array([[320.0, 240.0]])
    w = border_feather(uv, width=640, height=480, margin=40.0)
    np.testing.assert_allclose(w, [1.0])


def test_border_feather_edge_is_zero():
    uv = np.array([[0.0, 240.0], [639.0, 240.0], [320.0, 0.0]])
    w = border_feather(uv, width=640, height=480, margin=40.0)
    np.testing.assert_allclose(w, [0.0, 0.0, 0.0], atol=1e-9)


def test_border_feather_monotonic_into_frame():
    uv = np.array([[5.0, 240.0], [20.0, 240.0], [35.0, 240.0]])
    w = border_feather(uv, width=640, height=480, margin=40.0)
    assert w[0] < w[1] < w[2]
