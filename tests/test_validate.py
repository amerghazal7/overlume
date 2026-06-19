import numpy as np

from tpsprojector.validate import diff_heatmap, psnr, ssim


def test_psnr_identical_is_high():
    a = np.random.RandomState(0).rand(16, 16, 3)
    assert psnr(a, a) >= 99.0


def test_psnr_decreases_with_error():
    a = np.full((16, 16, 3), 0.5)
    near = a + 0.05
    far = a + 0.2
    assert psnr(a, near) > psnr(a, far)


def test_psnr_respects_mask():
    a = np.zeros((4, 4, 3))
    b = a.copy()
    b[0, 0] = 1.0  # one bad pixel
    mask = np.ones((4, 4), dtype=bool)
    mask[0, 0] = False  # exclude it
    assert psnr(a, b, mask) >= 99.0


def test_ssim_identical_is_one():
    a = np.random.RandomState(1).rand(32, 32, 3)
    np.testing.assert_allclose(ssim(a, a), 1.0, atol=1e-6)


def test_ssim_lower_for_different_images():
    a = np.random.RandomState(2).rand(32, 32, 3)
    b = np.random.RandomState(3).rand(32, 32, 3)
    assert ssim(a, b) < 0.5


def test_diff_heatmap_shape_and_range():
    a = np.zeros((8, 8, 3))
    b = np.ones((8, 8, 3))
    hm = diff_heatmap(a, b)
    assert hm.shape == (8, 8, 3)
    assert hm.min() >= 0.0 and hm.max() <= 1.0
