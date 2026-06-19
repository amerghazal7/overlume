"""Validation metrics for synthesized vs. ground-truth frames.

The synthetic world lets us render the *true* view at the virtual pose, so we
can measure synthesis error objectively. PSNR and a simplified global SSIM keep
the prototype dependency-free (no scipy/skimage). A diff heatmap gives a quick
visual read of where the synthesis breaks down (typically off-surface objects).
"""

from __future__ import annotations

import numpy as np

_MAX = 99.0


def _lum(img: np.ndarray) -> np.ndarray:
    img = np.asarray(img, dtype=float)
    return img[..., 0] * 0.299 + img[..., 1] * 0.587 + img[..., 2] * 0.114


def psnr(a: np.ndarray, b: np.ndarray, mask: np.ndarray | None = None) -> float:
    """Peak signal-to-noise ratio (dB) for images in ``[0, 1]``.

    With ``mask`` (H, W) only those pixels are scored. Returns ``99.0`` when the
    images match exactly.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if mask is not None:
        a = a[mask]
        b = b[mask]
    if a.size == 0:
        return 0.0
    mse = np.mean((a - b) ** 2)
    if mse <= 1e-12:
        return _MAX
    return float(min(_MAX, 10.0 * np.log10(1.0 / mse)))


def ssim(a: np.ndarray, b: np.ndarray) -> float:
    """Simplified global SSIM on luminance (single window over the image)."""
    x = _lum(a).ravel()
    y = _lum(b).ravel()
    c1 = 0.01 ** 2
    c2 = 0.03 ** 2
    mx, my = x.mean(), y.mean()
    vx, vy = x.var(), y.var()
    cov = np.mean((x - mx) * (y - my))
    luminance = (2 * mx * my + c1) / (mx * mx + my * my + c1)
    contrast_structure = (2 * cov + c2) / (vx + vy + c2)
    return float(luminance * contrast_structure)


def diff_heatmap(a: np.ndarray, b: np.ndarray, scale: float = 0.25) -> np.ndarray:
    """Cold(blue)->hot(red) heatmap of per-pixel error, values in ``[0, 1]``."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    err = np.abs(a - b).mean(axis=-1)
    h = np.clip(err / max(scale, 1e-6), 0.0, 1.0)
    r = h
    g = 1.0 - np.abs(2.0 * h - 1.0)
    bch = 1.0 - h
    return np.stack([r, g, bch], axis=-1)
