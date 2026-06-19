"""Blending candidate colors in camera-overlap regions.

When a virtual pixel's surface point reprojects into several real cameras we
get multiple candidate colors. Blending them with smooth weights (rather than
hard-selecting one) avoids visible seams and exposure jumps across the ~15
degree overlaps. The blend itself is just a normalized weighted average; the
interesting part is how weights are built (border feather + angular preference,
computed by the renderer).
"""

from __future__ import annotations

import numpy as np


def smoothstep(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def border_feather(uv: np.ndarray, width: int, height: int,
                   margin: float) -> np.ndarray:
    """Weight in ``[0, 1]`` that fades to 0 within ``margin`` px of any edge.

    Pixels deep inside the frame get weight 1; pixels on the border get 0, with
    a smooth ramp in between so contributions cross-fade across seams.
    """
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    dist_x = np.minimum(uv[:, 0], (width - 1) - uv[:, 0])
    dist_y = np.minimum(uv[:, 1], (height - 1) - uv[:, 1])
    d = np.minimum(dist_x, dist_y)
    if margin <= 0:
        return (d >= 0).astype(float)
    return smoothstep(d / margin)


def blend(colors: np.ndarray, weights: np.ndarray):
    """Normalized weighted average of candidate colors.

    Args:
        colors: ``(N, C, 3)`` candidate RGB per pixel per camera.
        weights: ``(N, C)`` non-negative weights (0 = candidate absent).

    Returns:
        ``(out, valid)`` with ``out`` ``(N, 3)`` (0 where invalid) and ``valid``
        a boolean ``(N,)`` mask where at least one candidate had weight > 0.
    """
    colors = np.asarray(colors, dtype=float)
    weights = np.asarray(weights, dtype=float)
    total = weights.sum(axis=1)
    valid = total > 0.0
    safe = np.where(valid, total, 1.0)
    out = (weights[:, :, None] * colors).sum(axis=1) / safe[:, None]
    out = np.where(valid[:, None], out, 0.0)
    return out, valid
