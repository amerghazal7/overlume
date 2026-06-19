"""Proxy geometry surfaces for reprojection.

A :class:`Surface` answers the single question the renderer needs: given world
rays ``(origins, dirs)``, where does each ray hit the proxy geometry?

This is the geometry swap point. Today: :class:`FlatSurface` (debug baseline)
and :class:`BowlSurface` (flat floor + parabolic wall). A future depth- or
learning-based surface only has to implement :meth:`Surface.intersect` with the
same contract to drop into the pipeline.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class Surface(ABC):
    @abstractmethod
    def intersect(self, origins: np.ndarray, dirs: np.ndarray):
        """Intersect world rays with the surface.

        Args:
            origins: ``(N, 3)`` ray origins in the rig/world frame.
            dirs: ``(N, 3)`` ray directions (need not be unit length).

        Returns:
            ``(hits, valid)`` where ``hits`` is ``(N, 3)`` (NaN where invalid)
            and ``valid`` is a boolean ``(N,)`` mask of rays that hit.
        """
        raise NotImplementedError


class FlatSurface(Surface):
    """The ground plane ``z = z0``. Closed-form intersection."""

    def __init__(self, z0: float = 0.0):
        self.z0 = float(z0)

    def intersect(self, origins: np.ndarray, dirs: np.ndarray):
        o = np.asarray(origins, dtype=float).reshape(-1, 3)
        d = np.asarray(dirs, dtype=float).reshape(-1, 3)
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.z0 - o[:, 2]) / dz
        valid = (np.abs(dz) > 1e-12) & (t > 1e-9)
        t_safe = np.where(valid, t, np.nan)
        hits = o + t_safe[:, None] * d
        return hits, valid


class BowlSurface(Surface):
    """Flat floor inside ``R0`` blending into a parabolic wall.

    Height profile (function of radius ``r = hypot(x, y)``)::

        f(r) = 0                         for r <= R0      (flat floor)
             = k * (r - R0)^2            for R0 < r < Rmax (parabolic wall)
             = k * (Rmax - R0)^2         for r >= Rmax     (clamped ceiling)

    The wall has zero slope at ``R0`` so it is tangent to the floor (no seam).
    Intersection uses a vectorized bisection root-find on the implicit function
    ``g(t) = z(t) - f(r(t))``, which is exact for the floor and general enough
    for any monotone profile a future surface might use.
    """

    def __init__(self, R0: float = 10.0, k: float = 0.04, Rmax: float = 30.0):
        assert Rmax > R0 >= 0
        self.R0 = float(R0)
        self.k = float(k)
        self.Rmax = float(Rmax)

    def height(self, r):
        r = np.asarray(r, dtype=float)
        d = np.clip(r - self.R0, 0.0, self.Rmax - self.R0)
        return self.k * d * d

    def _g(self, o, d, t):
        # signed height above the surface along each ray at parameter t
        P = o + t[:, None] * d
        r = np.hypot(P[:, 0], P[:, 1])
        return P[:, 2] - self.height(r)

    def intersect(self, origins: np.ndarray, dirs: np.ndarray,
                  t_max: float = 1.0e4, iters: int = 60):
        o = np.asarray(origins, dtype=float).reshape(-1, 3)
        d = np.asarray(dirs, dtype=float).reshape(-1, 3)
        n = o.shape[0]
        eps = 1e-6

        g_lo = self._g(o, d, np.full(n, eps))
        g_hi = self._g(o, d, np.full(n, t_max))
        # A hit exists when the ray starts above the surface and ends below it.
        valid = (g_lo > 0.0) & (g_hi < 0.0)

        lo = np.full(n, eps)
        hi = np.full(n, t_max)
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            g_mid = self._g(o, d, mid)
            # keep the bracket straddling the sign change
            go_high = g_mid > 0.0
            lo = np.where(go_high, mid, lo)
            hi = np.where(go_high, hi, mid)
        t = 0.5 * (lo + hi)

        t_safe = np.where(valid, t, np.nan)
        hits = o + t_safe[:, None] * d
        return hits, valid
