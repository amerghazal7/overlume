"""The reprojection core: synthesize the virtual camera frame.

This is the swap point for backends. :class:`NumpyRenderer` implements the
algorithm in vectorized NumPy (correctness-first). A future ``GLRenderer`` /
CUDA backend only has to satisfy the :class:`Renderer` contract:

    render(camera_images, cameras, surface, virtual_camera) -> (frame, valid)

Per virtual pixel: unproject to a world ray, intersect the proxy surface to get
a 3D point, reproject that point into every real camera that sees it, sample
each, and blend with feather + angular weights.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Sequence

import numpy as np

from .blend import blend, border_feather
from .camera import PinholeCamera
from .surface import Surface


def bilinear_sample(image: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Bilinearly sample ``image`` (H, W, 3) at pixel coords ``uv`` (N, 2).

    ``uv`` is ``(x=col, y=row)``. Coordinates are clamped to the image bounds.
    """
    image = np.asarray(image, dtype=float)
    H, W = image.shape[:2]
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    x = np.clip(uv[:, 0], 0.0, W - 1)
    y = np.clip(uv[:, 1], 0.0, H - 1)
    x0 = np.floor(x).astype(int)
    y0 = np.floor(y).astype(int)
    x1 = np.minimum(x0 + 1, W - 1)
    y1 = np.minimum(y0 + 1, H - 1)
    wx = (x - x0)[:, None]
    wy = (y - y0)[:, None]
    c00 = image[y0, x0]
    c10 = image[y0, x1]
    c01 = image[y1, x0]
    c11 = image[y1, x1]
    top = c00 * (1 - wx) + c10 * wx
    bot = c01 * (1 - wx) + c11 * wx
    return top * (1 - wy) + bot * wy


class Renderer(ABC):
    @abstractmethod
    def render(self, camera_images: Sequence[np.ndarray],
               cameras: Sequence[PinholeCamera], surface: Surface,
               virtual_camera: PinholeCamera):
        raise NotImplementedError


class NumpyRenderer(Renderer):
    def __init__(self, feather_margin: float = 30.0, fill_color=(0.0, 0.0, 0.0)):
        self.feather_margin = float(feather_margin)
        self.fill_color = np.asarray(fill_color, dtype=float)

    def render(self, camera_images: Sequence[np.ndarray],
               cameras: Sequence[PinholeCamera], surface: Surface,
               virtual_camera: PinholeCamera):
        W, H = virtual_camera.width, virtual_camera.height

        # 1. one ray per virtual pixel (row-major: y outer, x inner)
        ys, xs = np.mgrid[0:H, 0:W]
        uv = np.stack([xs.ravel(), ys.ravel()], axis=-1).astype(float)
        origins, dirs = virtual_camera.unproject(uv)

        # 2. intersect the proxy surface -> 3D points
        hits, valid_surf = surface.intersect(origins, dirs)

        n_pix = uv.shape[0]
        n_cam = len(cameras)
        colors = np.zeros((n_pix, n_cam, 3), dtype=float)
        weights = np.zeros((n_pix, n_cam), dtype=float)

        safe_hits = np.where(np.isfinite(hits), hits, 0.0)
        for ci, (cam, img) in enumerate(zip(cameras, camera_images)):
            # 3. reproject the surface point into this real camera
            proj, in_front = cam.project(safe_hits)
            in_bounds = cam.in_bounds(proj)
            usable = valid_surf & in_front & in_bounds
            if not usable.any():
                continue

            # 4. sample
            colors[:, ci, :] = bilinear_sample(img, proj)

            # 5. weight: border feather * angular alignment to optical axis
            feather = border_feather(proj, cam.width, cam.height, self.feather_margin)
            to_pt = safe_hits - cam.pose.t
            to_pt /= (np.linalg.norm(to_pt, axis=-1, keepdims=True) + 1e-12)
            fwd = cam.pose.R[:, 2]
            align = np.clip(to_pt @ fwd, 0.0, 1.0)
            weights[:, ci] = np.where(usable, feather * align * align, 0.0)

        out, valid = blend(colors, weights)
        out = np.where(valid[:, None], out, self.fill_color)

        frame = out.reshape(H, W, 3)
        valid_mask = valid.reshape(H, W)
        return frame, valid_mask
