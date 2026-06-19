"""Depth-derived rendering: forward point-cloud splatting.

Instead of assuming a static bowl, this backend uses an accurate **per-camera
depth map** to recover the true scene geometry each frame, then renders it from
the virtual camera. Every real pixel is back-projected to its true 3D point, all
cameras' points are fused, and the cloud is splatted into the virtual view with a
z-buffer (nearest wins). Because points sit at true depth, parallax is correct
for *all* objects — the bowl's off-surface ghosting disappears.

Depth source is swappable: :func:`synthetic_frames` uses the rasterizer's
ground-truth z-buffer now; a real system would supply depth from a model or
LIDAR fusion behind the same :class:`CameraFrame` contract.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from .camera import PinholeCamera
from .world.scene import Scene


@dataclass
class CameraFrame:
    """One onboard camera's image, per-pixel depth (camera-space z), and pose."""

    image: np.ndarray   # (H, W, 3)
    depth: np.ndarray   # (H, W); inf where there is no return
    camera: PinholeCamera


def synthetic_frames(scene: Scene, cameras: Sequence[PinholeCamera],
                     bg_color=(0.45, 0.6, 0.8)) -> List[CameraFrame]:
    """Render each camera to a :class:`CameraFrame` with ground-truth depth."""
    frames = []
    for cam in cameras:
        img, depth = scene.render(cam, bg_color=bg_color)
        frames.append(CameraFrame(image=img, depth=depth, camera=cam))
    return frames


class DepthRenderer:
    """Forward point-cloud splatting from depth-equipped camera frames."""

    def __init__(self, splat_radius: int = 1, fill_color=(0.0, 0.0, 0.0)):
        self.splat_radius = int(splat_radius)
        self.fill_color = np.asarray(fill_color, dtype=float)

    def _point_cloud(self, frames: Sequence[CameraFrame]):
        pts, cols = [], []
        for fr in frames:
            finite = np.isfinite(fr.depth)
            ys, xs = np.nonzero(finite)
            if xs.size == 0:
                continue
            uv = np.stack([xs, ys], axis=-1).astype(float)
            world = fr.camera.backproject(uv, fr.depth[ys, xs])
            pts.append(world)
            cols.append(fr.image[ys, xs])
        if not pts:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return np.concatenate(pts), np.concatenate(cols)

    def render(self, frames: Sequence[CameraFrame],
               virtual_camera: PinholeCamera):
        W, H = virtual_camera.width, virtual_camera.height
        P, C = self._point_cloud(frames)

        frame = np.zeros((H * W, 3), dtype=float) + self.fill_color
        valid = np.zeros(H * W, dtype=bool)
        if P.shape[0] == 0:
            return frame.reshape(H, W, 3), valid.reshape(H, W)

        uv, in_front = virtual_camera.project(P)
        zc = virtual_camera.pose.inverse().transform_points(P)[:, 2]
        vx = np.round(uv[:, 0]).astype(int)
        vy = np.round(uv[:, 1]).astype(int)
        base = in_front & (zc > 1e-6)

        # expand each point over its splat footprint to fill sparsity holes
        r = self.splat_radius
        txs, tys, tzs, tcs = [], [], [], []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                px, py = vx + dx, vy + dy
                keep = base & (px >= 0) & (px < W) & (py >= 0) & (py < H)
                txs.append(px[keep])
                tys.append(py[keep])
                tzs.append(zc[keep])
                tcs.append(C[keep])
        tx = np.concatenate(txs)
        ty = np.concatenate(tys)
        tz = np.concatenate(tzs)
        tc = np.concatenate(tcs)
        if tx.size == 0:
            return frame.reshape(H, W, 3), valid.reshape(H, W)

        idx = ty * W + tx
        depthbuf = np.full(H * W, np.inf)
        np.minimum.at(depthbuf, idx, tz)

        # a write wins its pixel if it is (within eps of) the nearest depth there
        winner = tz <= depthbuf[idx] + 1e-6
        frame[idx[winner]] = tc[winner]
        valid[idx[winner]] = True
        return frame.reshape(H, W, 3), valid.reshape(H, W)
