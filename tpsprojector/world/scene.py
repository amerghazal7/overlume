"""The synthetic 3D test world.

A :class:`Scene` is just a triangle soup (vertices, faces, per-face colors) that
can be rendered from any :class:`~tpsprojector.camera.PinholeCamera`. The world
holds a checkerboard ground plus a few boxes and walls that stand up off the
floor, so we can see how the bowl proxy handles both near ground and tall
objects.

The scene intentionally excludes the robot body: the onboard ring cameras look
outward and never see it, and the synthesized view composites a robot proxy
separately. Keeping the robot out of the scene means the ground-truth view is a
clean environment-only reference for measuring projection quality.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..camera import PinholeCamera
from .rasterizer import rasterize


@dataclass
class Scene:
    vertices: np.ndarray  # (V, 3)
    faces: np.ndarray     # (F, 3) int
    face_colors: np.ndarray  # (F, 3)

    def render(self, camera: PinholeCamera, bg_color=(0.45, 0.6, 0.8)):
        return rasterize(camera, self.vertices, self.faces, self.face_colors,
                         bg_color=bg_color)


class _Builder:
    """Accumulates vertices/faces/colors for assembling a scene."""

    def __init__(self):
        self.v = []
        self.f = []
        self.c = []

    def add_quad(self, p0, p1, p2, p3, color):
        base = len(self.v)
        self.v.extend([p0, p1, p2, p3])
        self.f.append([base, base + 1, base + 2])
        self.f.append([base, base + 2, base + 3])
        self.c.append(color)
        self.c.append(color)

    def add_box(self, center, size, color):
        cx, cy, cz = center
        sx, sy, sz = np.array(size) / 2.0
        x0, x1 = cx - sx, cx + sx
        y0, y1 = cy - sy, cy + sy
        z0, z1 = cz - sz, cz + sz
        # 8 corners
        c000 = (x0, y0, z0); c100 = (x1, y0, z0)
        c110 = (x1, y1, z0); c010 = (x0, y1, z0)
        c001 = (x0, y0, z1); c101 = (x1, y0, z1)
        c111 = (x1, y1, z1); c011 = (x0, y1, z1)
        shade = lambda f: tuple(np.clip(np.array(color) * f, 0, 1))
        self.add_quad(c001, c101, c111, c011, shade(1.0))   # top
        self.add_quad(c000, c010, c110, c100, shade(0.5))   # bottom
        self.add_quad(c000, c100, c101, c001, shade(0.8))   # -y
        self.add_quad(c010, c011, c111, c110, shade(0.7))   # +y
        self.add_quad(c000, c001, c011, c010, shade(0.6))   # -x
        self.add_quad(c100, c110, c111, c101, shade(0.9))   # +x

    def build(self) -> Scene:
        return Scene(
            vertices=np.array(self.v, dtype=float),
            faces=np.array(self.f, dtype=int),
            face_colors=np.array(self.c, dtype=float),
        )


def default_scene(extent: float = 20.0, cell: float = 2.0) -> Scene:
    """Checkerboard ground over ``[-extent, extent]^2`` plus boxes and walls."""
    b = _Builder()

    light = (0.55, 0.55, 0.5)
    dark = (0.3, 0.3, 0.28)
    n = int(2 * extent / cell)
    for ix in range(n):
        for iy in range(n):
            x0 = -extent + ix * cell
            y0 = -extent + iy * cell
            color = light if (ix + iy) % 2 == 0 else dark
            b.add_quad((x0, y0, 0.0), (x0 + cell, y0, 0.0),
                       (x0 + cell, y0 + cell, 0.0), (x0, y0 + cell, 0.0), color)

    # a few colored boxes standing on the ground (obstacles)
    b.add_box(center=(6.0, 0.0, 1.0), size=(2.0, 2.0, 2.0), color=(0.85, 0.2, 0.2))
    b.add_box(center=(-5.0, 4.0, 0.75), size=(1.5, 1.5, 1.5), color=(0.2, 0.7, 0.3))
    b.add_box(center=(0.0, -7.0, 1.5), size=(3.0, 1.0, 3.0), color=(0.2, 0.4, 0.9))
    b.add_box(center=(-6.0, -6.0, 0.5), size=(1.0, 1.0, 1.0), color=(0.9, 0.8, 0.2))

    # boundary walls (tall, to exercise the bowl wall projection)
    h = 3.0
    b.add_box(center=(0.0, extent, h / 2), size=(2 * extent, 0.5, h), color=(0.7, 0.6, 0.5))
    b.add_box(center=(0.0, -extent, h / 2), size=(2 * extent, 0.5, h), color=(0.6, 0.55, 0.7))
    b.add_box(center=(extent, 0.0, h / 2), size=(0.5, 2 * extent, h), color=(0.55, 0.7, 0.6))
    b.add_box(center=(-extent, 0.0, h / 2), size=(0.5, 2 * extent, h), color=(0.7, 0.5, 0.6))

    return b.build()
