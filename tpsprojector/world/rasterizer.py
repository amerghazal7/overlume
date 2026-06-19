"""Minimal flat-shaded triangle rasterizer with a z-buffer.

This exists only to render the *synthetic* test world (onboard camera images
and the ground-truth view). It is deliberately simple: flat per-face color,
screen-space barycentric fill, camera-space depth buffer. Triangles with any
vertex at/behind the camera are skipped (keep ground meshes finely tessellated
so the dropped triangles are tiny). Not performance-tuned; it renders a handful
of cameras per frame at modest resolution.
"""

from __future__ import annotations

import numpy as np

from ..camera import PinholeCamera

_NEAR = 1e-6


def rasterize(camera: PinholeCamera, vertices: np.ndarray, faces: np.ndarray,
              face_colors: np.ndarray, bg_color=(0.0, 0.0, 0.0)):
    """Render colored triangles to ``(image, depth)``.

    Args:
        camera: the view to render from.
        vertices: ``(V, 3)`` world-frame vertices.
        faces: ``(F, 3)`` integer vertex indices.
        face_colors: ``(F, 3)`` RGB per face in ``[0, 1]``.
        bg_color: background RGB for uncovered pixels.

    Returns:
        ``image`` ``(H, W, 3)`` float and ``depth`` ``(H, W)`` camera-space z
        (``inf`` where nothing was drawn).
    """
    H, W = camera.height, camera.width
    image = np.empty((H, W, 3), dtype=float)
    image[:] = np.asarray(bg_color, dtype=float)
    depth = np.full((H, W), np.inf, dtype=float)

    vertices = np.asarray(vertices, dtype=float).reshape(-1, 3)
    faces = np.asarray(faces, dtype=int).reshape(-1, 3)
    face_colors = np.asarray(face_colors, dtype=float).reshape(-1, 3)

    cam_pts = camera.pose.inverse().transform_points(vertices)
    z = cam_pts[:, 2]
    uv, _ = camera.project(vertices)

    for fi in range(faces.shape[0]):
        i, j, k = faces[fi]
        zi, zj, zk = z[i], z[j], z[k]
        if zi <= _NEAR or zj <= _NEAR or zk <= _NEAR:
            continue

        ax, ay = uv[i]
        bx, by = uv[j]
        cx, cy = uv[k]

        min_x = max(int(np.floor(min(ax, bx, cx))), 0)
        max_x = min(int(np.ceil(max(ax, bx, cx))), W - 1)
        min_y = max(int(np.floor(min(ay, by, cy))), 0)
        max_y = min(int(np.ceil(max(ay, by, cy))), H - 1)
        if max_x < min_x or max_y < min_y:
            continue

        area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        if abs(area) < 1e-12:
            continue

        xs = np.arange(min_x, max_x + 1)
        ys = np.arange(min_y, max_y + 1)
        px, py = np.meshgrid(xs, ys)  # (h, w)

        # barycentric weights via edge functions, normalized by signed area
        w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) / area
        w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) / area
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
        if not inside.any():
            continue

        z_interp = w0 * zi + w1 * zj + w2 * zk
        sub_depth = depth[min_y:max_y + 1, min_x:max_x + 1]
        write = inside & (z_interp < sub_depth)
        if not write.any():
            continue

        sub_depth[write] = z_interp[write]
        sub_img = image[min_y:max_y + 1, min_x:max_x + 1]
        sub_img[write] = face_colors[fi]

    return image, depth
