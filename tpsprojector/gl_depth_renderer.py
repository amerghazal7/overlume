"""GPU point-cloud splatting: a GL port of DepthRenderer.

Each finite-depth source pixel becomes one GL point. The world points are
back-projected on the CPU once (a static scene reuses the same cloud), uploaded
as a vertex buffer, and projected into the virtual camera in the vertex shader.
The hardware depth test implements nearest-wins, replacing the NumPy z-buffer.
"""

from __future__ import annotations

import numpy as np

_VERT = """
#version 330
in vec3 in_pos;
in vec3 in_col;
uniform vec3 vright, vdown, vfwd, vcenter;
uniform float vfx, vfy, vcx, vcy, out_w, out_h, point_size, zfar;
out vec3 col;
void main() {
    vec3 rel = in_pos - vcenter;
    float z = dot(vfwd, rel);
    col = in_col;
    if (z <= 1e-6) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }  // cull behind
    float xp = vfx * dot(vright, rel) / z + vcx;
    float yp = vfy * dot(vdown, rel) / z + vcy;
    float xndc = ((xp + 0.5) / out_w) * 2.0 - 1.0;
    float yndc = 1.0 - ((yp + 0.5) / out_h) * 2.0;       // image row -> NDC (y up)
    float zndc = clamp(z / zfar, 0.0, 1.0) * 2.0 - 1.0;  // monotone in z: nearest wins
    gl_Position = vec4(xndc, yndc, zndc, 1.0);
    gl_PointSize = point_size;
}
"""

_FRAG = """
#version 330
in vec3 col;
out vec4 frag;
void main() { frag = vec4(col, 1.0); }
"""


class GLDepthRenderer:
    def __init__(self, splat_radius: int = 1, fill_color=(0.0, 0.0, 0.0)):
        self.splat_radius = int(splat_radius)
        self.fill_color = tuple(float(c) for c in fill_color)
        # Cached for the lifetime of the singleton GL context (see gl_context.get_context).
        self._prog = None

    def _point_cloud(self, frames):
        pts, cols = [], []
        for fr in frames:
            finite = np.isfinite(fr.depth)
            ys, xs = np.nonzero(finite)
            if xs.size == 0:
                continue
            uv = np.stack([xs, ys], axis=-1).astype(float)
            pts.append(fr.camera.backproject(uv, fr.depth[ys, xs]))
            cols.append(fr.image[ys, xs])
        if not pts:
            return np.zeros((0, 3), "f4"), np.zeros((0, 3), "f4")
        return (np.concatenate(pts).astype("f4"),
                np.concatenate(cols).astype("f4"))

    def render(self, frames, virtual_camera):
        from .gl_context import get_context, get_fbo
        import moderngl
        ctx = get_context()
        if self._prog is None:
            self._prog = ctx.program(vertex_shader=_VERT, fragment_shader=_FRAG)
        prog = self._prog

        W, H = virtual_camera.width, virtual_camera.height
        P, C = self._point_cloud(frames)
        fbo = get_fbo(W, H, depth=True)
        fbo.use()
        fbo.clear(*self.fill_color, 0.0)
        if P.shape[0] == 0:
            raw = np.frombuffer(fbo.read(components=4, dtype="f4"), "f4").reshape(H, W, 4)
            raw = np.flipud(raw).copy()
            return raw[..., :3].astype(np.float64), raw[..., 3] > 0.5

        Rv, tv = virtual_camera.pose.R, virtual_camera.pose.t
        prog["vright"].value = tuple(Rv[:, 0])
        prog["vdown"].value = tuple(Rv[:, 1])
        prog["vfwd"].value = tuple(Rv[:, 2])
        prog["vcenter"].value = tuple(tv)
        prog["vfx"].value = float(virtual_camera.K[0, 0])
        prog["vfy"].value = float(virtual_camera.K[1, 1])
        prog["vcx"].value = float(virtual_camera.K[0, 2])
        prog["vcy"].value = float(virtual_camera.K[1, 2])
        prog["out_w"].value = float(W)
        prog["out_h"].value = float(H)
        prog["point_size"].value = float(2 * self.splat_radius + 1)
        prog["zfar"].value = 1.0e3

        vbo_pos = ctx.buffer(P.tobytes())
        vbo_col = ctx.buffer(C.tobytes())
        vao = ctx.vertex_array(prog, [(vbo_pos, "3f", "in_pos"),
                                      (vbo_col, "3f", "in_col")])
        ctx.enable(moderngl.DEPTH_TEST | moderngl.PROGRAM_POINT_SIZE)
        vao.render(mode=0)            # 0 = GL_POINTS
        ctx.disable(moderngl.DEPTH_TEST | moderngl.PROGRAM_POINT_SIZE)
        vao.release(); vbo_pos.release(); vbo_col.release()

        raw = np.frombuffer(fbo.read(components=4, dtype="f4"), "f4").reshape(H, W, 4)
        raw = np.flipud(raw).copy()
        frame = raw[..., :3].astype(np.float64)
        valid = raw[..., 3] > 0.5
        frame[~valid] = self.fill_color
        return frame, valid
