"""GPU bowl reprojection: a backward fragment-shader port of NumpyRenderer.

A full-screen triangle draws one fragment per virtual pixel. Each fragment
reconstructs its world ray, intersects the proxy surface (plane closed-form or
bowl bisection), reprojects the hit point into every real camera that sees it,
samples (hardware bilinear), and blends with the exact feather + angular weights
of the NumPy backend. Output is read back to NumPy so the Renderer contract is
intact.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from .camera import PinholeCamera
from .renderer import Renderer
from .surface import BowlSurface, FlatSurface

_VERT = """
#version 330
const vec2 verts[3] = vec2[3](vec2(-1.0,-1.0), vec2(3.0,-1.0), vec2(-1.0,3.0));
void main() { gl_Position = vec4(verts[gl_VertexID], 0.0, 1.0); }
"""

_FRAG = """
#version 330
#define NCAM {ncam}
uniform sampler2DArray cams;
uniform float cam_w, cam_h;
uniform vec3 cright[NCAM], cdown[NCAM], cfwd[NCAM], ccenter[NCAM];
uniform float cfx[NCAM], cfy[NCAM], ccx[NCAM], ccy[NCAM];

uniform float out_h;
uniform vec3 vright, vdown, vfwd, vcenter;
uniform float vfx, vfy, vcx, vcy;

uniform int surf_type;            // 0 = flat, 1 = bowl
uniform float flat_z0;
uniform float bowl_R0, bowl_k, bowl_Rmax;

uniform float feather_margin;
uniform vec3 fill_color;

out vec4 frag;

float heightf(float r) {{
    float d = clamp(r - bowl_R0, 0.0, bowl_Rmax - bowl_R0);
    return bowl_k * d * d;
}}
float gfun(vec3 o, vec3 dir, float t) {{
    vec3 P = o + t * dir;
    return P.z - heightf(length(P.xy));
}}

bool intersect(vec3 o, vec3 dir, out vec3 P) {{
    if (surf_type == 0) {{                 // flat plane z = z0
        float dz = dir.z;
        if (abs(dz) <= 1e-12) return false;
        float t = (flat_z0 - o.z) / dz;
        if (t <= 1e-9) return false;
        P = o + t * dir;
        return true;
    }}
    float eps = 1e-6, tmax = 1.0e4;       // bowl bisection (matches BowlSurface)
    if (!(gfun(o, dir, eps) > 0.0 && gfun(o, dir, tmax) < 0.0)) return false;
    float lo = eps, hi = tmax;
    for (int i = 0; i < 60; i++) {{
        float mid = 0.5 * (lo + hi);
        if (gfun(o, dir, mid) > 0.0) lo = mid; else hi = mid;
    }}
    P = o + (0.5 * (lo + hi)) * dir;
    return true;
}}

float feather(float u, float v, float w, float h) {{
    float dx = min(u, (w - 1.0) - u);
    float dy = min(v, (h - 1.0) - v);
    float d = min(dx, dy);
    if (feather_margin <= 0.0) return d >= 0.0 ? 1.0 : 0.0;
    return smoothstep(0.0, 1.0, d / feather_margin);   // == NumPy smoothstep(d/margin)
}}

void main() {{
    // image-space pixel coords matching NumpyRenderer (row 0 = top); framebuffer
    // is bottom-up, so we flip v here AND flip the readback array on the CPU.
    float u = gl_FragCoord.x - 0.5;
    float v = out_h - 0.5 - gl_FragCoord.y;

    vec3 dc = vec3((u - vcx) / vfx, (v - vcy) / vfy, 1.0);
    vec3 dir = normalize(vright * dc.x + vdown * dc.y + vfwd * dc.z);
    vec3 o = vcenter;

    vec3 P;
    if (!intersect(o, dir, P)) {{ frag = vec4(fill_color, 0.0); return; }}

    vec3 acc = vec3(0.0);
    float wsum = 0.0;
    for (int i = 0; i < NCAM; i++) {{
        vec3 rel = P - ccenter[i];
        float z = dot(cfwd[i], rel);
        if (z <= 1e-9) continue;
        float xp = cfx[i] * dot(cright[i], rel) / z + ccx[i];
        float yp = cfy[i] * dot(cdown[i], rel) / z + ccy[i];
        if (xp < 0.0 || xp > cam_w - 1.0 || yp < 0.0 || yp > cam_h - 1.0) continue;
        vec3 col = texture(cams, vec3((xp + 0.5) / cam_w, (yp + 0.5) / cam_h, float(i))).rgb;
        float align = clamp(dot(normalize(rel), cfwd[i]), 0.0, 1.0);
        float w = feather(xp, yp, cam_w, cam_h) * align * align;
        acc += w * col;
        wsum += w;
    }}
    if (wsum > 0.0) frag = vec4(acc / wsum, 1.0);
    else frag = vec4(fill_color, 0.0);
}}
"""


class GLBowlRenderer(Renderer):
    def __init__(self, feather_margin: float = 30.0, fill_color=(0.0, 0.0, 0.0)):
        self.feather_margin = float(feather_margin)
        self.fill_color = tuple(float(c) for c in fill_color)
        # Cached for the lifetime of the singleton GL context (see gl_context.get_context).
        self._progs = {}        # ncam -> (program, vao)
        self._cam_tex = None    # (ncam, H, W) cached sampler2DArray

    def _program(self, ncam: int):
        from .gl_context import get_context
        prog = self._progs.get(ncam)
        if prog is None:
            ctx = get_context()
            program = ctx.program(vertex_shader=_VERT,
                                  fragment_shader=_FRAG.format(ncam=ncam))
            vao = ctx.vertex_array(program, [])
            self._progs[ncam] = prog = (program, vao)
        return prog

    def _render_to_fbo(self, camera_images: Sequence[np.ndarray],
                       cameras: Sequence[PinholeCamera], surface,
                       virtual_camera: PinholeCamera):
        from .gl_context import get_context, get_fbo
        ctx = get_context()
        n = len(cameras)
        program, vao = self._program(n)

        cam_h, cam_w = camera_images[0].shape[:2]
        stack = np.stack([np.asarray(im, dtype="f4") for im in camera_images])  # (n,H,W,3)
        # Reuse the same allocation when camera geometry is unchanged; write() below always re-uploads the pixel data.
        if self._cam_tex is None or self._cam_tex.size != (cam_w, cam_h) \
                or self._cam_tex.layers != n:
            if self._cam_tex is not None:
                self._cam_tex.release()
            self._cam_tex = ctx.texture_array((cam_w, cam_h, n), 3, dtype="f4")
            self._cam_tex.filter = (9729, 9729)             # GL_LINEAR, GL_LINEAR
            self._cam_tex.repeat_x = self._cam_tex.repeat_y = False  # CLAMP_TO_EDGE
        self._cam_tex.write(stack.tobytes())
        self._cam_tex.use(0)
        program["cams"] = 0
        program["cam_w"].value = float(cam_w)
        program["cam_h"].value = float(cam_h)

        cright_list, cdown_list, cfwd_list, ccenter_list = [], [], [], []
        cfx_list, cfy_list, ccx_list, ccy_list = [], [], [], []
        for cam in cameras:
            R, t = cam.pose.R, cam.pose.t
            cright_list.append(tuple(float(v) for v in R[:, 0]))
            cdown_list.append(tuple(float(v) for v in R[:, 1]))
            cfwd_list.append(tuple(float(v) for v in R[:, 2]))
            ccenter_list.append(tuple(float(v) for v in t))
            cfx_list.append(float(cam.K[0, 0]))
            cfy_list.append(float(cam.K[1, 1]))
            ccx_list.append(float(cam.K[0, 2]))
            ccy_list.append(float(cam.K[1, 2]))
        program["cright"].value = cright_list
        program["cdown"].value = cdown_list
        program["cfwd"].value = cfwd_list
        program["ccenter"].value = ccenter_list
        program["cfx"].value = cfx_list
        program["cfy"].value = cfy_list
        program["ccx"].value = ccx_list
        program["ccy"].value = ccy_list

        W, H = virtual_camera.width, virtual_camera.height
        Rv, tv = virtual_camera.pose.R, virtual_camera.pose.t
        program["out_h"].value = float(H)
        program["vright"].value = tuple(Rv[:, 0])
        program["vdown"].value = tuple(Rv[:, 1])
        program["vfwd"].value = tuple(Rv[:, 2])
        program["vcenter"].value = tuple(tv)
        program["vfx"].value = float(virtual_camera.K[0, 0])
        program["vfy"].value = float(virtual_camera.K[1, 1])
        program["vcx"].value = float(virtual_camera.K[0, 2])
        program["vcy"].value = float(virtual_camera.K[1, 2])

        if isinstance(surface, FlatSurface):
            program["surf_type"].value = 0
            program["flat_z0"].value = float(surface.z0)
            program["bowl_R0"].value = 1.0
            program["bowl_k"].value = 0.0
            program["bowl_Rmax"].value = 2.0
        elif isinstance(surface, BowlSurface):
            program["surf_type"].value = 1
            program["flat_z0"].value = 0.0
            program["bowl_R0"].value = float(surface.R0)
            program["bowl_k"].value = float(surface.k)
            program["bowl_Rmax"].value = float(surface.Rmax)
        else:
            raise NotImplementedError(
                "GLBowlRenderer supports only FlatSurface and BowlSurface; "
                f"got {type(surface).__name__}. Use the depth backend for "
                "arbitrary geometry.")

        program["feather_margin"].value = self.feather_margin
        program["fill_color"].value = self.fill_color

        fbo = get_fbo(W, H)
        fbo.use()
        fbo.clear(*self.fill_color, 0.0)
        vao.render(mode=6, vertices=3)   # 6 = GL_TRIANGLES
        return fbo

    def render(self, camera_images: Sequence[np.ndarray],
               cameras: Sequence[PinholeCamera], surface,
               virtual_camera: PinholeCamera):
        W, H = virtual_camera.width, virtual_camera.height
        fbo = self._render_to_fbo(camera_images, cameras, surface, virtual_camera)
        raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(H, W, 4)
        raw = np.flipud(raw).copy()      # framebuffer is bottom-up
        frame = raw[..., :3].astype(np.float64)
        valid = raw[..., 3] > 0.5
        frame[~valid] = self.fill_color
        return frame, valid
