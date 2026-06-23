"""Fullscreen-quad presentation: blit a texture / FBO / NumPy array to a target.

Used by the live OpenGL app to show the rendered FBO with no readback, and to
present the NumPy validation composite. All blits go through one present shader so
the app window stays a single OpenGL surface.
"""

from __future__ import annotations

import numpy as np

_PVERT = """
#version 330
const vec2 v[3] = vec2[3](vec2(-1.0,-1.0), vec2(3.0,-1.0), vec2(-1.0,3.0));
uniform int flip;
out vec2 uv;
void main() {
    vec2 p = v[gl_VertexID];
    gl_Position = vec4(p, 0.0, 1.0);
    vec2 t = 0.5 * (p + 1.0);
    uv = vec2(t.x, flip == 1 ? 1.0 - t.y : t.y);
}
"""

_PFRAG = """
#version 330
uniform sampler2D src;
in vec2 uv;
out vec4 frag;
void main() { frag = texture(src, uv); }
"""

_progs = {}      # id(ctx) -> (program, vao)
_arr_tex = {}    # (id(ctx), w, h, comp) -> texture


def _program():
    from .gl_context import get_context
    ctx = get_context()
    key = id(ctx)
    pv = _progs.get(key)
    if pv is None:
        program = ctx.program(vertex_shader=_PVERT, fragment_shader=_PFRAG)
        vao = ctx.vertex_array(program, [])
        _progs[key] = pv = (program, vao)
    return pv


def present_texture(tex, target=None, blend=False, flip=False):
    """Blit ``tex`` over a fullscreen quad into ``target`` (default: the screen)."""
    from .gl_context import get_context
    import moderngl
    ctx = get_context()
    program, vao = _program()
    tex.use(0)
    program["src"] = 0
    program["flip"] = 1 if flip else 0
    (target if target is not None else ctx.screen).use()
    if blend:
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = (moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA)
    vao.render(mode=6, vertices=3)   # 6 = GL_TRIANGLES
    if blend:
        ctx.disable(moderngl.BLEND)


def present_fbo(fbo, target=None, blend=False):
    """Blit an FBO's color texture 1:1 (FBO is already in screen orientation)."""
    present_texture(fbo.color_attachments[0], target=target, blend=blend, flip=False)


def present_array(arr, target=None, blend=False):
    """Upload a NumPy (H,W,3|4) float array (row 0 = top) and blit it upright."""
    from .gl_context import get_context
    ctx = get_context()
    a = np.ascontiguousarray(arr, dtype="f4")
    h, w = a.shape[:2]
    comp = a.shape[2] if a.ndim == 3 else 1
    key = (id(ctx), w, h, comp)
    tex = _arr_tex.get(key)
    if tex is None:
        tex = ctx.texture((w, h), comp, dtype="f4")
        tex.filter = (9729, 9729)    # GL_LINEAR
        _arr_tex[key] = tex
    tex.write(a.tobytes())
    present_texture(tex, target=target, blend=blend, flip=True)
