"""Headless GL context and offscreen render targets for the GL backends.

A single standalone EGL context is created lazily and shared by all GL
renderers. Render targets (FBOs) are pooled by size so repeated frames at the
same resolution reuse GPU memory. ``gl_available()`` lets tests skip cleanly on
machines without a usable GL/EGL context.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

_ctx: "moderngl.Context | None" = None
_fbos: Dict[Tuple[int, int, bool], object] = {}
_available: Optional[bool] = None


def gl_available() -> bool:
    """True iff a standalone GL context can be created on this machine."""
    global _available
    if _available is None:
        try:
            get_context()
            _available = True
        except Exception:
            _available = False
    return _available


def get_context():
    """Return the lazily-created singleton standalone GL context."""
    global _ctx
    if _ctx is None:
        import moderngl
        _ctx = moderngl.create_standalone_context()
    return _ctx


def use_window_context():
    """Bind the GL backend to the CURRENT GL context (e.g. a pygame OpenGL window).

    Call this ONCE after creating the OpenGL window and BEFORE any GL renderer or
    get_context() use, so the renderers' FBOs/textures are allocated in the
    window's context and can be presented to the screen. Clears the FBO pool,
    which belonged to any previous (e.g. standalone) context.
    """
    global _ctx, _fbos
    import moderngl
    _ctx = moderngl.create_context()
    _fbos = {}
    return _ctx


def get_fbo(width: int, height: int, depth: bool = False):
    """Return a size-keyed cached framebuffer (RGBA32F color, optional depth)."""
    key = (int(width), int(height), bool(depth))
    fbo = _fbos.get(key)
    if fbo is None:
        ctx = get_context()
        color = ctx.texture((width, height), 4, dtype="f4")
        attachments = {"color_attachments": [color]}
        if depth:
            attachments["depth_attachment"] = ctx.depth_texture((width, height))
        fbo = ctx.framebuffer(**attachments)
        _fbos[key] = fbo
    return fbo


def release_fbos() -> None:
    """Release all pooled framebuffers and their attachment textures."""
    global _fbos
    for fbo in _fbos.values():
        for tex in getattr(fbo, "color_attachments", ()):  # color textures
            tex.release()
        depth = getattr(fbo, "depth_attachment", None)
        if depth is not None:
            depth.release()
        fbo.release()
    _fbos = {}
