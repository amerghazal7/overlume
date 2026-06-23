"""Headless GL context and offscreen render targets for the GL backends.

A single standalone EGL context is created lazily and shared by all GL
renderers. Render targets (FBOs) are pooled by size so repeated frames at the
same resolution reuse GPU memory. ``gl_available()`` lets tests skip cleanly on
machines without a usable GL/EGL context.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

_ctx = None                       # type: ignore[var-annotated]
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
