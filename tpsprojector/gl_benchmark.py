"""Frame-rate benchmark for the GL backends.

Renders a fixed preset repeatedly through a GL-backed Engine and reports fps.
Times two paths: ``fps`` times the no-readback bowl render (``_render_to_fbo``
+ ``ctx.finish()`` sync — required because GL is asynchronous); ``fps_readback``
times the end-to-end ``Engine._render_env`` for the requested mode, which
includes a framebuffer readback. The difference quantifies the readback cost.
Used as a manual perf check and a coarse regression guard in tests.
"""

from __future__ import annotations

import time

from .app import Engine
from .presets import PRESET_NAMES, get_preset


def benchmark(width: int = 1280, height: int = 720, mode: str = "bowl",
              iters: int = 30) -> dict:
    """Measure GL render fps with and without framebuffer readback.

    ``fps`` times the bowl no-readback render path (``_render_to_fbo`` +
    ``ctx.finish()`` — the sync is required because GL is asynchronous).
    ``fps_readback`` times the end-to-end ``Engine._render_env`` for ``mode``.
    """
    from .gl_context import get_context
    eng = Engine.from_defaults(width=width, height=height, mode=mode, backend="gl")
    vc = eng.virtual_camera(get_preset(PRESET_NAMES[0]))
    ctx = get_context()
    r = eng.bowl_renderer

    # no-readback path (pure GPU render)
    r._render_to_fbo(eng.images, eng.cameras, eng.surface, vc)
    ctx.finish()                                   # warm up
    t0 = time.perf_counter()
    for _ in range(iters):
        r._render_to_fbo(eng.images, eng.cameras, eng.surface, vc)
        ctx.finish()
    dt = time.perf_counter() - t0

    # readback path (end-to-end for the requested mode)
    eng._render_env(vc)                            # warm up
    t1 = time.perf_counter()
    for _ in range(iters):
        eng._render_env(vc)
    dt_rb = time.perf_counter() - t1

    return {"frames": iters, "ms_per_frame": 1e3 * dt / iters,
            "fps": iters / dt if dt > 0 else float("inf"),
            "fps_readback": iters / dt_rb if dt_rb > 0 else float("inf")}
