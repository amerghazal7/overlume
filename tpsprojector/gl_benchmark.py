"""Frame-rate benchmark for the GL backends.

Renders a fixed preset repeatedly through a GL-backed Engine and reports fps.
Times only the GL render path (Engine._render_env), excluding the CPU
ground-truth rasterization and metrics that Engine.synthesize also performs, so
the number reflects GL rendering speed. Used as a manual perf check and a coarse
regression guard in tests.
"""

from __future__ import annotations

import time

from .app import Engine
from .presets import PRESET_NAMES, get_preset


def benchmark(width: int = 1280, height: int = 720, mode: str = "bowl",
              iters: int = 30) -> dict:
    eng = Engine.from_defaults(width=width, height=height, mode=mode, backend="gl")
    vc = eng.virtual_camera(get_preset(PRESET_NAMES[0]))
    eng._render_env(vc)                  # warm up (compile shaders, upload textures)
    t0 = time.perf_counter()
    for _ in range(iters):
        eng._render_env(vc)
    dt = time.perf_counter() - t0
    return {"frames": iters, "ms_per_frame": 1e3 * dt / iters,
            "fps": iters / dt if dt > 0 else float("inf")}
