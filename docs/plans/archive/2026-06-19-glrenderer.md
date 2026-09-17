# GLRenderer + GPU Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the bowl, depth, and hybrid render backends to the GPU via moderngl, satisfying the existing `Renderer` contract unchanged, with provable pixel-parity to the NumPy backends and real-time frame rates.

**Architecture:** New `gl_context.py` owns a lazy headless EGL context + size-keyed FBO pool. `GLBowlRenderer` is a backward fragment-shader 1:1 port of `NumpyRenderer`; `GLDepthRenderer` is a GPU point-splat port of `DepthRenderer`. Both return NumPy `(frame, valid)` via framebuffer readback so `app.Engine` swaps them in as drop-in fields — hybrid then works through the existing `Engine._render_env` compositing unchanged.

**Tech Stack:** Python 3.10, NumPy 1.26, moderngl (EGL backend), GLSL 330, pytest.

## Global Constraints

- Test runner: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest` (the env var is REQUIRED — a stale anyio plugin breaks pytest otherwise). `./run_tests.sh` wraps this.
- Use `python3` (no `python` on this box).
- The `Renderer` contract is fixed: `render(camera_images, cameras, surface, virtual_camera) -> (frame, valid)` with `frame` `(H,W,3)` float in [0,1]-ish and `valid` `(H,W)` bool. Do NOT change it.
- All GL tests MUST skip cleanly when `gl_available()` is False so the existing 87 tests pass on any machine.
- Camera/Pose conventions (do not deviate): camera local frame is `+Z` forward, `+X` right, `+Y` down; `Pose` maps local→reference as `p_ref = R @ p_local + t`, so `t` is the camera center and `R`'s columns are `(right, down, fwd)`. `Pose.inverse().transform_points(P) = R^T (P - t)`.
- Pixel convention (match `NumpyRenderer`): pixel `(col, row)` uses image coords `uv = (col, row)` as floats (integer indices, NOT +0.5 centers) in `camera.project`/`unproject`; row 0 is the top of the image; `frame.reshape(H, W, 3)` with row-major (y outer, x inner).
- Bowl profile (match `BowlSurface`): `height(r) = k * clamp(r - R0, 0, Rmax - R0)^2`; intersection by bisection on `g(t) = P.z - height(hypot(P.x,P.y))`, bracket `[eps=1e-6, t_max=1e4]`, **60 iterations**, `valid = (g(eps) > 0) and (g(t_max) < 0)`.
- Blend (match `blend.py`): weight `w_i = border_feather(uv_i, W_i, H_i, margin) * align_i^2`, `align_i = clamp(dot(normalize(P - C_i), fwd_i), 0, 1)`; `border_feather` = `smoothstep(d/margin)` where `d = min(min(u, (W-1)-u), min(v, (H-1)-v))` and `smoothstep(x) = clamp(x,0,1)^2*(3-2*clamp(x,0,1))`; final pixel = `sum(w_i*col_i)/sum(w_i)`, valid where `sum(w_i) > 0`.
- `NumpyRenderer` defaults: `feather_margin=30.0`, `fill_color=(0,0,0)`. `DepthRenderer` defaults: `splat_radius=1`, `fill_color=(0,0,0)`.

---

### Task 1: GL context + FBO pool (`gl_context.py`)

**Files:**
- Create: `tpsprojector/gl_context.py`
- Test: `tests/test_gl_context.py`
- Modify: `requirements.txt` (add `moderngl`)

**Interfaces:**
- Produces:
  - `gl_available() -> bool` — True iff a standalone GL context can be created.
  - `get_context() -> moderngl.Context` — lazily-created singleton standalone context.
  - `get_fbo(width: int, height: int, depth: bool = False) -> moderngl.Framebuffer` — size-keyed (and depth-keyed) cached FBO with one `RGBA32F` (`f4`) color attachment, plus a depth attachment when `depth=True`.

- [ ] **Step 1: Add the dependency**

Append to `requirements.txt`:
```
moderngl>=5.8
```
Then install: `python3 -m pip install "moderngl>=5.8"` (uses the system EGL/NVIDIA already present). If the box has no network, note it and continue — tests skip without it.

- [ ] **Step 2: Write the failing test**

Create `tests/test_gl_context.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available, get_context, get_fbo

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def test_context_is_singleton():
    assert get_context() is get_context()


def test_fbo_has_requested_size_and_is_cached():
    fbo = get_fbo(64, 48)
    assert fbo.size == (64, 48)
    assert get_fbo(64, 48) is fbo  # cached by size


def test_fbo_clear_and_read_roundtrips_rgba32f():
    fbo = get_fbo(8, 8)
    fbo.use()
    fbo.clear(0.25, 0.5, 0.75, 1.0)
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(8, 8, 4)
    assert np.allclose(raw[..., :3], [0.25, 0.5, 0.75], atol=1e-3)


def test_depth_fbo_has_depth_attachment():
    fbo = get_fbo(16, 16, depth=True)
    assert fbo.depth_attachment is not None
```

- [ ] **Step 3: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_context.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tpsprojector.gl_context'` (or all-skipped if moderngl is missing — in that case install it first).

- [ ] **Step 4: Write the implementation**

Create `tpsprojector/gl_context.py`:
```python
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
```

- [ ] **Step 5: Run test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_context.py -v`
Expected: PASS (or SKIPPED on a box without GL — acceptable).

- [ ] **Step 6: Commit**

```bash
git add tpsprojector/gl_context.py tests/test_gl_context.py requirements.txt
git commit -m "feat(gl): headless EGL context + FBO pool with skip guard"
```

---

### Task 2: GLBowlRenderer pipeline on FlatSurface

Establish the full GL bowl pipeline (texture-array upload, per-camera uniforms, full-screen draw, RGBA readback, blend) on the simplest geometry, the plane, and prove parity vs `NumpyRenderer` with `FlatSurface`.

**Files:**
- Create: `tpsprojector/gl_renderer.py`
- Test: `tests/test_gl_bowl_parity.py`

**Interfaces:**
- Consumes: `gl_context.get_context`, `gl_context.get_fbo`; `Renderer` ABC from `renderer.py`; `FlatSurface`/`BowlSurface` from `surface.py`; `PinholeCamera` (`.K`, `.pose.R`, `.pose.t`, `.width`, `.height`).
- Produces: `GLBowlRenderer(Renderer)` with `__init__(feather_margin=30.0, fill_color=(0,0,0))` and `render(camera_images, cameras, surface, virtual_camera) -> (frame, valid)` returning `((H,W,3) float64, (H,W) bool)`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_bowl_parity.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import FlatSurface
from tpsprojector.transforms import Pose
from tpsprojector.renderer import NumpyRenderer
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


def _setup(width=96, height=72):
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0,
                            width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cameras)]
    vc = PinholeCamera.from_fov(width, height, 70.0,
                                Pose(R=np.eye(3), t=[0.0, -3.0, 2.0]))
    return images, cameras, vc


def test_gl_bowl_matches_numpy_on_flat_surface():
    from tpsprojector.gl_renderer import GLBowlRenderer
    images, cameras, vc = _setup()
    surf = FlatSurface(z0=0.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    np_frame, np_valid = NumpyRenderer().render(images, cameras, surf, vc)
    assert gl_frame.shape == np_frame.shape == (vc.height, vc.width, 3)
    # masks agree on the vast majority of pixels (edge rounding aside)
    assert (gl_valid == np_valid).mean() > 0.97
    both = gl_valid & np_valid
    assert _psnr(gl_frame[both], np_frame[both]) > 40.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_bowl_parity.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tpsprojector.gl_renderer'`.

- [ ] **Step 3: Write the implementation**

Create `tpsprojector/gl_renderer.py`:
```python
"""GPU bowl reprojection: a backward fragment-shader port of NumpyRenderer.

A full-screen triangle draws one fragment per virtual pixel. Each fragment
reconstructs its world ray, intersects the proxy surface (plane closed-form or
bowl bisection), reprojects the hit point into every real camera, samples
(hardware bilinear), and blends with the exact feather + angular weights of the
NumPy backend. Output is read back to NumPy so the Renderer contract is intact.
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

uniform float out_w, out_h;
uniform vec3 vright, vdown, vfwd, vcenter;
uniform float vfx, vfy, vcx, vcy;

uniform int surf_type;            // 0 = flat, 1 = bowl
uniform float flat_z0;
uniform float bowl_R0, bowl_k, bowl_Rmax;

uniform float feather_margin;
uniform vec3 fill_color;

out vec4 frag;

float heightf(float r) {
    float d = clamp(r - bowl_R0, 0.0, bowl_Rmax - bowl_R0);
    return bowl_k * d * d;
}
float gfun(vec3 o, vec3 dir, float t) {
    vec3 P = o + t * dir;
    return P.z - heightf(length(P.xy));
}

bool intersect(vec3 o, vec3 dir, out vec3 P) {
    if (surf_type == 0) {                 // flat plane z = z0
        float dz = dir.z;
        if (abs(dz) <= 1e-12) return false;
        float t = (flat_z0 - o.z) / dz;
        if (t <= 1e-9) return false;
        P = o + t * dir;
        return true;
    }
    float eps = 1e-6, tmax = 1.0e4;       // bowl bisection (matches BowlSurface)
    if (!(gfun(o, dir, eps) > 0.0 && gfun(o, dir, tmax) < 0.0)) return false;
    float lo = eps, hi = tmax;
    for (int i = 0; i < 60; i++) {
        float mid = 0.5 * (lo + hi);
        if (gfun(o, dir, mid) > 0.0) lo = mid; else hi = mid;
    }
    P = o + (0.5 * (lo + hi)) * dir;
    return true;
}

float feather(float u, float v, float w, float h) {
    float dx = min(u, (w - 1.0) - u);
    float dy = min(v, (h - 1.0) - v);
    float d = min(dx, dy);
    if (feather_margin <= 0.0) return d >= 0.0 ? 1.0 : 0.0;
    return smoothstep(0.0, 1.0, d / feather_margin);   // == NumPy smoothstep(d/margin)
}

void main() {
    // image-space pixel coords matching NumpyRenderer (row 0 = top); framebuffer
    // is bottom-up, so we flip v here AND flip the readback array on the CPU.
    float u = gl_FragCoord.x - 0.5;
    float v = out_h - 0.5 - gl_FragCoord.y;

    vec3 dc = vec3((u - vcx) / vfx, (v - vcy) / vfy, 1.0);
    vec3 dir = normalize(vright * dc.x + vdown * dc.y + vfwd * dc.z);
    vec3 o = vcenter;

    vec3 P;
    if (!intersect(o, dir, P)) { frag = vec4(fill_color, 0.0); return; }

    vec3 acc = vec3(0.0);
    float wsum = 0.0;
    for (int i = 0; i < NCAM; i++) {
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
    }
    if (wsum > 0.0) frag = vec4(acc / wsum, 1.0);
    else frag = vec4(fill_color, 0.0);
}
"""


class GLBowlRenderer(Renderer):
    def __init__(self, feather_margin: float = 30.0, fill_color=(0.0, 0.0, 0.0)):
        self.feather_margin = float(feather_margin)
        self.fill_color = tuple(float(c) for c in fill_color)
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

    def render(self, camera_images: Sequence[np.ndarray],
               cameras: Sequence[PinholeCamera], surface,
               virtual_camera: PinholeCamera):
        from .gl_context import get_context, get_fbo
        ctx = get_context()
        n = len(cameras)
        program, vao = self._program(n)

        cam_h, cam_w = camera_images[0].shape[:2]
        stack = np.stack([np.asarray(im, dtype="f4") for im in camera_images])  # (n,H,W,3)
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

        # per-camera uniforms: R columns are (right, down, fwd); center is pose.t
        for i, cam in enumerate(cameras):
            R, t = cam.pose.R, cam.pose.t
            program[f"cright[{i}]"].value = tuple(R[:, 0])
            program[f"cdown[{i}]"].value = tuple(R[:, 1])
            program[f"cfwd[{i}]"].value = tuple(R[:, 2])
            program[f"ccenter[{i}]"].value = tuple(t)
            program[f"cfx[{i}]"].value = float(cam.K[0, 0])
            program[f"cfy[{i}]"].value = float(cam.K[1, 1])
            program[f"ccx[{i}]"].value = float(cam.K[0, 2])
            program[f"ccy[{i}]"].value = float(cam.K[1, 2])

        W, H = virtual_camera.width, virtual_camera.height
        Rv, tv = virtual_camera.pose.R, virtual_camera.pose.t
        program["out_w"].value = float(W)
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
        raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(H, W, 4)
        raw = np.flipud(raw).copy()      # framebuffer is bottom-up
        frame = raw[..., :3].astype(np.float64)
        valid = raw[..., 3] > 0.5
        frame[~valid] = self.fill_color
        return frame, valid
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_bowl_parity.py::test_gl_bowl_matches_numpy_on_flat_surface -v`
Expected: PASS (PSNR > 40 dB vs NumpyRenderer on the flat plane).

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_renderer.py tests/test_gl_bowl_parity.py
git commit -m "feat(gl): GLBowlRenderer pipeline + FlatSurface parity vs NumpyRenderer"
```

---

### Task 3: GLBowlRenderer on BowlSurface + ground-truth check

The bowl bisection path is already coded in Task 2's shader; this task proves it against `NumpyRenderer` with `BowlSurface` and against ground truth (closing spec §9.2/§9.3).

**Files:**
- Modify: `tests/test_gl_bowl_parity.py` (add two tests)

**Interfaces:**
- Consumes: `GLBowlRenderer`, `NumpyRenderer`, `BowlSurface`, `default_scene().render(vc)` (ground-truth view).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_gl_bowl_parity.py`:
```python
def test_gl_bowl_matches_numpy_on_bowl_surface():
    from tpsprojector.gl_renderer import GLBowlRenderer
    from tpsprojector.surface import BowlSurface
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    np_frame, np_valid = NumpyRenderer().render(images, cameras, surf, vc)
    assert (gl_valid == np_valid).mean() > 0.97
    both = gl_valid & np_valid
    assert _psnr(gl_frame[both], np_frame[both]) > 40.0


def test_gl_bowl_meets_ground_truth_threshold():
    from tpsprojector.gl_renderer import GLBowlRenderer
    from tpsprojector.surface import BowlSurface
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    truth, _ = default_scene().render(vc)
    # same regime as the NumPy integration test: above the floor on overlap
    assert _psnr(gl_frame[gl_valid], truth[gl_valid]) > 12.0
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_bowl_parity.py -v`
Expected: all PASS. If the bowl parity test fails on a thin ring of pixels (bisection edge), confirm the mask-agreement line is `> 0.97`; the `both`-masked PSNR should still clear 40 dB. If PSNR is low everywhere, the v-flip or a uniform is wrong — debug against `NumpyRenderer` intermediate `hits`.

- [ ] **Step 3: Commit**

```bash
git add tests/test_gl_bowl_parity.py
git commit -m "test(gl): bowl-surface parity + ground-truth threshold for GLBowlRenderer"
```

---

### Task 4: GLDepthRenderer (GPU point splatting)

**Files:**
- Create: `tpsprojector/gl_depth_renderer.py`
- Test: `tests/test_gl_depth_parity.py`

**Interfaces:**
- Consumes: `gl_context.get_context`/`get_fbo` (with `depth=True`); `CameraFrame` from `depth_renderer.py` (`.image`, `.depth`, `.camera`); `PinholeCamera.backproject(uv, depth)`.
- Produces: `GLDepthRenderer` with `__init__(splat_radius=1, fill_color=(0,0,0))` and `render(frames, virtual_camera) -> (frame, valid)`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_depth_parity.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.transforms import Pose
from tpsprojector.depth_renderer import DepthRenderer, synthetic_frames
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


def test_gl_depth_matches_numpy_depth():
    from tpsprojector.gl_depth_renderer import GLDepthRenderer
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    frames = synthetic_frames(scene, cameras)
    vc = PinholeCamera.from_fov(96, 72, 70.0, Pose(R=np.eye(3), t=[0.0, -3.0, 2.0]))

    gl_frame, gl_valid = GLDepthRenderer(splat_radius=1).render(frames, vc)
    np_frame, np_valid = DepthRenderer(splat_radius=1).render(frames, vc)
    assert gl_frame.shape == (72, 96, 3)
    # splat footprints are square in both; masks agree on most pixels
    assert (gl_valid == np_valid).mean() > 0.90
    both = gl_valid & np_valid
    assert _psnr(gl_frame[both], np_frame[both]) > 28.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_depth_parity.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tpsprojector.gl_depth_renderer'`.

- [ ] **Step 3: Write the implementation**

Create `tpsprojector/gl_depth_renderer.py`:
```python
"""GPU point-cloud splatting: a GL port of DepthRenderer.

Each finite-depth source pixel becomes one GL point. The world points are
back-projected on the CPU once (a static scene reuses the same cloud), uploaded
as a vertex buffer, and projected into the virtual camera in the vertex shader.
The hardware depth test implements nearest-wins, replacing the NumPy z-buffer.
"""

from __future__ import annotations

from typing import Sequence

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_depth_parity.py -v`
Expected: PASS. If mask agreement is below 0.90, the square point sprite vs the NumPy box footprint differ at edges — confirm `point_size = 2*r+1`. If colors are off, check the `yndc` flip and that nearest-wins (smaller z) is selected by `GL_LESS` (moderngl default depth func).

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_depth_renderer.py tests/test_gl_depth_parity.py
git commit -m "feat(gl): GLDepthRenderer point-splat port + parity vs DepthRenderer"
```

---

### Task 5: Engine GL backend swap + hybrid parity

Wire the GL backends into `Engine` as drop-in fields so all three modes (bowl/depth/hybrid) run on GPU through the existing `_render_env` compositing.

**Files:**
- Modify: `tpsprojector/app.py` (add a `backend` param to `Engine.__init__` and `from_defaults`)
- Test: `tests/test_gl_hybrid_parity.py`

**Interfaces:**
- Consumes: `GLBowlRenderer`, `GLDepthRenderer`, existing `Engine` (`.bowl_renderer`, `.depth_renderer`, `.synthesize`, `.mode`, `RENDER_MODES`).
- Produces: `Engine(..., backend="numpy"|"gl")` and `Engine.from_defaults(..., backend=...)` selecting which renderer classes populate `self.bowl_renderer` / `self.depth_renderer`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_hybrid_parity.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.app import Engine, RENDER_MODES
from tpsprojector.presets import get_preset, PRESET_NAMES

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


@pytest.mark.parametrize("mode", RENDER_MODES)
def test_gl_engine_matches_numpy_engine(mode):
    shot = get_preset(PRESET_NAMES[0])
    np_eng = Engine.from_defaults(width=96, height=72, mode=mode, backend="numpy")
    gl_eng = Engine.from_defaults(width=96, height=72, mode=mode, backend="gl")
    np_res = np_eng.synthesize(shot)
    gl_res = gl_eng.synthesize(shot)
    assert (gl_res.valid == np_res.valid).mean() > 0.90
    both = gl_res.valid & np_res.valid
    assert _psnr(gl_res.synth[both], np_res.synth[both]) > 28.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_hybrid_parity.py -v`
Expected: FAIL with `TypeError` (unexpected keyword `backend`).

- [ ] **Step 3: Edit `Engine.__init__` to accept a backend**

In `tpsprojector/app.py`, change the `__init__` signature line:
```python
    def __init__(self, scene, cameras, frames, surface, robot, fov_deg,
                 width, height, mode="hybrid"):
```
to:
```python
    def __init__(self, scene, cameras, frames, surface, robot, fov_deg,
                 width, height, mode="hybrid", backend="numpy"):
```
and replace the two renderer-construction lines:
```python
        self.bowl_renderer = NumpyRenderer()
        self.depth_renderer = DepthRenderer(splat_radius=1)
```
with:
```python
        self.backend = backend
        if backend == "gl":
            from .gl_renderer import GLBowlRenderer
            from .gl_depth_renderer import GLDepthRenderer
            self.bowl_renderer = GLBowlRenderer()
            self.depth_renderer = GLDepthRenderer(splat_radius=1)
        else:
            self.bowl_renderer = NumpyRenderer()
            self.depth_renderer = DepthRenderer(splat_radius=1)
```

- [ ] **Step 4: Edit `from_defaults` to pass the backend through**

In `tpsprojector/app.py`, change the `from_defaults` signature to add `backend="numpy"`:
```python
    def from_defaults(cls, width=320, height=240, n_cameras=6, rig_fov_deg=85.0,
                      mount_radius=0.25, mount_height=0.55, body_radius=0.5,
                      tilt_deg=None, cam_width=320, cam_height=240, mode="hybrid",
                      backend="numpy"):
```
and change the `cls(...)` construction:
```python
        eng = cls(scene, cameras, frames, surface, robot, fov_deg=70.0,
                  width=width, height=height, mode=mode)
```
to:
```python
        eng = cls(scene, cameras, frames, surface, robot, fov_deg=70.0,
                  width=width, height=height, mode=mode, backend=backend)
```

- [ ] **Step 5: Run test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_hybrid_parity.py -v`
Expected: all three modes PASS. (The NumPy backend tests with `backend="numpy"` must still match its old behavior — run the full suite next step.)

- [ ] **Step 6: Run the full suite (no regressions)**

Run: `./run_tests.sh`
Expected: all prior tests still pass; new GL tests pass or skip.

- [ ] **Step 7: Commit**

```bash
git add tpsprojector/app.py tests/test_gl_hybrid_parity.py
git commit -m "feat(gl): Engine backend swap (numpy|gl) + hybrid GL parity"
```

---

### Task 6: GPU optimization — persistent textures + benchmark

`GLBowlRenderer` already re-uploads only into a cached texture array; this task adds a benchmark harness proving real-time speed and guarding regressions.

**Files:**
- Create: `tpsprojector/gl_benchmark.py`
- Test: `tests/test_gl_benchmark.py`

**Interfaces:**
- Consumes: `Engine.from_defaults(backend="gl")`, `get_preset`.
- Produces: `benchmark(width, height, mode, iters) -> dict` with keys `fps`, `ms_per_frame`, `frames`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_benchmark.py`:
```python
import pytest

from tpsprojector.gl_context import gl_available

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


@pytest.mark.slow
def test_gl_bowl_is_realtime_at_720p():
    from tpsprojector.gl_benchmark import benchmark
    stats = benchmark(width=1280, height=720, mode="bowl", iters=30)
    assert stats["frames"] == 30
    # generous floor to catch gross regressions, not to pin exact hardware speed
    assert stats["fps"] > 30.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_benchmark.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tpsprojector.gl_benchmark'`.

- [ ] **Step 3: Write the implementation**

Create `tpsprojector/gl_benchmark.py`:
```python
"""Frame-rate benchmark for the GL backends.

Renders a fixed preset repeatedly through a GL-backed Engine and reports fps.
Used both as a manual perf check and as a coarse regression guard in tests.
"""

from __future__ import annotations

import time

from .app import Engine
from .presets import PRESET_NAMES, get_preset


def benchmark(width: int = 1280, height: int = 720, mode: str = "bowl",
              iters: int = 30) -> dict:
    eng = Engine.from_defaults(width=width, height=height, mode=mode, backend="gl")
    shot = get_preset(PRESET_NAMES[0])
    eng.synthesize(shot)                 # warm up (compile shaders, upload textures)
    t0 = time.perf_counter()
    for _ in range(iters):
        eng.synthesize(shot)
    dt = time.perf_counter() - t0
    return {"frames": iters, "ms_per_frame": 1e3 * dt / iters,
            "fps": iters / dt if dt > 0 else float("inf")}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_benchmark.py -v -m slow`
Expected: PASS. Note the printed fps; if `synthesize` dominates on CPU (ground-truth render + metrics run every frame), the renderer itself is faster than this end-to-end number — acceptable, the floor is generous.

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_benchmark.py tests/test_gl_benchmark.py
git commit -m "feat(gl): fps benchmark harness + 720p real-time regression guard"
```

---

### Task 7: Visual verification — backend toggle + comparison montage

Give the user a way to eyeball GL vs NumPy vs truth (their preferred verification), per spec §10.

**Files:**
- Modify: `tpsprojector/app.py` (add a `[g]` key toggling backend in `main`)
- Create: `scripts/gl_montage.py`

**Interfaces:**
- Consumes: `Engine.from_defaults`, `get_preset`, `PRESET_NAMES`, `validate.psnr`.
- Produces: a `gl_montage.png` with rows = preset views, cols = `NumPy | GL | TRUTH`, PSNR annotated.

- [ ] **Step 1: Add a backend-toggle key to the pygame shell**

In `tpsprojector/app.py`, inside `main`'s `KEYDOWN` handling, after the `K_h` branch add:
```python
                elif e.key == pygame.K_g:
                    new = "numpy" if eng.backend == "gl" else "gl"
                    eng = Engine.from_defaults(width=W, height=H, mode=eng.mode,
                                               backend=new)
```
and add `[g]pu-toggle` to the `keys_help` string.

- [ ] **Step 2: Write the montage script**

Create `scripts/gl_montage.py`:
```python
"""Render NumPy | GL | TRUTH montages for visual GL verification.

Usage: python3 scripts/gl_montage.py  ->  writes gl_montage.png
"""

import numpy as np

from tpsprojector.app import Engine
from tpsprojector.gl_context import gl_available
from tpsprojector.presets import PRESET_NAMES, get_preset
from tpsprojector.validate import psnr


def _to_u8(img):
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def main():
    assert gl_available(), "no GL context — cannot build GL montage"
    W, H, mode = 240, 180, "bowl"
    np_eng = Engine.from_defaults(width=W, height=H, mode=mode, backend="numpy")
    gl_eng = Engine.from_defaults(width=W, height=H, mode=mode, backend="gl")

    rows = []
    for name in PRESET_NAMES[:4]:
        shot = get_preset(name)
        npr = np_eng.synthesize(shot)
        glr = gl_eng.synthesize(shot)
        p = psnr(glr.synth, npr.synth, np.ones(glr.synth.shape[:2], bool))
        row = np.concatenate([_to_u8(npr.synth), _to_u8(glr.synth),
                              _to_u8(npr.truth)], axis=1)
        rows.append(row)
        print(f"{name:10s}  GL-vs-NumPy PSNR = {p:5.2f} dB")
    montage = np.concatenate(rows, axis=0)

    try:
        from PIL import Image
        Image.fromarray(montage).save("gl_montage.png")
    except ImportError:
        import pygame
        pygame.image.save(
            pygame.surfarray.make_surface(np.transpose(montage, (1, 0, 2))),
            "gl_montage.png")
    print("wrote gl_montage.png  (cols: NumPy | GL | TRUTH)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Run the montage script**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 scripts/gl_montage.py`
Expected: prints per-preset GL-vs-NumPy PSNR (should be high, >35 dB) and writes `gl_montage.png`. Open it and confirm the NumPy and GL columns are visually identical.

- [ ] **Step 4: Update the README roadmap**

In `README.md`, mark the GLRenderer step done in the roadmap section (change the moderngl/GL line from planned to "implemented: `GLBowlRenderer`, `GLDepthRenderer`, backend toggle"). Keep wording consistent with the existing roadmap list.

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/app.py scripts/gl_montage.py README.md
git commit -m "feat(gl): backend toggle key + NumPy|GL|TRUTH montage + README roadmap"
```

---

## Self-Review

**Spec coverage:**
- §4 architecture (gl_context, gl_renderer, gl_depth_renderer) → Tasks 1, 2/3, 4. ✓
- §4 hybrid via existing compositing (gl_hybrid.py dropped per YAGNI — `Engine._render_env` already composites) → Task 5. ✓ (Deviation from spec §4/§7 noted: no separate module; hybrid is the Engine field-swap. Cleaner and DRY.)
- §5 bowl backward shader (ray, bisection, reproject, blend) → Tasks 2, 3. ✓
- §6 depth point splatting → Task 4. ✓
- §8 GPU optimizations (persistent textures, single pass, precomputed uniforms, benchmark) → Tasks 2 (cached texture array, CPU-precomputed uniforms) + 6 (benchmark). ✓ No-readback live path is explicitly optional in §8 and left out of this plan (the contract requires readback); the backend toggle in Task 7 still gives the live visual.
- §9 testing (context gate, bowl parity, bowl-vs-truth, depth parity, hybrid parity, benchmark, all skip-guarded) → Tasks 1–6. ✓
- §10 visual verification (toggle + montage) → Task 7. ✓
- §11 file plan → matches, minus `gl_hybrid.py` (intentional). ✓

**Placeholder scan:** No TBD/TODO; all code blocks complete. ✓

**Type consistency:** `render(...) -> (frame float64 (H,W,3), valid bool (H,W))` consistent across `GLBowlRenderer`, `GLDepthRenderer`, and the NumPy backends they mirror. `Engine(..., backend=...)` and `from_defaults(..., backend=...)` names match across Tasks 5/6/7. R-column convention `(R[:,0],R[:,1],R[:,2]) = (right,down,fwd)` used identically in both shaders. `gl_available`/`get_context`/`get_fbo` signatures consistent across all consumers. ✓

**Known tolerances (intentional, not bugs):** bowl parity PSNR>40 / mask>0.97; depth parity PSNR>28 / mask>0.90; engine parity PSNR>28 / mask>0.90 — looser for depth/hybrid because GL square point sprites vs NumPy box footprints and depth-buffer quantization differ at splat edges.
