# GPU Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the GL backend fast — eliminate per-frame texture re-upload (upload-once), add a no-readback offscreen render path proven by benchmark to exceed 60 fps @720p, and display the cinematic view live through an OpenGL pygame window with no per-frame readback.

**Architecture:** Refactor each GL renderer so the GPU work lives in `_render_to_fbo(...) -> fbo`; `render()` = `_render_to_fbo` + readback (contract unchanged). Add identity-based upload-once caching. A new `gl_present.py` blits a texture/FBO/array fullscreen. `app.main()` runs an OpenGL window, binding the GL backend to that window's context so FBOs can be presented directly.

**Tech Stack:** Python 3.10, NumPy, moderngl 5.12 (EGL/standalone for tests, window context for app), pygame, GLSL 330, pytest.

## Global Constraints

- Test runner: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest` (env var REQUIRED). Use `python3` (no `python`).
- The `Renderer` contract is fixed: `GLBowlRenderer.render(camera_images, cameras, surface, virtual_camera) -> (frame (H,W,3) float64, valid (H,W) bool)`; `GLDepthRenderer.render(frames, virtual_camera) -> (frame, valid)`. Refactors must NOT change `render()`'s observable output — the existing parity tests (`tests/test_gl_bowl_parity.py`, `tests/test_gl_depth_parity.py`, `tests/test_gl_hybrid_parity.py`) must stay green.
- All GL tests skip cleanly when `gl_available()` is False (`pytestmark = pytest.mark.skipif(not gl_available(), reason=...)`).
- Shaders' numeric behavior must not change (parity to NumPy preserved): bowl bisection 60 iters bracket [1e-6,1e4]; blend `feather*align^2` normalized; v-flip in shader + `np.flipud` on readback.
- Upload-once is identity-based (cache `id()` of inputs); document that in-place mutation needs `invalidate()`.
- No-readback benchmark MUST call `ctx.finish()` per iteration (GL is async; otherwise the timer is meaningless).
- Benchmark acceptance: no-readback bowl @1280×720 → `fps > 60`.
- moderngl: array uniforms are assigned as a whole Python list (not per-element); a declared-but-unused uniform is optimized out and raises `KeyError` on access — only touch uniforms the shader uses.

---

### Task 1: GLBowlRenderer — extract `_render_to_fbo` (pure refactor)

Split the GPU work out of `render()` so it can be reused without readback. No behavior change.

**Files:**
- Modify: `tpsprojector/gl_renderer.py` (replace the `render` method)
- Test: `tests/test_gl_render_to_fbo.py` (new)

**Interfaces:**
- Consumes: `gl_context.get_context`/`get_fbo`; existing module-level `_VERT`/`_FRAG`, `self._program`, `self._cam_tex`.
- Produces: `GLBowlRenderer._render_to_fbo(camera_images, cameras, surface, virtual_camera) -> moderngl.Framebuffer` (draws into the pooled FBO, no readback, returns it). `render(...)` unchanged in signature/output, now implemented via `_render_to_fbo` + readback.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_render_to_fbo.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _setup(width=96, height=72):
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cameras)]
    vc = PinholeCamera.from_fov(width, height, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    return images, cameras, vc


def test_render_to_fbo_readback_matches_render():
    from tpsprojector.gl_renderer import GLBowlRenderer
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl = GLBowlRenderer()
    frame, valid = gl.render(images, cameras, surf, vc)

    fbo = gl._render_to_fbo(images, cameras, surf, vc)
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(vc.height, vc.width, 4)
    raw = np.flipud(raw).copy()
    fbo_frame = raw[..., :3].astype(np.float64)
    fbo_valid = raw[..., 3] > 0.5
    fbo_frame[~fbo_valid] = gl.fill_color

    assert np.array_equal(fbo_valid, valid)
    assert np.allclose(fbo_frame, frame, atol=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_render_to_fbo.py -v`
Expected: FAIL with `AttributeError: 'GLBowlRenderer' object has no attribute '_render_to_fbo'`.

- [ ] **Step 3: Refactor — replace the entire `render` method**

In `tpsprojector/gl_renderer.py`, replace the whole `def render(self, camera_images, ...): ... return frame, valid` method (currently the last method of the class) with these TWO methods (the body is the same code, just split at the draw/readback boundary):
```python
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
```

- [ ] **Step 4: Run tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_render_to_fbo.py tests/test_gl_bowl_parity.py -v`
Expected: all PASS (refactor preserves output; parity unchanged).

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_renderer.py tests/test_gl_render_to_fbo.py
git commit -m "refactor(gl): extract GLBowlRenderer._render_to_fbo (no-readback render path)"
```

---

### Task 2: GLBowlRenderer — upload-once

Skip the per-frame texture write and per-camera uniform set when inputs are unchanged.

**Files:**
- Modify: `tpsprojector/gl_renderer.py` (`__init__`, `_render_to_fbo`, add `invalidate`)
- Test: `tests/test_gl_upload_once.py` (new)

**Interfaces:**
- Consumes: `_render_to_fbo` from Task 1.
- Produces: `GLBowlRenderer.invalidate() -> None` (forces re-upload next render). New cache fields `self._img_sig`, `self._cam_sig`, `self._uniforms_set`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_upload_once.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _setup(width=96, height=72):
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cameras)]
    vc = PinholeCamera.from_fov(width, height, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    return images, cameras, vc


def test_bowl_uploads_textures_only_once():
    from tpsprojector.gl_renderer import GLBowlRenderer
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl = GLBowlRenderer()
    first, _ = gl.render(images, cameras, surf, vc)   # allocates + uploads once

    writes = []
    real_write = gl._cam_tex.write
    gl._cam_tex.write = lambda data, *a, **k: (writes.append(1), real_write(data, *a, **k))[1]

    again, _ = gl.render(images, cameras, surf, vc)    # same images -> no re-upload
    assert writes == []
    assert np.allclose(again, first)                   # output still correct

    gl.invalidate()
    gl.render(images, cameras, surf, vc)               # forced re-upload
    assert len(writes) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_upload_once.py -v`
Expected: FAIL — `writes` will contain a `1` (texture re-uploaded every frame), so `assert writes == []` fails.

- [ ] **Step 3: Add cache fields + `invalidate` to `__init__`**

In `tpsprojector/gl_renderer.py`, replace the `__init__` body's cache lines:
```python
        # Cached for the lifetime of the singleton GL context (see gl_context.get_context).
        self._progs = {}        # ncam -> (program, vao)
        self._cam_tex = None    # (ncam, H, W) cached sampler2DArray
```
with:
```python
        # Cached for the lifetime of the singleton GL context (see gl_context.get_context).
        self._progs = {}        # ncam -> (program, vao)
        self._cam_tex = None    # (ncam, H, W) cached sampler2DArray
        # Upload-once caches: re-upload only when inputs change (by identity).
        self._img_sig = None    # tuple(id(img)) of last-uploaded camera images
        self._cam_sig = None    # tuple(id(cam)) of last camera set
        self._uniforms_set = set()  # ncam values whose per-cam uniforms are current
```
Then add this method to the class (e.g. right after `__init__`):
```python
    def invalidate(self):
        """Force re-upload of camera textures and per-camera uniforms next render.

        Call after mutating an input image/camera in place (identity-based
        caching cannot detect in-place changes).
        """
        self._img_sig = None
        self._cam_sig = None
        self._uniforms_set = set()
```

- [ ] **Step 4: Make `_render_to_fbo` skip redundant uploads**

In `_render_to_fbo`, replace the texture block:
```python
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
```
with:
```python
        cam_h, cam_w = camera_images[0].shape[:2]
        if self._cam_tex is None or self._cam_tex.size != (cam_w, cam_h) \
                or self._cam_tex.layers != n:
            if self._cam_tex is not None:
                self._cam_tex.release()
            self._cam_tex = ctx.texture_array((cam_w, cam_h, n), 3, dtype="f4")
            self._cam_tex.filter = (9729, 9729)             # GL_LINEAR, GL_LINEAR
            self._cam_tex.repeat_x = self._cam_tex.repeat_y = False  # CLAMP_TO_EDGE
            self._img_sig = None    # new allocation: force the upload below
        # Upload pixel data only when the image set changed (by identity).
        img_sig = tuple(id(im) for im in camera_images)
        if img_sig != self._img_sig:
            stack = np.stack([np.asarray(im, dtype="f4") for im in camera_images])
            self._cam_tex.write(stack.tobytes())
            self._img_sig = img_sig
        self._cam_tex.use(0)
```
Then wrap the per-camera uniform block. Replace:
```python
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
```
with:
```python
        # Per-camera uniforms live on the program; set them only when the camera
        # set changes (by identity) or this program has not been populated yet.
        cam_sig = tuple(id(c) for c in cameras)
        if cam_sig != self._cam_sig:
            self._cam_sig = cam_sig
            self._uniforms_set = set()
        if n not in self._uniforms_set:
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
            self._uniforms_set.add(n)
```
(Note: `program["cam_w"]/["cam_h"]/["cams"]` and all virtual-camera/surface uniforms stay set every call — they are cheap and the virtual camera changes per frame.)

- [ ] **Step 5: Run tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_upload_once.py tests/test_gl_bowl_parity.py tests/test_gl_render_to_fbo.py -v`
Expected: all PASS (1 upload across repeated identical renders; parity intact).

- [ ] **Step 6: Commit**

```bash
git add tpsprojector/gl_renderer.py tests/test_gl_upload_once.py
git commit -m "perf(gl): upload-once camera textures + per-camera uniforms in GLBowlRenderer"
```

---

### Task 3: GLDepthRenderer — `_render_to_fbo` + point-cloud cache

**Files:**
- Modify: `tpsprojector/gl_depth_renderer.py`
- Test: extend `tests/test_gl_render_to_fbo.py` and `tests/test_gl_upload_once.py`

**Interfaces:**
- Produces: `GLDepthRenderer._render_to_fbo(frames, virtual_camera) -> moderngl.Framebuffer`; `GLDepthRenderer.invalidate()`. Cache fields `self._cloud_sig`, `self._vbo_pos`, `self._vbo_col`, `self._vao`, `self._n_pts`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_gl_render_to_fbo.py`:
```python
def test_depth_render_to_fbo_readback_matches_render():
    from tpsprojector.gl_depth_renderer import GLDepthRenderer
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    frames = synthetic_frames(scene, cameras)
    vc = PinholeCamera.from_fov(96, 72, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    gl = GLDepthRenderer(splat_radius=1)
    frame, valid = gl.render(frames, vc)
    fbo = gl._render_to_fbo(frames, vc)
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(72, 96, 4)
    raw = np.flipud(raw).copy()
    fb_frame = raw[..., :3].astype(np.float64); fb_valid = raw[..., 3] > 0.5
    fb_frame[~fb_valid] = gl.fill_color
    assert np.array_equal(fb_valid, valid)
    assert np.allclose(fb_frame, frame, atol=1e-6)
```
Append to `tests/test_gl_upload_once.py`:
```python
def test_depth_rebuilds_cloud_only_when_frames_change():
    from tpsprojector.gl_depth_renderer import GLDepthRenderer
    from tpsprojector.world.scene import default_scene
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    frames = synthetic_frames(scene, cameras)
    vc = PinholeCamera.from_fov(96, 72, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    gl = GLDepthRenderer(splat_radius=1)

    builds = []
    real_build = gl._point_cloud
    gl._point_cloud = lambda fr: (builds.append(1), real_build(fr))[1]

    gl.render(frames, vc)            # first build
    gl.render(frames, vc)            # same frames -> no rebuild
    assert len(builds) == 1
    gl.invalidate()
    gl.render(frames, vc)            # forced rebuild
    assert len(builds) == 2
```

- [ ] **Step 2: Run to verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_render_to_fbo.py::test_depth_render_to_fbo_readback_matches_render tests/test_gl_upload_once.py::test_depth_rebuilds_cloud_only_when_frames_change -v`
Expected: FAIL — `_render_to_fbo` missing; cloud rebuilt every call (`builds` length 3).

- [ ] **Step 3: Rewrite `GLDepthRenderer`**

In `tpsprojector/gl_depth_renderer.py`, replace `__init__` and `render` (keep `_point_cloud` and the shaders as-is) with:
```python
    def __init__(self, splat_radius: int = 1, fill_color=(0.0, 0.0, 0.0)):
        self.splat_radius = int(splat_radius)
        self.fill_color = tuple(float(c) for c in fill_color)
        # Cached for the lifetime of the singleton GL context (see gl_context.get_context).
        self._prog = None
        # Upload-once cache: rebuild the cloud/VBOs only when frames change.
        self._cloud_sig = None
        self._vbo_pos = self._vbo_col = self._vao = None
        self._n_pts = 0

    def invalidate(self):
        """Force a point-cloud rebuild + re-upload on the next render."""
        self._cloud_sig = None

    def _ensure_cloud(self, frames):
        """(Re)build and upload the point cloud when ``frames`` changed by identity."""
        from .gl_context import get_context
        sig = tuple(id(f) for f in frames)
        if sig == self._cloud_sig:
            return
        ctx = get_context()
        if self._prog is None:
            self._prog = ctx.program(vertex_shader=_VERT, fragment_shader=_FRAG)
        P, C = self._point_cloud(frames)
        for buf in (self._vao, self._vbo_pos, self._vbo_col):
            if buf is not None:
                buf.release()
        self._vao = self._vbo_pos = self._vbo_col = None
        self._n_pts = int(P.shape[0])
        if self._n_pts:
            self._vbo_pos = ctx.buffer(P.tobytes())
            self._vbo_col = ctx.buffer(C.tobytes())
            self._vao = ctx.vertex_array(self._prog,
                                         [(self._vbo_pos, "3f", "in_pos"),
                                          (self._vbo_col, "3f", "in_col")])
        self._cloud_sig = sig

    def _render_to_fbo(self, frames, virtual_camera):
        from .gl_context import get_context, get_fbo
        import moderngl
        ctx = get_context()
        self._ensure_cloud(frames)
        W, H = virtual_camera.width, virtual_camera.height
        fbo = get_fbo(W, H, depth=True)
        fbo.use()
        fbo.clear(*self.fill_color, 0.0)
        if self._n_pts == 0:
            return fbo
        prog = self._prog
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
        ctx.enable(moderngl.DEPTH_TEST | moderngl.PROGRAM_POINT_SIZE)
        self._vao.render(mode=0)          # 0 = GL_POINTS
        ctx.disable(moderngl.DEPTH_TEST | moderngl.PROGRAM_POINT_SIZE)
        return fbo

    def render(self, frames, virtual_camera):
        W, H = virtual_camera.width, virtual_camera.height
        fbo = self._render_to_fbo(frames, virtual_camera)
        raw = np.frombuffer(fbo.read(components=4, dtype="f4"), "f4").reshape(H, W, 4)
        raw = np.flipud(raw).copy()
        frame = raw[..., :3].astype(np.float64)
        valid = raw[..., 3] > 0.5
        frame[~valid] = self.fill_color
        return frame, valid
```

- [ ] **Step 4: Run tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_render_to_fbo.py tests/test_gl_upload_once.py tests/test_gl_depth_parity.py tests/test_gl_hybrid_parity.py -v`
Expected: all PASS (depth parity + hybrid parity intact; cloud built once per unchanged frames).

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_depth_renderer.py tests/test_gl_render_to_fbo.py tests/test_gl_upload_once.py
git commit -m "perf(gl): GLDepthRenderer _render_to_fbo + point-cloud upload-once cache"
```

---

### Task 4: `gl_present.py` — fullscreen blit (FBO / array)

**Files:**
- Create: `tpsprojector/gl_present.py`
- Test: `tests/test_gl_present.py` (new)

**Interfaces:**
- Produces:
  - `present_fbo(fbo, target=None, blend=False)` — blit `fbo`'s color texture to `target` (default = `get_context().screen`), 1:1 orientation.
  - `present_array(arr, target=None, blend=False)` — upload NumPy `(H,W,3|4)` float array (row 0 = top) to a cached texture and blit it upright (vertically flipped so row 0 shows at the target's top).
  - `present_texture(tex, target=None, blend=False, flip=False)` — core blit.

- [ ] **Step 1: Write the failing test**

Create `tests/test_gl_present.py`:
```python
import numpy as np
import pytest

from tpsprojector.gl_context import gl_available, get_fbo, get_context

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _read(fbo, W, H):
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(H, W, 4)
    return raw


def test_present_array_roundtrips_upright():
    from tpsprojector.gl_present import present_array
    W, H = 16, 12
    # distinct per-row values so an accidental flip is caught
    arr = np.zeros((H, W, 3), dtype="f4")
    arr[:, :, 0] = (np.arange(H)[:, None] / H)      # red ramps top->bottom
    arr[:, :, 1] = 0.5
    target = get_fbo(W, H)
    target.use(); target.clear(0, 0, 0, 1)
    present_array(arr, target=target)
    out = np.flipud(_read(target, W, H)).copy()[..., :3]   # flipud -> row 0 = top
    assert np.allclose(out, arr, atol=1e-3)


def test_present_fbo_copies_1to1():
    from tpsprojector.gl_present import present_array, present_fbo
    W, H = 16, 12
    src = get_fbo(W, H)
    src.use(); src.clear(0, 0, 0, 1)
    arr = np.zeros((H, W, 3), dtype="f4"); arr[:, :, 2] = 0.7
    present_array(arr, target=src)                  # put a known image into src
    src_raw = _read(src, W, H).copy()

    dst = get_fbo(W, H)
    dst.use(); dst.clear(0, 0, 0, 1)
    present_fbo(src, target=dst)
    assert np.allclose(_read(dst, W, H), src_raw, atol=1e-3)


def test_present_array_alpha_blends_over_target():
    from tpsprojector.gl_present import present_array
    W, H = 8, 8
    target = get_fbo(W, H)
    target.use(); target.clear(0.0, 0.0, 0.0, 1.0)  # black background
    rgba = np.zeros((H, W, 4), dtype="f4")
    rgba[:, :, 0] = 1.0                              # red
    rgba[:, :, 3] = 0.5                              # half alpha
    present_array(rgba, target=target, blend=True)
    out = _read(target, W, H)
    assert np.allclose(out[..., 0], 0.5, atol=2e-2)  # 0.5*red over black
```

- [ ] **Step 2: Run to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_present.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tpsprojector.gl_present'`.

- [ ] **Step 3: Implement `gl_present.py`**

Create `tpsprojector/gl_present.py`:
```python
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
```

- [ ] **Step 4: Run tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_present.py -v`
Expected: all PASS (upright round-trip, 1:1 copy, alpha blend).

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_present.py tests/test_gl_present.py
git commit -m "feat(gl): gl_present — fullscreen FBO/array blit with flip + alpha blend"
```

---

### Task 5: Benchmark the no-readback path (prove > 60 fps @720p)

**Files:**
- Modify: `tpsprojector/gl_benchmark.py`
- Modify: `tests/test_gl_benchmark.py`

**Interfaces:**
- Produces: `benchmark(width, height, mode, iters) -> dict` now returns keys `frames`, `fps` (no-readback bowl render + `ctx.finish()`), `ms_per_frame`, and `fps_readback` (the end-to-end `_render_env` for `mode`).

- [ ] **Step 1: Update the test**

Replace the body of `tests/test_gl_benchmark.py`'s slow test with:
```python
@pytest.mark.slow
def test_gl_bowl_no_readback_is_realtime_at_720p():
    from tpsprojector.gl_benchmark import benchmark
    stats = benchmark(width=1280, height=720, mode="bowl", iters=30)
    assert stats["frames"] == 30
    assert "fps_readback" in stats
    assert stats["fps"] > 60.0          # no-readback GPU render clears the spec target
```
(Keep the module-level `pytestmark = pytest.mark.skipif(not gl_available(), ...)` and imports.)

- [ ] **Step 2: Run to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_benchmark.py -v -m slow`
Expected: FAIL — `benchmark` does not yet return `fps_readback`, and `fps` currently includes readback (~38, below 60).

- [ ] **Step 3: Rewrite `benchmark`**

Replace `benchmark` in `tpsprojector/gl_benchmark.py` with:
```python
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
```
Also update the module docstring's second paragraph to describe both metrics.

- [ ] **Step 4: Run the test (note both numbers)**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_benchmark.py -v -m slow -s`
Expected: PASS — `fps` (no-readback) > 60 (likely far higher). Record both `fps` and `fps_readback` for the README.

- [ ] **Step 5: Commit**

```bash
git add tpsprojector/gl_benchmark.py tests/test_gl_benchmark.py
git commit -m "perf(gl): benchmark no-readback render path (assert >60 fps @720p) + report readback delta"
```

---

### Task 6: Live OpenGL window (manual-verify) + README

Wire the interactive app to an OpenGL surface and present the cinematic view with no readback. `main()` is `# pragma: no cover` — verified by running, not unit tests. The building blocks it uses (Tasks 1–4) are all tested.

**Files:**
- Modify: `tpsprojector/gl_context.py` (add `use_window_context`)
- Modify: `tpsprojector/app.py` (`main()`)
- Modify: `README.md`
- Test: `tests/test_gl_context.py` (add a trivial assertion that `use_window_context` exists / is callable-guarded)

**Interfaces:**
- Consumes: `gl_present.present_fbo`/`present_array`; `GLBowlRenderer._render_to_fbo`; `Engine` fields `images`, `cameras`, `surface`, `bowl_renderer`, `robot`, `sky_color`, `synthesize`, `virtual_camera`.
- Produces: `gl_context.use_window_context() -> moderngl.Context` (binds the GL backend to the CURRENT GL context, e.g. a pygame OpenGL window, so its FBOs can be presented to the window).

- [ ] **Step 1: Add `use_window_context` (+ a guard test)**

In `tpsprojector/gl_context.py`, add after `get_context`:
```python
def use_window_context():
    """Bind the GL backend to the CURRENT GL context (e.g. a pygame OpenGL window).

    Standalone-context mode (the default) renders headless; the live app instead
    needs the renderers' FBOs in the window's own context so they can be blitted
    to the screen. Call this once after creating the OpenGL window, before any
    GL renderer is used.
    """
    global _ctx
    import moderngl
    _ctx = moderngl.create_context()
    return _ctx
```
Add to `tests/test_gl_context.py`:
```python
def test_use_window_context_is_exposed():
    import tpsprojector.gl_context as gc
    assert hasattr(gc, "use_window_context") and callable(gc.use_window_context)
```
(We do not call it in tests — it requires a current window GL context. It is exercised manually by running the app.)

- [ ] **Step 2: Verify the guard test passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gl_context.py -v`
Expected: PASS (existing 4 tests + the new existence check).

- [ ] **Step 3: Rewrite `main()` for an OpenGL window**

Replace the `main()` function in `tpsprojector/app.py` with the following (it keeps preset/orbit/backend-toggle behavior; the display path is now GL). Key points: create an OpenGL window, bind the GL backend to it, present the cinematic FBO directly for the bowl fast path, fall back to `present_array` of the readback result otherwise, and show metrics in the window caption.
```python
def main():  # pragma: no cover
    import pygame
    from . import gl_present
    from .gl_context import use_window_context

    W, H = 960, 720
    pygame.init()
    pygame.display.set_mode((W, H), pygame.OPENGL | pygame.DOUBLEBUF)
    pygame.display.set_caption("TPSProjector — GL live")
    use_window_context()                       # GL backend renders into this window's context
    clock = pygame.time.Clock()

    eng = Engine.from_defaults(width=W, height=H, backend="gl")
    ctx = None
    from .gl_context import get_context
    ctx = get_context()

    cur = get_preset(PRESET_NAMES[0]); src = dst = cur; t = 1.0
    show_validation = False
    orbit = False
    az, el, dist = np.radians(180.0), np.radians(28.0), 4.5

    def orbit_shot():
        ex = dist * np.cos(el) * np.cos(az)
        ey = dist * np.cos(el) * np.sin(az)
        ez = dist * np.sin(el) + 0.5
        return Shot(eye=[ex, ey, ez], target=[0.0, 0.0, 0.3])

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_v:
                    show_validation = not show_validation
                elif e.key == pygame.K_o:
                    orbit = not orbit
                elif e.key == pygame.K_b:
                    eng.mode = "bowl"
                elif e.key == pygame.K_d:
                    eng.mode = "depth"
                elif e.key == pygame.K_h:
                    eng.mode = "hybrid"
                elif e.key == pygame.K_g:
                    new = "numpy" if eng.backend == "gl" else "gl"
                    eng = Engine.from_defaults(width=W, height=H, mode=eng.mode, backend=new)
                elif pygame.K_1 <= e.key <= pygame.K_9:
                    idx = e.key - pygame.K_1
                    if idx < len(PRESET_NAMES):
                        src, dst, t, orbit = cur, get_preset(PRESET_NAMES[idx]), 0.0, False

        keys = pygame.key.get_pressed()
        if orbit:
            if keys[pygame.K_LEFT]:  az -= 0.04
            if keys[pygame.K_RIGHT]: az += 0.04
            if keys[pygame.K_UP]:    el = min(el + 0.03, np.radians(85))
            if keys[pygame.K_DOWN]:  el = max(el - 0.03, np.radians(5))
            if keys[pygame.K_EQUALS]: dist = max(dist - 0.1, 1.5)
            if keys[pygame.K_MINUS]:  dist += 0.1
            cur = orbit_shot()
        else:
            if t < 1.0:
                t = min(1.0, t + 0.05); cur = tween(src, dst, t)
            else:
                cur = dst

        vc = eng.virtual_camera(cur)
        hud_extra = ""
        fast = (eng.backend == "gl" and eng.mode == "bowl" and not show_validation)
        if fast:
            # No-readback path: env FBO over a sky clear, robot composited on GPU.
            env_fbo = eng.bowl_renderer._render_to_fbo(eng.images, eng.cameras,
                                                       eng.surface, vc)
            robot_rgb, robot_depth = eng.robot.render(vc)
            alpha = np.isfinite(robot_depth).astype("f4")
            robot_rgba = np.dstack([np.clip(robot_rgb, 0, 1).astype("f4"), alpha])
            ctx.screen.use()
            ctx.clear(*eng.sky_color, 1.0)
            gl_present.present_fbo(env_fbo, blend=True)        # env where valid, else sky
            gl_present.present_array(robot_rgba, blend=True)   # robot over env
        else:
            # Readback path: full synthesize (metrics available), present the frame.
            res = eng.synthesize(cur)
            gl_present.present_array(np.clip(res.synth, 0, 1))
            hud_extra = f" PSNR={res.psnr:5.2f} SSIM={res.ssim:4.2f}"

        pygame.display.flip()
        clock.tick(0)   # uncapped, to see real fps
        pygame.display.set_caption(
            f"TPSProjector — mode={eng.mode} backend={eng.backend} "
            f"{'ORBIT' if orbit else 'preset'} fps={clock.get_fps():4.1f}{hud_extra}  "
            f"[1-4]preset [b/d/h]mode [g]pu [v]alidation [o]rbit [esc]")

    pygame.quit()
```

- [ ] **Step 4: Manually verify the live app**

Run: `python3 -m tpsprojector.app`
Confirm: window opens; the cinematic bowl view renders; the caption fps is high (well above the old 38; orders of magnitude expected); `b/d/h` switch modes (depth/hybrid use the readback path), `v` toggles the validation/metrics readback path, `g` toggles numpy/gl, presets `1-4` tween, `o`+arrows orbit, `esc` quits. (If running over SSH without a display, note that this step needs the local display `:1`.)

- [ ] **Step 5: Update the README**

In `README.md`, update the GLRenderer roadmap item / add a short note: the GL optimizations are implemented — upload-once (camera textures + per-camera uniforms; point cloud) and a no-readback render path; the live app runs on an OpenGL window presenting the FBO directly. Quote the measured no-readback fps and the readback fps from Task 5 (e.g. "720p bowl: <NR> fps no-readback vs <RB> fps with readback"). Match the existing roadmap style.

- [ ] **Step 6: Commit**

```bash
git add tpsprojector/gl_context.py tpsprojector/app.py tests/test_gl_context.py README.md
git commit -m "feat(gl): live OpenGL window with no-readback present (+ window-context binding)"
```

---

## Self-Review

**Spec coverage:**
- §3.1 upload-once (bowl textures+uniforms; depth cloud) → Tasks 2, 3. ✓
- §3.2 no-readback `_render_to_fbo` (bowl, depth) → Tasks 1, 3. ✓
- §3.3 live OpenGL window + `gl_present` (present_fbo/present_array) → Tasks 4, 6. ✓
- §3.4 robot composited on GPU (small RGBA layer + alpha blend in present) → Task 6 fast path (`present_array(robot_rgba, blend=True)`). ✓
- §3.5 benchmark reports both fps; assert >60 no-readback → Task 5. ✓
- §4 testing (upload-once spies, _render_to_fbo parity, gl_present round-trip, benchmark, regression via existing parity suites, all skip-guarded) → Tasks 1–5. ✓
- §5 file plan → matches (gl_present.py new; renderers/benchmark/app/context modified; 3 new test files). ✓
- Window-context sharing (renderers' standalone context vs pygame window context) → `use_window_context` in Task 6 (an addition beyond the spec text, required for correctness; flagged). ✓

**Placeholder scan:** No TBD/TODO; all steps carry complete code. The README fps numbers (Task 5 → Task 6) are real measured values the implementer records, not placeholders.

**Type consistency:** `_render_to_fbo` returns a `moderngl.Framebuffer` in both renderers; `render()` signatures/outputs unchanged; `invalidate()` present on both renderers; `present_fbo`/`present_array`/`present_texture` signatures consistent across Task 4 and their Task 6 callers; `use_window_context` name consistent (Task 6 step 1 defines, step 3 imports). `benchmark` returns `frames/fps/ms_per_frame/fps_readback` consistently (Task 5 test + impl).

**Known intentional choices (not defects):** upload-once is identity-based (documented caveat + `invalidate()`); `main()` is manual-verify (`# pragma: no cover`) so its display wiring has no unit test — its components are all tested. The no-readback benchmark uses `ctx.finish()` for honest timing.
