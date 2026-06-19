# GLRenderer + GPU Optimization — Design Spec

**Date:** 2026-06-19
**Status:** Approved design, pre-implementation
**Author:** brainstorming session
**Builds on:** `2026-06-19-tpsprojector-design.md` (roadmap §10.2: "GLRenderer
(moderngl) — same interface, real-time")

## 1. Summary

Port the validated NumPy reprojection backends to the GPU via **moderngl**,
satisfying the existing `Renderer` contract unchanged so `app.py`, the validation
panel, and tests keep working. All three render modes are ported:

- **bowl** — backward per-fragment reprojection (1:1 port of `NumpyRenderer`),
- **depth** — forward GPU point-cloud splatting (port of `DepthRenderer`),
- **hybrid** — depth-where-valid composited over bowl (port of current hybrid).

Goal: real-time synthesis (target 720p bowl > 60 fps on the on-box NVIDIA GPU)
while remaining **pixel-faithful** to the NumPy backends, proved by parity tests.

## 2. Goals & Non-Goals

### Goals
- Implement `GLBowlRenderer(Renderer)` with the same
  `render(camera_images, cameras, surface, virtual_camera) -> (frame, valid)`
  contract; output NumPy arrays identical (to tolerance) to `NumpyRenderer`.
- Implement `GLDepthRenderer` matching `DepthRenderer.render(frames, virtual_camera)`.
- Implement GL hybrid compositing matching current behavior.
- Run headless (offscreen EGL FBO) so the renderer works in tests and on servers.
- Achieve real-time frame rates; provide a benchmark harness.
- Add a NumPy↔GL backend toggle + comparison montage for visual verification.

### Non-Goals (YAGNI for this phase)
- Arbitrary `Surface` support on GPU — only `BowlSurface` and `FlatSurface` are
  analytic in the shader; other surfaces raise `NotImplementedError`.
- Photometric harmonization, temporal smoothing, lens distortion (already future).
- The C++/CUDA backend (the next roadmap step after this one).
- Multi-GPU, FP16 paths (noted as optional later optimizations only).

## 3. Environment (verified on this box)

- EGL present: `libEGL_mesa`, `libEGL_nvidia` (driver 580.159.03).
- GPU device nodes `/dev/dri/card1`, `/dev/dri/renderD128`; `DISPLAY=:1` also set.
- System `python3.10`, `numpy 1.26.4`. `moderngl` to be added to `requirements.txt`.
- Headless standalone EGL context is therefore feasible; tests still must guard
  with skip-if-unavailable so the suite runs anywhere.

## 4. Architecture

New modules mirror the NumPy ones. The `Renderer`/render-call contracts are
unchanged — `render()` returns the same `((H,W,3) float, (H,W) bool)` tuple via
framebuffer readback.

| New module | Mirrors | Responsibility |
|---|---|---|
| `gl_context.py` | — | Lazy headless EGL context; size-keyed FBO pool; skip detection |
| `gl_renderer.py` (`GLBowlRenderer`) | `renderer.py` | Backward fragment-shader bowl reprojection |
| `gl_depth_renderer.py` (`GLDepthRenderer`) | `depth_renderer.py` | Forward point-splat depth rendering |
| `gl_hybrid.py` | current hybrid composite | Composite depth over bowl by validity |

Shaders are **inline triple-quoted GLSL strings** in their modules
(self-contained, matches the one-purpose-per-module style).

### 4.1 Context strategy (`gl_context.py`)
- `get_context()` returns a lazily-created singleton
  `moderngl.create_standalone_context()` (EGL backend) for headless use; if a
  GL window/context already exists it may attach instead (for the app's live
  blit path later).
- A small FBO pool keyed by `(W, H)` provides offscreen render targets sized to
  the virtual camera, with one color attachment (RGBA32F: RGB = color,
  A = coverage/validity) and, for the depth path, a depth attachment.
- `gl_available()` probes context creation once and is used by tests to skip.

## 5. Bowl mode — backward fragment shader (`GLBowlRenderer`)

A full-screen triangle draws one fragment per virtual pixel. Per fragment:

1. **Ray:** reconstruct the world ray from `gl_FragCoord` using the virtual
   camera's inverse `K` and its pose `R|t`, both passed as uniforms (computed
   once on CPU per frame — no per-fragment matrix inversion).
2. **Surface intersect:** port the exact `BowlSurface` bisection —
   `height(r) = k * clamp(r - R0, 0, Rmax - R0)^2`, `g(t) = Pz - height(r)`,
   bracket `[eps, t_max]`, **60 iterations**, `valid = (g_lo > 0) && (g_hi < 0)`.
   `FlatSurface` uses the closed-form plane intersect. This makes GL output match
   NumPy to floating-point tolerance.
3. **Reproject + sample (loop N cameras):** world→camera via `pose^-1`, reject if
   `z_cam <= 1e-9`, project with `K_i`, reject if out of `[0, W-1]×[0, H-1]`,
   sample the camera texture (hardware bilinear, `CLAMP_TO_EDGE` = the NumPy edge
   clamp).
4. **Weight:** `w_i = border_feather(uv_i, feather_margin) * align_i^2` where
   `align_i = clamp(dot(normalize(P - C_i), fwd_i), 0, 1)`. `border_feather`
   ports `blend.border_feather` exactly.
5. **Blend:** accumulate `sum(w_i * color_i)` and `sum(w_i)`; output
   `sum / sum_w` with `A = (sum_w > 0)`; fragments with no camera get `fill_color`.

### 5.1 Inputs to GPU
- Camera images uploaded into a `sampler2DArray` (all 320×240 in the default rig;
  if sizes ever differ, fall back to N separate `sampler2D`s).
- Per-camera calibration (`K_i`, `R_i`, `C_i`, `fwd_i`) in a uniform block / arrays.
- `N` is a shader `#define` (recompile when camera count changes; cached per N).
- Bowl params (`R0`, `k`, `Rmax`), `feather_margin`, `fill_color` as uniforms.

### 5.2 Surface support
`isinstance` dispatch: `BowlSurface` → bisection path (its `R0/k/Rmax`);
`FlatSurface` → plane path (`z0`); anything else → `NotImplementedError` with a
clear message that GL supports only analytic surfaces (depth path covers the rest).

## 6. Depth mode — GPU point splatting (`GLDepthRenderer`)

Replaces `DepthRenderer`'s forward NumPy splatting with the rasterizer:

- For each `CameraFrame`, upload `image` and `depth` (depth as R32F texture, or
  pre-expanded vertex buffer of finite-depth pixels).
- Draw one **GL point** per source pixel with finite depth; the **vertex shader**
  back-projects `(u, v, depth)` → world (`backproject`) → virtual-camera clip
  space; `gl_PointSize` encodes the splat footprint (`splat_radius`).
- **Hardware depth test** (`GL_LESS`) implements nearest-wins, replacing
  `np.minimum.at` + winner mask.
- Fragment writes the point's color; coverage in the alpha channel gives `valid`.
- Output read back to NumPy as `(frame, valid)`.

## 7. Hybrid mode (`gl_hybrid.py`)

Compose GL-depth over GL-bowl by validity — the same rule the current hybrid
uses: take depth color where depth is valid, else bowl color, else fill. Baseline
implementation composites the two readback results in NumPy (reuses existing
logic, guarantees parity); GPU-side compositing in a single pass is an
optimization noted in §8.

## 8. GPU optimizations (explicit)

1. **Persistent textures:** for a static synthetic scene, upload camera images
   once and reuse; re-upload only when frames change.
2. **Single pass, precomputed uniforms:** one draw per mode; virtual inverse pose
   and ray basis computed once on CPU per frame; calibration in a uniform block.
3. **No-readback live path:** an optional app path blits the FBO straight to the
   pygame GL window (the real-time win). The readback path is retained for
   tests and the validation panel.
4. **Benchmark harness:** measures fps at 320×240 and 1280×720; target 720p bowl
   > 60 fps on the on-box NVIDIA GPU (vs NumPy's seconds/frame). Doubles as a
   performance-regression guard.
5. Optional/later: FP16 textures, tighter bisection bracket from radial bounds,
   GPU-side hybrid compositing.

## 9. Testing strategy (TDD, GPU-guarded)

All GL tests **skip cleanly when `gl_available()` is False** so the existing 87
tests pass on any machine. Written test-first:

1. **Context gate:** `gl_available()` / `get_context()` succeeds headless (or skips).
2. **Bowl parity (anchor):** `GLBowlRenderer` vs `NumpyRenderer` on an identical
   trivial scene → **PSNR > 40 dB** (allows tiny bilinear/precision diffs). This is
   the core correctness proof.
3. **Bowl vs ground truth:** meets the same PSNR threshold the NumPy integration
   test uses.
4. **Depth parity:** `GLDepthRenderer` vs `DepthRenderer` → high PSNR and matching
   valid mask (tolerant at splat edges from rounding).
5. **Hybrid parity:** GL hybrid vs current hybrid composite.
6. **Benchmark:** records fps; optional assert above a floor to catch regressions
   (marked slow / informational).

## 10. Visual verification (user works by judging visuals)

- Add a NumPy↔GL backend toggle key in `app.py`.
- Generate a `NumPy | GL | TRUTH` comparison montage PNG (rows = preset views)
  with PSNR/coverage annotations, the same way prior changes were verified.

## 11. File plan

```
tpsprojector/
  gl_context.py          # context + FBO pool + gl_available()
  gl_renderer.py         # GLBowlRenderer(Renderer) + inline GLSL
  gl_depth_renderer.py   # GLDepthRenderer + inline GLSL
  gl_hybrid.py           # composite helper
tests/
  test_gl_context.py     # skip-guarded context gate
  test_gl_bowl_parity.py # GL vs NumPy + vs truth
  test_gl_depth_parity.py
  test_gl_hybrid_parity.py
  test_gl_benchmark.py   # fps, marked slow
requirements.txt         # += moderngl
README.md                # roadmap: GLRenderer built
```

## 12. Risks & mitigations

- **Headless context flakiness** → `gl_available()` probe + universal skip guard;
  document that the app's live path can use the existing `DISPLAY=:1`.
- **Parity drift** (bilinear/precision) → 40 dB tolerance, not exact equality;
  match CLAMP_TO_EDGE and 60-iter bisection exactly.
- **Per-camera texture size differences** → `sampler2DArray` assumes uniform size;
  fall back to N `sampler2D` if violated.
- **N changes at runtime** → shader cached per `#define N`, recompiled on change.

## 13. Out of scope (next roadmap step)

C++/CUDA backend; depth from a real model/LIDAR; learned `Surface`. The seams
(`Renderer`, `Surface`, `blend`) remain the swap points for those.
