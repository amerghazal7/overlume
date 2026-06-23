# GPU Optimization (upload-once + no-readback live display) — Design Spec

**Date:** 2026-06-23
**Status:** Approved design, pre-implementation
**Author:** brainstorming session
**Builds on:** `2026-06-19-glrenderer-design.md` §8 (GPU optimizations) and the
merged GL backend (master merge commit 70d8337).

## 1. Summary

The GL backend is correct but CPU-bound: the 720p bowl benchmark measures
~38 fps on an RTX 3090 because `GLBowlRenderer.render` re-uploads all camera
textures and re-sets per-camera uniforms every frame, and reads the full
framebuffer back to NumPy every frame. This phase implements the two optimizations
the GL spec deferred:

1. **Upload-once** — skip per-frame texture/point-cloud re-upload when the input
   is unchanged (transparent; the `Renderer` contract is untouched).
2. **No-readback path** — factor out an offscreen render that skips `glReadPixels`,
   used by (a) the benchmark to prove true GPU render speed and (b) a new live
   pygame OpenGL display that blits the rendered texture straight to the window.

Target: **> 60 fps @ 1280×720** for the no-readback bowl render (the benchmark
floor; the real number is expected to be far higher).

## 2. Goals & Non-Goals

### Goals
- Eliminate redundant per-frame GPU uploads for a static scene.
- Provide a no-readback offscreen render path and prove the GPU render rate.
- Display the cinematic view live through an OpenGL pygame window with no
  per-frame readback, while keeping the validation panel working as today.
- Keep the `Renderer` contract and all existing behavior/tests intact.

### Non-Goals (YAGNI)
- Changing the shaders' numeric behavior (parity to NumPy must be preserved).
- FP16 textures, multi-GPU, async double-buffered readback.
- Reworking the validation panel's rendering (it stays NumPy-composited; it is
  not performance-critical).
- The C++/CUDA backend (next roadmap step).

## 3. Design

### 3.1 Upload-once (transparent)
- `GLBowlRenderer` caches an upload signature: a tuple of `id(img)` for each
  image plus the camera count. On `render()`, compare the incoming signature to
  the cached one; call `self._cam_tex.write(...)` only when it differs (or when
  the texture was (re)allocated). Per-camera uniforms are likewise re-set only
  when the camera list identity changes (cache `id(cameras)` + the cameras'
  identities). The virtual-camera uniforms and surface uniforms still update
  every frame (they change per frame).
- `GLDepthRenderer` caches the built point cloud `(P, C)` and its VBOs keyed by
  the `frames` identity (tuple of `id(frame)`); skip `_point_cloud` rebuild and
  buffer upload when unchanged. Because the depth renderer currently creates and
  releases VBOs per call, caching means the VBOs persist across calls and are
  released on invalidation / replacement instead.
- Both expose `invalidate()` to force re-upload, for the in-place-mutation case.
- **Caveat (documented):** mutating an input array in place without creating a
  new object will not be detected; callers that mutate must call `invalidate()`.

### 3.2 No-readback render path
- Refactor each renderer so the GPU work lives in `_render_to_fbo(...) -> fbo`
  (sets uniforms, binds textures, draws into the pooled FBO; no readback).
- `render(...)` becomes `_render_to_fbo(...)` + framebuffer readback + the
  existing `(frame, valid)` post-processing. Output is byte-for-byte what it is
  today (the readback path is unchanged in result).
- Honest benchmarking: a no-readback timing loop calls `_render_to_fbo` then
  `ctx.finish()` each iteration (GL is asynchronous; without `finish()` the
  timer would not capture GPU execution).

### 3.3 Live OpenGL display (`gl_present.py` + `app.py`)
- New `gl_present.py` renders a fullscreen quad with a present shader that
  samples a source texture, giving two entry points:
  - `present_fbo(fbo, target=None)` — blit `fbo`'s color texture to `target`
    (the default framebuffer = the window, or a given FBO for tests).
  - `present_array(rgb, target=None)` — upload a NumPy `(H,W,3)` array to a
    cached texture and blit it (used for the validation composite).
- `main()` creates the window with `pygame.OPENGL | pygame.DOUBLEBUF`:
  - **Cinematic mode** (validation off): `eng._render_to_fbo(vc)` →
    `present_fbo(fbo)` → `pygame.display.flip()`. No readback — the fast path.
  - **Validation mode** (validation on): build the `synth | truth | diff` + text
    composite in NumPy exactly as today (synth via the readback `render()`), then
    `present_array(composite)` → `flip()`. Readback here is acceptable.
- The window is always an OpenGL surface, so toggling modes never recreates it.

### 3.4 Robot compositing in cinematic mode
The robot proxy is CPU-rendered to RGB + depth from the virtual pose and is
normally composited over the environment in NumPy (`robot.composite`). To keep
the environment on the GPU in cinematic mode, the robot is composited **on the
GPU**: the CPU robot layer is uploaded as a small `(H,W)` RGBA texture each frame
(alpha = `isfinite(robot_depth)` mask) and the present shader blends it over the
environment FBO (`out = mix(env, robot.rgb, robot.a)`). This is decisive for the
phase. Rationale: the robot layer depends on the virtual pose so it must refresh
each frame, but it is a single small upload — far cheaper than the eliminated
per-frame 6-camera re-upload and full-frame readback. The benchmark measures the
**environment-only** render (no robot), which is the pure-GPU number; robot
compositing adds only one small upload + a blend in the present pass.

### 3.5 Performance measurement
- `gl_benchmark.benchmark(...)` reports both `fps` (no-readback, with `finish()`)
  and `fps_readback` (the current end-to-end `_render_env`) so the readback cost
  is quantified explicitly.
- Acceptance: no-readback bowl @1280×720 → **fps > 60** (regression floor).

## 4. Testing strategy (TDD, GPU-guarded)

All GL tests skip when `gl_available()` is False.
1. **Upload-once:** wrap/spy `self._cam_tex.write` (e.g., monkeypatch a counter);
   call `render` 3× with the same images → exactly 1 write; call `invalidate()`
   then `render` → 1 more write. Output unchanged across calls (parity with the
   first frame). Same shape of test for `GLDepthRenderer` point-cloud rebuild.
2. **`_render_to_fbo` parity:** the readback of `_render_to_fbo(vc)` equals the
   `render(...)` frame (and the existing NumPy-parity tests still pass through the
   refactored `render`).
3. **`gl_present` round-trip:** `present_fbo`/`present_array` blitting into an
   offscreen target FBO, read back, equals the source image (within tolerance) —
   verifies the present shader headlessly without a real window.
4. **No-readback benchmark:** `benchmark` returns both fps numbers; assert
   `fps > 60` @720p (marked slow). Confirms the optimization target.
5. **Regression:** full suite stays green; readback `render()` output identical to
   pre-refactor (covered by the existing parity tests).

Manual-verify only: the `pygame.OPENGL` window creation + `flip()` wiring in
`main()` (it is `# pragma: no cover`), checked by running `python3 -m tpsprojector.app`.

## 5. File plan

```
tpsprojector/
  gl_renderer.py        # + upload-once + _render_to_fbo (render refactored to use it)
  gl_depth_renderer.py  # + point-cloud cache + _render_to_fbo
  gl_present.py         # NEW: present_fbo, present_array, present shader
  gl_benchmark.py       # + no-readback timing (ctx.finish), report both fps
  app.py                # OpenGL window; cinematic (present_fbo) vs validation (present_array)
tests/
  test_gl_upload_once.py
  test_gl_render_to_fbo.py
  test_gl_present.py
  test_gl_benchmark.py  # extend: assert no-readback fps > 60
README.md               # roadmap: optimizations done, fps numbers
```

## 6. Risks & mitigations
- **Identity-based cache staleness** → documented caveat + `invalidate()` hook;
  the Engine's static-scene usage is the common case and is safe.
- **Async timing dishonesty** → `ctx.finish()` in the no-readback benchmark.
- **pygame OpenGL + 2D text mixing** → all on-screen content goes through the
  present shader as textures (cinematic = FBO texture; validation = NumPy
  composite uploaded), so the window stays a single OpenGL surface.
- **Robot compositing in no-readback mode** → composited on the GPU via a small
  per-frame robot-layer texture + a blend in the present shader (§3.4); the
  benchmark stays robot-free for the pure-GPU number.

## 7. Out of scope (next roadmap step)
C++/CUDA backend; FP16; async readback double-buffering; depth from a real
model/LIDAR. Seams (`Renderer`, `Surface`, `blend`) unchanged.
