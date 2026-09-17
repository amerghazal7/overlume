# TPSProjector — Design Spec

**Date:** 2026-06-19
**Status:** Approved design, pre-implementation
**Author:** brainstorming session

## 1. Summary

TPSProjector synthesizes a cinematic third-person-shooter-style "follow-me"
camera view of a rigid body (a robot) that carries **N overlapping outward-facing
cameras** providing ~360° coverage. The virtual camera is placed behind/above the
robot and its frame is **computed in real time** by reprojecting the real cameras'
imagery onto a 3D proxy surface and re-rendering from the virtual pose.

This document specifies a **Python + pygame prototype** whose purpose is to perfect
the algorithm. It is explicitly architected so the core reprojection step can be
swapped from NumPy → OpenGL (moderngl) → a future GPU-accelerated **C++/CUDA library**
without changing the surrounding pipeline.

## 2. Goals & Non-Goals

### Goals
- Take N calibrated cameras (position, intrinsics, extrinsics) rigidly mounted on a
  robot and synthesize a virtual TPS camera view in real time (target after GL swap).
- Validate synthesis quality **objectively** against ground truth (synthetic world
  gives us a true view at the virtual pose).
- Keep the heavy reprojection math behind a fixed `Renderer` interface so backends
  are swappable; keep the proxy geometry behind a `Surface` interface so the geometry
  model can be upgraded later.
- Composite a 3D proxy of the robot body into the view (no real camera sees it).

### Non-Goals (prototype, YAGNI)
- Lens-distortion models (assume pinhole; design leaves a hook).
- Temporal smoothing across frames.
- Depth estimation / stereo-from-overlap / learned geometry (future `Surface` impl).
- Real camera capture / live feeds / multi-camera sync.
- The GL and C++/CUDA backends themselves (designed-for, not built in the prototype).

## 3. Key Decisions (from brainstorming)

| # | Decision | Choice |
|---|----------|--------|
| 1 | Input source | Synthetic 3D test world (gives ground truth) |
| 2 | Scene geometry model | **Bowl proxy surface** (flat floor + curved wall); pluggable `Surface` |
| 3 | Robot body in view | Render a **3D proxy model** of the robot, composite by depth |
| 4 | Virtual camera control | **Preset cinematic angles** with tweening (any pose valid internally) |
| 5 | Compute backend | **NumPy first**, behind a `Renderer` interface for later GL/CUDA swap |
| 6 | Validation | **Side-by-side + metrics**: synthesized \| ground-truth \| diff, PSNR/SSIM |

## 4. Architecture

### 4.1 Per-frame data flow

```
   Synthetic 3D World ──► renders N onboard camera images (RGB)
   (ground, boxes,        + renders GROUND-TRUTH view @ virtual pose (validation only)
    walls, robot body)
                          │  N images + calibration + virtual pose
                          ▼
        ┌──────────────── TPSProjector core ────────────────┐
        │ 1. Surface (bowl): ray → 3D hit point P            │
        │ 2. For each virtual-cam pixel: unproject → ray     │
        │ 3. Reproject P into every real camera that sees it │
        │ 4. Blend candidate colors (feather over overlaps)  │
        │ 5. Composite robot 3D proxy (depth-correct)        │
        └──────────────────────┬─────────────────────────────┘
                               │  synthesized RGB frame
                               ▼
        ┌──── pygame display / compare panel ────────────────┐
        │ [ synthesized | ground-truth | diff heatmap ]      │
        │   live PSNR / SSIM                                 │
        └─────────────────────────────────────────────────────┘
```

### 4.2 Modules (each independently testable)

| Module | Responsibility | Depends on |
|---|---|---|
| `world` | Synthetic scene + camera renderer (generates inputs & ground truth) | tiny 3D rasterizer, numpy |
| `camera` | Pinhole intrinsics/extrinsics; project / unproject; FOV test | numpy |
| `surface` | **Interface** + `BowlSurface`: ray → 3D point; params (floor radius, wall curve) | numpy |
| `renderer` | **Interface** + `NumpyRenderer`: the reprojection + blend | camera, surface |
| `blend` | `blend(candidates, weights)` → color; feather/angular weighting | numpy |
| `robot` | 3D proxy model + pose; rendered to RGB+depth; composited | camera |
| `validate` | PSNR / SSIM / diff heatmap | numpy |
| `app` | pygame loop; preset poses + tweening; layout & toggles | all |

**Critical seams (extension points):**
- `Renderer` interface — fixed contract
  `(camera_images, calibrations, surface, virtual_pose, robot) → synthesized_frame`.
  Swap point: `NumpyRenderer` → `GLRenderer` → C++/CUDA.
- `Surface` interface — fixed contract `ray(origin, dir) → (hit_point, valid_mask)`.
  Swap point: `BowlSurface` → flat-plane (debug baseline) → future depth/learned geometry.

## 5. Geometry & Reprojection Math

All cameras are pinhole models with known pose in the **robot rig frame**. The virtual
camera also lives in the rig frame. The bowl is defined in the rig frame, centered on
the robot.

### 5.1 Bowl proxy surface
- **Floor**: flat plane `z = 0` for radius `r ≤ R₀` (near ground stays crisp).
- **Wall**: for `r > R₀`, surface curves smoothly upward (parabolic/elliptical) up to a
  max height `H`. Distant objects "stand up" on the wall instead of smearing.
- The floor→wall transition is **continuous and tangent** at `R₀` (no visible seam).
- Tunable params exposed for tuning: `R₀` (floor radius), curvature, `H` (max height).

### 5.2 Reprojection (per virtual pixel, vectorized in NumPy)
1. **Unproject** pixel `(u,v)` → ray in virtual-cam frame → transform to rig frame:
   origin `Cᵥ`, direction `d`.
2. **Intersect** ray with surface → 3D point `P` (closed-form plane; quadratic solve in
   the wall region; guarded against degenerate/near-parallel rays).
3. **Reproject** `P` into each real camera `i`: `pᵢ = Kᵢ · [Rᵢ|tᵢ] · P`. Keep camera `i`
   only if `P` is in front of camera `i` AND `pᵢ` is within image bounds.
4. **Sample** each valid camera image at `pᵢ` (bilinear).
5. **Blend** candidates → final pixel (§6).

### 5.3 Why this yields parallax-correct follow-me
Every pixel resolves to a concrete 3D point `P` on the bowl and we reproject *that*
point, so synthesis is geometrically consistent for anything near the bowl surface.
Objects far off the surface show the known bowl distortion — the accepted trade-off of
this chosen geometry model; the validation panel quantifies it.

### 5.4 Edge cases
- **Pixel sees no camera** (ray misses all FOVs) → hole; fill neutral color or
  nearest-valid (configurable).
- **Ray would hit robot body first** → robot proxy wins via depth compositing (§7); we
  never paint environment over the robot.
- **Numerical**: rays near-parallel to floor; degenerate wall quadratic → clamp/guard.

## 6. Blending the overlaps

In the ~15° overlap zones a pixel's `P` reprojects into 2+ cameras. To avoid hard seams
and exposure jumps:
- **Feather weights**: each candidate's weight falls off toward its image border and
  toward the edge of its FOV → smooth cross-fade across seams.
- **Angular preference**: weight by alignment of camera view direction to `P` (a camera
  looking near-straight at `P` beats one catching it at the fringe of its view).
- **Photometric harmonization** (stretch, behind a flag): per-camera gain/bias to
  equalize overlap brightness. Add only if validation shows seams hurting metrics.
- All logic behind `blend(candidates, weights)` so feather vs. multiband is swappable
  without touching reprojection. Weights normalize to sum 1 over valid candidates.

## 7. Robot compositing, presets, validation

- **Robot proxy & compositing**: robot 3D proxy rendered from the virtual pose into a
  separate RGB+depth buffer. Bowl synthesis sits at "bowl depth"; composite
  robot-over-environment by depth so the robot occludes correctly and environment fills
  behind. Ghost/translucent mode is a later toggle.
- **Cinematic presets**: `behind`, `top-down`, `3/4-left`, `3/4-right` — each a pose in
  the rig frame. Switching tweens position + orientation with an ease curve (~0.5s).
  Any pose is valid internally, so a hidden free-orbit debug mode comes for free.
- **Validation panel**: render the true view by placing a real camera at the virtual
  pose in the synthetic world; display `[synthesized | ground-truth | diff-heatmap]`
  with live PSNR/SSIM. A keypress toggles validation layout vs. full-screen cinematic.

## 8. Testing strategy

- **Unit**: `camera` (project∘unproject round-trips), `surface` (known ray→point
  intersections), `blend` (weights sum to 1; symmetric overlaps balanced).
- **Integration**: synthesize from a trivial scene; assert PSNR above a threshold vs.
  ground truth.
- NumPy-first keeps every step inspectable, which is the point of the "perfecting" phase.

## 9. Default rig configuration (starting point, all parameterized)

- **N = 6** cameras in a horizontal ring around the robot, evenly spaced (60° apart),
  each with horizontal FOV ≈ 75° → ~15° overlap with each neighbor.
- Optional later: add up/down cameras for full sphere; the algorithm is general over N.
- Robot proxy: simple box/mesh at rig origin with a marked "front."
- Image resolution: modest (e.g. 320×240 per camera) for NumPy speed; configurable.

## 10. Roadmap beyond the prototype

1. **NumpyRenderer** (this prototype) — validate correctness against ground truth.
2. **GLRenderer** (moderngl) — same interface, real-time; bowl as textured mesh.
3. **C++/CUDA library** — port the validated design; CUDA kernels for per-pixel
   reproject + blend; clean C++ API consumable by other projects.
4. Geometry upgrades behind `Surface`: flat-plane baseline, depth/stereo, learned.

## 10b. Implementation findings (prototype build)

Two refinements emerged while building and are reflected in the code:

- **Surface intersection** is a vectorized **bisection root-find** on the
  implicit profile `g(t) = z(t) - f(r(t))`, not a closed-form quadratic. A
  strictly C1-tangent floor→wall transition at `R0 > 0` cannot be a single
  quadric of revolution, so the root-find is both correct *and* general over any
  future `Surface` profile — serving the swappable-geometry goal.
- **Validated limitations** (now in README): off-surface parallax/occlusion
  bleed grows with virtual-camera distance from the rig; a near-field blind zone
  rings the robot because the onboard cameras look horizontally from height.
  Round-trip reproduction (synthesize at a real camera's own pose) hits
  26–28 dB, confirming the reprojection math; `top_down` best-case is ~22 dB.

## 11. Open extension points (explicitly designed-for)

- `Renderer` backend swap (NumPy → GL → CUDA).
- `Surface` geometry swap (bowl → plane → depth/learned).
- `blend` strategy swap (feather → multiband → harmonized).
- Robot rendering mode (solid → ghost/translucent).
- Camera model (pinhole → +distortion).
