# TPSProjector

A cinematic third-person "follow-me" virtual camera synthesized in real time
from **N overlapping outward-facing cameras** rigidly mounted on a robot
(~360° coverage). The virtual camera is placed behind/above the robot and its
frame is computed by reprojecting the real cameras onto a 3D **bowl proxy
surface** and re-rendering from the virtual pose — the technique used by
automotive 360° surround-view systems.

This repository is the **NumPy + pygame prototype** for perfecting the
algorithm. It is architected so the reprojection core can later be swapped to
OpenGL and then a C++/CUDA library without changing the surrounding pipeline.

See the design spec: `docs/superpowers/specs/2026-06-19-tpsprojector-design.md`.

## How it works

```
onboard ring cameras ──▶ images + calibration
                         │
   virtual pixel ──▶ ray ──▶ Surface (bowl) hit point P
                         │
   P ──▶ reproject into every real camera that sees it
                         │
   sample + blend (feather + angular) ──▶ environment pixel
                         │
   composite robot proxy (depth) ──▶ final frame
                         │
   compare to ground-truth view ──▶ PSNR / SSIM / diff heatmap
```

## Module map

| Module | Role |
|---|---|
| `transforms` | SE(3) poses, rotations, `look_at` |
| `camera` | pinhole project / unproject / FOV tests |
| `surface` | `Surface` interface; `BowlSurface`, `FlatSurface` (geometry swap point) |
| `blend` | feather + normalized weighted blend for overlaps |
| `renderer` | `Renderer` interface; `NumpyRenderer` (backend swap point → GL → CUDA) |
| `robot` | robot proxy mesh + depth compositing |
| `validate` | PSNR / SSIM / diff heatmap |
| `presets` | cinematic shots + smoothstep tween |
| `world/` | synthetic test scene, triangle rasterizer, camera rig (ground truth) |
| `app` | `Engine` (testable core) + pygame display shell |

## Run

```bash
pip install -r requirements.txt
python -m tpsprojector.app
```

Controls: `1`–`4` cinematic presets · `v` toggle validation panel ·
`o` free-orbit debug (arrows orbit, `+`/`-` distance) · `esc` quit.

## Tests

A stale `anyio` pytest plugin must be disabled in this environment:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest
```

## Known limitations (inherent to the bowl method, by design)

- **Off-surface parallax**: objects far from the bowl surface (tall, near) smear
  and bleed, worsening as the virtual camera moves further from the rig center.
  The diff panel quantifies this; `top_down` (small parallax, ground-dominated)
  is the best-case view.
- **Near-field blind zone**: cameras mounted at height looking horizontally have
  a blind cone around the robot base → a ring with no source pixels. Mitigated by
  the robot proxy overlay and by the rig tuning below.

## Rig & bowl tuning

`Engine.from_defaults` exposes the knobs that matter most for realism:

- `tilt_deg` (default 12°) — pitches the ring cameras downward for near-field
  ground coverage. This is the main lever that shrinks the blind ring (orbit
  blind zone ~9% → ~3%).
- `mount_height` (default 0.45 m) and `rig_fov_deg` (default 85°) — lower mount +
  wider FOV reinforce the tilt and keep neighbour seams overlapping.
- `BowlSurface(R0, k, Rmax)` (default `6, 0.08, 20`) — smaller `R0` / steeper `k`
  make objects beyond the robot "stand up" on the wall sooner instead of smearing
  flat. Trade-off: too steep curves the distant ground and can push the orbiting
  virtual camera *outside* the bowl, so `R0` must stay larger than the orbit
  radius. PSNR slightly favors a flatter bowl; perceived realism favors a tighter
  one.

**Why far objects still distort during wide orbits:** a single fixed surface
cannot place an object that sits at the *orbit radius* (e.g. a box ~6 m out while
orbiting at ~5 m) at its true depth, so it ghosts. This is inherent to projection
without scene depth — the real fix is a depth-driven `Surface` (roadmap item 4),
which the architecture is already set up to accept.

## Roadmap

1. ✅ NumPy prototype — validated against ground truth.
2. `GLRenderer` (moderngl) — same `Renderer` interface, real time.
3. C++/CUDA library — per-pixel reproject + blend kernels, clean C++ API.
4. Geometry upgrades behind `Surface`: depth/stereo, learned.
