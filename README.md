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

## Render modes

The app renders the environment three ways (toggle live with `b`/`d`/`h`):

| Mode | Geometry | Strength | Weakness |
|---|---|---|---|
| `bowl` | static bowl proxy | always covers the frame | off-surface objects ghost/smear |
| `depth` | per-camera depth point cloud | parallax-correct for all objects | disocclusion holes (unseen geometry) |
| `hybrid` (default) | depth + bowl fallback | correct geometry *and* complete | bowl-filled holes are approximate |

Depth uses accurate per-camera depth (synthetic ground truth from the
rasterizer's z-buffer now; a real system would supply a depth model or LIDAR
fusion behind the same `CameraFrame` interface). This is the fix for the
far-object ghosting that a single static surface cannot solve.

## Module map

| Module | Role |
|---|---|
| `transforms` | SE(3) poses, rotations, `look_at` |
| `camera` | pinhole project / unproject / backproject / FOV tests |
| `surface` | `Surface` interface; `BowlSurface`, `FlatSurface` (geometry swap point) |
| `blend` | feather + normalized weighted blend for overlaps |
| `renderer` | `Renderer` interface; `NumpyRenderer` bowl backend (→ GL → CUDA) |
| `depth_renderer` | `CameraFrame`, `DepthRenderer` (point-cloud splatting), depth provider |
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

- `tilt_deg` (default: auto) — when left as `None`, each camera's downward tilt
  is computed by `tilt_for_body_edge` so its nearest visible ground lands at the
  robot body edge (body boundary at the bottom of frame, no blind ground ring),
  matching how rigs are physically mounted. ~31° for a 0.55 m mount here.
- `mount_height` — scalar **or per-camera list**: real rigs place cameras at
  different heights, and each camera's body-edge tilt follows from its own height
  (e.g. heights `[0.4, 0.55, 0.7, 0.85]` → tilts `[23.5°, 31.1°, 35.8°, 39.1°]`).
- `mount_radius` vs `body_radius` — the mounting tradeoff: cameras near the body
  rim keep the body to a thin sliver but need steep tilt (losing far view);
  inset cameras keep the far view but show more of the body. `rig_fov_deg`
  (default 85°) keeps neighbour seams overlapping despite the tilt.
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
2. ✅ Depth-derived geometry (point-cloud splatting) + hybrid fallback — fixes
   far-object ghosting using accurate per-camera depth.
3. Real depth source behind `CameraFrame`: monocular/stereo depth model or LIDAR
   fusion (replacing synthetic ground-truth depth).
4. ✅ `GLRenderer` — implemented: `GLBowlRenderer`, `GLDepthRenderer`, backend toggle (`[g]` key live-switches numpy ↔ GL).
5. Disocclusion handling: temporal accumulation / inpainting to fill unseen
   geometry instead of bowl fallback.
