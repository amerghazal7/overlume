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

Controls: `1`–`4` cinematic presets · `b`/`d`/`h` render mode (bowl/depth/hybrid) ·
`v` validation panel (adds the Python ground-truth render + PSNR/SSIM; slower) ·
`o` free-orbit debug (arrows orbit, `+`/`-` distance) · `esc` quit.
Env reprojection runs on the C++ CUDA core via the `tpscuda` bindings; the live
view is the fast path, validation is on-demand.

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
4. ✅ `GLRenderer` — implemented: `GLBowlRenderer`, `GLDepthRenderer`, backend toggle (`[g]` key live-switches numpy ↔ GL). Note: the current GL path includes a per-frame framebuffer readback to NumPy (to keep the `Renderer` contract identical), so the 720p bowl benchmark measures ~38 fps; reaching the >60 fps target needs the no-readback direct-blit path and upload-once-for-static-scene optimizations (designed-for, not yet built).
5. ✅ GPU optimizations — upload-once caching (camera textures + per-camera uniforms for `GLBowlRenderer`; point-cloud VBO for `GLDepthRenderer`) and a no-readback `_render_to_fbo` path for both renderers. The live app opens a real OpenGL window (pygame `OPENGL|DOUBLEBUF`) and presents the rendered FBO directly via `gl_present.present_fbo` — no CPU readback on the fast path. Measured on RTX 3090 @ 720p bowl: **882.6 fps no-readback** vs 45.3 fps with readback (19.5× speedup). The `[v]` key switches to the readback path to show PSNR/SSIM.
6. ✅ C++/CUDA library (`micropilot_rendering`) — `Reprojector` class: bowl, depth, hybrid
   render modes, upload-once-for-static-scene optimisation. Pure C++17/CUDA; no ROS, no GL, no
   display dependencies. Installs as a manager-convention CMake package
   (`find_package(micropilot_rendering)` → `micropilot_rendering::reprojector`). Pybind11
   bindings provide Python-side bit-identical results vs the NumPy reference. Measured on
   RTX 3090 @ 720p bowl: **682 fps** (headless, no readback).
7. ✅ Headless consumer integrations:
   - **ROS2 LifecycleNode** (`micropilot_rendering_node`, `cuda/src/ros_apps/`) — wraps the
     CUDA reprojector in an `rclcpp_lifecycle::LifecycleNode`. Subscribes to N
     `sensor_msgs/Image` + `sensor_msgs/CameraInfo` topics, renders at 30 Hz, publishes a
     virtual-camera `sensor_msgs/Image` on `/rendering/image`. Camera extrinsics read from a
     ROS parameter (flat list of N×12 floats); production upgrade path is tf2 lookup (noted in
     code). Built with `colcon` (see `cuda/scripts/ros_apps_build/colcon_build.sh`).
     Headless smoke test (`cuda/src/ros_apps/src/micropilot_rendering_node/test/smoke_test.py`)
     validates a non-blank rendered frame end-to-end and exits 0.
     Runtime virtual-cam control: `~/set_virtual_cam` service (presets 1-5, eased tween),
     `~/set_look` topic (6 floats `[eye|target]`, immediate free look) and `~/vcam_state`
     telemetry (7 floats `[eye|target|active_preset]`, 0 = free look, each render tick).
     Robot proxy: `robot_model_path` (OBJ+MTL, e.g. the M02P model) is loaded at
     configure time, rasterized on the GPU, and depth-composited into every
     rendered frame — the robot's own body is visible in the virtual view even
     though no real camera sees it. `robot_model_transform` ([R|t], default
     Blender-export → rig) is the model-orientation calibration knob.
   - **Virtual-cam GUI + WebSocket bridge** (`tools/`) — realtime orbit + preset control with
     live video. `vcam_ws_bridge.py` exposes a generic WS JSON API (`set_look`/`set_preset`
     in, `state`/`ack` out; default `:8765`) for third-party integration; `vcam_gui.py` is a
     GTK3 client embedding `rosimagesrc ! videoconvert ! gtksink` (left-drag orbits, scroll
     dollies, buttons switch presets). Design:
     `docs/superpowers/specs/2026-07-06-vcam-gui-ws-bridge-design.md`. Run (node
     configured+activated, ROS + `cuda/install/ros_apps` sourced):
     `python3 tools/vcam_ws_bridge.py` then `python3 tools/vcam_gui.py`.
   - Video-file / GL consumer: designed-for seam in `Reprojector` interface; not yet wired.
8. Disocclusion handling: temporal accumulation / inpainting to fill unseen
   geometry instead of bowl fallback.

## Visual-mode docs

For the ROS2 `micropilot_visualization_node` (Filament-based third render
mode, `cuda/src/ros_apps/src/micropilot_visualization_node/`):

- [`docs/visual_mode/profile_authoring.md`](docs/visual_mode/profile_authoring.md)
  — add a topic to the visualization via profile YAML only (autonomy-team
  guide).
- [`docs/visual_mode/environment_bake.md`](docs/visual_mode/environment_bake.md)
  — bake OSM building footprints into the environment-chunk format the node
  loads.

(Deployment/architecture notes for this node are deferred to post-cutover —
see [`docs/visual_mode/README.md`](docs/visual_mode/README.md).)
