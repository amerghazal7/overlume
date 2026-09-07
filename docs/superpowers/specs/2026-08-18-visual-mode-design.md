# Visual Mode — Stylized Autonomy Data Visualization (Design)

**Date:** 2026-08-18
**Status:** Accepted (Epics 0–2 implemented against it); amended by the plan review of 2026-09-07 — see `[review 2026-09-07]` markers and the changelog in `docs/superpowers/plans/2026-08-18-visual-mode.md`. Load-bearing decisions: `docs/adr/0001`–`0004`.
**Reference assets:** `assets/visualization-reference-1.jpg` (dark ADAS style),
`assets/visualization-reference-2.jpg` (light clay style), `assets/urban_config.rviz`,
`assets/offroad_config.rviz`

## 1. Context and goals

Today the rendering stack offers two modes of one product: a photographic
surround view stitched from the robot's 6 cameras (mode 1 = bowl, mode 2 =
lidar hybrid), rendered by the CUDA `micropilot_rendering_node` and streamed
as `/rendering/image`, with a virtual camera driven over WebSockets.

The product team wants a third mode — **Visual** — that renders *all*
autonomy-software data (perception objects, planner paths, HD map, occupancy
grids, collision checks) as a modern, stylish 3D scene in the spirit of
production ADAS HMIs: clay-shaded 3D vehicle models instead of bounding
boxes, glowing path ribbons, matte stylized world, HUD chips and alert
callouts. The autonomy team's two rviz configs (urban, offroad) define the
data inventory; nothing they can see today may become invisible.

**Untouchable contract:** the output stays a rendered frame stream, and a
virtual camera that orbits the world, receiving config/orbit commands through
the existing WebSocket pipeline. Existing modes 1–2 keep working unchanged.

### Success criteria

- Mode 3 selectable at runtime through the same channel as modes 1–2; the
  consumer-visible stream (`/rendering/image`) switches seamlessly.
- Dark-theme scene visually comparable to reference 1 (clay traffic objects,
  emissive planned-path ribbon, styled lane geometry, readable HUD) AND
  light-clay theme comparable to reference 2, switchable at runtime via an
  animated day/night toggle (§4.3).
- Every display in the two rviz configs has a Visual-mode representation
  (stylized or generic-fallback — see §7 parity matrix).
- Sustained 30 fps at 1280×720 on the robot's onboard GPU while perception +
  surround view run, with graceful degradation knobs.
- Orbit/preset/look camera behavior identical (same message semantics) to the
  existing node, verified by the existing WS smoke tests extended to mode 3.

## 2. Technology decision

**Renderer: Google Filament** (C++, real-time PBR) rendering to a headless
swapchain (`SwapChain::CONFIG_READABLE`, EGL/Vulkan backend) with pixel
readback into a `sensor_msgs/Image`. Chosen over (a) extending the in-house
CUDA/GL renderers — months of engine work for lighting, AO, bloom, shadows,
glTF loading, tone mapping that Filament provides out of the box — and
(b) web-tech headless three.js — fastest to pretty pixels but drags a
Node/Chromium runtime into the robot stack and headless-gl is WebGL1-bound.
Filament is a plain C++ library, fits the micropilot C++/CMake conventions,
and is proven in headless server-side use.

**Toolchain boundary (known risk, handled by design):** Filament's official
binaries — and its recommended source build — use clang + libc++, while the
ROS 2 workspace builds with gcc + libstdc++. Mixing the two C++ runtimes in
one binary is undefined-behavior territory at any std-type boundary.
Mitigation: the visualization library compiles as a self-contained
clang/libc++ static unit whose public header exposes **only POD structs,
fixed-width types, and raw pointers/spans** (no std:: types cross the ABI).
The ROS node stays on the stock gcc toolchain. A header check (`scripts/check_pod_header.sh`, a ctest of the lib project; wired into the colcon build by VM-037 — there is no hosted CI, `[review 2026-09-07]`) asserts the public
header includes nothing from the standard library beyond `<cstdint>`/
`<cstddef>`. If the clang/libc++ static-link route proves brittle in practice,
fallback is building Filament from source with gcc (supported but less
tested) — the POD boundary makes either choice invisible to the node.

## 3. Architecture overview

```
                       ┌────────────────────────────────────────────┐
 6 cams + lidar ──────▶│ micropilot_rendering_node (CUDA, existing) │──┐
                       │   modes 1 (bowl) / 2 (hybrid)              │  │
                       └────────────────────────────────────────────┘  │
                                                                        ├──▶ /rendering/image
 autonomy topics ─────▶┌────────────────────────────────────────────┐  │    /rendering/camera_info
 (markers, paths,      │ micropilot_visualization_node (NEW)        │──┘
  OGMs, TF)            │   mode 3 (visual)                          │
                       │  ┌──────────────┐  ┌─────────────────────┐ │
                       │  │ Ingest       │─▶│ micropilot::         │ │
                       │  │ adapters     │  │ visualization lib    │ │
                       │  │ (ROS msgs →  │  │ (SceneGraph → Filament│ │
                       │  │  SceneGraph) │  │  headless → RGB8)    │ │
                       │  └──────────────┘  └─────────────────────┘ │
                       └────────────────────────────────────────────┘
                                  ▲                    ▲
                    /rendering/set_mode      ~/set_virtual_cam, ~/set_look,
                    (Int32 1|2|3, global)    ~/vcam_state  (same semantics
                                             as existing node)
                                  ▲
                       tools/vcam_ws_bridge.py (extended) ◀── WebSocket clients
```

### 3.1 Two nodes, one stream (mode mux)

The Visual renderer is a **separate lifecycle node**, not a third code path
inside the CUDA node. Rationale: different GPU API (Filament GL/Vulkan
context vs CUDA), different input set (autonomy topics vs camera images),
crash isolation, independently deployable. The "one product, three modes"
experience is preserved by a mux protocol:

- New **global** topic `/rendering/set_mode` (`std_msgs/Int32`: 1 bowl,
  2 hybrid, 3 visual). Both nodes subscribe.
  - `rendering_node`: 1/2 behave exactly as today's `~/set_render_mode`;
    on 3 it stops publishing (subscriptions stay alive, buffers keep
    filling, so switching back is instant). The node-private
    `~/set_render_mode` topic remains for backward compatibility and
    implies leaving mode 3. `[review 2026-09-07]` As shipped this is one-sided: the CUDA node leaves idle locally but never announces it, so visualization_node keeps publishing (two publishers). Target (VM-037): the legacy handler re-publishes the value on `/rendering/set_mode`.
  - `visualization_node`: renders + publishes only while mode == 3;
    otherwise it keeps ingesting (cheap CPU-side scene updates) but skips
    the render/readback/publish, so it costs ~zero GPU when inactive.
- Both publish to the **same** `/rendering/image` + `/rendering/camera_info`.
  Exactly one node publishes at a time; consumers never re-subscribe.
- Mode state is included in `~/vcam_state` telemetry (already carries
  render_mode today) from whichever node is active.
- Race handling: on startup both nodes default to the last configured mode
  parameter (`initial_mode`, default 1) so exactly one publisher is active
  from the first frame. `[review 2026-09-07]` This only holds while the mode never changes at runtime: `/rendering/set_mode` is VOLATILE as shipped, so a restarted or late-joining node rejoins at `initial_mode`, not the live mode. Target (VM-037): `transient_local, depth 1, reliable` on every publisher and subscriber of the topic, so the last command is retained. Also as shipped, `initial_mode:=1` leaves the CUDA node's `render_mode_` at 2 (hybrid) — same item.

### 3.2 Repository layout

Additive, following the existing workspace conventions (the `cuda/` root is
historical; renaming it is deliberately out of scope):

```
cuda/src/libs/visual_renderer/            # micropilot::visualization (ROS-free)
  include/visual_renderer/api.h           # POD-boundary public API
  src/…                                   # Filament backend, scene, themes, hud
  assets/…                                # clay glTF models, fonts, IBL, themes/*.yaml
  tests/…                                 # golden-image + unit tests
cuda/src/ros_apps/src/micropilot_visualization_node/
  src/…  include/…  config/…  launch/…  test/
tools/vcam_ws_bridge.py                   # extended (mode 3, theme, layers)
tools/vcam_gui.py                         # extended (mode button, layer panel)
```

## 4. The visualization library (`micropilot::visualization`)

ROS-free, testable standalone. Internal components:

### 4.1 SceneGraph (domain model)

Plain-data description of one displayable moment; the node writes it, the
renderer reads it. All poses in the **map frame**; the ego pose comes from
TF (`map → base_link`).

`[review 2026-09-07]` Categories: 8 shipped through Epic 2; `PointCloud[]` is appended in Epic 3 (VM-035, ADR-0004).

- `Ego` — pose, speed (finite-differenced from TF, smoothed), the robot's
  own glTF model.
- `TrackedObject[]` — id, class enum {CAR, TRUCK_VAN, BUS, PEDESTRIAN,
  CYCLIST, UNKNOWN}, pose, dimensions, heading, velocity arrow data,
  predicted path polyline, label text.
- `PathRibbon[]` — role enum {BEHAVIOR, GLOBAL, LOCAL}, polyline, styled per
  theme (the behavior path is the hero emissive ribbon of reference 1).
- `MapElement[]` — lane polylines/polygons/crosswalks from the HD-map
  MarkerArray, cached (static per session, rebuilt only on new latched data).
  `[review 2026-09-07]` Carries `kind`, `lane_id` and `last_update_sec`
  (VM-036); paired left/right boundaries also produce a filled **road
  surface** in `palette.road`, so the road reads darker than the clay ground
  as in reference 2.
- `GroundGrid` — occupancy-grid raster layers (dynamic OGM, gradient OGM)
  as ground-projected textures with theme-colored transfer functions.
- `AlertPolygon[]` — collision-checker footprint sweeps / predicted
  polygons / merged polygons, translucent warning materials.
- `GenericMarker[]` — pass-through primitives for the fallback renderer (§7).
- `PointCloud[]` — decimated `PointCloud2` layers as colored points; colors
  baked node-side per the profile row's `color_mode` (`auto` = rgb when the
  cloud carries it → intensity ramp when it carries that → height ramp;
  or forced `rgb`/`intensity`/`height`/`flat`). Added in Epic 3 (VM-035) by
  appending a category — `[review 2026-09-07]` the scene header is
  additive-only and versioned (ADR-0004), not frozen.
- `Hud` — speed value, active mode, alert chips (text + 3D anchor for
  leader-line callouts, e.g. distance-to-obstacle).

Double-buffered: the publisher deep-copies into a staging slot and the active index swaps under a mutex. **Single-threaded as shipped (`[review 2026-09-07]`):** the same thread must call `set_scene()` and `render_frame()` — `SceneBuffer::active()` returns a reference aliased into slot storage, so cross-thread ingest needs an owned snapshot first (a SceneBuffer design change, not a locking tweak). Ingest-thread writes are the intended future shape, not today's contract. The freeze-frame property holds either way — same freeze-frame philosophy as the existing node (render
every tick from the latest cached data; stale data keeps rendering so the
stream never freezes with the sim paused).

### 4.2 FilamentBackend

- Engine + headless SwapChain (`CONFIG_READABLE`), one View/Scene/Camera.
- Post: SSAO, bloom (drives the emissive path/brake-light glow), FXAA or
  TAA, ACES tone mapping — all individually switchable (quality presets).
- Lighting: one directional sun + prefiltered IBL per theme (dark theme =
  low-key night IBL, light theme = bright overcast). Contact shadows on;
  shadow-map resolution is a preset knob.
- Geometry strategies:
  - glTF assets instanced per object class (gltfio + one material remap to
    the theme's clay material).
  - Ribbons/polylines: extruded triangle strips generated CPU-side per
    frame (paths are small: hundreds of points).
  - OGM layers: one textured quad per grid, texture updated in place.
  - Generic markers: pooled primitive renderables (cube/sphere/cylinder/
    line-strip/text) to avoid per-frame allocation.
- Output: `render() → readPixels → uint8 RGB` into a caller-provided buffer
  (the node wraps it in a `sensor_msgs/Image` without copy where possible).

### 4.3 ThemeSystem

`[review 2026-09-07]` Hidden coupling to know about: `fog.density` is the only authored fog knob; the renderer also scales `palette.fog` radiance by an empirical fit on `ibl.intensity` (`50·(8750/I)^1.159`, `renderer.cpp`) calibrated on the two shipped themes, so retuning a theme's IBL intensity changes its horizon fog brightness. Revisit when a third theme ships.

A theme is a YAML token file (committed under `assets/themes/`):
palette (ground, sky/fog, lane paint, ribbon core/glow, per-class object
tints, alert colors), material params (roughness/metallic for clay),
emissive strengths, grid style, HUD colors/typography scale. **Both themes
ship in v1**: `dark_adas.yaml` and `light_clay.yaml`. Because every visual
property is a token on parameterized materials (no shader rebuild), theme
switching is an **animated transition**, not a swap: on `set_theme` the
renderer eases all tokens from the current theme to the target over a
configurable duration (default 0.8 s, smoothstep) — palette lerp in a
perceptual color space (Oklab), sun/IBL intensity crossfade, HUD colors
included — the ref-1 night world visibly "brightens into" the ref-2 clay
world. The GUI exposes this as a day/night toggle; the WS `set_theme`
command drives the same transition.

### 4.4 AssetLibrary

- Clay vehicle/pedestrian/cyclist models: low-poly CC0 glTF (e.g. Kenney /
  Quaternius packs), normalized to unit footprint and scaled per-object to
  the perception bbox dims — so a "car" model stretches to the measured
  box rather than clipping through it. UNKNOWN class renders as a rounded
  clay box.
- Ego robot: the M02P OBJ converted to glTF (one-time script), path via
  param like today's `robot_model_path`, load failure non-fatal (falls back
  to clay box, warns).
- SDF font atlas for in-scene (3D-anchored) text and callouts.
  `[review 2026-09-07]` **Accepted (user 2026-09-07):** the screen-space HUD (speed chip, mode)
  is composited node-side onto the RGB8 buffer after readback with
  stb_truetype instead of an SDF overlay pass through Filament — same pixels,
  no font pipeline in the lib (VM-030). Re-entry trigger: HUD text must be
  depth-tested, lit or fogged.

### 4.5 EnvironmentLayer (clay buildings — 3D-tiles-style)

The reference images' matte building blocks are footprint extrusions — the
same technique as Mapbox GL's `fill-extrusion` buildings layer. Because the
robot operates in a bounded area, v1 uses a **deployment-time bake pipeline**
instead of runtime tile streaming:

- `scripts/bake_environment.py`: given the operating-area bounding box,
  pulls OSM building footprints + height/levels tags (Overpass API or a
  local planet extract; Mapbox vector tiles usable as an alternative source
  where licensed), extrudes them into **chunked, georeferenced glTF tiles**
  (quadtree chunks, ~256 m), and writes a small tileset index JSON.
- At runtime the AssetLibrary loads the chunk index once and enables chunks
  by distance to ego (simple culling; the whole operating area typically
  fits in memory as clay boxes). Buildings take the theme's building
  material — no textures, pure clay.
- **Geo-anchor:** buildings are in WGS84; the scene is in the map frame.
  The robot publishes `sensor_msgs/NavSatFix` in WGS84 — the **primary**
  datum source: sample fixes + the `gps_link` TF while localized to solve
  the map↔WGS84 anchor (position from averaged fixes, heading from the fix
  track vs map-frame track). A `geo_datum` param (lat, lon, heading of the
  map-frame origin) is a manual override for GPS-denied replays. No anchor
  from either source → environment layer disabled with one WARN, everything
  else unaffected.
- The layer sits behind an `EnvironmentSource` seam. **v1.1 (committed,
  starts immediately after v1.0; `[review 2026-09-07]` deferral was proposed
  and rejected by the user):** a cesium-native-based OGC 3D Tiles
  streaming adapter (Cesium OSM Buildings via Cesium ion — user handles
  registration) replaces/augments the baked source, with clay
  re-materialization, geo placement via the same anchor, a disk tile cache,
  and automatic fallback to baked chunks on network loss — renderer code
  unchanged. Backlog Epic 6 (VM-060…VM-063).

## 5. Ingest adapters (ROS node side)

Config-driven, mirroring the rviz configs as **profiles**. A profile YAML
(`config/urban_profile.yaml`, `config/offroad_profile.yaml`, `config/sim_profile.yaml` — the sim profile adds the latched full-extent map row; `[review 2026-09-07]` three ship, not two) lists
`(topic, msg type, adapter, style role)` rows; the node subscribes
accordingly. Adding a topic for the autonomy team = one YAML row, not code.

| Adapter | Input | Produces |
|---|---|---|
| `DynamicObjectsAdapter` | `/perception/dynamic_objects_list` (MarkerArray; namespaces `*_bbox`, `*_arrow`, `*_text`, `*_hd_map_path`, `*_hd_map_path_dots`) | `TrackedObject[]` — bbox marker → pose/dims; text marker → label + **class inference**; arrow → velocity; hd_map_path → predicted path |
| `HdMapAdapter` | **`/hd_map_local_elements` (primary — live-sim finding 2026-08-19: continuous ~10 Hz stream, always joinable)**; `/hd_map_global_elements` best-effort only (its publisher is VOLATILE publish-once-at-startup — late joiners get nothing); sim profile may also use latched `/sim/hd_map/markers` (TRANSIENT_LOCAL) | `MapElement[]`, cached until a new message arrives |
| `PathAdapter` ×N (3 roles, 4 shipped rows) | `/behavior_path_planner/output_path_visualization` (BEHAVIOR), `/local_vel_path` (LOCAL — the only local output live in the 2026-08-19 recording, `[review 2026-09-07]`), `/local_path` (LOCAL, offroad display, silent in the recording), `/navigation/global_path` (GLOBAL, silent) (nav_msgs/Path) | `PathRibbon` per row slot (not per role) |
| `OgmAdapter` ×2 | `/perception/dynamic_ogm`, `/perception/gradient_ogm` (+`_updates`) (OccupancyGrid + updates) | `GroundGrid` textures |
| `CollisionAdapter` | the 5 collision-checker MarkerArray topics | `AlertPolygon[]` |
| `TfAdapter` | TF (`map → base_link`); speed prefers `/robot/feedback/robot_speed_mps` (Float32, live-sim finding 2026-08-19) with TF finite-difference as fallback | `Ego` pose + speed |
| `GenericMarkerAdapter` | any additional MarkerArray topic named in the profile | `GenericMarker[]` (§7 fallback) |
| `PointCloudAdapter` (VM-035, Epic 3) | any `sensor_msgs/PointCloud2` row in the profile; ingest-decimated (`max_points`, `stride`, `max_rate_hz`) | `PointCloud` — rgba8 baked per `color_mode: auto\|rgb\|intensity\|height\|flat`; `auto` falls back rgb → intensity (normalized over `intensity_range`, auto-ranged when unset) → height |

**Class inference** (until a typed perception topic exists — the adapter
seam accepts one later without touching the renderer): live-sim sampling
(2026-08-19) shows text labels are `<prefix>_<track_id>` (e.g. `V_1105`),
so the config map keys on the label PREFIX (V=vehicle, …; table completed
from fixture data); if inconclusive, classify by bbox footprint
(length/width/height thresholds: pedestrian < ~1 m² tall-thin; cyclist
elongated-narrow; car/van/bus by length bands). Misclassification degrades
to a *differently shaped clay model of the right size* — cosmetic, not
misleading, because scale always comes from the measured bbox.

**Frame/time handling:** everything renders in the map frame at "latest
available" per topic (rviz semantics). Per-topic staleness timeouts fade
objects out (opacity ramp over e.g. 0.5 s) instead of popping.

## 6. Virtual camera & control-surface compatibility

The visualization node re-implements the existing vcam surface with
identical message/service contracts under its own namespace:

- `~/set_virtual_cam` (SetVirtualCam srv, presets 1–5 with eased tween)
- `~/set_look` (Float64MultiArray[6] `[eye|target]`, immediate, cancels tween)
- `~/vcam_state` (Float64MultiArray[8] telemetry incl. active preset + mode). `[review 2026-09-07]` Index 7 is NOT comparable across nodes as shipped: rendering_node publishes its bowl/hybrid `render_mode_` (1|2), visualization_node its `active_mode_` (1|2|3); the WS bridge works around it by trusting one namespace per commanded mode. VM-037 appends index 8 `mux_mode` (identical on both nodes) and leaves index 7 unchanged for compatibility.
- `virtual_pose` / `virtual_vfov_deg` params, live-settable like today.

The pose→Filament camera mapping reuses the same look_at/[R|t] convention as
the CUDA node so presets/orbits produce the same framing in both worlds.
`tools/vcam_ws_bridge.py` fans commands out to **both** nodes (they share
semantics; the inactive one just updates state), so a client orbiting in
mode 2 and switching to mode 3 keeps its viewpoint. New WS commands:
`set_render_mode` extended to 3 (publishes `/rendering/set_mode`),
`set_theme <name>`, `set_layers {layer: bool}` (per-SceneGraph-category
visibility), `set_quality <preset>`.

The GUI (`tools/vcam_gui.py`) gains a mode-3 button, a theme dropdown, and a
layers checklist; its tuning panel plumbing (`set_param` → node parameters)
already generalizes to the new node.

## 7. Feature-parity matrix (nothing gets lost)

Existing modes 1–2 are untouched, so all surround-view features survive by
construction. Within Visual mode, every rviz display maps as:

| rviz display (configs) | Visual mode representation |
|---|---|
| Grid | Themed ground grid (fading with distance) |
| TF frames | Ego pose consumed always; full TF axes as a **debug layer** (off by default, generic renderer) |
| `/hd_map_local_elements` (primary, ~10 Hz) + `/hd_map_global_elements` (best-effort, publish-once) | Stylized lane geometry (lane paint material, crosswalk hatching — `[review 2026-09-07]` hatching is dead code on recorded 5-point closed crosswalks until VM-036) |
| `/perception/dynamic_objects_list` | Clay 3D models + velocity arrows + predicted-path ribbons + optional labels |
| 5 collision-checker topics | Translucent alert polygons (theme warning colors), ego sweep as ghost trail |
| `/behavior_path_planner/output_path_visualization` | Hero emissive ribbon (ref-1 green glow) |
| `/navigation/global_path`, `/local_path` | Route ribbons (distinct theme roles) |
| `/perception/dynamic_ogm`, `gradient_ogm` | Ground-projected raster layers with theme transfer functions |
| Any other MarkerArray | **Generic marker fallback**: faithful rendering of Marker primitives (CUBE, SPHERE, CYLINDER, ARROW, LINE_STRIP/LIST, POINTS, TEXT, TRIANGLE_LIST, MESH by path) with theme-neutral materials — added by one profile row |

The generic fallback is the parity guarantee: any topic the autonomy team
visualizes tomorrow is displayable immediately, stylized later.

rviz *tools* (SetInitialPose, SetGoal, PublishPoint, Measure) are authoring
tools, not displays — out of scope for a rendered stream; rviz remains the
authoring tool. Interactive selection/click-to-inspect is future work
(requires picking + a richer WS protocol).

## 8. Performance budget & degradation

- Target: 1280×720 @ 30 fps on the onboard GPU with perception + the CUDA
  node co-resident. `[review 2026-09-07]` This is an assumption; it was never
  measured on the robot. Proxy numbers (dev box, bag replay) are in
  `cuda/src/libs/visual_renderer/tools/budget_probe.md`; the on-robot table is
  a blocking item of VM-043. Scene scale is small (tens of objects, few-thousand map
  segments, 2 grid textures) — well inside Filament's envelope; the risks
  are readback latency and GPU contention, both bounded by:
- Quality presets (`quality` param + WS command): `high` (TAA, SSAO, bloom,
  2048 shadows), `medium` (FXAA, SSAO half-res, bloom, 1024), `low` (FXAA
  only, no shadows, 960×540 upscaled). Preset auto-drop (hysteresis) if the
  measured frame time exceeds budget for N consecutive seconds, with a log
  line — never silent.
- Inactive mode costs: no render pass, no readback; ingest only.
- Readback is synchronous v1 (simple, proven by the existing GL tests’ FBO
  path); double-buffered async readback is a listed optimization if profiling
  demands it.

## 9. Error handling

- **Missing/silent topics:** render continues with what exists; per-layer
  staleness fades (§5). A `~/diagnostics`-style status (active layers, last
  msg age per topic) is published for the GUI to surface.
- **Asset load failure** (model/font/IBL/theme): non-fatal, fall back to
  clay boxes / default theme, WARN once — same philosophy as
  `robot_model_path` today.
- **Malformed markers** (empty polylines, NaN poses): adapter-level
  validation drops the primitive, counts it in diagnostics; never crashes
  the render thread.
- **GPU/context loss:** lifecycle node transitions to `inactive`, publishes
  nothing; the mux means consumers can switch back to modes 1–2, which run
  in a separate process by design.
- **Mode-switch races:** both nodes treat `/rendering/set_mode` as
  last-write-wins; a 1-frame overlap or gap during switch is acceptable and
  bounded by topic latency.

## 10. Testing strategy

Following the repo's TDD convention:

- **Adapter unit tests** (gtest): recorded-message fixtures (captured from
  bags into headers/json) → SceneGraph assertions, incl. class-inference
  table tests and malformed-input tests.
- **Golden-image tests** — `[review 2026-09-07]` as shipped: block-SSIM 0.98 at 320×240, `quality=1` (medium, the shipped default — not the "low preset" below), single-theme for every geometry golden except empty-world and map. **Accepted (user 2026-09-07):** require per-theme pairs only for goldens whose subject is theming (empty world, map/lane paint, one lit-geometry reference); other categories keep one golden. Trigger back: a theme-only regression escapes. Original text: deterministic synthetic SceneGraph → Filament
  headless render → perceptual-diff (SSIM threshold) against committed
  goldens, per theme, low preset, fixed seeds. Skip without GPU (same
  pattern as the GL tests).
- **Contract tests**: vcam message semantics vs the existing node (shared
  test vectors: preset poses, set_look echo, state telemetry layout).
- **Smoke test** (extends `smoke_test.py`): launch both nodes, drive
  `/rendering/set_mode` 1→3→2, assert exactly-one-publisher and stream
  continuity; WS bridge E2E for the new commands.
- **Perf benchmark**: scripted scene at target complexity, frame-time
  budget assertion per preset (informational in CI, hard gate on-robot).

## 11. Out of scope (v1) — explicit

- Runtime-streamed 3D Tiles are out of **v1.0** only — they are the
  committed v1.1 scope (§4.5, backlog Epic 6), starting immediately after
  v1.0 (`[review 2026-09-07]` deferral was proposed and rejected by the user). Photorealistic tiles stay out entirely (clay style is the product).
- Minimap inset (needs a 2D map raster source; the bake pipeline's data
  could feed this later).
- Real camera imagery composited into Visual mode (modes 1–2 cover
  photographic needs; a hybrid "surround ground + synthetic overlays" mode
  is deferred, not scheduled — backlog Future VM-072, no epic assigned; `[review 2026-09-07]`).
- Interactive picking / click-to-inspect via WS.
- rviz authoring tools (goal/initialpose publishing).
- Renaming the historical `cuda/` directory.

## 12. Risks

| Risk | Mitigation |
|---|---|
| Filament clang/libc++ vs ROS gcc ABI | POD-only public API boundary (§2); CI header check; gcc-source-build fallback |
| Class inference wrong on real labels | Scale always from bbox (cosmetic-only errors); adapter seam for typed topics; label-parsing table driven by a config map, tunable per deployment |
| Onboard GPU contention with perception | Quality presets + auto-drop; inactive mode costs ~0; measured early via the Phase-0 spike on robot hardware |
| MarkerArray formats drift (autonomy team refactors namespaces) | Profile YAML + generic fallback keep data visible; adapter tests use recorded fixtures so drift is caught by re-recording |
| Filament build complexity in CI | Pin one Filament release; prebuilt-binary first, vendored source build script as fallback; build cached |
| Geo-anchor wrong/absent → misplaced buildings | Environment layer independently toggleable; bake script renders a verification overlay (footprints vs recorded ego track) before deployment; no datum = layer off, not wrong |
| OSM height data sparse in operating area | Extrude `building:levels`×3 m fallback, then a default height; clay style makes approximate heights acceptable (context, not measurement) |

## 13. Implementation execution note

Per project directive (2026-08-18): implementation phases run as **dynamic
workflows** — orchestrator Fable, implementer agents Sonnet, reviewer agents
Opus. See the companion plan
`docs/superpowers/plans/2026-08-18-visual-mode.md` and backlog
`docs/superpowers/specs/2026-08-18-visual-mode-backlog.md`.
