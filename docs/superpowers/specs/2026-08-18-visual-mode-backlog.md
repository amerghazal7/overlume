# Visual Mode — Backlog

Companion to `2026-08-18-visual-mode-design.md`. Ordered by phase; each story
lists acceptance criteria (AC). Execution: dynamic workflows (orch Fable /
impl Sonnet / review Opus) per project directive. Load-bearing decisions live in
`docs/adr/0001`–`0004`; interface changes follow ADR-0004 (additive-only, versioned).
Status per task: the **Status ledger** at the top of each epic plan.

## Epic 0 — Contract spike (de-risk everything first)

- **VM-001 Filament headless hello-frame.** Build a minimal clang/libc++
  static `visual_renderer` lib (POD API) that renders a lit cube on a grid
  to a readable headless swapchain and returns RGB8.
  AC: standalone example writes a PNG on the dev box; no X server; header
  passes the POD-only check. **DONE** (66c1340, bc2007c, 76ce179, acd26a8, 73f100c).
- **VM-002 Skeleton visualization node + mode mux.** Lifecycle node
  publishing VM-001's frame to `/rendering/image` at 30 Hz; global
  `/rendering/set_mode` handling in BOTH nodes (CUDA node idles on 3).
  AC: smoke test drives 1→3→2 with exactly-one-publisher and no consumer
  re-subscribe; existing node tests still green. **DONE** (dfab33b, 2968ebc); residual mux defects → VM-037.
- **VM-003 vcam parity.** `~/set_virtual_cam` / `~/set_look` / `~/vcam_state`
  on the new node; WS bridge fan-out.
  AC: contract test proves identical framing math vs CUDA node for the 5
  presets; GUI orbit (`set_look`) works against mode 3 with no GUI change (the GUI later gained a mode button/3-way view cycle by directive). **DONE** (6a43f55, e619d41).
- **VM-004 GPU budget measurement.** `[review 2026-09-07]` Split: (a) PROXY on
  the dev box (same CPU/GPU class), bag replay, alone and beside the CUDA node
  — DONE 2026-09-07, `cuda/src/libs/visual_renderer/tools/budget_probe.md`;
  (b) ON-ROBOT beside CUDA node + perception with real camera input, frame
  time p50/p99 via the `render_ms` instrumentation from VM-034 — OPEN,
  blocks VM-043 sign-off. The original "gate for Epic 1+" was overridden in
  practice (Epic 1 proceeded on assumptions); the docs now say so.
  AC: proxy table recorded; on-robot table recorded; go/adjust on 720p30.

## Epic 1 — Core scene & dark theme

- **VM-010 SceneGraph + double buffer.** Domain structs, staging/render swap.
  AC: unit tests for swap semantics and staleness fade timers.
- **VM-011 Theme system.** YAML tokens → materials; BOTH `dark_adas.yaml`
  and `light_clay.yaml` ship here (v1, per product direction 2026-08-18).
  AC: golden image of an empty themed world (ground, grid, sky/fog) per
  theme; no per-theme code branches.
- **VM-014 Animated theme toggle.** `set_theme` eases all tokens
  current→target (default 0.8 s smoothstep, Oklab palette lerp, sun/IBL
  crossfade); GUI day/night toggle + WS command.
  AC: golden at t=0/0.4/0.8 s of a transition (deterministic clock);
  no frame drop > 1 during switch; toggle mid-transition retargets smoothly.
- **VM-012 Ego robot.** M02P→glTF conversion script; TF-driven ego pose +
  speed; clay-box fallback.
  AC: golden image with ego; speed matches TF finite difference on a
  recorded fixture. **Partial `[review 2026-09-07]`:** conversion ran (`~/Downloads/M02P.glb`, 2026-08-20) but the asset is outside git and `default_params.yaml` carries a per-user absolute path; the only committed ego golden is the clay-box fallback. Closes via VM-044 (install the asset, re-shoot the ego golden). Blocks VM-043 sign-off.
- **VM-013 Camera presets/tween port.** Eased tween identical to CUDA node.
  AC: contract test vectors pass.

## Epic 2 — Autonomy data ingestion

- **VM-020 Profile YAML loader** (urban/offroad/sim topic rows → subscriptions; `[review 2026-09-07]` three profiles ship — sim adds the latched `/sim/hd_map/markers` row).
  AC: bad rows rejected with clear errors; all three shipped profiles load.
- **VM-021 DynamicObjectsAdapter + class inference.** MarkerArray →
  TrackedObjects; label keywords + bbox-dims fallback table (config-driven).
  AC: fixture tests from a recorded bag; inference table test; malformed
  markers dropped and counted.
- **VM-022 Clay object rendering.** CC0 glTF pack normalized; instancing;
  bbox-driven scaling; velocity arrows; predicted-path ribbons.
  AC: golden image with a mixed-class scene; 50 objects < 2 ms scene-update.
- **VM-023 Path ribbons.** Behavior (hero emissive) + global + local.
  AC: golden image; ribbon regenerates correctly on path change.
- **VM-024 HdMapAdapter + lane styling.**
  AC: golden from recorded HD-map fixture; cached (no per-frame rebuild).
- **VM-025 OGM ground layers** (dynamic + gradient, with `_updates`).
  AC: fixture test incl. partial updates; offroad profile golden.
- **VM-026 Collision alert polygons.**
  AC: golden with sweep + predicted polygons in warning materials.
- **VM-027 Generic marker fallback renderer.** All Marker primitive types,
  pooled renderables.
  AC: parity test rendering a synthetic MarkerArray of every type; any
  extra topic displayable via one profile row.

## Epic 3 — HUD, polish, controls

- **VM-030 HUD overlay.** Speed chip, mode indicator.
  **Accepted (user 2026-09-07) `[review 2026-09-07]`:** draw the 2D HUD node-side by compositing
  text onto the RGB8 buffer after readback (stb_truetype, vendored like
  stb_image_write) instead of an SDF atlas + Filament overlay pass inside the
  lib — identical pixels for screen-space chips, no font pipeline in the
  clang/libc++ archive, no new public entry point **(amended, user 2026-09-07: `get_hud_colors()` accepted as the one exception — see Epic 3 plan's Task 3 for the rationale; P2's substance otherwise stands)**. The SDF/in-scene path is
  kept only for 3D-anchored text (VM-031). Re-entry trigger: HUD text must
  be depth-tested, lit, or fogged.
  Prerequisite (`[review 2026-09-07]`): the node never populates `SceneGraph::hud` today — add `scene.hud.speed_mps = ego.speed_mps; scene.hud.active_mode = active_mode_` next to the `~/ego_state` publish.
  AC: node-side test renders a known frame + HUD; HUD reads `SceneGraph::hud`; text legible at 720p low preset.
- **VM-031 Alert callouts.** Leader-line chips anchored to 3D objects
  (e.g. nearest-obstacle distance from collision topics).
  AC: golden; callout tracks object across camera moves.
- **VM-032 Layer visibility + quality presets** end to end (params + WS
  commands + GUI panel; theme toggle already shipped in VM-014).
  `[review 2026-09-07]` `RenderConfig::quality` today drives only SSAO enable+resolution and FXAA-vs-TAA. This item must also map spec §8's three deferred knobs: shadow-map resolution (2048 high / 1024 medium), shadow enable (low = none), and the 960×540 render-scale upscale for `low`. Live switching needs an appended `set_quality()` entry point or a renderer re-create (see VM-040, decision P4 accepted: settle it in the Epic 5 plan).
  AC: WS E2E test toggles each layer and preset; each preset measurably changes frame cost (`render_ms`).
- **VM-033 (moved into VM-011/VM-014 — both themes + animated toggle ship
  in Epic 1.)**
- **VM-034 Staleness fades + diagnostics topic** surfaced in GUI.
  `[review 2026-09-07]` Also: (1) HD-map layer fade — needs
  `MapElement.last_update_sec` (appended under ADR-0004; closes the recorded
  "pops, does not fade" deviation); (2) **`render_ms`**: time `render_frame()`
  in the node tick and publish it in the diagnostics message — prerequisite
  for VM-040's governor and for the on-robot VM-004(b) table.
  AC: silencing a topic fades its layer (map included); diagnostics shows
  per-topic age, dropped-primitive counts and render_ms.
- **VM-035 PointCloudLayer** (user request 2026-08-20). `sensor_msgs/PointCloud2`
  ingestion + point rendering. Adds a `PointCloud` category to `SceneGraph`
  (appended, `kSceneVersion` bump — ADR-0004; `[review 2026-09-07]` the former
  "Epic-3 freeze lift" wording is superseded). Node side: `PointCloudAdapter`, one
  profile row per cloud topic with ingest decimation knobs (`max_points`,
  `stride`, `max_rate_hz`) — full-rate lidar is never drawn raw. **Coloring is
  per-row config: `color_mode: auto | rgb | intensity | height | flat` (default
  `auto`)** — the adapter inspects the cloud's actual fields per message: `rgb`/
  `rgba` used directly when present; else `intensity` normalized over the row's
  `intensity_range` (auto-ranged when unset) through a theme color ramp; else
  height through the same ramp; `flat` = single theme point color. The adapter
  bakes rgba8 per point node-side, so the renderer keeps ONE points path:
  per-vertex COLOR, layer-wide fade via the shared `staleness_alpha`, chunked
  under the uint16 index ceiling like every polyline.
  AC: goldens from three synthetic clouds (rgb, intensity-only, bare XYZ)
  proving each `auto` tier; adapter test proves field detection + decimation
  counts; one YAML row displays any PointCloud2 (§7 parity extended to clouds).

- **VM-036 MapElement.kind — per-kind lane styling** (user request 2026-08-20).
  Epic 2 renders every HD-map polyline in one `palette.lane_paint` style, so a
  lane reads as three identical stripes (centerline + left/right boundary) —
  live feedback called it "repeated, shifted, a mess". The wire already
  carries the semantics; the frozen `MapElement {points, point_count,
  is_polygon}` just cannot cross them to the renderer. Marker-array
  convention (authoritative, from the autonomy stack, 2026-08-20):
  - lane centerline: one marker per lane, LINE_STRIP, ns `centerline_{lane_id}`, id `{lane_id}`
  - lane boundaries: one marker per side, LINE_STRIP, ns `left_boundary_{lane_id}` / `right_boundary_{lane_id}`, id `{lane_id}`
  - centerline arrows: one marker per point, ARROW, ns `centerline_arrows_{lane_id}`, id `{point_index}` (dropped by rule in Epic 2 — 93% of map volume; direction is derivable from centerline point order)
  - crosswalks: one marker each, LINE_STRIP, ns `crosswalks` (sim, plural, no suffix) or `crosswalk_{id}` (urban publisher) — both matched by the shipped bare `crosswalk` prefix rule since 2026-08-20
  `[review 2026-09-07]` `MapElement` gains `kind` (CENTERLINE, LEFT_BOUNDARY,
  RIGHT_BOUNDARY, CROSSWALK, STOPLINE, JUNCTION, OTHER), `lane_id` (uint32,
  0 = none) and `last_update_sec` — all appended (ADR-0004). `HdMapAdapter`
  fills them from the row's namespace rules (`classify()` already knows; today
  it collapses to polyline/polygon). Renderer: per-kind theme tokens
  (`palette.lane_centerline`, `palette.lane_boundary`, `palette.crosswalk`) +
  per-kind width/z-lift so boundaries read thin, centerlines read as the lane
  spine, and coplanar strips stop z-fighting. **Road surface:** pair
  `left_boundary_{id}`/`right_boundary_{id}` by `lane_id` and fill the strip
  between them in a new `palette.road` token — ref-2's primary value
  separation (road darker than the clay ground) is unexpressible today because
  every map element is a stroke and the whole ground plane carries the road
  tone (`light_clay.yaml` `palette.ground` comment). Ground returns to a light
  clay value once the road is its own surface. Move centerline dashing
  renderer-side (one element per centerline) and retire the ingest chop.
  AC: golden per theme with all kinds visually distinct AND road darker than
  ground; adapter test proves ns→kind+lane_id mapping for both crosswalk
  spellings; dashed-centerline golden unchanged after the chop is retired.
  **STYLING GROUND TRUTH (user directive 2026-08-20, binding for every theme/
  styling task):** `assets/visualization-reference-1.jpg` is THE dark-theme
  target and `assets/visualization-reference-2.jpg` THE light-theme target —
  compare colors, value separation, and accent saturation against them, not
  against taste. What they teach that the shipped `light_clay` currently
  gets wrong: ref-2 separates VALUES (road visibly darker than buildings/
  ground; sky a real blue, not fog-white), keeps lane paint semantic
  (yellow centerlines vs white dashes), and spends saturation only on
  meaning (blue hero ribbon, coral alert vehicle, dark navy ego) while
  everything inert stays clay. Ref-1's dark world does the same with a
  near-black blue base, white lane paint, and the green emissive hero.
  **Debt found by the 2026-09-07 review (fix inside VM-036, one re-authoring pass, goldens re-promoted):**
  1. **Crosswalk hatch is dead code**: recorded crosswalks arrive as closed 5-point polylines; `build_crosswalk_hatch()` guards `n != 4`. Drop the duplicate closing vertex in `hd_map.cpp` (mirror `collision.cpp`'s dedupe) + a test on the recorded geometry. Do this first; it is the only pure defect here.
  2. **Dash assignment must FLIP, not just gain colour**: Epic 2 dashes `centerline_` (the lane spine, no painted analogue) and leaves boundaries solid — the inverse of ref-2's "yellow centerlines vs white dashes". Target: CENTERLINE solid, semantic yellow-family; BOUNDARY dashed white (or solid at road edge once `kind` distinguishes them).
  3. **Token debt**: (a) `dark_adas.palette.lane_paint` [0.45,0.5,0.55] is mid-gray; ref-1 wants near-white (light_clay already ships [0.92,0.92,0.90]); (b) both themes saturate every inert object class — re-author car/truck_van/unknown toward clay, keep saturation for pedestrian/cyclist and alerts; (c) `dark_adas.ribbon_local` [0.95,0.70,0.15] is within 0.05/channel of `alert.warning` — move ribbon_local to a cool hue and reserve amber for alerts.
  4. **Guards**: split the horizon/sky convergence bound per theme (dark ~37 → <45, light ~45 → <55) instead of one shared 55; add `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml`; add a `blend()` field-count static_assert so an appended token cannot silently blend to black; add a `map_element_rebuild_count` hook + test for the "cached, no per-frame rebuild" AC.
  **Already shipped in Epic 2 (2026-08-20, do not redo):** centerlines render
  DASHED — the adapter chops them into per-dash `MapElement`s at ingest
  (profile rule flag `dashed: true`, 1.5 m dash / 1.5 m gap constants), the
  one differentiation expressible without the `kind` field. VM-036's color/
  width tokens layer on top; when `kind` lands, consider moving dashing
  renderer-side (one element per centerline again) and retiring the chop.

- **VM-037 Epic-0 debt: mux hardening + build hygiene** (`[review 2026-09-07]`, small, do first in Epic 3).
  (a) `/rendering/set_mode` QoS → `transient_local, depth 1, reliable` on every publisher/subscriber (restart/late-join rejoins the live mode); (b) legacy `~/set_render_mode` re-publishes on the global topic (no one-sided exit from mode 3); (c) `initial_mode:=1` also sets `render_mode_` (today starts hybrid); (d) append `vcam_state[8] = mux_mode` on both nodes, index 7 unchanged; (e) `kSceneVersion` constant in `scene.h` + a node-side gtest mirroring the sizeof/offsetof table (ADR-0004); (f) `check_pod_header.sh` as an `ament_add_test` in the node package, glob widened to `*.h*`; (g) `FILAMENT_VERSION` single-sourced (node CMake reads it from `GetFilament.cmake`); (h) log `GL_VENDOR/GL_RENDERER/GL_VERSION` once at `create_renderer()` and make `test_hello_frame` skip (not fail) when no hardware GL device is found; (i) link-probe assertion on the hand-declared `bluegl::bind()` signature.
  AC: smoke test drives restart-in-mode-3 and legacy-topic exit with exactly-one-publisher; `colcon_build.sh` runs the POD check; a deliberate `scene.h` layout change fails the node build; hello-frame log names the GPU.
  Scheduled: Epic 3 Task 7 (user decision 2026-09-07); item (e) ships earlier, inside Epic 3 Task 1.

## Epic 4 — Clay buildings (EnvironmentLayer, §4.5)

- **VM-050 Geo-anchor.** `geo_datum` param (lat/lon/heading of map origin)
  or NavSatFix+`gps_link` sampling → local-ENU transform; layer disabled
  with WARN when absent.
  AC: unit test round-trips WGS84↔map-frame within 0.1 m over a 2 km area.
- **VM-051 Bake pipeline.** `scripts/bake_environment.py`: bbox → OSM
  footprints+heights (Overpass or local extract; Mapbox MVT as alternative
  source) → extruded, chunked glTF + tileset index JSON; verification
  overlay image (footprints vs a recorded ego track).
  AC: bake of the CARLA-town / real operating area completes offline from a
  cached extract; overlay image sanity-approved.
- **VM-052 Runtime chunk loading + culling.** Index → distance-enabled
  chunks, theme building material.
  AC: golden with baked town; frame-time delta < 2 ms at target preset.
- **VM-053 → promoted to Epic 6 (v1.1).** See below.

## Epic 6 — v1.1: 3D Tiles streaming (committed, starts immediately after v1.0)

`[review 2026-09-07]` The review proposed deferring this epic; the user rejected
that on 2026-09-07 — it stays committed v1.1.

- **VM-060 Cesium ion registration + tileset access.** User performs the
  registration (offered 2026-08-18); obtain a Cesium OSM Buildings token,
  store like the Mapbox token (env var, never committed).
  AC: token retrieves tileset.json for the operating area.
- **VM-061 cesium-native build integration.** Pin a cesium-native release
  behind the same clang/libc++ + POD-boundary rules as Filament.
  AC: builds in CI alongside Filament; POD header check still passes.
- **VM-062 3D Tiles streaming EnvironmentSource.** cesium-native tile
  selection/loading behind the `EnvironmentSource` seam (VM-052); glTF tile
  payloads re-materialized with the theme's clay building material; geo
  placement via the VM-050 anchor; disk tile cache for offline robustness.
  AC: baked-source goldens still pass with the streaming source swapped in
  over the same area; frame-time budget held while streaming.
- **VM-063 Source selection + fallback.** Profile/param chooses
  `baked | streamed`; streamed falls back to baked chunks on network loss.
  AC: fallback e2e test (kill network mid-run → baked chunks appear, WARN).

## Epic 5 — Hardening & delivery

- **VM-064 Google Photorealistic 3D Tiles: original-materials mode + 3-preset
  source config** (added 2026-09-11, user decision). `materials=original|clay`
  query key on the `ion://` source URI; `original` skips the clay
  `buildingMaterial` remap so Google's textured photoreal tiles keep their own
  materials; config file documents all three presets (OSM Buildings 96188 clay
  / own clipped clay / Google photorealistic). Includes attribution surfacing
  + cache-terms compliance (documented checks) and a fixture-redistribution
  legality check before any golden.
  AC: switching between the three documented presets is a one-line YAML edit;
  `original` mode provably skips the remap (test); attribution + cache terms
  recorded in the runbook.
- **VM-040 Quality auto-drop with hysteresis.** `[review 2026-09-07]` Depends
  on VM-034's `render_ms`; the governor lives node-side (it already owns
  `quality` as a parameter and the lib's `RenderConfig` is create-time only —
  a live preset change needs either a `set_quality()` entry point (appended,
  ADR-0004) or a renderer re-create; decide in the Epic 5 plan).
  AC: synthetic-load test triggers drop + log; recovers.
- **VM-041 Perf benchmark + repo-local CI gate.** `[review 2026-09-07]` The
  repo has no hosted CI (no `.github/workflows`, no `.gitlab-ci.yml`). "CI
  wiring" means one `tools/ci_visual_mode.sh` running POD check, lib ctest,
  node gtests, WS bridge pytest and goldens with GPU-skip, documented as the
  pre-merge gate; hosted CI follows when a platform exists.
  AC: clean-checkout build + `ci_visual_mode.sh` green, documented and reproducible.
- **VM-042 Docs + runbook.** README section, profile-authoring guide for the
  autonomy team, environment-bake guide, deployment notes.
  `[2026-09-11]` Docs half shipped (commit 85013dd):
  `docs/visual_mode/profile_authoring.md` + `environment_bake.md` + README
  section. Deployment/two-node-topology notes DEFERRED to VM-095 (unified-
  engine cutover) — the migration deletes that topology; see
  `docs/visual_mode/README.md`.
  AC: autonomy-team member can add a topic via profile YAML using only docs.
- **VM-044 Package theme + ego assets for a real install** (`[review 2026-09-07]`). Today `DEFAULT_THEME_ASSETS_DIR` compiles in this checkout's path, the node leaves `RenderConfig::theme_assets_dir` null, no ROS param selects the initial theme (always dark_adas at launch), and `ego_model_path` defaults to a per-user `~/Downloads` path — off this dev box the node silently runs the compiled-in fallback theme with a clay-box ego (WARN only). Install `assets/themes` and the converted ego `.glb` (Git LFS or a fetch script), resolve them via `ament_index`, add an `initial_theme` param.
  AC: clean clone + build on another machine shows both themes and the ego mesh; `ros2 param get` shows the resolved paths. Blocks VM-043.

  **Done (2026-09-11):** `micropilot_visualization_node/CMakeLists.txt` installs
  `assets/themes` and the class-model glTF pack (both plain-git, following the
  existing car/pedestrian/truck_van convention) from `visual_renderer` into
  this package's own `share/`; the converted M02P ego glTF (~72 MB, past that
  convention's practical size, and no Git LFS remote is configured in this
  repo) is provisioned instead by a new
  `scripts/provision_ego_model.sh` (runs the existing
  `obj2gltf_m02p.py` against the real `M02P.obj`; gitignored output,
  installed only if present, WARNs at configure time otherwise — an honest
  gap, not a build error). `visualization_node.cpp`'s `on_configure()` gained
  `theme_assets_dir` (default `""` → resolved via `ament_index_cpp` to the
  installed `assets/themes`), `initial_theme` (default `dark_adas`, configure
  FAILS if the yaml isn't actually installed), and resolves `ego_model_path`
  (default `""`) to the installed `assets/ego/M02P.glb` when present; all
  three write the resolved value back via `set_parameter()` so `ros2 param
  get` reports it, not the `""` default. No `visual_renderer` change needed —
  `RenderConfig::theme_assets_dir`/`initial_theme` already existed, the node
  simply never populated them. AC verified by running the node from a scratch
  colcon install with `HOME` pointed at an empty temp dir (empty
  `~/Downloads`, simulating a clean machine): both `dark_adas` and
  `light_clay` load with no fallback WARN, `ros2 param get` shows
  `theme_assets_dir`/`ego_model_path` resolved into the install `share/`
  path, not a per-user one. Node suite green (20/20 ctest). `hud_font_path`/
  `environment_chunks_dir` are the same CLASS of per-checkout-path gap but
  outside this entry's verbatim scope — left open, comments updated to say so
  honestly rather than implying VM-044 closed them.
- **VM-043 Live validation.** Full stack on CARLA bridge + a real-robot bag;
  side-by-side review vs rviz for parity sign-off. `[review 2026-09-07]`
  Includes the on-robot budget table (VM-004(b)) as a blocking checklist item.
  AC: product + autonomy sign-off checklist complete, budget table recorded.

- **VM-078 Object-rendering opacity control** (added 2026-09-11, user request:
  "can we offer apacity control for the objects rendering? (boxes or clay
  models)"). A theme style token `objects.opacity` (soft-default 1.0,
  explicit in both shipped themes) controlling TrackedObject entity opacity —
  boxes and glTF class models alike. Fresh-opaque convention holds: at 1.0
  (default) objects render on today's opaque clay path byte-identically; below
  1.0 they ride the EXISTING per-entity clay_translucent staleness-swap
  machinery with alpha = opacity * staleness_alpha (never a permanently
  fade-blended material — the 2026-09-10 flicker rule). Blended into theme
  transitions (sentinel field-coverage test extended, not a parallel test).
  AC: default renders pixel-identical to today; a 0.5 opacity theme shows
  see-through objects whose staleness fade still works; both theme YAMLs
  carry explicit values.

  **Done (2026-09-11):** `objects.opacity` added to `Theme`/parse()/ Known ceiling (gate minor, deliberate): `objects.opacity` governs the object BODY only — the velocity arrow and predicted-path ribbon keep their shared opaque material (see objects.cpp's own note; upgrade path recorded there).
  blend()/both YAMLs; `update_entity_staleness()` (objects.cpp) now binds
  alpha = staleness_alpha * objects.opacity through the existing
  clay_translucent swap — a two-line change, no new material or path.
  Sentinel field-coverage test extended (test_theme_transition.cpp), not
  duplicated. New tests: ThemeObjects.* (test_theme.cpp, parse/soft-default/
  blend) and Objects.HalfOpacity* (test_objects.cpp, fresh-translucent +
  staleness-ramps-down-from-ceiling), plus the `objects_half_opacity.yaml`
  fixture. Full suite green (199/199), goldens pixel-identical.

## Future (explicitly deferred)

`[review 2026-09-07]` IDs and re-entry triggers so other docs can cite them:

- **VM-070 Minimap inset** (can reuse bake data) — trigger: product asks for orientation context the 3D view cannot give.
- **VM-071 Typed perception-topic adapter** — trigger: the autonomy stack publishes a typed object list (the `DynamicObjectsAdapter` seam already accepts one).
- **VM-072 Hybrid mode** (camera-imagery ground + synthetic overlays) — trigger: a product request for photographic ground that modes 1–2 cannot serve.
- **VM-073 Interactive picking over WS** — trigger: a client needs click-to-inspect.
- **VM-074 Async readback** — trigger: on-robot `render_ms` p99 shows readback dominating.
- **VM-075 Wheel/turn animations on clay models**; **VM-076 `cuda/` directory rename** — cosmetic, no trigger.
