# Height-Grid Terrain Layer (Design)

**Date:** 2026-10-06
**Status:** Approved for spec (user, 2026-10-06); implementation plan pending.
**Branch:** `feat/height-grid` (off `main`).
**Load-bearing rules:** ADR-0003 (POD-only public headers), ADR-0004 (additive scene versioning),
the standing element-config directive (every rendered element ships style tokens and a disable
knob), and the fresh-opaque / stale-translucent material convention.

## 1. Context and goal

The offroad perception stack (the perception repo, `perception_offroad_stack`) builds a
per-cell terrain height map inside the geometric costmap node, then turns its gradients into
`/perception/geometric_costmap`. Overlume already draws that costmap and the dynamic costmap as
flat coloured grids. The height map itself is never shown.

Goal: render that height map as a shaded 3D terrain surface in visual mode (mode 3), so the
operator can see berms, ditches, slopes and walls the costmaps are derived from.

Non-goals: draping cost colours onto the terrain, multiple simultaneous terrain layers,
height maps from sources other than an `OccupancyGrid`, and modes 1 and 2.

### What the topics carry today (measured live, 2026-10-06)

- `/debug_ogm_1` (geometric node, ~10 Hz): 250x250 at 0.2 m, frame `base_link`. Cell values are
  the height map **min-max normalised per frame** into 1..100, 255 (int8 −1) = unknown.
  Shape is correct; scale is not metric and changes whenever the min or max in view changes.
- `/debug_ogm_2`: publishers are declared by both costmap nodes, but no code publishes on it
  (the geometric node's call is commented out). A free slot.

## 2. User decisions (2026-10-06)

| # | Question | Decision |
|---|----------|----------|
| D1 | Where metric heights come from | The geometric node publishes a fixed-scale metric encoding on `/debug_ogm_2`. |
| D2 | Terrain vs the existing cost grids | Terrain under the costs: cost grids keep drawing at ground level; raised terrain rises through them. |
| D3 | Renderer | A new heightfield mesh (chosen by the agent, user deferred). The point-cloud renderer draws fixed-pixel sprites, which break into dots on a 0.2 m grid near the camera and add nothing over the raw lidar cloud. |
| D4 | Flat clay ground inside the terrain footprint | Replaced by the terrain (§6). Without this every ditch and every downhill slope is buried under the opaque z = 0 ground quad. |
| D5 | Encoding window | −2.0 .. +3.0 m above nominal ground, 0.05 m steps. |
| D6 | Branch | `feat/height-grid` off `main`. |

## 3. Data contract: `/debug_ogm_2` (perception side)

Published by `geometric_cost_map_node` once per processed lidar frame, with the same header
stamp, frame (`base_link`) and `info` (resolution, width, height, origin) as the
`/perception/geometric_costmap` message built from the same frame.

Cell value for local cell (i, j):

```
h_above = h_map − (ego_z + origin_z)      // metres above the grid origin plane
v       = clamp(round((h_above − min_m) / (max_m − min_m) · 100), 0, 100)
v       = −1                              // when the cell height is unknown
```

- `h_map` is the cell's value in the existing float height image (`ogm_height_image`,
  `GeometricCostMap::update_cost_map`), after the 3x3 gap fill and before the existing
  per-frame debug normalisation. Its unknown sentinel is `m_unknown_height_value` (255.0).
- `ego_z` is `translation(2)` of `local_to_global_trans` in the same `update_cost_map` call:
  the ego height in `map` that the point heights were compensated against.
- `origin_z` is `cost_map.m_position_z` (= −`lidar_fixation_height`), the same value published
  as `info.origin.position.z`. Flat ground under the ego therefore encodes as height ≈ 0.
- `min_m`, `max_m` come from a new block in
  `config/off_road/m2/geometric_cost_map_config.yaml`:
  ```yaml
  debug_height_encoding:   # /debug_ogm_2 metric heights; must match the overlume profile row
    min_m: -2.0
    max_m:  3.0
  ```
  Parsed in `GeometricCostMap::init` with those defaults when the block is absent.
  `max_m > min_m` is required; otherwise the defaults are used and a warning is printed.

Code touched in the perception repo (left uncommitted for the user; the repo is on a
detached HEAD):

- `perception_interfaces/.../cost_map.h`: `cv::Mat m_debug_height_metric_image` plus
  `float m_debug_height_min_m, m_debug_height_max_m`.
- `perception_grid_map/src/geometric_cost_map.cpp`: parse the config block in `init`,
  allocate the image, and fill it in `update_cost_map` from a small pure function
  `int8_t encode_height_cell(float h_above, float min_m, float max_m)`.
- `perception_offroad_stack/src/geometric_cost_map_node.cpp`: replace the commented-out
  `/debug_ogm_2` block with `build_occupancy_grid_from_cost_map(...)` plus a data swap from
  `m_debug_height_metric_image`, then `m_debug_ogm_2_publisher->publish(...)`, mirroring the
  existing `/debug_ogm_1` publish.

`/debug_ogm_1` is unchanged.

## 4. Public API (`overlume/include/overlume/scene.h`)

Added above `SceneGraph`:

```cpp
struct HeightGridLayer {
    Vec3 origin;              // world position of the corner of cell (0,0); the height-zero plane
    double yaw_rad;           // rotation of the grid's +x axis about world +z
    double resolution_m;      // cell edge length
    uint32_t width_cells, height_cells;
    const float* heights_m;   // width*height values, row-major (row j runs along +y);
                              // metres above origin.z; NaN = unknown
    double last_update_sec;
};
```

Appended to the end of `SceneGraph`:

```cpp
    const HeightGridLayer* height_grids;
    uint32_t height_grid_count;
```

- `kSceneVersion` 8 → 9 (additive per ADR-0004).
- LP64 layout: `sizeof(HeightGridLayer) == 64` (origin 0, yaw_rad 24, resolution_m 32,
  width_cells 40, height_cells 44, heights_m 48, last_update_sec 56);
  `sizeof(SceneGraph)` 216 → 232 (`height_grids` 216, `height_grid_count` 224). The layout
  asserts in `overlume/tests/test_scene_buffer.cpp` and
  `ros/src/overlume_ros/test/test_scene_layout.cpp` gain these values.
- Doxygen comments only; no `std::` anywhere in the header (`check_pod_header.sh`).
- The library renders every entry; the ground hole (§6) follows entry 0 only. The node sends at
  most one (§7).

## 5. Library renderer (`overlume/src/height_grid.{hpp,cpp}`)

### Data path

- `OwnedScene::assign` (`scene_buffer.{hpp,cpp}`) deep-copies each layer's `heights_m`
  (`width*height` floats) like the other layers.
- `render_frame` calls `update_height_grids(r, scene)` next to `update_ground_grids`.
- Slots: `VisualRenderer::HeightGridSlot` vector, resized to `height_grid_count`.

### Mesh

- One vertex per cell centre. Local vertex position (relative to the layer origin, to keep
  floats small): `R(yaw) · ((i + 0.5)·res, (j + 0.5)·res, 0) + (0, 0, z)`, with
  `z = h + ground_bias_m` for known cells and `z = ground_bias_m` for unknown cells. The
  renderable's transform carries `origin`.
- Two triangles per quad of four neighbouring vertices: `(w−1)(h−1)·2` triangles. The index
  buffer is `uint32` (no 65,536-vertex ceiling) and is rebuilt only when `width`/`height`
  change.
- Normals by central differences of `z` (one-sided at the edges); tangent frames via the
  existing `fill_tangent_frames`.
- Vertex attributes: `POSITION` float3, `TANGENTS` float4, `CUSTOM0` float2 =
  `(height_m, known)` where `height_m` is the raw height (0 for unknown) and `known` is 1 or 0.
- The vertex buffer is re-uploaded only when `last_update_sec`, the dimensions, the
  resolution, the yaw or `ground_bias_m` change. A test hook counts uploads.

> Amendment (Task 3): the layer origin is not a re-upload trigger. Vertex positions are stored
> relative to the origin and the renderable's transform carries it, so an origin-only change moves
> the mesh without touching the vertex buffer. `ground_bias_m` is a trigger, as the "Theme push"
> subsection already states. Pinned by HeightGrid.OriginMoveWithoutNewDataDoesNotReupload and
> HeightGrid.ThemeSwitchChangingGroundBiasReuploads. The theme cross-fade lerps `ground_bias_m`,
> so a switch between themes with different biases re-uploads every slot on every frame of the
> transition (0.8 s by default); both shipped themes use -0.05, so only custom themes hit it.
>
> Known transient (Task 3): while a slot is in its stale fade (alpha in (0,1), at most 0.5 s) the
> faded terrain renderable sits in the blended queue at default priority, so it overpaints the
> ground-grid cost quads even though it lies below them (the documented Filament blended-queue
> trap). Accepted; fix by lowering the faded renderable's priority if it is ever visible in review.

### Materials

- `assets/materials/height_grid.mat`: lit, opaque, culling off, casts no shadows, receives
  shadows. Parameters: `rampTexture` (sampler2d), `rampMinM`, `rampMaxM`, `unknownColor`
  (float3), `roughness`.
  `baseColor = mix(unknownColor, ramp(clamp((height_m − rampMinM)/(rampMaxM − rampMinM))), known)`.
- `assets/materials/height_grid_faded.mat`: same, `blending: fade`, premultiplied output
  (`rgb · alpha`, `alpha`), plus an `alpha` parameter.
- Staleness: `SceneBuffer::staleness_alpha(sim_time, last_update_sec, 0.5, 1.0)`. At alpha 1
  the slot's renderable uses the opaque instance; below 1 it is switched to a per-slot faded
  instance; at alpha 0 it is removed from the scene. (Pattern: `ribbon.cpp` fade instances.)

### Theme push

`push_theme_to_scene` bakes the theme ramp into a 256x1 RGBA32F texture spanning
[first stop, last stop] (linear interpolation, clamped ends), and sets `rampMinM`, `rampMaxM`,
`unknownColor` and `roughness` on both instances. `ground_bias_m` is read at mesh-build time;
a change re-uploads the vertex buffer.

## 6. Ground replacement inside the terrain footprint

- New materials `assets/materials/ground_clay.mat` (copy of `clay.mat`) and
  `assets/materials/ground_lines.mat` (copy of `clay_faded.mat`), each with a footprint discard:
  parameters `holeCenter` (float2, world xy), `holeAxisX` (float2, unit vector of the grid's +x),
  `holeHalfExtent` (float2, metres), `holeEnabled` (float). A fragment whose world xy falls
  inside the rotated rectangle is discarded when `holeEnabled > 0.5`.
- `groundMaterial` and `gridMaterial` instances are created from these two materials instead
  of `clay` / `clay_faded`. With `holeEnabled = 0` they render exactly as before. The shared
  `clay` and `clay_faded` materials are untouched.
- The hole follows `height_grids[0]`: centre = the mesh's centre, axes from `yaw_rad`,
  half extents `((w − 1)/2 · res, (h − 1)/2 · res)`, the extent spanned by the cell-centre
  vertices. It is enabled while that slot's staleness alpha is above 0 and the slot is in the
  scene, and disabled otherwise.
- Unknown cells render flat at `ground_bias_m` in `unknown_color` (default `palette.ground`),
  so the footprint never shows the background through the terrain.
- The cost grids keep their existing z-lifts (+0.010 m gradient, +0.015 m dynamic). With
  `ground_bias_m = −0.05`, flat terrain sits under them and centimetre-level height noise does
  not speckle through; terrain above ~+0.06 m rises through the cost grids (D2).
- When the Cesium environment provides its own ground, the clay quad is already removed; the
  terrain is drawn as is. Interplay with photoreal ground is out of scope.

## 7. Node adapter and profile (`ros/src/overlume_ros`)

### Profile row

```yaml
- {topic: /debug_ogm_2, type: nav_msgs/msg/OccupancyGrid, adapter: height_grid, role: terrain,
   encoding: height_linear, height_min_m: -2.0, height_max_m: 3.0, best_effort: true}
```

`profile.cpp` changes:

- `height_grid` is added to `KnownAdapters`.
- `RoleSets[height_grid] = {terrain}`.
- `TypeSets[height_grid] = {nav_msgs/msg/OccupancyGrid}`.
- New row keys `height_min_m` and `height_max_m` (doubles), valid only on `height_grid` rows,
  both required there, with `height_max_m > height_min_m`.
- `encoding` on `height_grid` rows accepts `height_linear` (default) or `height_normalized`.
  The `ogm` adapter's `occupancy`/`costmap` values are unchanged.
- `frame_id` is allowed on `height_grid` rows; `update_topic` is rejected.
- At most one `height_grid` row per profile (validation error otherwise).

`ProfileRow` gains `double height_min_m{0.0}, height_max_m{0.0}`.

### Decoding (`HeightGridAdapter`, `src/adapters/height_grid.{hpp,cpp}`)

| Cell (int8) | `height_linear` | `height_normalized` |
|---|---|---|
| −1 | NaN (unknown) | NaN (unknown) |
| 0..100 | `min + v/100 · (max − min)` | `min + clamp((v − 1)/99, 0, 1) · (max − min)` |
| any other | NaN; the message counts once in `dropped_malformed` | same |

`height_normalized` exists to A/B `/debug_ogm_1` against `/debug_ogm_2`; it is documented as
not metric.

Size checks match `OgmAdapter` (zero dims or `data.size() != w·h` → `dropped_malformed`, message
dropped).

### Placement

The TF lookup, `frame_id` override, origin transform, `flatten_z` and yaw composition currently
inline in `OgmAdapter::ingest` move into one helper (`src/adapters/grid_placement.{hpp,cpp}`)
that both adapters call. `OgmAdapter` behaviour is unchanged; `test_ogm_adapter` stays green
without edits.

### Node wiring

- `overlume_node.cpp`: a `height_grid` row loop modelled on the OGM loop (QoS depth 10,
  `best_effort` / `transient_local` honoured), a `HeightGridRow{adapter, timeout_sec, topic}`
  vector, the per-tick staleness gate and `fill`, and a diagnostics row. Rows are cleared in
  the same two places as the others.
- `SceneAssembly` gains `std::vector<overlume::HeightGridLayer> height_grids`, handled in
  `clear()` and `point_at()`.
- Disable knob: node param `layer_height_grids` (default `true`, live-settable in `on_params`,
  listed in `config/default_params.yaml`). `LayerFlags` gains `bool height_grids = true`
  (its `static_assert` becomes 9). `mode_content_mask`: false for BOWL and HYBRID, true for
  FREE_LOOK. `compose_layer_gates` and `apply_layer_gates` handle it.
- `config/robot-offroad_profile.yaml` gets the row above.

## 8. Theme tokens

New optional block, soft-defaulted in `theme.cpp`, present in `kFallbackTheme()`, blended in
`theme_transition.cpp::blend` (Oklab colours, lerped floats, ramp stops blended pairwise when
the stop counts match, otherwise snapped at t = 0.5), and pushed in `push_theme_to_scene`:

```yaml
height_grid:
  ramp:                      # colour by height above nominal ground, metres
    - { height_m: -1.0, color: [...] }
    - { height_m:  0.0, color: [...] }
    - { height_m:  0.5, color: [...] }
    - { height_m:  1.5, color: [...] }
    - { height_m:  2.5, color: [...] }
  unknown_color: [...]       # default: palette.ground
  roughness: 0.9
  ground_bias_m: -0.05
```

Fallback when the block or ramp is absent: two stops, `palette.ground` at 0.0 m and
`palette.alert.warning` at 2.5 m.

Starting values (tuned by eye later):

| stop (m) | dark_adas | light_clay |
|---|---|---|
| −1.0 | [0.08, 0.12, 0.24] | [0.55, 0.62, 0.72] |
| 0.0 | [0.055, 0.055, 0.078] (= palette.ground) | [0.762, 0.716, 0.672] (= palette.ground) |
| 0.5 | [0.16, 0.26, 0.18] | [0.66, 0.70, 0.56] |
| 1.5 | [0.55, 0.46, 0.26] | [0.80, 0.66, 0.46] |
| 2.5 | [0.85, 0.84, 0.78] | [0.95, 0.93, 0.88] |

`unknown_color` = palette.ground in both, so unknown patches read as plain ground.

## 9. Testing and checks

Each item fails if the behaviour it guards is reverted.

- **Library (`overlume/tests/test_height_grid.cpp`, hooks in `height_grid_test_hooks.hpp`):**
  - vertex count and triangle count for a w×h grid;
  - a known cell's vertex z = origin.z + h + bias, an unknown cell's = origin.z + bias, and its
    `known` attribute is 0;
  - yaw rotates vertex positions;
  - no re-upload when `last_update_sec` is unchanged; a re-upload when it changes;
  - stale fade: alpha 1 binds the opaque instance, alpha in (0,1) the faded one, alpha 0 removes
    the renderable;
  - ground hole: enabled with the expected centre, axis and half extents while a layer is
    present, disabled when it is absent or fully stale;
  - layout asserts and `kSceneVersion == 9` in `test_scene_buffer.cpp`.
- **Theme (`test_theme.cpp`, `test_theme_transition.cpp`):** the block parses, the fallback
  applies when absent, and a transition midpoint blends ramp colours and `ground_bias_m`.
- **Goldens:** `height_grid_terrain_dark_adas.png` and `height_grid_terrain_light_clay.png` from a
  synthetic scene (a berm, a ditch, a sloped half and an unknown patch, with a cost grid
  overlaid). Candidates are shown to the user; they are committed only after the user promotes
  them. Existing goldens must stay green; the ground-material swap with the hole disabled must
  not move any of them.
- **Node:** `test_height_grid_adapter.cpp` (both decodes, NaN on −1, malformed counting, size
  checks, flatten and yaw, `frame_id` override), `test_profile.cpp` (new keys, required keys,
  `max > min`, encoding values, one-row limit, key scoping), `test_scene_assembly.cpp`
  (`height_grids` flag and mode masks), `test_scene_layout.cpp` (layout and version).
- **Live probe (`tools/probe_height_grid.py`):** subscribes to `/debug_ogm_1` and `/debug_ogm_2`
  (best effort), pairs messages by stamp, and prints PASS/FAIL for:
  `/debug_ogm_2` at ≥ 5 Hz; identical unknown masks; Pearson r > 0.99 between `/debug_ogm_1`
  values and decoded `/debug_ogm_2` heights over known, unclamped cells. `--self-test` runs the
  same comparison on a synthetic round trip without ROS traffic.
- **Gate:** every task ends green on `tools/ci_visual_mode.sh` (foreground).

## 10. Delivery

- Spec: this file. Plan: `docs/plans/2026-10-06-height-grid-terrain.md`, with its own status
  ledger.
- Implementation as a dynamic workflow (session-model orchestrator, Sonnet implementers, Opus
  review gates, at most two fix rounds per task), one commit per task.
- Docs: `docs/runbooks/profile_authoring.md` (adapter list, table and row-key semantics),
  `CHANGELOG.md` `[Unreleased]` (`kSceneVersion` 8 → 9), `docs/status.md` ledger entry, theme
  token documentation in `docs/runbooks/theme_showcase.md`.
- Perception change in the perception repo, built with its usual colcon build and left
  uncommitted for the user.

## 11. Acceptance

1. `tools/ci_visual_mode.sh` green, including the promoted goldens.
2. Live CARLA offroad run (`LIVE_SIM_TIME=true tools/validate_visual_mode.sh --live --profile
   robot-offroad`, costmap nodes restarted on the new perception build):
   `tools/probe_height_grid.py` PASS; the terrain is visible in mode 3; berms rise through the
   cost grids; dips are visible; `layer_height_grids:=false` hides the terrain and restores the
   clay ground; modes 1 and 2 show no terrain.
3. Per-update CPU cost of a 250x250 grid (decode, mesh, normals, upload) under 3 ms on the dev
   box, read from `render_ms` on the diagnostics topic during the live run.

## 12. Risks and known limits

- The height map is accumulated in `map`; heights far behind the ego after a long climb or
  descent clamp at the window edges (−2.0 / +3.0 m). The window is a config value on both sides.
- During the 0.5 s fade after data stops, the hole is still open under translucent terrain, so
  the background shows through briefly.
- With `flatten_z: false` the grid sits on its origin plane in `base_link` (1.2 m below the
  sim's ground), exactly like the existing cost grids. Default is `flatten_z: true`.
- The encoding window is duplicated in two configs (perception yaml and the overlume profile),
  and each carries a comment naming the other. The live probe's checks are scale-invariant, so
  a window mismatch passes them; it shows only as wrong terrain heights by eye.
