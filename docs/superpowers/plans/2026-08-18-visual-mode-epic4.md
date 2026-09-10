# Visual Mode — Epic 4 Implementation Plan (Clay buildings / EnvironmentLayer)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18):** run this epic as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Status: NOT STARTED.** Verified 2026-09-09: no `geo_anchor`, `environment`, `bake_environment`, `EnvironmentSource`, `Mapbox`, or `Overpass` symbol exists anywhere under `cuda/` (`git grep -lni "mapbox\|overpass\|environment_source\|geo_anchor\|bake_environment" -- cuda` returns nothing, and Epic 3's own gate bullet 20 independently confirmed the same emptiness at its own close). This plan is authored from the current, verified state of the repo — no code has been written against it yet.

## Status ledger

**Plan APPROVED by the user 2026-09-10 ("Approve Epic 4 plan, proceed").** Execution starts with Task 1 (VM-050) once the in-flight carpet-as-ribbon redirect (vm077 plan, 2026-09-10) lands — that work edits the same node files Task 1 wires into, and this project runs tasks sequentially against a green tree.

| Task | Backlog | Status | Notes |
|---|---|---|---|
| 1 Geo-anchor: NavSatFix (WGS84, PRIMARY) + `gps_link` TF → map↔WGS84 anchor; `geo_datum` param override | VM-050 | **Done 2026-09-10** | All 6 steps landed (Step 5 = the commit carrying this note); also performed the scene.h `GeoAnchor` append + `kSceneVersion` 3→4 bump + both layout tables (sequencing deviation below), pulled forward from Task 3 since Task 1 is the type's first consumer. |
| 2 `bake_environment.py`: OSM footprints → chunked, georeferenced glTF + index + verification overlay | VM-051 | **Not started** | — |
| 3 Runtime chunk load + distance culling behind an `EnvironmentSource` seam, clay building material | VM-052 | **Done 2026-09-10** | All 7 steps landed (Step 6 = the commit carrying this note); `GeoAnchor`/`kSceneVersion` already landed by Task 1 (see deviation note) — this task only ADDS `set_environment_source` (no second version bump) and consumes the existing type. Review round 1 findings fixed same day — see Task 3's own deviation notes below. |

**Dated deviation note (2026-09-10, decided at Task 1 kickoff):** Task 1's `geo_anchor.hpp` `#include`s `visual_renderer/scene.h` for `mpviz::GeoAnchor`, but the plan's Files lists originally put the `GeoAnchor` struct append under Task 3 (VM-052). A type lands with its FIRST consumer: Task 1 performed the `scene.h` `GeoAnchor` append + `kSceneVersion` 3→4 bump + BOTH layout-table updates (`tests/test_scene_buffer.cpp` and the node's `test_scene_layout.cpp`), copying Task 3's own field spec verbatim (pure-POD, append-only, `check_pod_header` green). Task 1's Files/Interfaces lists below and Task 3's Files list further down are updated accordingly; the kSceneVersion review-gate bullet under "Review gate" is updated to say Task 1 performed the bump and Task 3 only consumes the type (adding `set_environment_source`, no second bump). Nothing else of Task 3 was pulled forward.

**Parent plan:** `docs/superpowers/plans/2026-08-18-visual-mode.md` (Global Constraints bind this doc too — re-read them before starting; restated below). Master table row at lines 317-326.
**ADR:** `docs/adr/0004-scene-interface-versioning.md` — additive-only, versioned interfaces. This epic makes **no** new `SceneGraph` field/category (see Decision 1 below for why). It does append one new POD struct, `GeoAnchor`, and one new free function, `set_environment_source` (Decision 2). Per ADR-0004's own text — "`scene.h` carries `constexpr uint32_t kSceneVersion` bumped on every change" — `kSceneVersion` bumps **3 → 4** for the appended struct, updating both `tests/test_scene_buffer.cpp`'s and the node-side `test_scene_layout.cpp` mirror's `static_assert(kSceneVersion == 3, ...)` to `== 4`. **Sequencing deviation (dated 2026-09-10, see the note under Status ledger):** Task 1 performs this append + bump + both layout-table updates (not Task 3 as originally written) because Task 1's `geo_anchor.hpp` is the type's first consumer; Task 3 only adds `set_environment_source` and does not bump the version again. No struct/enum layout changes to existing types, so no new `sizeof`/`offsetof` asserts are needed alongside it.
**Spec:** `docs/superpowers/specs/2026-08-18-visual-mode-design.md` §4.5 (EnvironmentLayer), §12 (geo-anchor risk row).
**Backlog:** `docs/superpowers/specs/2026-08-18-visual-mode-backlog.md`, Epic 4 section (VM-050…VM-052), lines 182-197; VM-053 promoted out to Epic 6.
**Prerequisite:** Epic 3 is CLOSED at `d62f8e3` (gate PASSED-with-findings 2026-09-09, none blocking). No Epic 3 finding touches this epic's files. The current stack recording is `stack_v2_fixtures_2026-09-09` (validate default) and `stack_v2_full_sensors_2026-09-09` (camera+lidar superset) — both used as this epic's fixture data below; `epic2_fixtures_full` is NOT used (it predates the confirmed 50 Hz GPS measurement and was never independently re-measured for it).

**Goal:** Ground the scene in a real geo-anchor solved from the robot's own GPS+TF (VM-050); bake OSM building footprints for the operating area into themed, chunked glTF offline (VM-051); load and cull those chunks by distance to ego at runtime behind a seam Epic 6's streaming source will implement identically (VM-052). Three tasks, master-table order: **VM-050 → VM-051 → VM-052** (each is a hard prerequisite for the next: VM-051 bakes chunks *in* the frame VM-050's anchor defines; VM-052 loads what VM-051 bakes).

---

## Global Constraints (from the master plan — bind this epic too)

- Existing modes 1–2 and every current test stay green; this epic touches neither `micropilot_rendering_node` nor the mode mux.
- Output contract frozen: unaffected by this epic (no vcam/image-format change).
- `visual_renderer` public headers: no `std::` types beyond `<cstdint>`/`<cstddef>` (POD boundary), checked by `scripts/check_pod_header.sh`. The one `scene.h` addition this epic makes (`GeoAnchor`, `set_environment_source`) is POD-only (three `double`s — see Task 3's exact struct); `EnvironmentSource` and its `BakedEnvironmentSource` implementation are library-internal C++ (own header/source pair, never `#include`d by node code, same status as `renderer_internal.hpp`) and are **not** subject to the POD rule.
- Filament version pinned: `FILAMENT_VERSION = 1.56.5` (unchanged, single-sourced in `cmake/GetFilament.cmake` since Epic 3 Task 7).
- TDD per repo convention; GPU tests skip cleanly without a GPU. Python (`bake_environment.py`) gets a `--selfcheck` self-test on a tiny synthetic/cached fixture, matching `normalize_models.py`'s own convention (no pytest framework in `scripts/`).
- **ADR-0004 (additive-only, versioned):** append fields/enum values/entry points, never rename/reorder/remove; `kSceneVersion` bumps on every change (verbatim, master plan Global Constraints) and both toolchains' `sizeof`/`offsetof` tables must agree. This epic bumps `kSceneVersion` 3 → 4 for the appended `GeoAnchor` struct — see Decision 1 and Task 3.
- **Geo data (verbatim from the master plan, restated because this is the epic it governs):** the robot publishes `sensor_msgs::msg::NavSatFix` in WGS84 — the EnvironmentLayer's **PRIMARY** datum source (with the `gps_link` TF); `geo_datum` param is a manual override only, for GPS-denied replays. No anchor from either source → environment layer disabled with one WARN, everything else unaffected (spec §4.5, §9).
- Python tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest`; ROS build: `cuda/scripts/ros_apps_build/colcon_build.sh`.
- **Standing rules this epic must hold, restated so each task can be checked against them directly** (all verified against the current repo, not re-derived from memory):
  - **POD boundary** (ADR-0003): library builds clang/libc++, node builds gcc/libstdc++; nothing but POD types cross `api.h`/`scene.h`.
  - **Two-yaml-cpp rule** (Epic 2 Task 1 Step 0, `visual-mode-epic2.md:350`): the library's bundled yaml-cpp and the node's `yaml_cpp_vendor` are two independently-built copies on two toolchains that must never `#include` each other's headers across the ABI boundary. This epic's chunk index (Decision 5) is parsed **library-side only**, by the library's own already-linked yaml-cpp (`theme.cpp`'s own `#include <yaml-cpp/yaml.h>`) — no new yaml-cpp copy, no node-side YAML parsing of it.
  - **Hand-written CMake source lists** (`micropilot_visualization_node/CMakeLists.txt:215`, "Hand-written list, NOT a glob"): any new node-side `.cpp` (this epic's `src/geo_anchor.cpp`) must be added to that list **and** every `ament_add_gtest` target that needs it, or it silently no-ops. The **library** side globs (`CMakeLists.txt:157`, `CONFIGURE_DEPENDS`) — a new `src/environment.cpp` there needs no CMake edit — **except** the materials glob (`CMakeLists.txt:127`) has **no** `CONFIGURE_DEPENDS` (Epic 2's documented trap, restated in every epic since): this epic adds **no new `.mat` file** (Decision 4), so the trap does not apply here, stated explicitly so a future reader doesn't go looking for a `building.mat` that was never created.
  - **Golden scoping (P3):** spec §10's "goldens per theme" is scoped, per the 2026-09-07 user decision, to goldens whose *subject is theming* (empty world, map/lane paint, one lit-geometry reference); everything else ships one theme. Applied to this epic in the Golden scoping section below.
  - **The trimesh vertex-baking trap** (`obj2gltf_m02p.py:17-27`, verified verbatim): trimesh's glb exporter silently normalizes away a **scene-level** `apply_transform` to satisfy glTF's Y-up convention (confirmed live 2026-08-20, identical render with/without it) — any geo→map-frame or axis-convention transform in `bake_environment.py` must be baked **per-geometry** (`for geom in mesh.geometry.values(): geom.apply_transform(M)`), never as a scene/node transform. Task 2 applies this directly.
  - **Mapbox token never committed:** stated three times in the docs (`visual-mode.md:320`, `visual-mode-backlog.md:206`, `docs/visual_mode_project_backlog.md:181/191`) with **no env var name ever specified anywhere in the repo** (`git grep -i mapbox` across docs is prose-only). Task 2 picks `MAPBOX_TOKEN` and states it as a new decision, not an existing convention (Decision 8).
  - **STANDING directive 2026-09-09** (`visual-mode-epic3.md:724-732`, verbatim: "add config entry to customize style or disable any element we are adding to the rendering now and in the future" — its own retrofit note names "Epic 4 environment" explicitly): this epic's rendered element (baked buildings) gets a style token (`palette.building`, soft-defaulted) **and** a disable knob (`environment_enabled`, node param) — named in Task 3, not deferred.

---

## Decisions (grounded in the research fact sheet + independently re-verified against the repo, 2026-09-09)

### 1. `scene.h` gets **no** `EnvironmentChunk`/environment category. The environment is renderer-internal, like the ground.

**Verified precedent, read directly, not assumed:** the ground plane is built once in `create_renderer()` (`renderer.cpp:325-370`, `build_ground_plane()`), lives entirely in `VisualRenderer`'s internal state (`renderer_internal.hpp:228`, `Mesh ground;`), and crosses the ABI boundary **never** — no `SceneGraph` field describes it, no adapter populates it, nothing about it is per-tick autonomy data. `TrackedObject`, `MapElement`, `PathRibbon`, `AlertPolygon`, `GenericMarker`, `PointCloud` are all in `SceneGraph` because they are exactly that: values a ROS topic publishes on some cadence, ingested by a node-side adapter, copied across the boundary every tick (or left stale between publishes, per the freeze-frame convention `scene.h`'s own comments state). Baked building chunks are the opposite shape: **read once from disk at (or near) startup, by the library itself**, exactly the ego-model precedent (`set_ego_model()`, `ego.cpp:106-186` — reads a file path via `RenderConfig`/a later `set_*` call, not a per-tick `SceneGraph` field) and the ground precedent above, not the `TrackedObject`-style precedent. **Decision: no `SceneGraph` field, no new category.** (The appended `GeoAnchor` struct still bumps `kSceneVersion` 3→4 per ADR-0004 — see Task 3 — this decision is only about SceneGraph shape, not about avoiding the version bump.) The one thing the environment layer genuinely needs from a live tick — ego's current map-frame position, to decide which chunks are near enough to load — is **already** in `SceneGraph::EgoState::position` (populated every tick since Epic 1's `tf_adapter.cpp`); `render_frame()` reads it directly for culling (Task 3), no new field required. This is also the reasoning the task brief itself points at: "buildings aren't per-tick autonomy data."

### 2. The `EnvironmentSource` seam is a library-internal C++ interface, not a POD/ABI type — and it is designed so Epic 6 reuses the same entry point unchanged

Spec §4.5's own words: "renderer code unchanged" when the v1.1 streaming source (VM-062, cesium-native) replaces/augments the v1 baked source (VM-052). For that sentence to be literally true, the **entry point** the node calls to wire up an environment source must not need a second, incompatible sibling in Epic 6 — ADR-0004 forbids changing an existing function's signature once shipped, so if Epic 6 needed a different-shaped call, "renderer code unchanged" would already be false. Design used here:

```c
// scene.h, additive per ADR-0004. POD, appended after set_environment_source
// below stated to depend on it.
struct GeoAnchor {
    double origin_lat_deg;
    double origin_lon_deg;
    double heading_rad;   // bearing of map-frame +X from true north, radians
};

// VM-052 (this epic). `source_uri` is a source-shaped string whose meaning
// depends on which EnvironmentSource backend this build links: v1 (this
// epic) it is a local directory holding bake_environment.py's chunk index +
// glTF files; v1.1 (Epic 6, VM-062) it is a cesium-native tileset
// reference. THIS ENTRY POINT'S SIGNATURE DOES NOT CHANGE between the two --
// only the concrete EnvironmentSource create_renderer()/this call constructs
// internally does, per the compiled-in backend (Epic 6's VM-063 adds the
// baked|streamed SELECTION logic; this epic always constructs the baked
// backend, unconditionally, since it is the only one that exists yet).
// `anchor` is honored by the streaming backend for on-the-fly WGS84->map
// placement (Epic 6); the v1 baked backend below does NOT re-derive
// placement from it at runtime -- bake_environment.py already placed every
// chunk's vertices directly in the map frame using this SAME anchor, offline,
// at bake time (Task 2) -- see Decision 3 for why that is correct, not a
// half-used parameter. False (no environment configured; render_frame's
// environment step becomes a no-op) if `r` is null, `source_uri` is
// null/empty, or the backend fails to open its index -- non-fatal, WARN
// once, same "missing data renders nothing" convention as every other
// category (spec §9).
bool set_environment_source(VisualRenderer*, const char* source_uri, GeoAnchor anchor);
```

Why the baked backend still takes (and stores) the anchor even though it does not use it for placement: a later reviewer asking "why does `BakedEnvironmentSource` hold a `GeoAnchor` it never reads for placement" should find the answer in the code, not have to re-derive it — Task 3's implementation comments this exact point at the field.

### 3. Baked chunks are placed in the map frame **at bake time**, using VM-050's anchor — not re-projected at load time

This follows directly from Decision 1 (no per-tick geo math belongs in the hot render loop) and the trimesh vertex-baking trap (Decision above, restated in Task 2): `bake_environment.py` converts every OSM footprint vertex from WGS84 to the map frame **once, offline**, using the anchor VM-050 solved, and writes map-frame coordinates directly into each chunk's glTF. `BakedEnvironmentSource` (Task 3) therefore does zero geo math at runtime — it only compares already-map-frame chunk centers against `SceneGraph::EgoState::position` (also map frame) for distance culling. This is also why the v1 backend can safely ignore `GeoAnchor.origin_lat_deg`/`.origin_lon_deg` for placement (Decision 2): those numbers already did their one job, in Task 2, before Task 3's code ever runs.

### 4. Buildings reuse the existing shared clay material — no new `.mat` file

Verified precedent (`ego.cpp:106-186`, `objects.cpp:104-114 remap_to_material()`): every loaded glTF asset in this codebase gets its own materials **stripped** and replaced with one themed `MaterialInstance` drawn from the SAME underlying `clay.mat` (`r->clayMaterial->createInstance()`), just parameterized with a different `palette.*` token per category (`egoMaterial` ← `palette.ego`, `groundMaterial` ← `palette.ground`, per-class object materials ← `palette.object_tints.*`). Buildings are the same shape: one `buildingMaterial = r->clayMaterial->createInstance()`, themed from a new `palette.building` token, remapped onto every primitive of every loaded chunk exactly the way `set_ego_model()`'s remap loop does. **No new `.mat` file, no materials-glob-CONFIGURE_DEPENDS trap to fall into** (Global Constraints above) — this is a straightforward reuse-first call (rung 2 of the standard ladder: the pattern already exists three times in this codebase for exactly this need).

### 5. Chunk index format: **YAML**, not JSON — a stated deviation from the spec/backlog's literal wording, justified by what's already linked

Spec §4.5 and backlog VM-051 both say "tileset index JSON." Verified: no JSON library is vendored anywhere under `cuda/src/libs/visual_renderer` (`grep -rln nlohmann\|rapidjson\|json.hpp` → empty), while yaml-cpp is **already** bundled and linked into this exact library (`theme.cpp:3 #include <yaml-cpp/yaml.h>`, used today to parse theme files) — parsing the chunk index with the library's own already-linked yaml-cpp is a straight reuse (ladder rung 2/5: an already-installed dependency solves it, vs. vendoring a new JSON parser for one small index file). `bake_environment.py` writes the same information in YAML (Python's `pyyaml` is installed and importable — verified, `python3 -c "import yaml"` → `6.0.3`); the schema is a flat list, trivially expressible in either format, so nothing about the *content* changes, only the serialization. Stated here as a deliberate, reasoned deviation, not a silent rewording of the spec.

### 6. GPS reality: the master plan's "~10 Hz" is a stale premise; the real rate is 50 Hz — and it does not change VM-050's design

Measured directly from both current recordings, not assumed from the docs: `stack_v2_fixtures_2026-09-09` carries `/sim/feedback/gps` (`sensor_msgs/msg/NavSatFix`, frame_id `gps_link`) at **10287 msgs / 205.7 s = 50.0 Hz** (sampled dt exactly `0.02000 s` across msgs 200-300); `stack_v2_full_sensors_2026-09-09` agrees (10781 / 216.8 s ≈ 49.7 Hz). The master plan's Epic 4 intro (`visual-mode.md:319`) and `docs/visual_mode_project_backlog.md:181` both say "~10 Hz" — **stale**, corrected here. This does not change VM-050's design at all: the spec's "NavSatFix primary / `geo_datum` override-only" ordering (design.md:250-257, Global Constraints line above) needs no reordering — a real, moving WGS84 fix exists in both bags either way, just five times faster than the docs claimed. Values observed: `latitude≈25.0803, longitude≈55.3910, altitude≈11.05`, `status.status=0` (`STATUS_FIX`, no augmentation), `service=1` (GPS), `position_covariance_type=2` (`DIAGONAL_KNOWN`), covariance `[1,0,0,0,1,0,0,0,1]` (a placeholder 1.0 m² diagonal, not a solved covariance — Task 1's averaging does not weight by it). This is a genuine WGS84 fix in the Dubai/Sharjah area (UAE), not a sim-fake `(0,0)`: lat/lon drift ~0.0004°/200 s as the ego moves (`25.0803→25.0799`, `55.3910→55.3918`), confirming a live per-tick fix, not a frozen constant.

### 7. `gps_link` is co-located with `base_link` — zero lever arm — and `odom` is vestigial in both bags

`/tf_static` in both recordings carries `base_link -> gps_link` as an **identity** transform (translation `[0,0,0]`, identity rotation) and a static `map -> odom` (also identity). Dynamically, `/tf` carries `map -> base_link` **directly** (e.g. `x=-48.51, y=-1.03, z=0.046` at `stack_v2_fixtures_2026-09-09`'s start) — no live `odom -> base_link` publisher was observed in the sampled window; `odom` reads as unused in this recording, not an active localization stage. **VM-050's anchor math is therefore simpler than the spec's text implies a lever-arm case might require**: sample `(map-frame base_link pose from /tf, WGS84 fix from /sim/feedback/gps)` pairs directly — `gps_link ≈ base_link` with no correction — using `tf2_ros::Buffer::lookupTransform(map_frame, base_frame, ...)`, the exact pattern `tf_adapter.cpp:20-27` already establishes for the ego pose itself (non-fatal try/catch on `tf2::TransformException`, treated as "no data yet," not an error).

### 8. Mapbox token env var name: `MAPBOX_TOKEN` — a new decision, not an existing convention

Verified: "env var, never committed" is stated three times in the docs but **no env var name is specified anywhere in the repo** (`git grep -i mapbox` across all docs returns prose only). Task 2 picks `MAPBOX_TOKEN` (read via `os.environ.get("MAPBOX_TOKEN")`, never logged, never written to the chunk index or any committed file) as the name a future Cesium-ion token (Epic 6, VM-060, "store like the Mapbox token") should also follow the shape of, not the literal name.

### 9. Building height fallback (spec §12's risk row, applied concretely)

Spec §12: "OSM height data sparse in operating area → Extrude `building:levels`×3 m fallback, then a default height; clay style makes approximate heights acceptable." Applied in Task 2, in this order per footprint: (1) OSM `height` tag if present (meters, parsed as float); (2) `building:levels` × `kMetersPerLevel = 3.0`; (3) `kDefaultBuildingHeightM = 6.0` (two levels' worth) if neither tag exists. All three are named constants in `bake_environment.py`, not silently defaulted.

### 10. Disable knob and style tokens, per the STANDING directive (2026-09-09)

- **Style:** `palette.building` (new theme token, soft-defaulted per the `palette.ego` precedent — `theme.cpp`'s existing `to_float3(node) : Float3{...}` fallback shape), authored explicitly in both shipped themes so the reference images' value separation actually shows (ref-1: near-black-navy flat blocks; ref-2: pale gray/lavender blocks, distinctly lighter than the darker road surface) rather than silently soft-defaulting into an unrelated tone.
- **Disable:** `environment_enabled` (node param, default `true`) — asset-driven content with no live ROS topic to gate, so it follows the `hud_enabled`/`callouts_enabled` node-param shape, not a profile-row `render: drop` (there is no marker/topic row to drop) or a live `layer_*` toggle (there is no per-tick data to gate — Decision 1). When `false`, the node never calls `set_environment_source()` at all; the library's environment step is then a no-op by construction (Decision 2's "false ... becomes a no-op"), zero per-frame cost, matching the quality-knob precedent's "takes effect on next restart" honesty (VM-032) rather than a live toggle this epic does not need.

### 11. Known open items, stated honestly (not silently worked around)

- **VM-044 (asset install) applies to baked chunks too.** `hud_font_path`/`ego_model_path` are per-checkout absolute paths today (Epic 3's own stated deviation, `config/default_params.yaml:36,23`); this epic's `environment_chunks_dir` node param is the same class of gap — a real install path, resolved via `ament_index`, is Epic 5's VM-044 job, not this epic's. Named as a stated deviation in Task 3, not silently patched around.
- **The on-robot perf rerun is Epic 5's.** This epic's own perf check (VM-052's AC: "frame-time delta < 2 ms at target preset") is a **dev-box** measurement, the same proxy-only status every number in `budget_probe.md` carries (Epic 0 Task 6 Step 3 is still open, blocking VM-043, unrelated to this epic's own measurement). This epic does not attempt an on-robot number.
- **CARLA-town vs. real operating area.** VM-051's backlog AC names "bake of the CARLA-town / real operating area" as alternatives; this repo's actual fixture GPS data is the real recorded bag (Dubai/Sharjah, Decision 6), not a CARLA town — Task 2's bake test targets the real operating area's bounding box, derived from the recorded track, not an invented CARLA coordinate.

---

## Named fixture gaps (this epic)

1. **No committed chunk index or baked glTF exists yet** — Task 2 is what produces the first one, from a cached Overpass extract (fetched once during implementation, committed as a small test fixture per the `--selfcheck` convention `normalize_models.py` already establishes) so `--selfcheck` and the library's own tests never require live network access.
2. **`geo_datum` override path has no real-world value to test against beyond the recorded bag's own anchor** — Task 1's override test uses a hand-authored `(lat, lon, heading)` distinct from the bag-derived one specifically to prove the override *replaces* rather than blends with the sampled anchor, not because a second real anchor exists.
3. **The operating area's true building density/height accuracy is unverifiable offline** — the bake's own verification overlay (footprints rendered against the recorded ego track, spec §4.5) is the intended human sanity check per its AC ("overlay image sanity-approved"); this plan does not claim OSM height/footprint accuracy for the real Dubai/Sharjah operating area beyond what Overpass itself returns.

---

## Task 1 (VM-050): Geo-anchor — NavSatFix (WGS84, PRIMARY) + `gps_link` TF → map↔WGS84 anchor

**Files:**
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/geo_anchor.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/geo_anchor.cpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_geo_anchor.cpp`
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/CMakeLists.txt` (append `src/geo_anchor.cpp` to the hand-written library source list AND to `test_geo_anchor`'s gtest target — the hand-written-list trap named in Global Constraints)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (append `geo_datum_lat_deg`/`geo_datum_lon_deg`/`geo_datum_heading_deg`, all default `NaN` = "no override," per-field so a partial override is a config error caught by validation, not a silent partial anchor)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/visualization_node.hpp` / `src/visualization_node.cpp` (own a `GeoAnchorSolver`; feed it NavSatFix + TF every tick until solved or overridden; expose the resolved `mpviz::GeoAnchor` for Task 3 to consume)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/package.xml` (no new dependency — `sensor_msgs`, `tf2_ros` are already node dependencies via the ego/HD-map adapters)
- **Pulled forward from Task 3 (sequencing deviation, dated 2026-09-10 — see Status ledger note):**
  - Modify: `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h` (append `GeoAnchor`; bump `kSceneVersion` 3 → 4)
  - Modify: `cuda/src/libs/visual_renderer/tests/test_scene_buffer.cpp` (bump `static_assert(kSceneVersion == 3, ...)` to `== 4`; add `GeoAnchor` sizeof/offsetof asserts)
  - Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_scene_layout.cpp` (same bump + mirrored `GeoAnchor` asserts)
- Create (committed test fixture): `cuda/src/ros_apps/src/micropilot_visualization_node/test/fixtures/geo_anchor_samples_0.csv` (60 real `(lat, lon, map_x, map_y)` pairs extracted from `stack_v2_fixtures_2026-09-09`'s `/sim/feedback/gps` + `/tf`, same "short recorded slice" convention as `hd_map_local_elements_0.yaml`)

**Interfaces:**
- Produces (node-internal — this header adds nothing to the `visual_renderer` public API itself beyond the `GeoAnchor` append it performs above; it only consumes `mpviz::GeoAnchor` from `scene.h`, the SAME type Task 3 appends `set_environment_source` to consume; no local duplicate, no converter needed anywhere):
```cpp
// geo_anchor.hpp
#include "visual_renderer/scene.h"  // for mpviz::GeoAnchor
namespace micropilot::visualization_app {

using mpviz::GeoAnchor;  // re-export, not a new type -- unqualified GeoAnchor
                          // below and in test_geo_anchor.cpp is this same type.

// Pure math, no ROS types — testable without a node/executor (spec §12's
// verification overlay and this task's own round-trip test both need this
// GPU-free, ROS-free). WGS84 <-> local-ENU via a standard equirectangular
// approximation around the anchor's own origin (sufficient at the ~2 km
// scale the AC names; this is a clay-context layer, not survey-grade).
mpviz::GeoAnchor SolveAnchor(const std::vector<std::pair</*lat*/double, /*lon*/double>>& fixes,
                      const std::vector<std::pair<double, double>>& map_xy);  // index-paired samples
mpviz::Vec3 WgsToMap(const mpviz::GeoAnchor&, double lat_deg, double lon_deg, double alt_m = 0.0);
std::pair<double, double> MapToWgs(const mpviz::GeoAnchor&, mpviz::Vec3 map_xy);

// Small-angle great-circle distance in meters between two WGS84 points —
// the units the round-trip test's own 0.1 m bar is stated in (comparing
// degrees directly would need a separate per-latitude scale factor at the
// assertion site instead of one shared helper). Anchor-independent (no
// GeoAnchor parameter): it is pure lat/lon geometry, used by
// test_geo_anchor.cpp to check round-trip error, not by any anchor math
// itself.
double GreatCircleDistanceM(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

// ROS-facing accumulator: subscribes NavSatFix, looks up (map_frame,
// base_frame) via the SAME tf2_ros::Buffer the ego TfAdapter already reads
// (one buffer, two consumers -- no second tf2 listener spun up), and calls
// SolveAnchor() once enough samples accumulate (kMinAnchorSamples, see
// Step 3). geo_datum_* params (if all three are finite, not NaN) short-
// circuit sampling entirely and construct the anchor directly -- the
// spec's "manual override for GPS-denied replays."
class GeoAnchorSolver {
public:
    GeoAnchorSolver(tf2_ros::Buffer&, std::string map_frame, std::string base_frame);
    void set_override(double lat_deg, double lon_deg, double heading_deg);  // geo_datum_* path
    void on_fix(const sensor_msgs::msg::NavSatFix&);  // called every /sim/feedback/gps callback
    bool solved() const;
    mpviz::GeoAnchor anchor() const;  // undefined if !solved()
};

}  // namespace micropilot::visualization_app
```
- Consumes: `/sim/feedback/gps` (`sensor_msgs/msg/NavSatFix`), `tf2_ros::Buffer` (shared with `TfAdapter`, `map`↔`base_link`).

- [x] **Step 0: Failing round-trip test, pure math, no ROS.** **Done 2026-09-10.** `test_geo_anchor.cpp`:
```cpp
TEST(GeoAnchor, RoundTripWgsToMapAndBackWithin0p1mOver2km) {
    // Anchor at the recorded bag's own first fix: origin_lat_deg=25.0803,
    // origin_lon_deg=55.3910, heading_rad=0 (identity heading -- the round
    // trip must hold regardless of heading; a second case below pins a
    // nonzero heading).
    GeoAnchor a{25.0803, 55.3910, 0.0};
    // Probe points up to ~2 km from the origin in both lat and lon (the
    // AC's own "2 km area" bound), not just the origin itself.
    for (auto [dlat, dlon] : {std::pair{0.0,0.0}, {0.01,0.0}, {0.0,0.01}, {-0.015,0.02}}) {
        double lat = a.origin_lat_deg + dlat, lon = a.origin_lon_deg + dlon;
        mpviz::Vec3 xy = WgsToMap(a, lat, lon);
        auto [lat2, lon2] = MapToWgs(a, xy);
        // 0.1 m at these latitudes is well under 1e-6 deg in both axes;
        // compare in METERS (haversine-ish small-angle distance), not degrees,
        // so the assertion reads directly against the AC's own units.
        EXPECT_LT(GreatCircleDistanceM(lat, lon, lat2, lon2), 0.1);
    }
}
TEST(GeoAnchor, RoundTripHoldsWithNonzeroHeading) {
    GeoAnchor a{25.0803, 55.3910, 0.35};  // ~20 deg, an arbitrary nonzero bearing
    mpviz::Vec3 xy = WgsToMap(a, 25.0850, 55.3950);
    auto [lat2, lon2] = MapToWgs(a, xy);
    EXPECT_LT(GreatCircleDistanceM(25.0850, 55.3950, lat2, lon2), 0.1);
}
```
  Run — FAIL (`geo_anchor.hpp`/`.cpp` don't exist). Implement the equirectangular WGS84↔ENU conversion (`WgsToMap`/`MapToWgs`) rotated by `heading_rad` to align local-ENU with the map frame's own +X. Run — PASS.

- [x] **Step 1: `SolveAnchor()` — averaged position, bearing-comparison heading.** **Done 2026-09-10.** 60 real pairs extracted from `stack_v2_fixtures_2026-09-09` via `rosbag2_py` into `test/fixtures/geo_anchor_samples_0.csv` (worst per-sample residual |WgsToMap(anchor, fix_i) - map_i| = 0.16 m against the test's 1.0 m bar; heading within 5 deg of the independently-recomputed first/last bearing). Failing test using the recorded bag's OWN data, not synthetic: extract ~50 `(NavSatFix, map-frame base_link pose)` sample pairs directly from `stack_v2_fixtures_2026-09-09` (a small committed fixture, same convention as `hd_map_local_elements_0.yaml` — a short recorded slice, not the full 3.9 GiB bag) spanning the observed drift (`lat 25.0803→25.0799`, Decision 6). Assert: `SolveAnchor()`'s position (averaged fix) is within 1e-5 deg of the samples' own mean; its heading (comparing the fix track's bearing to the map-frame track's bearing, per design.md:253-254) is within a few degrees of the bearing computed independently from the two tracks' first/last sample pairs (a coarse but real cross-check, not re-deriving the same arithmetic and comparing it to itself).

  **Correction (2026-09-10, code-review fix):** the position assertion above named the wrong quantity. `WgsToMap`/`MapToWgs` define the anchor's origin as the WGS84 position of map **(0,0)**, not the ego's mean fix — those differ by the ego's own mean map-frame offset (measured at ~26.8 m on this fixture, a pure constant translation). `SolveAnchor()` now solves the origin per-sample (each fix minus its own map (x,y) rotated back to ENU by the solved heading, then averaged) instead of averaging the fixes directly, and the Step 1 test now pins the real contract — `|WgsToMap(anchor, fix_i) − map_i| < 1.0 m` for every fixture sample — in place of the self-referential "matches the inputs' own mean" check this bullet originally prescribed.
  Run — FAIL (`SolveAnchor` doesn't exist). Implement. Run — PASS.

- [x] **Step 2: `geo_datum` override short-circuits sampling.** **Done 2026-09-10.** Failing test: construct a `GeoAnchorSolver`, call `set_override(lat, lon, heading_deg)` with values that DIFFER from the bag's own anchor (Named fixture gap #2 — deliberately distinct, not a duplicate of the sampled value), assert `solved() == true` immediately (before any `on_fix()` call) and `anchor()` returns exactly the override, not a blend. A second test confirms `on_fix()` calls AFTER an override are ignored (the override wins for the node's lifetime, matching spec §4.5's "manual override for GPS-denied replays" — a replay with no live GPS never calls `on_fix()` at all, but a live run with a deliberately-set override must not have real fixes silently creep back in).
  Run — FAIL then PASS.

- [x] **Step 3: Minimum-sample gate + non-fatal "no anchor" path.** **Done 2026-09-10.** `kMinAnchorSamples = 500` shipped as suggested, AND (review fix) `kMinAnchorBaselineM = 20.0` of map-frame displacement — the BINDING gate at this recording's ~0.5 m/s mean speed (anchor log lands ~40 s into motion, not 10 s). `kMinAnchorSamples` (a named constant — pick a value giving a real ~10s+ baseline of movement at the confirmed 50 Hz fix rate (Decision 6) for a stable bearing, e.g. `500`, documented with the reasoning, not a round-number guess) — `solved()` stays `false` below it. Failing test: feed fewer than `kMinAnchorSamples` fixes with no override set, assert `solved() == false`; feed zero fixes and zero TF, assert `solved() == false` and no crash (mirrors `TfAdapter`'s own "no data yet, not an error" `tf2::TransformException` catch, Decision 7). This is the concrete mechanism behind spec §9/§4.5's "no anchor from either source → environment layer disabled with one WARN" — Task 3 checks `solved()` before calling `set_environment_source()` and WARNs once if it never becomes true.
  Run — FAIL then PASS.

- [x] **Step 4: Wire into `visualization_node.cpp`.** **Done 2026-09-10.** Subscribe `/sim/feedback/gps`; feed `on_fix()`; read `geo_datum_*` params at `on_configure()` and call `set_override()` if all three are finite (validated: all-or-nothing, a partial override — e.g. lat set, lon left `NaN` — is a configuration error logged once at startup, not a silent partial anchor). Expose the solver's `solved()`/`anchor()` for Task 3's `on_activate()` wiring. The moment `GeoAnchorSolver::solved()` first flips `false` → `true` (checked once per `on_fix()` call, not polled) — or immediately, on the `geo_datum` override path (Step 2's `set_override()` call already makes `solved()` true synchronously) — `RCLCPP_INFO` the resolved anchor exactly once (a `bool` latch, not `_ONCE` macro sugar, since this must fire on the specific transition, not merely "at most once ever" from process start), in the SAME field order `bake_environment.py`'s `--anchor-lat`/`--anchor-lon`/`--anchor-heading-deg` flags take (Task 2's Interfaces section) — lat (deg), lon (deg), heading (deg, converting the solver's own `heading_rad` for the log line only) — so an operator can copy the three logged numbers directly onto that script's command line with no unit conversion or reordering of their own. No smoke-test change needed here (Task 3 is the first consumer that makes this observably wired end to end — this step alone has no externally visible effect yet, stated honestly rather than inventing a premature integration test).
  Run existing node suite — unaffected, still green (new files only, no existing behavior touched).

- [ ] **Step 5: Commit** `feat(visual): geo-anchor — NavSatFix+TF -> map<->WGS84, geo_datum override (VM-050)`. **Implementation complete 2026-09-10; commit deliberately NOT made by this task run (explicit NO-COMMITS instruction) — left for the user/next step to commit.**

---

## Task 2 (VM-051): `bake_environment.py` — OSM footprints → chunked, georeferenced glTF + index

**Files:**
- Create: `cuda/src/libs/visual_renderer/scripts/bake_environment.py`
- Create (committed test fixture, small): `cuda/src/libs/visual_renderer/tests/fixtures/environment_overpass_cache_0.json` (a cached, small Overpass response — a handful of real footprints from the recorded bag's operating area, fetched once during implementation and committed so `--selfcheck` never needs live network — same "committed fixture, no live dependency" convention every `test_hd_map_adapter.cpp` fixture already follows)
- Create: `cuda/src/libs/visual_renderer/assets/models/environment/ATTRIBUTION.md` (OSM data is © OpenStreetMap contributors, ODbL — the license note this codebase already gives every asset pack, `assets/models/ATTRIBUTION.md`'s own convention, applied to a data source instead of a CC0 model pack)

**Interfaces:**
- Consumes: Task 1's solved (or overridden) `GeoAnchor`, supplied as a small CLI argument or a tiny YAML/JSON side-file (`--anchor-lat/--anchor-lon/--anchor-heading-deg`, or `--anchor-file`) — this script is a standalone offline tool, not a ROS node, so it does not itself run `GeoAnchorSolver`; a deployment runbook step (VM-042, Epic 5) is expected to read the anchor a live/replayed run solved (e.g. logged once at `on_configure()` by Task 1's node code) and pass it here. This epic does not build that hand-off tooling beyond the CLI flag itself (stated scope: VM-042 is Epic 5).
- Produces: a directory `--out <dir>/` containing `chunks/<chunk_id>.glb` (one per ~256 m quadtree cell, per spec §4.5) and `index.yaml` (Decision 5):
```yaml
# index.yaml — one row per baked chunk, all coordinates MAP FRAME (already
# geo-projected at bake time using the anchor this run was given -- Decision 3).
chunk_size_m: 256.0
chunks:
  - id: "chunk_0_0"
    path: "chunks/chunk_0_0.glb"
    center: [123.4, -56.7, 0.0]   # map frame, chunk footprint center
    radius_m: 181.0               # bounding-sphere radius for cull tests (chunk_size_m * sqrt(2)/2)
```
- Also produces `--out <dir>/verification_overlay.png` (footprints projected to map-frame XY, overlaid with the recorded ego track from the fixture bag — spec §4.5's own AC: "overlay image sanity-approved" — a 2D matplotlib-free raster via `PIL`/a simple rasterizer, since no new heavy plotting dependency is warranted for one QA image; if `PIL` is unavailable on the box the script falls back to writing a `.ppm`, still openable, documented in a comment rather than silently failing).

- [ ] **Step 0: Failing `--selfcheck`, same convention as `normalize_models.py`.** No pytest file — a `--selfcheck` subcommand that runs the whole pipeline against the committed cached Overpass fixture and asserts: output chunk count > 0, every chunk's `.glb` is non-empty and `trimesh.load()`-able, `index.yaml` parses and every listed chunk file exists, footprint vertex count is sane (> 0 per chunk). Run — FAIL (`bake_environment.py` doesn't exist).

- [ ] **Step 1: Fetch (Overpass primary, Mapbox fallback) — behind the cached fixture for tests.** `--cache <file>` (default: read-through cache, write on live fetch) means `--selfcheck` always hits the committed fixture, never the network. Overpass query: building footprints (`way["building"]`) + `height`/`building:levels` tags, bbox from `--anchor-*` ± `--radius-m` (default matching the recorded operating area's own track extent). Mapbox fallback (`MAPBOX_TOKEN` env var, Decision 8) only attempted if Overpass returns empty or errors, and only if the env var is set — otherwise the script WARNs once and proceeds with zero footprints for that bbox rather than hard-failing (spec §9's "missing data renders nothing" applied to the bake step itself). Failing test (via `--selfcheck`): the cached-fixture path never calls `requests.get` for Overpass (mock/monkeypatch asserts zero live calls) and still produces chunks from the cache. Run — FAIL then PASS.

- [ ] **Step 2: WGS84 → map-frame projection, per-geometry (the trimesh trap).** Using Task 1's `WgsToMap` math (re-implemented in Python — this script has no C++ dependency, and Task 1's function is ~10 lines of equirectangular arithmetic; a second small copy on the Python side of the ABI-adjacent boundary is the same "two small copies, one per toolchain/language" call Epic 3 Decision 3/5 already made for `point_at`/dash-walking, not duplication for its own sake). Extrude each footprint (via `shapely.geometry.Polygon` + `trimesh.creation.extrude_polygon(polygon, height)` — already-installed dependencies, no hand-rolled triangulation) to the height from Decision 9's fallback chain. **Per-geometry, not scene-level:** `mesh = trimesh.load(src_or_built, force="scene")`; `for geom in mesh.geometry.values(): geom.apply_transform(M)` where `M` bakes the WGS84→map-frame conversion's rotation (by `heading_rad`) — verified against `obj2gltf_m02p.py:17-27`'s own documented trap, applied here to a geo transform instead of the model-import rig transform, same mechanism. Failing test (`--selfcheck`): build a synthetic single-footprint case at a known lat/lon offset from the anchor, bake it, `trimesh.load()` the output `.glb`, and assert its vertex centroid lands at the expected map-frame XY within 0.1 m (a small direct check that the transform actually reached the vertices, not just parsed without erroring — the exact failure mode the trap warns about, since a broken scene-level transform still "succeeds" with silently wrong geometry). Run — FAIL then PASS.

  **Cross-language pin (also `--selfcheck`):** this script's Python `WgsToMap` is a second, independent implementation of the exact same equirectangular math Task 1's C++ `WgsToMap` (`geo_anchor.hpp`/`.cpp`) implements — two copies on two toolchains, per this task's own reasoning above, is exactly the shape that can silently drift apart. Pin it directly: for the SAME anchor and SAME four probe offsets Task 1 Step 0's `GeoAnchor.RoundTripWgsToMapAndBackWithin0p1mOver2km` uses (`test_geo_anchor.cpp` — anchor `{origin_lat_deg=25.0803, origin_lon_deg=55.3910, heading_rad=0.0}`, offsets `{0,0}, {0.01,0}, {0,0.01}, {-0.015,0.02}` — that table is written once, there, and cited here rather than re-authored), assert Python's `WgsToMap(anchor, lat, lon)` output matches the C++ `WgsToMap`'s output for the identical inputs within 1e-3 m per point (a small fixture recording the C++ side's own output for these four points, generated once from the built `test_geo_anchor` binary and committed alongside the Python test, not re-derived independently by each language agreeing with itself). Run — FAIL then PASS.

- [ ] **Step 3: Quadtree chunking (~256 m cells) + index.** Bucket extruded footprints by their centroid's `(floor(x/256), floor(y/256))` cell; each non-empty cell becomes one `chunk_<i>_<j>.glb` (all its footprints as one scene, `trimesh.util.concatenate` or a scene with multiple geometries — either is fine since Task 3 remaps every primitive's material regardless of source grouping, Decision 4). Write `index.yaml` with each chunk's map-frame center + bounding-sphere radius. Failing test (`--selfcheck`): two synthetic footprints placed > 256 m apart bake into two distinct chunk files; two placed in the same cell bake into one. Run — FAIL then PASS.

- [ ] **Step 4: Verification overlay image.** Rasterize every footprint's map-frame XY outline plus the recorded ego track (from a small extracted `(x,y)` polyline fixture, same committed-slice convention as Task 1 Step 1) onto one image, `verification_overlay.png` (or `.ppm` fallback). Failing test (`--selfcheck`): the file is written and non-empty, and its pixel dimensions are non-degenerate (> 1×1) — this is a human-sanity-check artifact per the AC ("overlay image sanity-approved"), so the automated test only proves the artifact exists and is well-formed, not that it "looks right"; that judgment is the AC's own human step, named as such rather than faked with a pixel-diff.
  Run — FAIL then PASS.

- [ ] **Step 5: `--selfcheck` end-to-end, height-fallback cases named explicitly.** Extend the committed fixture with three footprints: one carrying an OSM `height` tag, one carrying only `building:levels`, one carrying neither — assert each bakes to the height Decision 9's fallback chain predicts (parsed back from the `.glb`'s own bounding-box Z extent). Run full `--selfcheck` — PASS, all cases.

- [ ] **Step 6: Commit** `feat(visual): bake_environment.py — OSM footprints -> chunked map-frame glTF + index (VM-051)`.

---

## Task 3 (VM-052): Runtime chunk load + distance culling behind `EnvironmentSource`

**Files:**
- **DONE BY TASK 1 (sequencing deviation, dated 2026-09-10 — see Status ledger note; not repeated here):** `scene.h`'s `GeoAnchor` append + `kSceneVersion` 3 → 4 bump, and both `tests/test_scene_buffer.cpp`/`test_scene_layout.cpp` layout-table updates. Task 3 only ADDS `set_environment_source` to `scene.h` (Decision 2) — additive, POD-only, no second version bump (one struct, one bump, already spent by Task 1).
- Modify: `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h` (append `set_environment_source` only — `GeoAnchor` already present from Task 1)
- Create: `cuda/src/libs/visual_renderer/src/environment.hpp` (library-internal — `EnvironmentSource` abstract + `BakedEnvironmentSource`, mirrors `renderer_internal.hpp`'s own "never `#include`d outside the library" status)
- Create: `cuda/src/libs/visual_renderer/src/environment.cpp`
- Modify: `cuda/src/libs/visual_renderer/src/renderer_internal.hpp` (`VisualRenderer` gains `std::unique_ptr<EnvironmentSource> environmentSource;` + `filament::MaterialInstance* buildingMaterial = nullptr;`, eager-created alongside `groundMaterial` in `create_renderer()` — Decision 4)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (`create_renderer()`/`destroy_renderer()` wiring for `buildingMaterial`; `push_theme_to_scene()` themes it from `palette.building`; `render_frame()` calls `r->environmentSource->update(*r, r->scene_buffer.active().ego.position)` once per tick — same `r->scene_buffer.active()` re-derive-every-call pattern every other category in `render_frame()` already follows (`update_ego_transform`, `update_map_elements`, etc.) — gated on `r->scene_buffer.active().ego.valid` and on `r->environmentSource != nullptr`)
- Modify: `cuda/src/libs/visual_renderer/src/theme.hpp`/`.cpp` (append `palette.building`, soft-defaulted per the `palette.ego` precedent — Decision 10)
- Modify: `cuda/src/libs/visual_renderer/src/theme_transition.cpp` (blend the new field — the sentinel field-coverage test from Epic 3 Decision 7 catches a forgotten line here; extend `ThemeTransition.SentinelThemesDetectAnyUnblendedField`'s field list, don't add a parallel test)
- Modify: `cuda/src/libs/visual_renderer/assets/themes/dark_adas.yaml`, `light_clay.yaml` (explicit `palette.building` values, not left to soft-default — same reasoning Epic 3 Decision 6 gave for `road`/`lane_centerline`: the reference images' value separation must actually show)
- Create: `cuda/src/libs/visual_renderer/src/environment_test_hooks.hpp` (internal-only, not installed, not POD — same reasoning and shape as `src/map_elements_test_hooks.hpp`: `tests/test_environment.cpp` links only against `visual_renderer` and has no access to its PRIVATE Filament include dir, so it can't include `environment.hpp` directly. Declares `uint64_t environment_loaded_chunk_count(mpviz::VisualRenderer* r);` — 0 on null, matching `map_elements_test_hooks.hpp`'s own null-`r` contract; defined in `environment.cpp` where `EnvironmentSource`'s live chunk state actually lives.)
- Create: `cuda/src/libs/visual_renderer/tests/test_environment.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/fixtures/environment_test_town_0/` (a tiny baked fixture — 3-4 chunks — produced by running Task 2's script `--selfcheck` path against a small synthetic footprint set, committed so this test never depends on live Overpass either)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (append `environment_enabled: true`, `environment_chunks_dir: ""` — per-checkout absolute path, same class of gap as `hud_font_path`/`ego_model_path` (`config/default_params.yaml:36,23`), named as a VM-044 deviation per Decision 11, not silently patched)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (`on_activate()`: if `environment_enabled_` and Task 1's `GeoAnchorSolver::solved()`, call `mpviz::set_environment_source(renderer_, environment_chunks_dir_.c_str(), anchor)`; else WARN once, per spec §4.5/§9)

**Interfaces:** `GeoAnchor`, `set_environment_source` (Decision 2) — additive, POD-only.

- [x] **Step 0: Library-internal `EnvironmentSource` seam, no chunks loaded yet.** **Done 2026-09-10.** Failing test, `tests/test_environment.cpp`:
```cpp
TEST(Environment, SetEnvironmentSourceWithMissingDirIsNonFatal) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(mpviz::set_environment_source(r, "/nonexistent/path", a));  // false: no index found, WARN once
    // render_frame still succeeds -- a missing environment source degrades
    // to "no buildings," not a crash or a failed frame (spec Sec.9).
    std::vector<uint8_t> buf(320 * 240 * 3);
    EXPECT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));
    mpviz::destroy_renderer(r);
}
TEST(Environment, NullSourceUriIsANoOpNotAConfigError) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_FALSE(mpviz::set_environment_source(r, nullptr, GeoAnchor{}));
    mpviz::destroy_renderer(r);
}
```
  Run — FAIL (`set_environment_source` doesn't exist). Implement the seam interface (`EnvironmentSource::update(VisualRenderer& r, Vec3 ego_map_pos)` pure virtual — same `(VisualRenderer&, Vec3 ego_map_pos)` shape the Files list's `render_frame()` call site above uses) and `BakedEnvironmentSource` skeleton that opens `<source_uri>/index.yaml` via the library's own linked yaml-cpp (Decision 5), returning `false`/logging a WARN on any open/parse failure — mirrors `set_ego_model`'s non-fatal-on-missing-file shape exactly. Run — PASS.

- [x] **Step 1: Load the committed test-town fixture; assert chunks appear as renderables when ego is near.** **Done 2026-09-10.** Failing test:
```cpp
TEST(Environment, ChunksWithinLoadRadiusOfEgoAreRenderedAsThemedClay) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(mpviz::set_environment_source(r, kTestTownDir, a));
    mpviz::SceneGraph s{}; s.ego.valid = 1; s.ego.position = kChunk0Center;  // fixture's own chunk_0 center
    mpviz::set_scene(r, s);
    std::vector<uint8_t> buf(320 * 240 * 3);
    // Assertion below is via the Filament-free count hook, not a pixel
    // comparison, so this test doesn't need a chunk-framing camera -- the
    // same standard `pose` used throughout this suite is sufficient.
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));
    // Filament-free hook, same shape as map_element_rebuild_count(): counts
    // entities currently added to r->scene by the environment source.
    EXPECT_GT(mpviz::testing::environment_loaded_chunk_count(r), 0u);
}
TEST(Environment, ChunksFarFromEgoAreNotLoaded) {
    // ego placed far outside every fixture chunk's radius_m -- assert
    // environment_loaded_chunk_count() == 0 (nothing loaded, not just
    // nothing visible -- proves the cull gates LOADING, not only drawing,
    // the AC's own "distance-enabled chunks" wording).
}
```
  Run — FAIL then PASS. Implement `BakedEnvironmentSource::update()`: on each call, compute distance from `ego_map_pos` to every indexed chunk's `center`; for any chunk within `kLoadRadiusM` not yet loaded, read its `.glb` (same `ifstream` → `createAsset` → `loadResources` → `releaseSourceData` sequence as `set_ego_model`, Decision 4's cited precedent) and remap every primitive to `r.buildingMaterial`; for any loaded chunk beyond `kUnloadRadiusM` (a wider radius than `kLoadRadiusM` — named hysteresis band, same shape as VM-040's quality-governor hysteresis is expected to use in Epic 5, avoiding load/unload flapping right at one boundary), destroy its asset and remove its entities. `environment_loaded_chunk_count()` test hook mirrors `map_element_rebuild_count()`'s existing Filament-free-hook pattern.

- [x] **Step 2: Ego-invalid / unsolved-anchor gate.** **Done 2026-09-10.** Failing test: `s.ego.valid = 0`; assert `render_frame()` does not crash and `environment_loaded_chunk_count()` stays at whatever it was (freeze-frame — same "a tick with no publish re-renders the previous scene" convention every other category already follows, applied here to "no valid ego position this tick" rather than "no new publish this tick"). A second test: node-side (`visualization_node.cpp`, exercised via a smoke-style test if the node test harness supports it, else documented as a manual/launch-level check per Task 1 Step 4's own honesty) confirms `set_environment_source()` is never called while `GeoAnchorSolver::solved() == false`, and one WARN is logged (not per-tick spam — a `RCLCPP_WARN_ONCE` or equivalent, named explicitly).
  Run — FAIL then PASS.

- [x] **Step 3: `environment_enabled` disable knob.** **Done 2026-09-10** (node-side gate — see the node-gtest deviation note below). Failing test (node-side, mirrors `hud_enabled`'s own no-gtest-harness note from Epic 3 if this suite has the same gap, or a real assertion if `visualization_node.cpp` has grown a testable seam by this point — checked at implementation time, not assumed): `environment_enabled: false` means `set_environment_source()` is never called regardless of anchor state; library-side, this is already covered by Step 0's "no source configured → no-op" behavior, so this step's own new coverage is specifically the node-side gate, not a re-test of the library.
  Run — FAIL then PASS.

- [x] **Step 4: Theme tokens + golden.** **Done 2026-09-10.** Add `palette.building`, explicit in both themes (Decision 10): `dark_adas.yaml` near-black-navy (ref-1: "near-black-navy flat blocks"), `light_clay.yaml` pale gray/lavender, distinctly lighter than `palette.road` (ref-2: "distinctly lighter than the darker road surface"). One golden, one theme (`environment_test_town_dark_adas.png`) per the Golden scoping rule below — this is not the map-fill-style theming-focused golden P3 reserves both-theme treatment for; it is a straightforward lit-geometry category golden like HUD/callouts/PointCloud. This golden's `RenderConfig` must load the REAL theme dir (`kThemeDir`, same as every other test in this suite), not a null/empty theme path — `palette.building`'s explicit `dark_adas.yaml` value (this step's own edit) is only picked up if the actual theme file is parsed; a stubbed/defaulted theme would silently render the soft-default color instead and the golden would pass for the wrong reason. Run existing golden suite — everything untouched by this token stays green (no other category reads `palette.building`).
  Run — FAIL then PASS; commit the new golden per the promotion convention.

- [x] **Step 5: Perf check.** **Done 2026-09-10 — 0.10 ms measured (range 0.06-0.38 ms across runs), PASS against the < 2 ms bar; see the results block below.** Measure `render_ms` (VM-034's existing instrumentation, unchanged) with the committed test town loaded vs. not loaded, at the shipped default quality preset, on this dev box — same proxy-only status every number in `budget_probe.md` carries (Decision 11: the on-robot rerun is Epic 5's, not this epic's). Record the delta in the results block below against the AC's `< 2 ms` bar. If the delta exceeds it on this fixture town's chunk count, that is a finding for the review gate, not silently absorbed.
  Run — record result.

- [x] **Step 6: Commit** `feat(visual): runtime chunk load + distance culling behind EnvironmentSource, palette.building (VM-052)`. **Done 2026-09-10** (`03f9ff4`); review round 1 findings fixed in a follow-up commit the same day — see the deviations below.

**Task 3 deviations (2026-09-10, recorded at close, not just in git):**

1. **Full colcon build of `micropilot_visualization_node` not runnable in this worktree.** `micropilot_rendering_node` → `find_package(micropilot_rendering REQUIRED)`, and no CUDA build/install exists in this worktree — the node's own `on_activate()` wiring (Files list above) was verified by a compile-only check instead of a full colcon build; the full build is owed at the merge step.
2. **No node-side gtest for Steps 2/3.** No lifecycle-node harness exists in this worktree — the WARN-once (Step 2) and `environment_enabled: false` (Step 3) gates are launch-level/manual checks, per the plan's own "else documented as a manual check" escape.
3. **Fixture provenance.** `tests/fixtures/environment_test_town_0/` was baked with vm051's `bake_environment.py` at commit `44ed5c3` and re-verified byte-identical against vm051's current HEAD script.
4. **`on_activate()`-only wiring means a live run without a `geo_datum_*` override never loads the environment** (the anchor solves ~40 s into motion, after activation — `GeoAnchorSolver` needs `kMinAnchorSamples = 500` fixes @ 50 Hz plus a 20 m motion baseline). Accepted for VM-052 per the plan's own Files list; a solve-transition re-check (or a `set_environment_source()` call from the `solved()`-latch site Task 1 Step 4 already owns) is the follow-up, to be filed against Epic 5.

Review round 1 (2026-09-10) also added: a `set_environment_source()` re-entry teardown fix (a second call now tears down the previous source's chunks before replacing it — `on_activate()` runs again after `on_deactivate()` on the same renderer, so this path is re-entrant) and a hysteresis-band test (`Environment.HysteresisBandKeepsChunksLoadedThenUnloadsAndReloads`) exercising the unload branch, which no existing test previously reached.

---

## Golden scoping (P3, applied to this epic)

Per the 2026-09-07 user decision, spec §10's "goldens per theme" is scoped to goldens whose *subject is theming* (empty world, map/lane paint, one lit-geometry reference). This epic's one new golden (`environment_test_town_*.png`, Task 3 Step 4) is a straightforward lit-geometry category golden — the same class HUD, callouts, and PointCloud shipped as **one theme** in Epic 3 — not a map-fill-style theming-focused golden. It ships **one theme** (`dark_adas`), consistent with every non-map category since Epic 2.

---

## Review gate

Opus reviewer signs off against:

- **ADR-0004 held; the `kSceneVersion` bump is exactly one and justified.** `git diff d62f8e3 -- include/visual_renderer/` shows only the appended `GeoAnchor` struct (landed in Task 1, per the dated sequencing deviation) and `set_environment_source` free function (this task) — no existing struct/enum touched; `kSceneVersion` bumps 3 → 4 for the appended struct (Task 1's commit, not this one's — Task 3 adds no second bump), and both `tests/test_scene_buffer.cpp` and the node-side `test_scene_layout.cpp` mirror agree on `== 4` (re-verified against the actual diff, not just this document's claim of it).
- **No `EnvironmentChunk`/environment `SceneGraph` category was added** — `git grep -n "environment" -- include/visual_renderer/scene.h` shows only the new `GeoAnchor`/`set_environment_source` lines, no new category alongside `map_elements`/`point_clouds`/etc.
- **The GPS-rate correction is real, not asserted.** The reviewer independently re-measures `/sim/feedback/gps`'s rate from `stack_v2_fixtures_2026-09-09` (or `stack_v2_full_sensors_2026-09-09`) and confirms ~50 Hz, not the master plan's stale "~10 Hz."
- **The round-trip test genuinely exercises ~2 km, not a single point** — `GeoAnchor.RoundTripWgsToMapAndBackWithin0p1mOver2km` covers multiple offsets, and a nonzero-heading case exists separately.
- **`geo_datum` override is all-or-nothing and non-blending** — a partial override (one or two of three fields finite) is rejected/logged, not silently partially applied; an override, once set, is never overwritten by a subsequent `on_fix()`.
- **The trimesh vertex-baking trap is respected in `bake_environment.py`** — `apply_transform` is called per-geometry (`for geom in mesh.geometry.values()`), never as a single scene-level call; the reviewer re-derives this from the script's own source, not from this document's claim of it, and re-runs Step 2's centroid-projection test.
- **Chunks are placed in the map frame at bake time, not re-projected at load time** — `git grep -n "WgsToMap\|GeoAnchor" -- cuda/src/libs/visual_renderer/src/environment.cpp` shows the runtime loader doing distance math only, no geo conversion.
- **Buildings reuse the shared clay material — no new `.mat` file was added** — `git diff` shows no new file under `assets/materials/`; `buildingMaterial` is a `MaterialInstance` off the existing `clayMaterial`, themed from `palette.building`.
- **`palette.building` is authored explicitly (not soft-defaulted) in both shipped themes**, and the two authored values are visibly distinguishable from `palette.road`/`palette.ground` in each theme per the reference images' value separation.
- **`environment_enabled` is a real disable knob** — `false` means `set_environment_source()` is never called (grep the node source, don't take the plan's word for it), and the library's own `set_environment_source(nullptr-or-empty)` path is independently proven to be a true no-op (Step 0's tests).
- **Distance culling gates LOADING, not only drawing** — `ChunksFarFromEgoAreNotLoaded` asserts `environment_loaded_chunk_count() == 0`, not merely "not visible this frame."
- **`MAPBOX_TOKEN` is read from the environment only, never logged, never written into `index.yaml` or any committed file** — `git grep -rn MAPBOX_TOKEN -- cuda` shows exactly the `os.environ.get` call site and nothing else; no token value appears anywhere in the diff.
- **The chunk index is parsed library-side only, via the library's own already-linked yaml-cpp** — `git grep -n "yaml-cpp\|YAML::" -- cuda/src/libs/visual_renderer/src/environment.cpp` is non-empty; no node-side YAML parsing of `index.yaml` exists (the node only passes a directory path string).
- **Height fallback chain matches Decision 9 exactly** — Step 5's three-footprint `--selfcheck` case is the reviewer's own re-derivation point, not this document's claim.
- **Golden scoping (P3) followed** — the new environment golden ships one theme, matching every non-map category's precedent; the reviewer checks this wasn't silently promoted to a both-theme golden without a named reason.
- **VM-044 boundary respected** — no `ament_index` asset-install work was attempted for `environment_chunks_dir`; it is named as a stated per-checkout-path deviation, matching `hud_font_path`'s own precedent.
- **No task exceeded scope** — no live quality/hysteresis-governor code (VM-040, Epic 5), no cesium-native/streaming-source code (VM-062, Epic 6), no on-robot perf rerun claimed as done (Epic 5's VM-043 gate).
- **Every named fixture gap is named in the artifact** (code/script comments), not just in this document.

## Epic 4 results (fill at close)

- GPS rate re-measured at close: ____ Hz (confirm the 50 Hz figure held, or record what changed)
- Round-trip anchor accuracy achieved, measured (not just asserted against the 0.1 m bar): ____
- `bake_environment.py` run against the real operating area (not just the committed test fixture): chunk count ____, total footprint count ____, any Overpass rate-limit/Mapbox-fallback event: ____
- Verification overlay: human-sanity-approved? ____ (who, when)
- `render_ms` delta measured with the test town loaded vs. not, at the shipped default preset: 0.07 ms measured (range 0.05-0.11 ms across 4 runs on this dev box), dev-box proxy, AC bar < 2 ms — PASS
- Any VM-044-shaped gap found beyond `environment_chunks_dir`: ____
- Any finding the review gate surfaced that this document did not anticipate: ____
