# VM-077 — New-Stack Rendering: `output_trajectory_carpet` + Topic Remap

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status: DONE — all 6 tasks implemented and verified 2026-09-09 (see Results section at the end for suite counts, files changed, and the live-rig verification that found and fixed one rendering bug not caught by any unit test).**

**Parent plan:** `docs/superpowers/plans/2026-08-18-visual-mode-epic3.md` — Epic 3 is CLOSED at `d62f8e3`; this plan is new-stack follow-on work staged there under "New-stack intake, PENDING user 'go' (2026-09-09) — VM-077" (that section's step 4: "Author the new-topic rendering work as its own scoped task(s) AFTER inspecting the recorded data"). Its "STANDING user directive 2026-09-09 — every rendered element ships with style + disable config" binds every task below.
**ADR:** `docs/adr/0004-scene-interface-versioning.md` — additive-only, versioned `scene.h`. This plan's one interface change (Task 1) appends a category and bumps `kSceneVersion` 2 → 3 (Epic 3 Task 1 introduced it at 1, Task 6 bumped it to 2).
**Measurement report:** VM-077 Step 4 measurement pass (offline `rosbag2_py`/rclpy inspection, `stack_v2_fixtures_2026-09-09`, the 31-topic light bag) — reproduced in full in this plan's Decisions section below; every row/skip decision cites it.
**Prerequisite:** Epic 3 closed. `visualization_node`'s `layer_*` gate mechanism (VM-032), `apply_layer_gates()` (VM-035 review fix), and the `PointCloudPoint`/`point_cloud.mat` per-vertex-color machinery (VM-035) are the three existing seams this plan reuses rather than re-inventing.

**Goal:** Render the one new topic the measurement pass found unambiguous rendering value in with genuinely new data (`output_trajectory_carpet`, a per-vertex velocity-colored trajectory ribbon), remap the five dead `/navigation_urban_collision_checker_testing_node/*` collision rows onto their content-verified successors (or a dormant disabled-row treatment where none exists), add two cheap rows via the existing `generic` adapter, and record explicit, evidence-cited SKIP/defer decisions for every duplicate or inconclusive topic the pass found — nothing else. Six tasks:

**Task 1 → Task 2 → Task 3 (the carpet: interface, then library rendering, then node adapter+wiring) → Task 4 (collision remap) → Task 5 (two new `generic` rows) → Task 6 (SKIP/defer decisions, documentation only).** Tasks 4-6 touch disjoint files from 1-3 and from each other (profile YAMLs only) and may run in parallel once Task 3 lands (Task 3 also edits `urban_profile.yaml`, so sequence profile-file edits to avoid merge conflicts — Task 4 and 5 after Task 3, or all three assigned to one agent).

---

## Verbatim user context

> "theres more new data and topics offered I want to include in our plan (like a new velocity profile trajectory named output_trajectory_carpet amd similar new markers), I will start the stack and wait for my go to recor a new bag that will replace the new one we are using in the validate script."

(Given 2026-09-09, before the recording; the recording, health-gate swap, rviz-reference diff, and this measurement pass are the "similar new markers" follow-through this plan is written from.)

---

## Scope discipline (binding on every task below)

Per the measurement pass's own §7 recommendation table: implement **only** what the recorded data justifies.
- **Render:** `output_trajectory_carpet` (genuinely new per-vertex data, no existing path can draw it — §6), the two collision successors (content-verified role mapping — §2), `/navigation/debug_cruise_obstacle_marker` and `/local_map_corners` (existing `generic` adapter already handles their geometry — §2, §5).
- **Retire-to-dormant, not delete:** the three collision rows with no measured successor (`sweep`/`merged_ego`/`merged_object` — §2, §7).
- **SKIP, with evidence, no code:** `output_path_line` (exact duplicate, §3), `output_path_velocity_visualization` (blank placeholders, no text renderer — §3/§6), `dynamic_objects_marker_1..6` (duplicates or silent — §4), `/sim/hd_map/local_markers` (duplicate road network, 3 MB/s — §5), `/navigation/bvp_crosswalk_attention_areas` (never publishes real content — §5), `/navigation/trajectory_following_status` (data-inconclusive — §5).
- **Unchanged:** `/navigation/global_path`, `/local_path`, OGM topics — still absent from the live stack, no new evidence, no action (Epic 2's existing FIXTURE GAP rows stand).
- Every rendered element ships a theme style token (soft-defaulted, reused where reuse is honestly justified — matching the VM-031 callout precedent of reusing `hud.accent_color` verbatim rather than inventing an unneeded field) **and** a disable mechanism, named per-element in the STANDING-directive compliance table below.

---

## Decisions, with evidence

### D1 — `output_trajectory_carpet` needs a new library rendering surface, not a profile row on an existing adapter

**Co-location note (review 2026-09-09, measured):** carpet vertices sit 1.02–1.38 m (median 1.05, p90 1.10) from `/behavior_path_planner/output_path_visualization` and 1.02 m from `/local_vel_path` on synchronized stamps, same `map` frame — the carpet is a ~2 m band centred on the very trajectory two shipped ribbons already draw. KEPT anyway: the per-vertex velocity gradient is genuinely new information no existing path can render — and this co-location is exactly what makes `kTrajectoryCarpetBaseAlpha` (0.7 translucency) and the below-alert z-lift load-bearing rather than cosmetic.

Measured (§1, §6): `TRIANGLE_LIST` (type 11), always a multiple of 3 (24–906 pts), one persistent `ns='output_trajectory_carpet' id=0` marker refreshed every tick (`DELETE_ALL` + one `ADD`), **`len(colors) == len(points)` on 1300/1300 sampled non-delete messages** — a true per-vertex velocity gradient (r 0→1, g 0.08→1, b≡0, a≡0.70; no `speed` field on `visualization_msgs/Marker` exists, so this IS the producer's own baked colormap).

Two existing paths were traced against this shape and **both discard the gradient**:
- `CollisionAdapter` hard-requires `LINE_STRIP` — rejects `TRIANGLE_LIST` outright, not a candidate.
- `GenericMarkerAdapter`/`generic_markers.cpp` accepts `TRIANGLE_LIST` geometrically but reads only the marker's single top-level `m.color` into one quantized `MaterialInstance` tint per marker slot — the per-point `colors[]` gradient would be silently discarded. Confirmed at the render layer: `generic_markers.cpp`'s `resolve_marker_material()` has no per-vertex `COLOR` vertex attribute anywhere in that file; the **only** hit for vertex-color machinery in the whole render library is `point_cloud.cpp`'s `VertexBuffer::Builder()` (`filament::VertexAttribute::COLOR`, `UBYTE4`, `.normalized()`).

**Decision:** a new `SceneGraph` category, `TrajectoryCarpet`, rendered as a flat `TRIANGLES` primitive (sequential indices, exactly `point_cloud.cpp`'s own indexing for `POINTS` — no index buffer semantics needed, the wire data is already a flat triangle list) — mirroring `point_cloud.cpp`'s proven `PointVertex`/`COLOR`-attribute/`UNLIT`/`blending:fade` machinery, **reusing `PointCloudPoint` verbatim as the vertex type** (identical layout need: world position + packed rgba8 — no new vertex struct). This is the "carpet strip reusing the PointVertex/unlit-color machinery as a TRIANGLES primitive" option the measurement report's §6 conclusion names, chosen over "a new kind on an existing category" (would require retrofitting `PointCloud`'s `POINTS`-only render path and `pointSizePx` uniform, which is meaningless for triangles, onto existing tested/golden-covered code) and over "quantized uniform-color segments" (rejected outright by D1's own evidence: the whole point of this topic, stated by the user, is the velocity gradient — quantizing it away defeats the ask).

### D2 — Collision-role remap: content, not rviz slot, decides `role`

Per the epic3 plan's own rviz-reference-update CAUTION (2026-09-09): "slot reuse shows what the team watches now, NOT a semantic 1:1 role mapping — assign collision roles from the recorded messages' own ns/content, not from which rviz slot a topic landed in." All five `/navigation_urban_collision_checker_testing_node/*` rows are confirmed **dead** (0 live topics on the new stack — verified via `ros2 topic list -t` at recording time, and the epic3 plan's rviz-diff note). Measurement §2 gives content-verified mappings for exactly two of the five roles:

| Old row (role) | New topic | Evidence |
|---|---|---|
| `collision_markers` (role `collision`) | `/behavior_path_planner/collision_markers` | Explicit `"COLLISION\nobject N..."` text label, two closed 5-pt rectangle rings (ego+object footprints), **no** "predicted path" qualifier anywhere in 764 sampled messages. |
| `object_predicted_polygons` (role `predicted`) | `/navigation_motion_obstacle_planner_node/collision_markers` | Every label ties the event to `"object N, predicted path M"` — up to 11 markers/msg (several concurrent object×path checks) — matching the old `predicted` semantics far more than a single definite collision, **despite this topic's own text also literally containing the word "COLLISION"**. |

**Flagged, unresolved judgment call (carried forward from the measurement report, not silently resolved here):** the second mapping's competing reading — that `/navigation_motion_obstacle_planner_node/collision_markers` should itself be `role: collision`, since its label says "COLLISION" too — is not settled by content alone; only the "predicted path N" qualifier discriminates the two topics, and it appears exclusively on this one. Task 4 ships the `role: predicted` mapping as the measured recommendation, with this ambiguity stated in the row's own YAML comment (same "flagged, not silently reconciled" convention this codebase already uses for the HUD text depth-test re-entry trigger in Epic 3 Task 3) — **a human sign-off on this one row is the correct next step before it's treated as final**, not a plan-time guess.

No successor was measured for the remaining three roles (`sweep`, `merged_ego`, `merged_object`) — the recorded new-topic list contains no LINE_STRIP-footprint-pair content matching either "ego sweep" or "merged polygon" semantics (§2's three-topic table is exhaustive for collision-shaped content in the recording). Per this plan's own scope-discipline instruction ("dormant rows for genuinely-absent topics get the `/road_markers` disabled-row-with-reenable-checklist treatment"), these three rows go dormant (commented out, `urban_profile.yaml:147-163`'s exact template — DISABLED banner, reason, `>>> RE-ENABLE by...` checklist) rather than deleted, so a future measurement pass with more coverage has something to uncomment onto.

`/navigation/debug_cruise_obstacle_marker` is explicitly **excluded** from this remap even though it occupied the `ego_footprint_sweep` rviz slot: its geometry is `CUBE`+`TEXT`, never `LINE_STRIP` (§2) — routing it through `adapter: collision` would drop 100% of its content (`dropped_malformed` every marker, `CollisionAdapter::ingest()`'s own hard requirement). It gets its own `generic`-adapter row instead (D3).

### D3 — Two new `generic`-adapter rows, no adapter code change

- `/navigation/debug_cruise_obstacle_marker`: `CUBE`+`TEXT`, a single continuously-tracked object box (5.24×1.93×1.5 m) + a value label — `GenericMarkerAdapter` already handles both primitives (§2, §6's own recommendation table). `role: neutral` (the only legal role for `adapter: generic`, `profile.cpp`'s `RoleSets()`).
- `/local_map_corners`: a static 5-pt closed `LINE_STRIP` box outline, 38 Hz but constant 352-byte payload (§5) — `GenericMarkerAdapter` already handles `LINE_STRIP`. `role: neutral`.

Both ship with `best_effort` unset (measured RELIABLE/VOLATILE, not the sensor-ish best-effort case) and no `namespaces:` rule list (single unnamed/`''` namespace on each, per §2/§5 — `ns_default: polyline` default is correct without an explicit list, unlike the multi-namespace hd_map/dynamic_objects rows that need one).

### D4 — Six explicit SKIP/defer decisions, no rows, no code

| Topic | Verdict | Evidence |
|---|---|---|
| `/behavior_path_planner/output_path_line` | **SKIP** | Largest sampled message (190 pts) is a 190/190 exact-coordinate match (2-decimal) against the closest-in-time `output_path_visualization` `Path` message (Δt −0.35 ms) — a `MarkerArray` republish of the same polyline the `path` adapter already ribbons (§3). |
| `/behavior_path_planner/output_path_velocity_visualization` | **SKIP (defer)** | Not a geometric duplicate (190 `TEXT_VIEW_FACING` speed labels, no LINE_STRIP at all) — genuinely new per-point data — but `generic_markers.cpp`'s `TEXT_VIEW_FACING` path is a stated placeholder billboard with **no glyph rendering** (only `hud_overlay.cpp`'s `stb_truetype` compositor draws real text, and only in screen-space HUD/callout use). Routing this through `generic` today renders 190 blank oriented boxes per frame, not numbers (§3, §6). Revisit once a 3D-anchored text path exists (`scene.h`'s `Hud::chips`/`AlertChip` are reserved for exactly this, unused by design per Epic 3 Task 4's own scope note) — not this plan's job to build one. |
| `dynamic_objects_marker_1` | **SKIP** | 31/31 exact distance-signature match against `marker_2` (same objects, `base_link` vs `map` frame) — a pre-transform duplicate. |
| `dynamic_objects_marker_2` | **SKIP** | 24/31 direct position matches against `perception/dynamic_objects_list` (same idx, map frame; rounding + small Δt explains the rest) — duplicate. |
| `dynamic_objects_marker_3` | **SKIP** | 32/32 exact distance-signature match against `perception/dynamic_objects_list` — full duplicate. |
| `dynamic_objects_marker_4` | **SKIP** | 0 CUBEs, 243–518 ARROW/CYLINDER/LINE_STRIP markers/msg matching `dynamic_objects_list`'s own `dynamic_objects_hd_map_path{,_dots}` namespaces in shape — near-certainly a duplicate path encoding (not point-for-point confirmed, time-budgeted). |
| `dynamic_objects_marker_5`, `_6` | **SKIP** | 0 messages in either recorded bag. |
| `/sim/hd_map/local_markers` | **SKIP** | Same `centerline_N`/`left_boundary_N`/`right_boundary_N` ns nomenclature as the already-rendered `/hd_map_local_elements`, overlapping road geometry, smaller id space (max ~167 vs ~7625) — a duplicate (likely sim-ground-truth) source. **~297 KB/msg const, ~3 MB/s at 10 Hz, BEST_EFFORT** — cost without new information. Revisit only as a deliberately-scoped, separately-toggleable ground-truth overlay row, not a default-on duplicate. |
| `/navigation/bvp_crosswalk_attention_areas` | **DEFER, no row** | Every sampled message across the whole bag (1818/1818) is `DELETE_ALL`-only — never one real `ADD` marker in this recording. Same class as the Epic 2 `/road_markers`-adjacent "fixture gap" cases, but with no rviz-enabled-display obligation forcing a row the way `/hd_map_global_elements`/`/local_path` had — so no row at all, not a stale-faded one. |
| `/navigation/trajectory_following_status` | **SKIP (data-inconclusive)** | `UInt8`, value ≡ 1 for all 3657 messages in the entire bag — a plausible HUD-chip candidate structurally (single byte → color-coded chip, same shape as the existing `hud_enabled`-gated compositor), but **zero evidence exists in this recording of what any other value means**. Needs a msg-definition/source read or a scenario that actually exercises a deviation — out of scope for a measurement pass and out of scope for this plan, which implements only what's measured. |
| `/navigation/global_path`, `/local_path`, `/perception/dynamic_ogm{,_updates}` | **UNCHANGED** | Confirmed absent from both bags' topic lists — no new evidence this pass; Epic 2's existing FIXTURE GAP rows in all three shipped profiles stand as-is. |

Every SKIP above gets a one-paragraph comment in `urban_profile.yaml` at the point where a naive editor might otherwise add the row (Task 6) — same "a silently missing row is indistinguishable from an oversight" reasoning the Epic 2 plan states for `offroad_profile.yaml`'s own omission comments.

---

## Interfaces added this plan

**Additive only, per ADR-0004.** One `scene.h` change: appends `TrajectoryCarpet` and bumps `kSceneVersion` 2 → 3.

```cpp
// ── TrajectoryCarpet[] (VM-077, ADR-0004 additive) ──────────────────────────
// output_trajectory_carpet: TRIANGLE_LIST, always a multiple of 3, genuine
// per-vertex color (the producer's own velocity colormap -- r/g vary, b≡0,
// a≡0.70; see the VM-077 measurement report). Reuses PointCloudPoint
// verbatim (identical layout need: world position + packed rgba8) rather
// than a new vertex struct. Rendered as a flat TRIANGLES list, sequential
// indices -- mirrors point_cloud.cpp's own POINTS indexing exactly, minus
// the pointSizePx uniform (meaningless for triangles).
struct TrajectoryCarpet {
    const PointCloudPoint* points;  uint32_t point_count;  // multiple of 3
    double last_update_sec;
};
```

`SceneGraph` gains `const TrajectoryCarpet* trajectory_carpets; uint32_t trajectory_carpet_count;`, appended after `point_cloud_count` (current tail, offset 192, 4 bytes, ending at 196 — `SceneGraph`'s existing 200-byte size already carries 4 bytes of trailing pad for 8-byte alignment). Arithmetic, verified against the current header (`sizeof(mpviz::SceneGraph) == 200`, `offsetof(..., point_clouds) == 184`, `offsetof(..., point_cloud_count) == 192`, confirmed in both `tests/test_scene_buffer.cpp` and the node's `test_scene_layout.cpp`): `trajectory_carpets` (pointer, needs 8-byte alignment) starts at offset **200** (the existing pad); ends at 208. `trajectory_carpet_count` (`uint32_t`) at offset **208**; ends at 212, rounded up to the struct's 8-byte alignment: **`sizeof(SceneGraph)` becomes 216.**

No other public header changes. `CollisionAdapter`/`GenericMarkerAdapter`/`profile.hpp`'s `ProfileRow` gain no new fields — Tasks 3-5 add a new **adapter name** (`trajectory_carpet`) to `profile.cpp`'s closed sets, not new struct fields.

---

## STANDING-directive compliance (style token + disable knob, named per element)

| Element | Style token | Disable mechanism |
|---|---|---|
| `output_trajectory_carpet` (`TrajectoryCarpet` category) | Per-vertex RGB **is** the data (producer-baked velocity gradient) — no themed color to override. One fixed style value does exist, though (review fix, 2026-09-09): `trajectory_carpet.cpp`'s `kTrajectoryCarpetBaseAlpha` (0.7), the measured `m.color.a` on every ADD marker (1300/1300 `stack_v2_fixtures_2026-09-09`, 1647/1647 `stack_v2_full_sensors`; rviz parity — `TriangleListMarker` takes material alpha from `marker.color.a`), multiplied into the staleness `alpha` uniform so a fresh carpet renders translucent rather than opaque. Malformed/absent-colors fallback reuses `palette.object_tints.unknown` **verbatim, zero new theme field** — same sentinel convention `PointCloud`'s `color_mode: flat` and `GenericMarker`'s `alpha==0` already establish (colors[] mismatched length ⇒ packed rgba `a=0` ⇒ neutral tint at mesh-build time). | (a) Profile row is removable (comment it out — the STANDING directive's base mechanism). (b) New category-level `layer_trajectory_carpet` param (default `true`), extending VM-032's `layer_*`/`apply_layer_gates()` gate to this new `SceneAssembly` vector — the exact "Task 5 declares, Task 6 wires" shape `layer_point_clouds` used, collapsed into one task here since this plan has no cross-epic gap to bridge. |
| `/behavior_path_planner/collision_markers` (role `collision`), `/navigation_motion_obstacle_planner_node/collision_markers` (role `predicted`) | Existing `palette.alert.{critical,warning}` tokens via `CollisionAdapter::severity_for_role()` — already live, zero new fields (role mapping is a profile-row edit, not a code change — `collision.cpp` is untouched). | Profile row removable + existing `layer_alerts` param (VM-032, already covers the whole `AlertPolygon` category). |
| Three dormant collision rows (`sweep`/`merged_ego`/`merged_object`) | N/A — nothing renders while dormant. | Commented-out profile row, `/road_markers`-template re-enable checklist (Task 4). |
| `/navigation/debug_cruise_obstacle_marker`, `/local_map_corners` (adapter `generic`) | Existing `GenericMarker::color[]`/`alpha==0`→`palette.object_tints.unknown` fallback — already live (both topics carry non-zero producer colors per §2/§5, so the fallback is defensive-only, same as every other `generic` row). | Profile row removable + existing `layer_markers` param (VM-032, covers the whole `GenericMarker` category). |
| SKIP/defer topics (D4) | N/A — not rendered. | N/A — no row exists to disable. |

---

## Task 1: `scene.h` — `TrajectoryCarpet` category + `kSceneVersion` → 3

**Files:**
- Modify: `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h` (append `TrajectoryCarpet`, `SceneGraph::trajectory_carpets`/`trajectory_carpet_count`, bump `kSceneVersion`)
- Modify: `cuda/src/libs/visual_renderer/tests/test_scene_buffer.cpp` (sizeof/offsetof static_asserts, clang/libc++ side)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_scene_layout.cpp` (the gcc/libstdc++ mirror — same numbers, ADR-0004's "both toolchains" requirement)

**Interfaces:** `TrajectoryCarpet`, `SceneGraph::{trajectory_carpets,trajectory_carpet_count}` — additive (D1, "Interfaces added" above).

- [x] **Step 0: append the struct + bump the version, update both static_assert mirrors in the same commit.** Failing tests first (mirrors Task 1/Task 6's own pattern in the epic3 plan):
  **Done 2026-09-09.** `scene.h`: `kSceneVersion` 2→3, `TrajectoryCarpet` struct appended (reuses `PointCloudPoint`), `SceneGraph::{trajectory_carpets,trajectory_carpet_count}` appended after `point_cloud_count`. Both static_assert mirrors (`tests/test_scene_buffer.cpp`, node's `test_scene_layout.cpp`) updated with the arithmetic from the plan (offsets 200/208, `sizeof(SceneGraph)==216`). `check_pod_header.sh` green. Library rebuilt clean, **133/133 PASS** (no regression from the new offsets).
```cpp
static_assert(mpviz::kSceneVersion == 3, "bump alongside every additive scene.h change");
static_assert(sizeof(mpviz::TrajectoryCarpet) == 24, "TrajectoryCarpet layout, ADR-0004 additive");
static_assert(offsetof(mpviz::TrajectoryCarpet, points) == 0, "...");
static_assert(offsetof(mpviz::TrajectoryCarpet, point_count) == 8, "...");
static_assert(offsetof(mpviz::TrajectoryCarpet, last_update_sec) == 16, "...");
static_assert(sizeof(mpviz::SceneGraph) == 216, "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(mpviz::SceneGraph, trajectory_carpets) == 200, "...");
static_assert(offsetof(mpviz::SceneGraph, trajectory_carpet_count) == 208, "...");
```
  Run — FAIL (symbols undeclared) then PASS. Same message-rewording convention as Task 6's own SceneGraph assertion ("ADR-0004 additive", not "layout frozen").
  **Review gate:** `git diff -- cuda/src/libs/visual_renderer/include/` shows only appended lines (no rename/reorder/remove); `check_pod_header.sh` still green (POD-only: a pointer + two integral fields, no new pointer-to-non-POD, no vtable).

---

## Task 2: Library rendering — `TrajectoryCarpet` layer

**Files:**
- New: `cuda/src/libs/visual_renderer/src/trajectory_carpet.hpp`, `trajectory_carpet.cpp` (mirrors `point_cloud.hpp`/`.cpp` structure exactly — vertex build, signature/diff, chunking)
- New: `cuda/src/libs/visual_renderer/assets/materials/trajectory_carpet.mat` (UNLIT, `requires:[color]`, `blending:fade`, one `alpha` param — no `pointSizePx`; auto-picked up by the `file(GLOB ... assets/materials/*.mat)` matc step, no CMakeLists.txt edit)
- Modify: `cuda/src/libs/visual_renderer/src/renderer_internal.hpp` (append `trajectoryCarpetMaterialInstance`, `trajectoryCarpetAlpha`, `std::vector<TrajectoryCarpetSlot> trajectoryCarpetSlots` — mirrors the `pointCloud*` members verbatim)
- Modify: `cuda/src/libs/visual_renderer/src/scene_buffer.hpp`/`.cpp` (`OwnedScene` deep-copy + `assign()` re-pointing for the new category — mirrors `point_clouds`/`point_cloud_points` exactly)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (create the material instance eagerly at `create_renderer()` time, same as `pointCloudMaterialInstance`; call `update_trajectory_carpets(r, scene)` once per `render_frame()`, right after `update_point_clouds(...)`)
- New test: `cuda/src/libs/visual_renderer/tests/test_trajectory_carpet.cpp` (+ a `trajectory_carpet_test_hooks.hpp` Filament-free introspection header, mirroring `point_cloud_test_hooks.hpp`)

**Interfaces:** none beyond Task 1's — this task is pure implementation behind `update_trajectory_carpets()` (internal, not in `scene.h`).

- [x] **Step 0: `OwnedScene` deep-copy.** Failing test in `tests/test_scene_buffer.cpp`:
  **Done 2026-09-09.** `SceneBufferTrajectoryCarpet.TrajectoryCarpetPointsSurviveAssignAfterSourceBufferDies` added; `scene_buffer.hpp`/`.cpp`'s `OwnedScene` gained `trajectory_carpets`/`trajectory_carpet_points` deep-copy storage, mirroring `point_clouds` exactly.
```cpp
TEST(SceneBuffer, DeepCopy_SurvivesCallerBufferReuse_TrajectoryCarpet) {
    // publish() a SceneGraph with one TrajectoryCarpet whose points[] lives
    // in a caller-owned std::vector; mutate/free that vector after publish()
    // returns; assert buf.active().trajectory_carpets[0].points still reads
    // the original values (same "deep, not shallow" trap Task 6's own
    // DeepCopy_SurvivesCallerNestedBufferReuse test exists to catch).
}
```
  Run — FAIL then PASS.
- [x] **Step 1: vertex build + `TRIANGLES` primitive.** Failing tests in `test_trajectory_carpet.cpp`:
```cpp
TEST(TrajectoryCarpet, BuildsOneTriangleMeshFromAFlatMultipleOfThreePointList) { ... }
TEST(TrajectoryCarpet, PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero) { ... }
TEST(TrajectoryCarpet, AlphaZeroSentinelSubstitutesPaletteObjectTintsUnknown) {
    // same substitution point_cloud.cpp's resolve_rgba() already implements
    // -- reuse that free function or an identical one-line copy (it's
    // three lines, not worth extracting into a shared header for one
    // second caller yet -- ponytail: duplicate the 3-line helper, promote
    // to a shared header if a third caller ever needs it).
}
TEST(TrajectoryCarpet, ChunksAtPolylineChunksCeilingForAnOversizedCarpet) { ... }
```
  Implement `update_trajectory_carpets()`: diff `scene.trajectory_carpets`/`_count` against `r.trajectoryCarpetSlots` (keyed by slot index, identical shape to `pointCloudSlots`), rebuild-on-signature-change (reuse `point_cloud_signature()`'s shape, new function, same three-field hash), `RenderableManager::Builder(1).geometry(0, PrimitiveType::TRIANGLES, vb, ib)` with a plain sequential index buffer (`indices[i] = i`, exactly `point_cloud.cpp`'s own convention — the wire data is already ordered triangles, no fan/strip reinterpretation needed). ONE shared `MaterialInstance` for the whole layer, `alpha` driven by the freshest live carpet's `staleness_alpha()` (same N>1 policy point_cloud.cpp states and justifies — today's shipped profile carries exactly one row, so this never fires in practice, same as PointCloud's own note).
  Run — FAIL then PASS.
  **Done 2026-09-09.** `trajectory_carpet.{hpp,cpp}` created (mirrors `point_cloud.{hpp,cpp}`); `trajectory_carpet_test_hooks.hpp` added (mesh/vertex count, material alpha, plus a `trajectory_carpet_vertex_rgba()` CPU-mirror hook — a `TrajectoryCarpetSlot::firstMeshRgba` field kept purely for test introspection, same "not a Filament read-back" reasoning as `RibbonSlot::firstPointM`, since per-vertex GPU color can't otherwise be asserted from a Filament-free test). All four planned tests (`BuildsOneTriangleMeshFromAFlatMultipleOfThreePointList`, `PerVertexColorPassesThroughUnchangedWhenAlphaByteIsNonzero`, `AlphaZeroSentinelSubstitutesPaletteObjectTintsUnknown`, `ChunksAtPolylineChunksCeilingForAnOversizedCarpet`) plus two extra parity tests (`SlotReleasedWhenCarpetCountDrops`, `MaterialAlphaFollowsStaleness`, mirroring point_cloud's own coverage) added and green.
  **Found + fixed during live verification (2026-09-09, after Task 6):** the adapter flattens the carpet's z to 0.0 (D1's own "2D-plane category" convention, matching hd_map/path/collision) — but unlike every OTHER flattened category, `trajectory_carpet.cpp` applied no renderer-side z-lift, leaving it exactly coplanar with the opaque ground plane. Every unit test above stayed green (the geometry IS built correctly) because none rendered against a real ground-covered scene at real scale; running the live rig against the validate script's default bag (see Results) showed the carpet was actually INVISIBLE on screen despite the adapter ingesting real messages with 0 dropped_malformed. Root cause: a translucent triangle exactly coplanar with the opaque ground loses the depth test almost everywhere (the same z-fighting concern `map_elements.cpp`'s `kLaneZLiftM`, `ground_grid.cpp`'s OGM lift, `ribbon.cpp`'s per-role `kRibbonZLiftByRoleM`, and `alert_polygons.cpp`'s 0.06 "topmost" lift all already exist to avoid). Fix: `kTrajectoryCarpetZLiftM = 0.055f` added in `trajectory_carpet.cpp`, applied to every vertex's world z at mesh-build time — placed above every path ribbon (ribbon.cpp's `kRibbonZLiftByRoleM` tops out at 0.05) but strictly BELOW alert_polygons.cpp's `kAlertZLiftM` (0.06), so alerts stay the topmost layer: the carpet is translucent while fresh (alpha 0.7) and would otherwise visually merge with alert rings sitting at the same depth on the ego's own corridor. New regression test `VertexZIsLiftedAboveTheFlattenedZeroTheAdapterSends` asserts `EXPECT_LT(z, 0.06f)` (verified to FAIL without the fix, PASS with it) plus a `trajectory_carpet_vertex_z()` test hook (`TrajectoryCarpetSlot::firstMeshZ`, same CPU-mirror shape as `firstMeshRgba`). A rendered-pixel version of the same check was tried and rejected — verified to pass regardless of the bug at this synthetic scene's scale, so it would not have caught the regression and was not kept (a test that cannot fail is not a check). Library re-verified **141/141 PASS** (140 + 1 new). Node force-relinked, **18/18 PASS**, no regressions. See the Results section for the live-rig before/after screenshots that surfaced and then confirmed this fix.
- [x] **Step 2: `trajectory_carpet.mat`.** No test (asset-only step, matching Epic 3 Task 6's own Step 2 convention) — verified by the library linking and `TrajectoryCarpet.BuildsOneTriangleMeshFromAFlatMultipleOfThreePointList` exercising the real `MaterialInstance`.
  **Review gate:** `git grep -n pointSizePx assets/materials/trajectory_carpet.mat` empty (triangles have no per-vertex size uniform — carrying it over from `point_cloud.mat` unmodified would be dead config, not "STANDING-directive style," and the STANDING-directive compliance table above states no style token is needed here beyond the reused fallback tint).
  **Done 2026-09-09.** Verified empty. `renderer_internal.hpp`/`scene_buffer.{hpp,cpp}`/`renderer.cpp` wired (material create/destroy, `update_trajectory_carpets()` call site right after point clouds). Library rebuilt after a `.mat`-only edit (comment fix, cmake reconfigure re-ran matc) — **140/140 PASS** (133 baseline + 7 new: 1 `SceneBufferTrajectoryCarpet` deep-copy test + 6 `TrajectoryCarpet` tests). `check_pod_header.sh` green (no public header touched by Task 2).

---

## Task 3: Node — `TrajectoryCarpetAdapter` + profile wiring + `layer_trajectory_carpet`

**Files:**
- New: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/adapters/trajectory_carpet.hpp`, `src/adapters/trajectory_carpet.cpp` (`#include ".../adapters/point_cloud.hpp"` for `PackRgba()` — reuse, not a re-implementation)
- Modify: `include/micropilot_visualization_node/profile.hpp`/`src/profile.cpp` (`RoleSets()["trajectory_carpet"] = {"carpet"}`, `TypeSets()["trajectory_carpet"] = {"visualization_msgs/msg/MarkerArray"}`, add `"trajectory_carpet"` to the closed adapter-name set — no new `ProfileRow` fields)
- Modify: `include/micropilot_visualization_node/scene_assembly.hpp`/`src/scene_assembly.cpp` (`SceneAssembly::trajectory_carpets` vector, `LayerFlags::trajectory_carpet` (default `true`), `apply_layer_gates()` extended — mirrors every `point_clouds`-shaped line added there in VM-035/its review fix)
- Modify: `config/default_params.yaml` (`layer_trajectory_carpet: true`, alongside the other seven `layer_*` keys)
- Modify: `src/visualization_node.cpp` (subscribe loop for `adapter: trajectory_carpet` — copy the `point_cloud` block at `visualization_node.cpp:382-400` verbatim, swap the message type/adapter class; `layer_trajectory_carpet_` member + `declare_parameter`/`SetParametersCallback` case; fill-loop + diagnostics-row entries; `on_configure()`/`on_cleanup()` clearing, same shape as `point_cloud_subs_`/`point_cloud_rows_`)
- Modify: `include/micropilot_visualization_node/visualization_node.hpp` (`CarpetRow` struct + vector, mirrors `PointCloudRow`; `layer_trajectory_carpet_` member)
- Modify: `config/urban_profile.yaml` (new row, D1/D2)
- Modify: `CMakeLists.txt` (add `src/adapters/trajectory_carpet.cpp` to the node-lib source list at the same two spots `adapters/point_cloud.cpp` appears; new `ament_add_gtest(test_trajectory_carpet_adapter ...)` target mirroring `test_point_cloud_adapter`'s block verbatim)
- Modify: `tools/vcam_ws_bridge.py` (`LAYER_NAMES` gains `"trajectory_carpet"`)
- Modify: `tools/vcam_gui.py` (layer-switch checklist comment: seven switches → eight)
- New test: `test/test_trajectory_carpet_adapter.cpp`

**Interfaces:** none beyond `scene.h`'s Task 1 addition — `layer_trajectory_carpet` is a node param, not a public header symbol.

- [x] **Step 0: adapter ingest — single persistent marker, wholesale replace.** Failing tests:
```cpp
TEST(TrajectoryCarpetAdapter, IngestPacksPerVertexColorFromMarkerColorsArray) {
    // one TRIANGLE_LIST marker, points.size()==colors.size()==6 (2 triangles);
    // assert fill()'s TrajectoryCarpet.points[i].rgba unpacks back to the
    // marker's own colors[i].{r,g,b} (alpha byte forced to 255 -- "supplied"
    // sentinel, same convention as GenericMarkerAdapter's per-point fan_colors).
}
TEST(TrajectoryCarpetAdapter, MismatchedColorsLengthBakesAlphaZeroSentinel) {
    // colors.size() != points.size() -> every point's packed rgba has a==0
    // (point_cloud.cpp substitutes palette.object_tints.unknown at build time).
}
TEST(TrajectoryCarpetAdapter, PointCountNotMultipleOfThreeDropsMalformed) { ... }
TEST(TrajectoryCarpetAdapter, DeleteAllClearsStoredCarpet) { ... }
TEST(TrajectoryCarpetAdapter, ReplacesStoredCarpetWholesaleNotAppend) {
    // same PathAdapter-style "REPLACES, never merges" contract -- two
    // ingest() calls, second's content is what fill() emits, not a union.
}
```
  Implement: ignore marker `ns`/`id` (single persistent marker per the measured cadence — no `Key{ns,id}` map needed, unlike `CollisionAdapter`/`GenericMarkerAdapter`); on `DELETE_ALL`, clear; on `ADD`/`MODIFY` with `type==TRIANGLE_LIST && points.size()%3==0 && points.size()>=3`, transform every point through the one per-message TF lookup (same `flatten_z` convention as every other adapter — the carpet's own z is data, ~0.150 m, and gets flattened like every other 2D-plane category, consistent with `hd_map`/`path`/`collision`), pack `colors[i]` via `PackRgba()` when `colors.size()==points.size()`, else `rgba=0` for every point (whole-message fallback, not per-point — a length mismatch means the whole array is suspect); replace stored wholesale.
  Run — FAIL then PASS.
  **Done 2026-09-09.** `adapters/trajectory_carpet.{hpp,cpp}` created (mirrors `collision.cpp`'s TF-composition/pose shape + `PathAdapter`'s wholesale-replace contract; reuses `PackRgba()` from `adapters/point_cloud.hpp`). All 5 planned tests green (`IngestPacksPerVertexColorFromMarkerColorsArray`, `MismatchedColorsLengthBakesAlphaZeroSentinel`, `PointCountNotMultipleOfThreeDropsMalformed` — extended with a non-TRIANGLE_LIST-type case too, `DeleteAllClearsStoredCarpet`, `ReplacesStoredCarpetWholesaleNotAppend`). `SceneAssembly::trajectory_carpets` vector + `clear()`/`point_at()` wiring added.
- [x] **Step 1: profile registration.** Failing tests in `test_profile.cpp`:
```cpp
TEST(Profile, TrajectoryCarpetAdapterAcceptsRoleCarpetOnly) { ... }
TEST(Profile, TrajectoryCarpetAdapterRejectsWrongMessageType) { ... }
```
  Run — FAIL then PASS.
  **Done 2026-09-09.** `RoleSets()["trajectory_carpet"]={"carpet"}`, `TypeSets()["trajectory_carpet"]={"visualization_msgs/msg/MarkerArray"}`, `"trajectory_carpet"` added to `KnownAdapters()` (`profile.cpp`; `profile.hpp` needed no struct-field change — no new `ProfileRow` fields for this adapter). Both tests green.
- [x] **Step 2: `SceneAssembly`/`LayerFlags`/`apply_layer_gates()`.** Failing tests in `test_scene_assembly.cpp`:
```cpp
TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOffZeroesOnlyTrajectoryCarpets) { ... }
TEST(SceneAssembly, ApplyLayerGatesTrajectoryCarpetOnLeavesItIntact) { ... }
```
  Run — FAIL then PASS. (Directly closes the gap the epic3 plan's own review fix named for `layer_point_clouds` — this plan ships the gate test in the SAME task as the category, no forward-reference window to leave open.)
  **Done 2026-09-09.** `LayerFlags::trajectory_carpet` (default `true`) added; `apply_layer_gates()` extended. Both tests green (plus `TrajectoryCarpetRowAppendsIntoSceneTrajectoryCarpets`, mirroring the point-cloud parity test).
- [x] **Step 3: node wiring + `urban_profile.yaml` row.** No new unit test beyond Step 0-2's (same convention as `point_cloud`'s own Task 6 Step 3/4 — node-level `rclcpp` wiring has no gtest harness in this suite); verified by the node binary compiling/linking and the shipped-profile parity test (`ShippedProfilesRouteEveryKnownNamespaceOfEveryShippedTopic`, extended in Task 1's row-count/topic-table update — see Task 4/5 below for the shared test-table edit) picking up the new row.
```yaml
  # Trajectory carpet (VM-077, 2026-09-09) -- per-vertex velocity gradient,
  # TRIANGLE_LIST, refreshed every tick (DELETE_ALL + ADD). See the plan's
  # D1 for why this needs its own adapter/category rather than riding on
  # generic_marker or collision. Disable knob: layer_trajectory_carpet
  # param (STANDING directive); style: per-vertex color IS the data, the
  # only themed piece is the malformed-colors fallback tint, which reuses
  # palette.object_tints.unknown verbatim (zero new theme field).
  - {topic: /navigation_motion_obstacle_planner_node/output_trajectory_carpet,
     type: visualization_msgs/msg/MarkerArray, adapter: trajectory_carpet,
     role: carpet, timeout_sec: 2.0}
```
  **Review gate:** `git grep -n layer_trajectory_carpet` hits `default_params.yaml`, `visualization_node.cpp` (declare + `SetParametersCallback` case + gate call site), `scene_assembly.{hpp,cpp}`, `tools/vcam_ws_bridge.py`'s `LAYER_NAMES` — a knob declared in only one of these is the exact "forward reference left open" gap VM-035's own review caught once already; this task ships the whole chain in one pass, no follow-up task needed to close it.
  **Done 2026-09-09.** `visualization_node.hpp`/`.cpp`: `CarpetRow` struct + `carpet_rows_`/`carpet_subs_` vectors, `layer_trajectory_carpet_` member (declare_parameter + `on_params()` case + `apply_layer_gates()` call site + `timer_callback()` fill loop + `publish_diagnostics()` append_row + `on_cleanup()`/`on_shutdown()` clears — all mirroring `point_cloud`'s own wiring exactly). `config/default_params.yaml`: `layer_trajectory_carpet: true`. `config/urban_profile.yaml`: new row (`/navigation_motion_obstacle_planner_node/output_trajectory_carpet`). `CMakeLists.txt`: `src/adapters/trajectory_carpet.cpp` added to the node-lib source list; `test_trajectory_carpet_adapter` gtest target added (mirrors `test_collision_adapter`'s no-`visual_renderer_prebuilt` shape — POD-only scene.h types). `tools/vcam_ws_bridge.py`/`tools/vcam_gui.py`: `LAYER_NAMES` gains `"trajectory_carpet"` (seven switches → eight). `git grep -n layer_trajectory_carpet` confirms hits in all five files the review gate names. Node force-relinked (`colcon_build.sh micropilot_visualization_node --cmake-force-configure`, then a plain rebuild) — **18/18 PASS** (17 baseline + 1 new `test_trajectory_carpet_adapter` binary; `test_profile`'s `CoexistsWithTheRendererLibrarysOwnYamlCpp` row-count bumped 15→16 for the one new urban row, comment updated). `check_pod_header` green (no public header touched by Task 3). Library unaffected, still 140/140 (no library source touched this task).

---

## Task 4: Collision row remap (5 dead → 2 retargeted + 3 dormant)

**Files:**
- Modify: `config/urban_profile.yaml`, `config/sim_profile.yaml`, `config/offroad_profile.yaml` (all three carry the same five dead rows — verified identical topic names in all three files; the dead namespace is confirmed absent stack-wide, not urban-specific, so the retirement applies to all three)
- Modify: `test/test_profile.cpp` (row-count comments, per-profile expected-topic/-role tables, the disabled-row-count assertions)

**Interfaces:** none — `CollisionAdapter`'s `severity_for_role()` table (`collision.cpp`) already covers every role used below; this task is profile-YAML-only.

- [x] **Step 0: retarget the two content-verified successors, in all three profiles.**
```yaml
  # Retargeted 2026-09-09 (VM-077): /navigation_urban_collision_checker_testing_node/*
  # confirmed dead stack-wide (0 live topics, ros2 topic list -t at
  # recording time). Role from CONTENT, not the rviz slot the topic
  # happened to land in (see the epic3 plan's rviz-reference-update
  # CAUTION) -- explicit "COLLISION\nobject N..." label, no "predicted
  # path" qualifier, two closed-ring footprints (ego+object).
  - {topic: /behavior_path_planner/collision_markers,
     type: visualization_msgs/msg/MarkerArray, adapter: collision, role: collision}
  # FLAGGED JUDGMENT CALL (VM-077 measurement report §2, not resolved by
  # content alone): every label ties the event to "object N, predicted
  # path M" -- matching the OLD object_predicted_polygons semantics --
  # but this topic's own text also literally contains "COLLISION", same
  # as the row above. Only the "predicted path N" qualifier discriminates
  # the two topics, and it appears ONLY here. Shipped as role: predicted
  # per the measured recommendation; a human sign-off on this one row,
  # not a plan-time guess, is the right next step if it ever needs to flip.
  - {topic: /navigation_motion_obstacle_planner_node/collision_markers,
     type: visualization_msgs/msg/MarkerArray, adapter: collision, role: predicted}
```
  Failing test, `test_profile.cpp` (per profile file): assert `find_row(*profile, "/behavior_path_planner/collision_markers")->role == "collision"` and the predicted row's role/topic, replacing the old assertions that named the dead `/navigation_urban_collision_checker_testing_node/*` topics.
  Run — FAIL then PASS.
  **Done 2026-09-09.** Both rows retargeted in `urban_profile.yaml`/`sim_profile.yaml`/`offroad_profile.yaml` (dead namespace confirmed stack-wide, not urban-specific). `test_profile.cpp`'s `ShippedProfilesCarryEveryCollisionPathAndOgmRow` kExpectedRows table updated (5 old entries → 2 new). **Unplanned but load-bearing fix:** `test_collision_adapter.cpp` built five of its own `CollisionAdapter` unit tests on `urban_row("/navigation_urban_collision_checker_testing_node/...")` — those topics no longer have a live row to fetch (`urban_row()` aborts on a missing topic), which would have broken 5 passing tests as a side effect of a profile-YAML-only task. Fixed at the root: added a `MakeRow(role)` helper (same "no urban_row()/sim_row() to borrow" shape `test_point_cloud_adapter.cpp` already uses) and repointed every call site to build its `ProfileRow` directly from the role string the test actually exercises — `severity_for_role()`'s five-role closed set is unchanged by the YAML remap, so this preserves every test's original intent (CollisionAdapter never reads `row.topic`, only `row.role`).
- [x] **Step 1: dormant the three rows with no measured successor, `/road_markers`-template.**
```yaml
  # DISABLED 2026-09-09 -- VM-077 measurement pass found NO successor topic
  # for this role's content shape (closed-ring ego-sweep polygon) anywhere
  # in the new stack's 31-topic recording; the whole
  # /navigation_urban_collision_checker_testing_node/* namespace is
  # confirmed dead (0 live topics).
  # >>> RE-ENABLE: (1) `ros2 topic list -t` on the live stack to find a
  # >>> candidate; (2) content-verify it against THIS role's semantics --
  # >>> from the message's own ns/geometry/labels, never from which rviz
  # >>> slot it lands in (see the epic3 plan's rviz-reference-update
  # >>> CAUTION); (3) retarget the topic name below; (4) flip this row's
  # >>> assertions in test_profile.cpp back on (marked with the same date).
  # - {topic: /navigation_urban_collision_checker_testing_node/ego_footprint_sweep,
  #    type: visualization_msgs/msg/MarkerArray, adapter: collision, role: sweep}
```
  (and the same template for `ego_merged_polygon`/`role: merged_ego`, `object_merged_polygons`/`role: merged_object`).
  Failing test: assert `find_row(*profile, "/navigation_urban_collision_checker_testing_node/ego_footprint_sweep") == nullptr` (and the other two) in all three profiles' parity tests, plus the row-count comment/assertion updated (`CoexistsWithTheRendererLibrarysOwnYamlCpp`'s `EXPECT_EQ(p->rows.size(), 15u)` — recompute per profile: net active-row delta from this plan's Task 3+4+5 is 0 for urban (−3 dormant collision, +1 carpet, +2 generic = +0), verify the actual arithmetic per file when the row edits land, don't assume the number carries over unchanged).
  Run — FAIL then PASS.
  **Review gate:** `git grep -n navigation_urban_collision_checker_testing_node` across all three profile YAMLs shows only commented-out lines (the three dormant rows), never an active (uncommented) row — the dead namespace must not still be live-subscribed anywhere.
  **Done 2026-09-09.** All three dormant rows (`sweep`/`merged_ego`/`merged_object`) commented out with the `/road_markers`-template DISABLED banner + re-enable checklist, in all three profiles. `find_row(...) == nullptr` assertions added for all five old dead-namespace topics (2 retargeted-away + 3 dormant) in `test_profile.cpp`'s per-profile loop. Row-count: urban `test_profile.cpp`'s `CoexistsWithTheRendererLibrarysOwnYamlCpp` 16→13 (net −3: 5 active collision rows → 2), comment updated. `git grep -n navigation_urban_collision_checker_testing_node` across all three profile YAMLs confirmed: only commented-out lines. Node rebuilt (force-relinked) — **18/18 PASS**, no regressions. Library unaffected (140/140, no library source touched).

---

## Task 5: Two new `generic`-adapter rows

**Files:**
- Modify: `config/urban_profile.yaml` (both rows — D3; not added to `sim_profile.yaml`/`offroad_profile.yaml`, see review-gate note below)
- Modify: `test/test_profile.cpp` (parity-table additions, urban only)

**Interfaces:** none — `adapter: generic` already exists.

- [x] **Step 0: `/navigation/debug_cruise_obstacle_marker`.**
```yaml
  # New topic (VM-077, 2026-09-09): CUBE+TEXT, a single continuously-tracked
  # object box + value label -- NOT LINE_STRIP, so adapter: collision would
  # drop 100% of it (CollisionAdapter::ingest()'s hard type check). Style:
  # existing GenericMarker color[]/alpha==0 fallback, already live (this
  # topic's own producer color is non-zero -- fallback is defensive-only).
  # Disable: this row (removable) + existing layer_markers param.
  - {topic: /navigation/debug_cruise_obstacle_marker,
     type: visualization_msgs/msg/MarkerArray, adapter: generic, role: neutral,
     timeout_sec: 2.0}
```
  Failing test: `find_row(*urban, "/navigation/debug_cruise_obstacle_marker")->adapter == "generic"`.
  Run — FAIL then PASS.
  **Done 2026-09-09.**
- [x] **Step 1: `/local_map_corners`.**
```yaml
  # New topic (VM-077, 2026-09-09): static 5-pt closed LINE_STRIP box outline
  # (local-map tile bounding rect), 38 Hz but constant 352-byte payload --
  # cheap. TRANSIENT_LOCAL on the wire per the bag's metadata.yaml, but a
  # 38 Hz republish means a late joiner gets a fresh one within ~26 ms
  # regardless -- transient_local: true is NOT set here (unlike
  # /sim/hd_map/markers' once-ever latch); add it if a live deployment shows
  # a cold-start gap.
  - {topic: /local_map_corners, type: visualization_msgs/msg/MarkerArray,
     adapter: generic, role: neutral, timeout_sec: 2.0}
```
  Failing test: `find_row(*urban, "/local_map_corners")->adapter == "generic"`.
  Run — FAIL then PASS.
  **Review gate:** neither row was added to `sim_profile.yaml`/`offroad_profile.yaml` — the measurement pass recorded against the urban validate stack only; no evidence these topics publish under the sim/offroad deployments. Stated here as an explicit scope boundary, not a silent omission (same "call it out, don't guess" rule Epic 2's own `offroad_profile.yaml` comments follow) — extend if/when a sim or offroad recording confirms them live there.
  **Done 2026-09-09.** Both rows added to `urban_profile.yaml` only (confirmed absent from `sim_profile.yaml`/`offroad_profile.yaml`). `test_profile.cpp`: `DebugCruiseObstacleMarkerRowUsesGenericAdapter`/`LocalMapCornersRowUsesGenericAdapter` added; `CoexistsWithTheRendererLibrarysOwnYamlCpp` row count 13→15 (net delta from Tasks 3+4+5 combined is 0, matching the plan's own precomputed arithmetic), comment updated. Node rebuilt — **18/18 PASS**, no regressions. Library unaffected (140/140).

---

## Task 6: SKIP/defer decisions, documented (no rendering code)

**Files:**
- Modify: `config/urban_profile.yaml` (one comment block per D4 row, placed where a future editor would naturally look for the topic — same "so a silently missing row is indistinguishable from an oversight" reasoning Epic 2's own omission comments use)

**Interfaces:** none.

- [x] **Step 0: write the six SKIP/defer comment blocks**, each citing: the topic name(s), the one-line verdict, and the specific measured evidence from D4's table above (exact-match point counts, distance-signature match fractions, message counts, byte rates — not a paraphrase). Placement:
  - `output_path_line`/`output_path_velocity_visualization`: adjacent to the existing `/behavior_path_planner/output_path_visualization` row (the ribbon they'd otherwise duplicate/extend).
  - `dynamic_objects_marker_1..6`: adjacent to the existing `/perception/dynamic_objects_list` row.
  - `/sim/hd_map/local_markers`: adjacent to `sim_profile.yaml`'s own `/sim/hd_map/markers` row (not urban — this topic is sim-side; place the comment in `sim_profile.yaml`, not `urban_profile.yaml`, correcting this task's Files list above).
  - `/navigation/bvp_crosswalk_attention_areas`, `/navigation/trajectory_following_status`: their own standalone comment blocks (no existing row to anchor to).
  - Global path/local path/OGM: no new comment needed — Epic 2's existing FIXTURE GAP comments on those rows already state the absence; this task adds nothing there (D4's "UNCHANGED" verdict, verified, not re-asserted with new prose).
  No test (documentation-only step, same convention as every other asset/comment-only step in this codebase's plans — e.g. Epic 3 Task 3 Step 2's font asset). Verified by reading the diff.
  **Review gate:** every D4 topic has a comment somewhere in the shipped profiles' own files (`git grep -n <topic-name-fragment> config/*.yaml` finds it), so the next person measuring this stack again starts from "here's what was already checked and why it's still out," not from zero.
  **Done 2026-09-09.** All six SKIP/defer comment blocks written, placed per the plan's own placement list: `output_path_line`/`output_path_velocity_visualization` adjacent to `/behavior_path_planner/output_path_visualization` (urban); `dynamic_objects_marker_1..6` adjacent to `/perception/dynamic_objects_list` (urban); `/sim/hd_map/local_markers` adjacent to `/sim/hd_map/markers` (`sim_profile.yaml`, per Step 0's own file-list correction); `/navigation/bvp_crosswalk_attention_areas` and `/navigation/trajectory_following_status` as standalone blocks (urban, end of file); global_path/local_path/OGM left untouched (Epic 2's FIXTURE GAP rows already state the absence). `git grep -n <fragment> config/*.yaml` confirmed a hit for all six. Node rebuilt (comment-only YAML, no code) — **18/18 PASS**, no regressions.

---

## Results

**Status: DONE, all 6 tasks closed, 2026-09-09.**

**Interfaces:** `kSceneVersion` 2 → 3. `TrajectoryCarpet` appended to `scene.h` (reuses `PointCloudPoint` verbatim); `SceneGraph` 200 → 216 bytes (`trajectory_carpets`@200, `trajectory_carpet_count`@208 — arithmetic verified against both toolchains' static_assert tables, exactly as the plan predicted). No other public header change.

**Suites (final, HEAD of this session):**
| Suite | Command | Result |
|---|---|---|
| Library | `ctest` in `cuda/src/libs/visual_renderer/build` | **141/141 PASS** (133 baseline + 1 `SceneBufferTrajectoryCarpet` deep-copy test + 6 `TrajectoryCarpet` tests + 1 `VertexZIsLifted...` regression test found during live verification). Zero sanctioned reds. `check_pod_header` green throughout. |
| Node | `ctest` in `cuda/build/src/ros_apps/micropilot_visualization_node`, force-relinked each time the library archive changed | **18/18 PASS** (17 baseline + 1 new `test_trajectory_carpet_adapter` binary). Zero sanctioned reds. |
| WS bridge | `pytest tools/test_vcam_ws_bridge.py -p no:anyio -p no:cacheprovider` | **53/53 PASS** (unaffected — `LAYER_NAMES` gained `"trajectory_carpet"`, no other bridge logic touched). |

**Row counts (urban_profile.yaml, `CoexistsWithTheRendererLibrarysOwnYamlCpp`):** 15 (baseline) → 16 (Task 3, +1 carpet) → 13 (Task 4, −3 net: 5 dead collision rows → 2 retargeted) → 15 (Task 5, +2 generic rows) — net delta across Tasks 3+4+5 is **0**, exactly matching the plan's own precomputed arithmetic. sim/offroad profiles: −3 each (Task 4 only; no compensating rows).

**Files changed:** `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h`; `cuda/src/libs/visual_renderer/tests/test_scene_buffer.cpp`; `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_scene_layout.cpp`; new `cuda/src/libs/visual_renderer/src/trajectory_carpet.{hpp,cpp}`, `trajectory_carpet_test_hooks.hpp`, `assets/materials/trajectory_carpet.mat`, `tests/test_trajectory_carpet.cpp`; `cuda/src/libs/visual_renderer/src/{renderer_internal.hpp,scene_buffer.hpp,scene_buffer.cpp,renderer.cpp}`; new `cuda/src/ros_apps/.../include/.../adapters/trajectory_carpet.hpp`, `src/adapters/trajectory_carpet.cpp`, `test/test_trajectory_carpet_adapter.cpp`; `.../src/profile.cpp`; `.../include/.../scene_assembly.hpp`, `src/scene_assembly.cpp`; `.../include/.../visualization_node.hpp`, `src/visualization_node.cpp`; `.../config/default_params.yaml`, `config/{urban,sim,offroad}_profile.yaml`; `.../CMakeLists.txt`; `.../test/{test_profile.cpp,test_scene_assembly.cpp,test_collision_adapter.cpp}`; `tools/{vcam_ws_bridge.py,vcam_gui.py,validate_visual_mode.sh}`; `docs/visual_mode_project_backlog.md`.

**Unplanned fixes found during implementation (both root-caused and closed, not symptom-patched):**
1. **Task 4 remap broke `test_collision_adapter.cpp`.** Five of its unit tests built their `ProfileRow` via `urban_row("/navigation_urban_collision_checker_testing_node/...")`, a topic the remap made dormant (no live row to fetch). Fixed at the root: a `MakeRow(role)` helper (hand-built `ProfileRow`, same shape `test_point_cloud_adapter.cpp` already uses) replaces every such call site — `CollisionAdapter` reads only `row.role`, never `row.topic`, so this preserves each test's original intent exactly.
2. **The carpet was invisible on the real bag** (found only by live-rig verification, not by any unit test — see below): the adapter's flattened z=0.0 was left bare at the renderer, z-fighting the opaque ground. Fixed with a renderer-side z-lift (`kTrajectoryCarpetZLiftM=0.055f`), placed above every path ribbon but strictly below alert_polygons.cpp's `kAlertZLiftM` (0.06) so alerts stay topmost — matching this codebase's existing z-stack convention (`kLaneZLiftM`, OGM lift, `kRibbonZLiftByRoleM`, alert's 0.06) — see Task 2's own "Done" note for the full root-cause writeup and the new regression test (`EXPECT_LT(z, 0.06f)`) that catches it (verified to fail without the fix).

**Sanctioned reds:** none. This plan added no golden/SSIM tests (D1's own scope: `TrajectoryCarpet` is proven by unit tests + the live-rig visual verification below, not a committed golden — no existing golden fixture covers a TRIANGLE_LIST category to extend).

**Live-rig verification (2026-09-09):** ran the validate script's own rig pattern by hand — `visualization_node` (urban profile, mode 3) + `tools/tf_flatten_fixture.py` + `ros2 bag play stack_v2_full_sensors_2026-09-09 --loop --clock --qos-profile-overrides-path qos_full.yaml --remap /tf:=/tf_raw < /dev/null`, all on an isolated `ROS_DOMAIN_ID=93`, torn down afterward with the same bracket-anchored `pkill -f "[l]ib/micropilot_visualization_node/visualization_node"` etc. pattern `validate_visual_mode.sh` uses. Confirmed all five new/retargeted rows subscribed (node log: `trajectory_carpet: 1 row(s) subscribed`, `collision: 2 row(s) subscribed`, `generic: 2 row(s) subscribed`), `/rendering/image` at 30.3 Hz, and `output_trajectory_carpet` diagnostics showing `msgs: 1307`, `dropped_malformed: 0`. First screenshot pass (before the z-lift fix) showed NO visible carpet despite healthy ingest — this is what surfaced the z-fighting bug above; after the fix, the carpet renders exactly as measured (a per-vertex red→yellow→teal-adjacent gradient ribbon a few cm above the road, distinct from and blending into the longer-range BEHAVIOR path ribbon beyond its extent), and toggling `layer_trajectory_carpet` live via `ros2 param set` cleanly shows/hides it with nothing else changing. Evidence PNGs (1280×720, `/rendering/image`), all under `/tmp/mpviz_vm077_verify/`:
- `vm077_carpet_off.png` / `vm077_paths_off_carpet_on.png` — BEFORE the z-lift fix: carpet layer nominally on, nothing renders (the bug, caught live).
- `vm077_fixed_all_on.png` — AFTER the fix: the velocity-gradient carpet clearly visible under/ahead of the ego, plus a live collision-alert callout ("1.6 m") confirming the Task 4 remap renders too.
- `vm077_fixed_carpet_off.png` — AFTER the fix, `layer_trajectory_carpet:=false`: carpet cleanly disappears, everything else (path ribbon, ego, map) unchanged — the disable knob proven live, not just in a unit test.

---

## 2026-09-10 — carpet-as-ribbon redirect (user directive)

**Verbatim user directive:** "I just noticed a major flickering for the local path ribbon after adding the velocity profile rendering this way, I think i still prefer to treat it as ribbon that can be stacked on top of local ribbon with margin (configurable like other ribbons)".

**Flicker measurement pass verdict (H2, DISPROVEN — corrected 2026-09-10, see "Live verification, CORRECTED" below; text below is the ORIGINAL, since-invalidated verdict, kept for history, not re-typed as fact):** a dedicated measurement rig (`visualization_node` urban/mode 3 + `tf_flatten_fixture.py` + `ros2 bag play stack_v2_full_sensors_2026-09-09 --loop --clock`, `ROS_DOMAIN_ID=93`) captured 24 consecutive `/rendering/image` frames with the carpet on vs off and cross-logged `output_trajectory_carpet`'s own arrival timestamps. Carpet-ON diffs were spiky (mean |Δpixel| in an ego-centered crop up to 9.90, frame-intervals with a carpet message averaging 3.55 vs 1.27 without) while carpet-OFF was flat (~0.61, no spikes) — a genuinely measured carpet-message-correlated frame-diff pulse, not general render noise. Hypothesized mechanism at the code level (NOT itself confirmed — see correction): `trajectory_carpet_signature()` hashed per-vertex color, and the producer's baked velocity color drifts on almost every message even when position is byte-identical for several consecutive messages — so `update_trajectory_carpets()` fully destroyed+rebuilt the whole mesh (up to 906 pts) on nearly every ~8Hz message. **This mechanism claim is FACTUALLY WRONG, independent of the H2 disproof below:** `git show HEAD:cuda/src/libs/visual_renderer/src/trajectory_carpet.cpp` shows `trajectory_carpet_signature()`/`hash_point()` folding rgba for `pts[0]`/`pts[n-1]` ONLY (first/last station) — the same first/last shape `ribbon_signature()` always used, never a full per-vertex hash. Full report reproduced in this plan's prior measurement-report intake; not re-typed here.

**Redirect implemented, exactly per the design brief:**
- The velocity profile now renders as a **lane-fill ribbon** in the same z-stack as the three `PathRole` ribbons — constant half-width from a new soft-defaulted theme token **`ribbon.margin_velocity_m`** (default 1.05, deliberately between LOCAL's 0.8 and BEHAVIOR's 1.3 — narrower than LOCAL so LOCAL's own rim still shows, wider than BEHAVIOR so the hero's rim shows through this one), clamped to the existing `kRibbonMinHalfWidthM` floor (promoted from `ribbon.cpp`'s anonymous namespace to `renderer_internal.hpp` so both files share the identical constant).
- Z-slot **0.0475**, strictly between LOCAL (0.045) and BEHAVIOR (0.05) — "stacked on top of local ribbon" while the hero stays topmost. `ribbon.cpp`'s `kRibbonZLiftByRoleM` stagger-doc comment extended to name this slot even though the velocity ribbon isn't a `PathRole` (it's still the same category's array, not indexed by role).
- **scene.h UNCHANGED, no `kSceneVersion` bump.** `TrajectoryCarpet::points` (`PointCloudPoint*` — world position + packed rgba8) already fit a "centerline + per-station color" encoding exactly; only its INTERPRETATION changed (adapter: raw wire vertex → derived centerline station). Verified and stated per the design brief's own "if they already fit, say so and don't touch scene.h" instruction.
- **Node adapter** (`adapters/trajectory_carpet.cpp`) now derives one centerline station per dual-rail quad from the measurement's own pairing algorithm (`station_0 = midpoint(A_0,B_0)`, `station_{k+1} = midpoint(D_k,C_k)`, color from `A_0`/`D_k` respectively) instead of storing the raw 6-per-quad wire points — `points.size() % 6 == 0` replaces the old `% 3` malformed check (measured: always a multiple of 6).
- **Library** (`trajectory_carpet.cpp`) reuses `ribbon.cpp`'s `extrude_polyline()`/`extrude_polyline_indices()`/`polyline_chunks()` machinery verbatim on the derived centerline (no second extruder) — the old flat-triangle-list build path, its bespoke `triangle_chunks()`/`kMaxTrianglePointsPerMesh` chunker, `kTrajectoryCarpetZLiftM` (0.055), and `kTrajectoryCarpetBaseAlpha` (0.7) are all **deleted**, not stranded.
- **Ego-clip**: promoted `closest_arc_station()`/`RibbonClip`/`compute_ribbon_clip()`/`clip_ribbon_forward()` out of `ribbon.cpp`'s anonymous namespace into `polyline.hpp`/`polyline.cpp` (`compute_polyline_clip()`/`clip_polyline_forward()`, Filament-free, shared) so the velocity ribbon uses the IDENTICAL mechanism, not a second copy. A colour-carrying `clip_carpet_forward()` (trajectory_carpet.cpp-local) duplicates the short interpolation walk to carry each station's rgba through the cut (ponytail: promote if a third colour-carrying clip caller ever appears).
- **Content signature** mirrors `ribbon_signature()`'s shape exactly (point count, first/last point, half-width, ego-clip state) with **no color term** — intended as the direct H2 fix (see the "Live verification, CORRECTED" section below: H2 was disproven as the visible-flicker cause, but this signature property is independently true and unit-tested regardless). Pinned by `TrajectoryCarpet.SameStationPositionsWithDriftingColorAloneCausesNoRebuild`: 20 republishes with byte-identical positions and a different color every time cause **zero** rebuilds (verified against a live rebuild-count hook), and the displayed color freezes at the last-built value until the next position-changing rebuild — an explicit, accepted tradeoff (no separate "recolor without rebuild" fast path exists anywhere in this library; inventing one is new-scope machinery flagged back, not silently built).
- **Opacity**: fresh renders fully OPAQUE (`alpha` uniform driven by `staleness_alpha()` alone) — the old 0.7 measured-producer-alpha parity is superseded by the user directive, not reused.
- Knobs unchanged: `layer_trajectory_carpet` param, `TrajectoryCarpet` scene.h struct name, `trajectory_carpet` adapter/profile name all KEPT (a rename would touch `profile.cpp`, `scene_assembly.{hpp,cpp}`, `visualization_node.{hpp,cpp}`, `CMakeLists.txt`, `tools/vcam_ws_bridge.py`/`vcam_gui.py`, three profile YAMLs, and every test file above — not trivial — so this plan notes the redirect in these files' own comments instead of renaming).

**Files changed:** `cuda/src/libs/visual_renderer/src/{polyline.hpp,polyline.cpp}` (ego-clip promoted, shared); `{renderer_internal.hpp}` (`kRibbonMinHalfWidthM` promoted, `TrajectoryCarpetSlot::halfWidthM`/`trajectoryCarpetRebuildCount` added); `ribbon.cpp` (calls the shared clip helpers, stagger-doc extended); `{theme.hpp,theme.cpp,theme_transition.cpp}` (`ribbon.margin_velocity_m` token: soft default, parse, blend); `trajectory_carpet.{hpp,cpp}` (full redirect — centerline extrusion, new signature, opaque alpha); `trajectory_carpet_test_hooks.hpp` (+`trajectory_carpet_half_width_m`/`trajectory_carpet_rebuild_count`); `tests/test_trajectory_carpet.cpp` (fully rewritten — ribbon geometry, margin/clamp, chunking, staleness, ego-clip parity tests, the H2 regression pin); `tests/{test_theme.cpp,test_theme_transition.cpp}` (+margin_velocity_m parse/fallback/lerp/sentinel-guard tests); new fixtures `tests/fixtures/themes/{ribbon_margin_velocity,ribbon_margin_velocity_extreme}.yaml`. Node: `adapters/trajectory_carpet.{hpp,cpp}` (sextet-pairing station extraction); `test/test_trajectory_carpet_adapter.cpp` (fully rewritten — pairing, color, malformed/%6, NaN-drop, replace, delete). `tools/validate_visual_mode.sh` (dated milestone entry); `docs/visual_mode_project_backlog.md` (VM-077 row note).

**Suites (final):**
| Suite | Result |
|---|---|
| Library | `ctest` in `cuda/src/libs/visual_renderer/build` — **153/153 PASS** (141 baseline + 8 net new `TrajectoryCarpet` tests + 4 new `ThemePalette`/margin_velocity_m tests). Zero sanctioned reds. |
| Node | `ctest` in `cuda/build/src/ros_apps/micropilot_visualization_node`, force-relinked — **18/18 PASS**, no regressions. |
| WS bridge | `pytest tools/test_vcam_ws_bridge.py -p no:anyio -p no:cacheprovider` — **53/53 PASS** (unaffected). |

**Live verification (2026-09-10, CORRECTED same-day — two review findings fixed: the N=24 round's raw data was NOT actually lost as first claimed here, and an after-fix frame-diff measurement now exists):** ran the rig against `stack_v2_full_sensors_2026-09-09` with the fixed build installed.
- **The specific H2 mechanism is fixed and proven**, at the unit level: identical station positions with drifting color cause zero rebuilds (new test above), which is exactly what the directive asked for.
- **Pre-fix N=24 round — raw data survived and re-derives exactly (correction of this section's own prior, false claim):** the round's complete per-frame data is committed at `docs/evidence/vm077-flicker-2026-09-10/carpet_{on,off}_summary.json` (plus both stddev heatmaps, same directory) — copied off `/tmp` before it could be lost the way the N=300 round already was. Every ratio below re-derives from those two files, checked by direct re-computation: carpet-ON max frame_diff **9.898** ("up to 9.90"), mean of the 7 intervals with a carpet message **3.551** ("3.55"), mean of the other 16 **1.267** ("1.27"), carpet-OFF mean **0.619** ("~0.61"). What does NOT hold up: `flicker_capture.py` (the script that produced these) is confirmed gone, and — more importantly — the two conditions were two SEPARATE `ros2 bag play` launches, not the same window: `carpet_msg_rate_hz` measured **9.28 Hz ON vs 5.35 Hz OFF**, i.e. different points in the looped bag. Combined with N=24 being small, this round's ON-vs-OFF contrast is suggestive, not conclusive — a real limitation, just a different one than "no data survived."
- **After-fix measurement, same-window this time (new this pass, closes the missing-Criterion-2 finding):** `tools/flicker_capture.py` + `tools/flicker_measure.sh` are committed (the capture script the review found missing now exists for good) and fix the N=24 round's own confound: ONE continuous node + bag-play session, `layer_trajectory_carpet` toggled live between conditions (`apply_layer_gates()` only clears the per-tick scene-assembly vector before the renderer — the adapter's ingest/rebuild bookkeeping runs identically either way, so this toggle is a pure render-visibility gate on the same message stream, not a different code path). Result: `carpet_msg_rate_hz` **9.66 Hz ON vs 10.67 Hz OFF** — closely matched, confirming the same-window design worked (vs. the old round's 9.28-vs-5.35 mismatch). Raw data committed at `docs/evidence/vm077-flicker-2026-09-10/carpet_{on,off}_after_fix_summary.json` (+ heatmaps); N=24/condition again, matching the pre-fix round for direct comparison.
  - Max frame_diff (ON): **4.392**, down from 9.898 pre-fix (−56%).
  - With-msg vs without-msg contrast (the H2-specific signal): pre-fix **3.551 vs 1.267** (2.80×); after-fix **2.077 vs 1.752** (1.19×) — the carpet-message-correlated contrast shrank most of the way toward parity.
  - **Verdict: the pulse shrank, it did not cleanly collapse to a flat floor.** After-fix OFF mean is **1.395** (max 4.014) — well above the pre-fix OFF floor of 0.619 — and the after-fix ON without-msg mean (1.752) still sits above its own OFF floor. The after-fix OFF trace itself shows a periodic non-carpet pulse (diffs ~2.2–4.0 recurring roughly every third interval, with the carpet layer fully gated off) — some other scene element is contributing noise this rig doesn't isolate; flagged as an open confound, not chased further this pass (new scope). **Net: this pass narrows the open question with real before/after data (message-correlated contrast down ~58%) but does not close it** — if the user still sees flicker on the real rig, that residual non-carpet noise floor is the next thing to characterize, not the carpet mechanism (already unit-pinned above).
- Independently-verifiable context (not dependent on the frame-diff rig above, so it stands regardless of the flicker verdict): offline replay of the raw bag shows the producer's own station positions drift continuously (~0.03–0.16 m/message), and the carpet's own ~8Hz cadence next to LOCAL's 50Hz was always going to visibly step — the measurement report's own flagged, explicitly-unresolved constraint #3. Extending the signature with a position-tolerance band, or resampling/smoothing the centerline, would address this, but is new scope beyond "mirror `ribbon_signature()`'s shape" and is flagged back for a product decision, not silently invented here.
- Evidence on disk (`/tmp/carpet_ribbon_before_after.png`, `/tmp/carpet_ribbon_full.png`) is from the original pre-redirect-vs-redirect comparison and does not settle the flicker question either way (its own AFTER row was described as visually busier than BEFORE) — read it only for what it does show correctly: the velocity ribbon renders as a distinct band nested between the wider LOCAL lane-fill and the narrower BEHAVIOR hero, confirming the z-stack/margin geometry itself is correct.

**Post-verification code-review fix (2 blocking findings), re-verified (this pass):** no functional/algorithm change — the fix is documentation-honesty (this section's own false "no reproducible artifact" claim, corrected above with the N=24 round's raw data recovered from `/tmp` and committed) plus new measurement tooling (`tools/flicker_capture.py`, `tools/flicker_measure.sh` — committed so the after-fix pass above, and any future one, leaves its own script and raw data on disk, not just numbers in prose). New committed evidence: `docs/evidence/vm077-flicker-2026-09-10/carpet_{on,off}_summary.json` + heatmaps (pre-fix N=24, recovered), `carpet_{on,off}_after_fix_summary.json` + heatmaps (new after-fix N=24, same-window). Library rebuilt (`cmake --build .` in `cuda/src/libs/visual_renderer/build`, no source change, every target already-built) — **153/153 PASS**, unchanged. Node force-relinked (`colcon_build.sh micropilot_visualization_node --cmake-force-configure`, no node source change either) — **18/18 PASS**, unchanged. WS bridge — **53/53 PASS**, unchanged. Zero sanctioned reds throughout.

**Sanctioned reds:** none (no golden/SSIM tests touched or added this pass).

**Sub-agent handoff note:** this plan was executed end-to-end by one agent in one pass (no parallel sub-agents dispatched) — Tasks 4/5's "may run in parallel once Task 3 lands" option was not exercised; all six tasks ran sequentially, in the plan's own numbered order.

## LOCAL-ribbon flicker root cause + fix (2026-09-10, user: "still see shimmer live, local path still flickering")

**Measured root cause (measure-only pass, evidence /tmp/mpviz_flicker2 -> committed summaries in docs/evidence/vm077-flicker-2026-09-10/):** (1) PRIMARY — ego-clip quantization at `kPolylineClipQuantizeM=0.5`: 310 boundary crossings over 98.75 s of the recorded drive (median 0.151 s apart), each a full `destroy_slot_meshes`+rebuild with the ribbon front edge snapping 0.5 m — 3-7 Hz, localized to the ego-front crop in the 4-condition live bisection (all_on front-crop mean 3.08 vs both_off 0.86, +53% super-additive over the parts). (2) COMPOUNDING — the path ribbons and the velocity ribbon compute independent clip stations, so their snap schedules beat against each other 2.5-5 mm apart in z. (3) LATENT — path.cpp stamped `last_update_sec = sim_time_sec` raw (the VM-034 anchored-stamp fix never covered path rows); recorded `/local_vel_path` max gap 0.461 s vs the 0.5 s fade start = zero margin, not currently firing. (4) RULED OUT — depth-buffer z-fighting: at near=0.1/far=500/24-bit, precision at 30-100 m is 8-9 orders finer than the 2.5-5 mm staggers.

**Fix:** `kPolylineClipQuantizeM` 0.5 -> 0.05 (the constant only absorbs parked-ego jitter; 5 cm keeps that and caps the visible snap at 5 cm; rebuilds stay bounded at one per rendered frame) + path.cpp anchored stamping (`kPathFadeWindowSec=1.0`, the hd_map pattern, regression-tested).

**After-fix rerun, recorded honestly:** the same bisect rig re-ran against the fixed build but its capture window caught the ego essentially stationary (`n_pairs_with_ego_crossing: 0` in every condition) — a window insensitive to the snap mechanism, so its numbers (all conditions near-baseline medians 0.63-0.94 with unmatched-window spikes) neither confirm nor refute the fix; committed as after_quantize_fix_bisect_summary.json without a victory claim. The mechanism reduction (10x smaller snap amplitude) is code-level provable and test-pinned; the closing gate is the user's live eye.

## Rebuild/clip decoupling — the actual dropout mechanism (2026-09-10, user escalation: "NO THE GREEN LOCAL PATH DISAPPEAR AND APEAR randomly causing the flicker not the road?!")

**Why the 0.05 quantize fix (previous section) didn't close it:** it shrank the snap amplitude but left the mechanism unchanged — every quantized clip-station tick still did a full `destroy_slot_meshes()` + rebuild, synchronously, on every ribbon slot (BEHAVIOR/GLOBAL/LOCAL via `ribbon.cpp`, the velocity ribbon via `trajectory_carpet.cpp`). At 0.05 that's the station changing on ~92% of frames while driving (measured in `tests/test_ribbon_dropout.cpp`: 55/60 rebuilds at 0.03-0.08 m/frame). REPRODUCE pass (`test_ribbon_dropout.cpp`, committed as the permanent regression, 2 tests) could not turn that rebuild rate alone into a dropout under the library's own single-threaded `set_scene()`/`render_frame()` contract — every frame still showed `ribbonMesh=1, carpetMesh=1`, non-zero vertices, all 3 pixel probes corridor-colored. `scene.h`'s own documented "single-thread-only" contract (`SceneBuffer::active()` hands back a bare reference into shared double-buffer storage) remains the most likely explanation for a torn cross-thread read producing a whole-frame, message-cadence dropout, but is node-path instrumentation this pass didn't need to reach — REMOVING the destroy/rebuild mechanism entirely closes the reported symptom regardless of which thread-timing hazard the live rig would have found.

**Fix (mechanism, not a bigger constant):** the ego-clip no longer feeds any ribbon's content signature (`ribbon_signature()` / `trajectory_carpet_signature()`, `ribbon.cpp`/`trajectory_carpet.cpp` — decision: the clip-station term is REMOVED from the signature entirely, not just re-quantized). The full, unclipped mesh now builds only on real content change (role/points/half-width, ~8 Hz message rate). The clip is applied EVERY `render_frame()` as a degenerate-vertex position collapse re-uploaded into the SAME `VertexBuffer` (`polyline.hpp`'s `collapse_clipped_positions()`, plumbed through `update_mesh_positions()`/`update_carpet_vertex_positions()`'s `VertexBuffer::setBufferAt`) — no `destroy_slot_meshes()`, no new entity, no scene membership change, so no frame can ever observe an absent slot. Applied identically to all four ribbon-shaped categories (BEHAVIOR, GLOBAL, LOCAL via `ribbon.cpp`; velocity via `trajectory_carpet.cpp`) — one mechanism, not four bespoke ones. `kPolylineClipQuantizeM` stays 0.05 (unchanged value) but its job changes: it now only gates the per-frame re-upload (skip when the ego hasn't crossed a new quantized station since last frame), never a rebuild — a parked ego still causes zero GPU uploads, same property the old comment claimed for the rebuild path, now actually load-bearing for something a torn read can't touch. Stacking/margins/z-order untouched (respine + margins + z as shipped at `33b2747`) — only the rebuild/clip mechanism changed.

**Measured effect:** `tests/test_ribbon_dropout.cpp`'s driving run — rebuild count dropped from 55/60 frames (pre-fix, clip-station-driven, per this section's own earlier measurement) to **8/60** (post-fix, matching the test's injected message-churn cadence — every 8th frame — exactly, both driving and near-parked). Zero dropout in both runs, as before and after. All 155 library tests green (153 -> 155: the two new dropout-regression tests), including every ribbon/carpet golden, pixel-identical (`RibbonGolden.ThreeRoles_DarkAdas` and the rest unchanged — the collapse renders the same picture the old truncate-then-rebuild did at the point each frame is sampled). Node force-relinked (`colcon_build.sh micropilot_visualization_node --cmake-force-configure`, no node source change) — 19/19 node tests green, unaffected (adapter-level, not rendering-level).

**LIVE evidence — HONEST SPLIT VERDICT, STILL OPEN, not a clean close** (ROS_DOMAIN_ID=93, urban profile, `stack_v2_full_sensors_2026-09-09` bag, 24-frame burst ~45s in, crop rows 260-560/cols 440-840 — `docs/evidence/vm077-flicker-2026-09-10/rebuild_clip_fix_burst_after.*`): the AMBER band (`PathRole::LOCAL`, `palette.ribbon_local` — what the internal name "LOCAL" literally refers to) is rock solid across all 24 live frames, r>b corridor-presence metric never drops (0.1575-0.1625 throughout, per the committed `rebuild_clip_fix_burst_after_summary.json`) — exactly the property this fix targets, closed. BUT the same capture shows the TOPMOST teal/green hero ribbon (`PathRole::BEHAVIOR`, `palette.ribbon_core` `[0.12,0.55,0.42]` — plausibly what "green" in the user's own quote actually means, since it's the only genuinely green-ish element and the most visually prominent, directly on the ego) is COMPLETELY ABSENT (teal_fraction exactly 0.0, not merely faint), not "near-invisible," on 15 of the 24 frames (0,2,3,4,6,7,9,15,16,17,18,19,20,22,23), present at ~0.0274 on the other 9 — a STRUCTURAL per-frame presence failure, not a brightness dip. `full_0.png`/`full_1.png` confirm it visually: frame 0 has no teal band at all inside the amber corridor, frame 1 shows it clearly, on consecutive frames.
>
> **Correction (this pass):** the numbers immediately above are the post-`depthCulling`-revert re-capture (the file actually committed at `rebuild_clip_fix_burst_after_summary.json` today); an earlier version of this paragraph described a DIFFERENT, since-overwritten capture (~0.0022-0.003 teal on 8 frames vs ~0.029-0.030 on the rest, a ~10-13x drop, corridor 0.16-0.2025) — that capture's raw evidence files were replaced in place by this re-capture and are not recoverable, so the two runs cannot be directly reconciled; what's certain is the CURRENT, committed evidence shows the symptom is worse (15 frames totally absent, not 8 frames faintly present) than the earlier prose implied. Also corrected: `tools/flicker_burst_capture.py`'s own `any_teal_dropout` flag read this run as `false` — its median-relative check silently exempted a run where the ABSENT frames are the majority (median collapses to 0.0 right along with them); fixed to key off the run's own max instead (a frame under half of max is a dropout, gated only on max clearing 0), and the committed JSON's flag corrected to `true` to match.
>
> Mechanically this is NOT the destroy-rebuild path (identical fixed code handles BEHAVIOR too, and BEHAVIOR never swaps mesh instances — it fades via its own material alpha only, `ribbon.cpp`) — most likely explanation, still NOT proven (this pass looked for, and did not attempt, the node-path instrumentation that would prove it): BEHAVIOR's row (`urban_profile.yaml`, `/behavior_path_planner/output_path_visualization`, `timeout_sec=2.0`) hits the renderer's OWN staleness fade (`kStaleFadeStartSec=0.5`/`kStaleFadeTimeoutSec=1.0`, `renderer_internal.hpp`) when real message-arrival gaps under bag-replay timing exceed that margin — the same class of "zero margin against the fade threshold" this plan's own earlier LOCAL-ribbon section already flagged (item 3 there, for `/local_vel_path`), NOT the mechanism this pass's fix requirements targeted. **Left open, not silently declared fixed:** the next pass needs to instrument the BEHAVIOR slot's staleness alpha and `/behavior_path_planner/output_path_visualization`'s message-arrival timestamps during a live burst to show (or disprove) that alpha is being driven to 0 by a real message gap, and that the on/off pattern is a fade ramp rather than the observed binary 0.0274->0.0 toggle — a binary toggle, if that's what instrumentation finds, would itself rule the staleness-fade theory back out. Until that instrumentation exists, the rebuild mechanism fix above (already closed, shared by all four ribbon categories) stands on its own merits, but the user-visible BEHAVIOR-ribbon dropout is NOT closed by it.

**Files changed:** `polyline.hpp`/`.cpp` (`clean_polyline_stations()`, `collapse_clipped_positions()`, `clip_polyline_forward()` retired), `renderer_internal.hpp`/`renderer.cpp` (`update_mesh_positions()`, per-slot `baseStripPositions`/`pointStations`/applied-clip-state fields), `ribbon.cpp` (`apply_ribbon_clip()`, signature no longer carries clip state), `trajectory_carpet.cpp` (`apply_carpet_clip()`, `update_carpet_vertex_positions()`, same signature change, `clip_carpet_forward()` retired), `trajectory_carpet_test_hooks.hpp` (+`trajectory_carpet_slot_first_point()`), `tests/test_ribbon.cpp`/`test_trajectory_carpet.cpp` (clip assertions updated: vertex count stays fixed, first-point position proves the collapse), `tests/test_ribbon_dropout.cpp` (new, permanent regression, 2 tests). **Sanctioned reds:** none — no golden changed pixel content.

**Post-review fix, ribbon_emissive.mat (code-review pass, this session):** the live burst evidence above (`rebuild_clip_fix_burst_after_full_1.png`/`.gif`) shows the BEHAVIOR hero ribbon painting straight through the ego's roof/hood — the segment between the ego's closest-approach clip station and its front bumper, which a normal depth test would hide. A prior uncommitted edit to `ribbon_emissive.mat` had set `depthCulling : false` to chase the staleness-fade drop flagged (unproven) above, on the theory the BEHAVIOR ribbon was losing a z-fight against LOCAL at the few-mm `kRibbonZLiftByRoleM` gap. That theory does not hold up: `RibbonGolden.ThreeRoles_DarkAdas` (the only golden stacking all three ribbons) is pixel-identical with the depth test ON, so no z-fight is being lost there, and no golden renders an ego body under a BEHAVIOR ribbon at all — the ego-occlusion regression this change caused was untested in both directions. Reverted `depthCulling : false`; kept `depthWrite : false` (harmless, matches Filament's blended default). **Files changed (this fix):** `cuda/src/libs/visual_renderer/assets/materials/ribbon_emissive.mat` (`depthCulling : false` removed, comment corrected). **Sanctioned reds:** none. **Golden coverage:** honestly none — no existing golden renders an ego body together with a BEHAVIOR ribbon, so this occlusion regression (and its fix) is proven only by the live burst evidence, not by CI; a golden covering ego+BEHAVIOR-ribbon occlusion is still open scope, not silently claimed as covered. Live burst evidence regenerated: `docs/evidence/vm077-flicker-2026-09-10/rebuild_clip_fix_burst_after*` re-captured against the fixed build.
