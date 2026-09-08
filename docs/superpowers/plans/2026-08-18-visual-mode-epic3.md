# Visual Mode — Epic 3 Implementation Plan (HUD, polish, controls)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18):** run this epic as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Status: NOT STARTED. This document is a proposal for user approval — no code has been written against it.**

## Status ledger

| Task | Backlog | Status | Notes |
|---|---|---|---|
| 1 `MapElement.kind`/`lane_id`/`last_update_sec` + road-surface fill + dash-kind flip (boundary dashed, centerline solid) + crosswalk-hatch fix + `kSceneVersion` | VM-036 | **Done 2026-09-07** | All 8 steps complete, both suites green (library: 97/99 unrelated-perf-flake aside, 2 sanctioned MapGolden reds; node: 12/12). STATED DEVIATION in Step 6: `palette.ground` left unchanged in both themes (protects unsanctioned goldens); debt items (b) object-tint retint and (c) ribbon_local hue move skipped for the same reason. Also carries the minimal slice of VM-037(e) this epic's first interface change requires — see "VM-037/VM-044 scope" below. Review-fix 2026-09-07: the `git grep -n dashed -- cuda/src/ros_apps` gate (line ~803) had 4 residual prose hits (comments describing the retired flip) plus one genuinely stale comment; reworded all four to drop the literal word (gate re-verified empty) rather than narrowing the gate bullet itself — the field-level flip (no `dashed` field/param anywhere) was already correct, only the prose tripped the word-level check |
| 2 Staleness fades (map layer) + diagnostics topic + `render_ms` + ego-invalid map-pop cosmetic fix | VM-034 | Not started | Depends on Task 1's `MapElement.last_update_sec` |
| 3 HUD overlay (node-side CPU composite) | VM-030 | Not started | Depends on Task 2's `render_ms` only insofar as both touch `timer_callback()`; otherwise independent |
| 4 Alert callouts (`project_to_screen()` + node-side leader lines) | VM-031 | Not started | Depends on Task 3's compositor plumbing |
| 5 Layer visibility + quality-preset plumbing | VM-032 | Not started | Depends on Task 2's `render_ms` for the AC's frame-cost measurement |
| 6 `PointCloudLayer` | VM-035 | Not started | Independent of Tasks 1-5; ordered last per the master table |
| 7 Epic-0 debt: mux hardening + build hygiene (mux QoS, legacy-topic re-publish, `initial_mode`, `vcam_state[8]`, `check_pod_header.sh` in colcon, `FILAMENT_VERSION` single-source, GL_RENDERER logging, bluegl link-probe) | VM-037 | Not started | Runs LAST — after VM-035 — unless the user reorders; items are independent mux/build-hygiene fixes with no coupling to Tasks 1-6; item (e) already shipped by Task 1 |

**Parent plan:** `docs/superpowers/plans/2026-08-18-visual-mode.md` (Global Constraints bind this doc too — re-read them before starting). Epic 3's master-table row is at lines ~300-312; the 2026-09-07 review re-sequenced it so VM-036/VM-034 (fix what autonomy sees today) run before VM-030/031/032 (HUD/controls polish), with VM-035 last.
**ADR:** `docs/adr/0004-scene-interface-versioning.md` — additive-only, versioned interfaces. Every `scene.h` change in this epic appends; nothing is renamed, reordered or removed; `kSceneVersion` is introduced in Task 1 and bumped again in Task 6.
**Spec:** `docs/superpowers/specs/2026-08-18-visual-mode-design.md` §4.1 (SceneGraph categories, `Hud`), §4.4 (AssetLibrary — HUD text amendment), §5 (adapters, staleness), §6 (vcam/WS), §9 (error handling), §10 (testing).
**Backlog:** `docs/superpowers/specs/2026-08-18-visual-mode-backlog.md`, Epic 3 section (VM-030…VM-036) plus VM-037/VM-044 (see scope decision below).
**Prerequisite:** **Epic 2 is CLOSED at `fd72331`** (gate PASSED 2026-08-20 at `37d41fe`). Read `docs/superpowers/plans/2026-08-18-visual-mode-epic2.md` before starting, in particular its "Interfaces added this epic", "Staleness", and "…and the material that can actually do it" sections — Task 1 of this epic extends the exact same staging/material machinery, not a parallel one. `FILAMENT_VERSION = 1.56.5`.

**Goal:** Close Epic 2's two recorded deviations (HD-map layer pops instead of fading; lane geometry is visually undifferentiated), add the diagnostics/`render_ms` instrumentation every later epic depends on, ship the HUD and alert-callout overlays as an accepted node-side CPU composite (P2), wire layer-visibility and quality-preset controls end to end, add a `PointCloud` category to `SceneGraph`, and (user decision 2026-09-07) close out VM-037's Epic-0 mux/build-hygiene debt in full. Seven tasks, in the master table's fixed order: **VM-036 → VM-034 → VM-030 → VM-031 → VM-032 → VM-035 → VM-037.**

---

## ACCEPTED (user, 2026-09-07): `get_hud_colors()` ships as specified — authorized amendment to P2

**Task 3 adds `get_hud_colors()` to `scene.h`.** P2 (master plan changelog line 309-310; backlog VM-030, `visual-mode-backlog.md:85`) accepted the node-side HUD explicitly on the terms "no font pipeline in the clang/libc++ archive, **no new public entry point**." This plan needs one anyway: `Theme::hud.{text_color,accent_color,scale}` lives only in `renderer_internal.hpp`'s `active_theme` (verified — the node has no theme parser of its own), which is also the only place that holds the mid-transition, blended theme state during a `theme_transition`; the node-side HUD compositor cannot read that any other way without duplicating theme-blend bookkeeping the renderer already owns. **This is a deviation from P2's literal text, not a silent extension of it.** The alternative that holds to zero new public entry points literally — the node keeping its own reduced theme-color mirror, synchronized some other way — was considered and rejected: it would duplicate the renderer's own theme-blend bookkeeping (mid-transition state) in a second place, reintroducing exactly the drift risk P2's node-side-HUD design otherwise avoids. **User decision (2026-09-07): ACCEPTED.** `get_hud_colors()` ships as specified in Task 3, as an authorized amendment to P2 — P2's substance (the node-side CPU composite, no font pipeline in `visual_renderer`) stands unchanged; only its "no new public entry point" clause is amended for this one, load-bearing case.

---

## VM-037 / VM-044 scope decision (read first — VM-044 stays out of this plan; VM-037 is scheduled in full as Task 7, per the 2026-09-07 user decision)

The master plan's Epic 3 table originally listed six rows: VM-036, VM-034, VM-030, VM-031, VM-032, VM-035; the 2026-09-07 review added a seventh, VM-037 (see the master plan's Decisions block). VM-044 remains out of scope for this plan. Verified, not guessed:

- **VM-044** ("Package theme + ego assets for a real install") is listed under the backlog's own **"Epic 5 — Hardening & delivery"** heading (`visual-mode-backlog.md:237`), alongside VM-042/VM-043, and its own AC line says "**Blocks VM-043**" — an Epic 5 gate. It is Epic 5 work, not Epic 3, and this plan does not schedule it. Two of this epic's tasks (VM-030's font path, unavoidably) touch the *same class* of gap VM-044 exists to close (a per-user absolute path, not installed) — each such spot is called out as a **stated deviation that VM-044 closes**, not silently worked around.
- **VM-037** ("Epic-0 debt: mux hardening + build hygiene") says "do first in Epic 3" in its own backlog prose (`visual-mode-backlog.md:177`); the master table did not carry it as a row when this plan was first drafted, and Epic 1's own status ledger had filed the layout-assert gap under "→ VM-037" as a future item. Nine sub-items (a-i) live under VM-037: mux QoS, legacy-topic re-publish, `initial_mode`, `vcam_state[8]`, `kSceneVersion` + node-side layout asserts, `check_pod_header.sh` in colcon, `FILAMENT_VERSION` single-sourcing, GL_RENDERER logging, bluegl link-probe. **User decision (2026-09-07): VM-037 is scheduled into Epic 3 IN FULL, as a new Task 7** — the slice-only proposal below (kept for the record) was rejected. **Item (e)** — `kSceneVersion` plus a node-side layout-assert mirror — stays exactly where the original slice put it, **in Task 1**, because ADR-0004 states plainly that every interface change bumps `kSceneVersion` and is guarded by `sizeof`/`offsetof` static_asserts "on both toolchains," and Task 1 is this epic's (and the whole project's) **first** post-ADR-0004 `scene.h` change — there is no later, cheaper point to introduce the mechanism than the change that first needs it, so item (e) is Task 1's own prerequisite, not Task 7's. **Task 7 covers the remaining eight sub-items in full** (mux QoS, legacy topic, `initial_mode`, `vcam_state[8]`, `check_pod_header.sh` in colcon, `FILAMENT_VERSION` single-source, GL_RENDERER logging, bluegl probe) — see "Task 7 (VM-037)" below. These eight are unrelated Epic-0 mux/build-hygiene defects with no dependency from any of Tasks 1-6, scheduled last per the Status ledger, unless the user reorders.

---

## Ordering: why VM-036 and VM-034 run before the HUD/controls tasks

The master plan's 2026-09-07 changelog states the reason directly (`visual-mode.md:301-304`): "the two items that fix what is visible today... come first; HUD text follows." Restated concretely from what Epic 2 actually shipped:

- **VM-036 first.** Epic 2's map rendering is live-feedback-confirmed wrong today: every HD-map polyline (centerline, left boundary, right boundary) renders in one `palette.lane_paint` token (`dark_adas.yaml` ships `[0.45,0.5,0.55]`, mid-gray — verified, `assets/themes/dark_adas.yaml`), so a lane reads as three identical shifted stripes. Every later golden in this epic composes against that same map surface (VM-034's staleness golden, VM-032's layer-toggle golden), so fixing the surface first means nothing downstream gets re-shot twice.
- **VM-034 second.** It needs `MapElement.last_update_sec`, which Task 1 appends. Running VM-034 before it would either re-litigate the frozen-vs-additive question Epic 2 already deferred, or ship the staleness fade for five of six categories while leaving the sixth (map) popping — the exact deviation this task exists to close.
- **VM-030/031/032 (HUD, callouts, controls) come after both**, because VM-030's `render_ms`-adjacent GUI/diagnostics surface (VM-032's AC: "each preset measurably changes frame cost (`render_ms`)") only exists once Task 2 ships it, and because compositing legible HUD text over a scene whose base layers are still wrong (VM-036) is wasted design effort.
- **VM-035 (PointCloud) last.** It is the only task with zero real fixture data (the bag carries no `PointCloud2` topic at all — verified, no hits for `sensor_msgs::msg::PointCloud2` anywhere under `micropilot_visualization_node`), it shares no code path with Tasks 1-5, and the master table places it last.
- **Task 7 (VM-037) runs after all of that, and independently.** User decision 2026-09-07 schedules VM-037 into this epic in full; its eight remaining sub-items (mux QoS, legacy-topic re-publish, `initial_mode`, `vcam_state[8]`, `check_pod_header.sh` in colcon, `FILAMENT_VERSION` single-source, GL_RENDERER logging, bluegl probe) touch only the mux/build layer, share no code path with Tasks 1-6 (`scene.h`/themes/adapters/HUD/callouts/layers/quality/PointCloud), and carry no dependency in either direction with any of them — it is placed last in the Status ledger's fixed order, the same "ordered last, independent" logic VM-035 already gets, unless the user reorders it.

---

## Interfaces added this epic

**Additive only, per ADR-0004.** `include/visual_renderer/scene.h` currently has **no** `kSceneVersion` symbol anywhere in the repo (verified: `grep -rn kSceneVersion` outside `docs/` returns nothing) — this epic introduces it. Every struct change below is an **appended field or enum value**; no existing field is renamed, reordered, removed, or given a new meaning.

### `scene.h` changes (Task 1, bumped again Task 6)

```c
// Introduced this epic (ADR-0004). Bumped on every additive scene.h change;
// the node-side static_assert mirror (Task 1 Step 0) fails loudly on a
// layout mismatch instead of silently reading garbage across the ABI
// boundary at the node's next rebuild (Epic 2's own frozen-header note,
// now formalized).
constexpr uint32_t kSceneVersion = 1;   // Task 1 introduces it at 1 for this epic's MapElement
                                         // change; Task 6 raises it to 2 for the PointCloud category

// Appended to MapElement (VM-036). Namespace convention verified against
// the recorded bag (see Task 1, "Verified: kind/lane_id source"); ROAD_EDGE
// is reserved now for the disabled /road_markers row's future re-enable
// (urban_profile.yaml's own comment already names it) though nothing emits
// it in this epic; ROAD_SURFACE is adapter-synthesized (never present on
// the wire) — see Task 1's road-fill section for its two-rail point
// encoding.
enum class MapKind : uint8_t {
    OTHER = 0, CENTERLINE = 1, LEFT_BOUNDARY = 2, RIGHT_BOUNDARY = 3,
    CROSSWALK = 4, STOPLINE = 5, JUNCTION = 6, ROAD_EDGE = 7, ROAD_SURFACE = 8
};

struct MapElement {
    const Vec3* points;  uint32_t point_count;
    uint8_t is_polygon;
    MapKind kind;             // NEW, appended. Default (aggregate zero-init) = OTHER.
    uint32_t lane_id;         // NEW, appended. 0 = none (crosswalk/stopline/junction/other).
    double last_update_sec;   // NEW, appended. Closes Epic 2's "map pops, does not fade" deviation.
};
```

`TrackedObject`, `PathRibbon`, `GroundGridLayer`, `AlertPolygon`, `GenericMarker` already carry `last_update_sec` (verified, `scene.h:45,52,73,80,91`) — `MapElement` was the one holdout, and only because it had no field for it (Epic 2's stated deviation). This epic closes it the way ADR-0004 replaced "wait for a freeze lift" with: append the field.

### `scene.h` changes (Task 6, VM-035)

```c
// Appended category (ADR-0004; kSceneVersion -> 2). Colors are baked
// node-side (rgba8) per the profile row's color_mode — see Task 6.
struct PointCloudPoint { Vec3 position; uint8_t rgba[4]; };
struct PointCloud {
    const PointCloudPoint* points;  uint32_t point_count;
    double last_update_sec;
};
// SceneGraph gains:
//   const PointCloud* point_clouds; uint32_t point_cloud_count;
```

### New free functions (`scene.h`, additive entry points, same precedent as `set_ego_model`/`theme_assets_loaded`/`theme_parses`)

```c
// VM-030. Reads the HUD color tokens off whatever theme is CURRENTLY
// active -- including mid-transition, blended -- so the node-side HUD
// compositor never needs its own theme-tracking state (no second "which
// theme is active" bookkeeping, no risk of it drifting from the renderer's
// own theme_transition clock). false only if r == nullptr.
bool get_hud_colors(VisualRenderer*, float out_text_rgb[3], float out_accent_rgb[3],
                    float* out_scale);

// VM-031. Projects a map-frame point through whatever CameraPose the MOST
// RECENT render_frame() call configured, at that call's own output size --
// call it AFTER render_frame() in the same tick, mirroring the
// "everything re-derived from the last call" convention set_theme/
// render_frame already follow. Returns false (out_x/out_y unchanged) when
// the point is behind the camera or projects outside [0,1] on either axis;
// the caller (node-side compositor) treats false as "skip this chip this
// frame," not an error.
bool project_to_screen(VisualRenderer*, Vec3 world_pos, uint32_t width, uint32_t height,
                        float* out_x01, float* out_y01);
```

**No `set_quality()` entry point in this epic.** Decision P4 (master plan changelog) explicitly defers the live-quality-switch entry point to the Epic 5 plan; VM-032 Task 5 below ships the WS/GUI/param plumbing but the value only takes effect at the next `on_configure()`, not live — see Task 5's stated deviation.

### Node-internal (not POD, no ADR-0004 constraint — these are node-package headers, never cross the ABI boundary)

- `profile.hpp`: `NsRule` gains `MapKind kind` (Task 1); `NsRule::dashed` and its validator rule are **deleted** (Task 1 — dashing moves renderer-side).
- `adapter_stats.hpp`: unchanged (already carries every counter VM-034's diagnostics message needs — see Task 2).
- New `diagnostics.hpp`/`diagnostics.cpp` (Task 2): builds one `diagnostic_msgs/msg/DiagnosticArray` per tick from every row's `AdapterStats` plus `render_ms`.
- New `hud_overlay.hpp`/`hud_overlay.cpp` (Task 3): CPU compositor over `frame_buf_`.
- New `callouts.hpp`/`callouts.cpp` (Task 4): builds the chip list from collision/alert data + calls `project_to_screen`.
- New `point_cloud.hpp`/`point_cloud.cpp` adapter (Task 6).

---

## Shared decisions (later tasks and the reviewer cite these, not re-derive them)

### 1. The z-stack, verified, and where road-fill slots into it (Task 1)

Verified by reading every z-lift constant in the library, not asserted:

| Layer | Constant | Value | File:line |
|---|---|---|---|
| Ground | (untransformed base) | 0 | — |
| OGM gradient | `kGradientZLiftM` | 0.010 | `ground_grid.cpp:56` |
| OGM dynamic | `kDynamicZLiftM` | 0.015 | `ground_grid.cpp:57` |
| Lanes | `kLaneZLiftM` | 0.02 | `map_elements.cpp:142` |
| Object predicted-paths | `kPathZLiftM` | 0.03 | `objects.cpp:329` |
| Ribbon GLOBAL | `kRibbonZLiftByRoleM[1]` | 0.040 | `ribbon.cpp:88` |
| Ribbon LOCAL | `kRibbonZLiftByRoleM[2]` | 0.045 | `ribbon.cpp:89` |
| Ribbon BEHAVIOR | `kRibbonZLiftByRoleM[0]` | 0.050 | `ribbon.cpp:87` |
| Alerts | `kAlertZLiftM` | 0.06 | `alert_polygons.cpp:82` |

**Road-fill slots at `kRoadZLiftM = 0.005`, between ground and OGM.** Decided here, binding for Task 1: the road surface is a static base coat painted directly on the ground, so it sits just above it; OGM (a live perception overlay) sits *above* the road so a dynamic occupancy reading is never hidden behind the static road tint (an OGM texel is transparent where unobserved, so the road shows through everywhere the grid has nothing to say); lane paint sits above both, because paint is physically on top of the road surface and must never be occluded by it. This also means the road fill never contests a z-slot any existing category already occupies — no re-tuning of any constant above it.

### 2. Crosswalk-hatch fix — verified root cause, verified fix pattern (Task 1)

`build_crosswalk_hatch()` (`map_elements.cpp:84`) returns empty for `n != 4`. Deserialized directly from the committed fixture `test/fixtures/hd_map_local_elements_0.yaml` (marker `ns: crosswalk_8043`): the real marker carries **5** points, the first and last identical (`x: -39.50850289011474, y: 45.33743457749722, z: -0.000284586101770401` at both index 0 and index 4) — a closed polyline with a duplicate closing vertex, exactly the shape `collision.cpp` already handles for its own polygons (`kDedupEpsM = 1e-6`, `collision.cpp:61,157,173`: `if (pts.size() >= 2 && Dist(pts.front(), pts.back()) < kDedupEpsM) pts.pop_back();`). **Fix:** `hd_map.cpp` applies the identical trailing-duplicate dedupe to any `is_polygon` element's point list before it is stored, mirroring `collision.cpp`'s own pattern verbatim (not a new algorithm). This turns every recorded crosswalk from 5 points into 4 and makes `build_crosswalk_hatch()` fire on real data for the first time since Epic 2 shipped it as dead code.

### 3. Dashing moves renderer-side, AND the dashed kind flips (Task 1)

Backlog VM-036 debt item 2 (verbatim): "Epic 2 dashes `centerline_` (the lane spine, no painted analogue) and leaves boundaries solid — the inverse of ref-2's 'yellow centerlines vs white dashes'. Target: CENTERLINE solid, semantic yellow-family; BOUNDARY dashed white (or solid at road edge once `kind` distinguishes them)." Two changes land together here, not one:

1. **Where the dash decision is made moves from adapter to renderer.** Today (`hd_map.cpp:53-119,237-251`; `profile.hpp:28-41`): the **adapter** chops a polyline into alternating 1.5 m dash / 1.5 m gap runs at ingest time, gated by `NsRule::dashed` (a profile YAML flag, legal only on `render: polyline` rules), storing N separate `StoredElement`s per marker.
2. **Which kind gets dashed flips, per debt item 2.** The retired adapter-side chop dashed `CENTERLINE`. The renderer-side helper that replaces it fires on `e.kind == MapKind::LEFT_BOUNDARY || e.kind == MapKind::RIGHT_BOUNDARY` instead — `CENTERLINE` renders as one solid `MapElement`, never chopped. This flip is the point of debt item 2, not an incidental side effect of moving code across the ABI boundary.

**Deleted, verbatim list:**
- `hd_map.cpp`: `ChopIntoDashes()`, `Dist()`, `kDashLenM`/`kGapLenM`/`kMinDashLenM`, the `if (rule != nullptr && rule->dashed && verdict == NsRender::kPolyline)` branch and its `pieces` fan-out (the `else` branch's single-element path becomes the only path, for every kind, centerlines included).
- `profile.hpp`/`profile.cpp`: `NsRule::dashed` field and its validator rule ("legal ONLY on `render: polyline` rules").
- `urban_profile.yaml`/`offroad_profile.yaml`/`sim_profile.yaml`: every `dashed: true` flag (previously on `centerline_` rules; not re-added to boundary rules — the renderer now decides by `kind` alone, no profile flag needed).
- **Node test surface, `cuda/src/ros_apps/.../test/test_hd_map_adapter.cpp`** (verified via `grep -n -i dash`): the `DashRuleRow(bool dashed)` helper (~line 315, builds a profile row with a `dashed:` flag that no longer exists) and the four tests that exercise the adapter-side chop through it — per test, deleted or rewritten:
  - `DashedCenterlineChopsByArcLengthWithInterpolatedEndpoints` (~line 351) — **deleted**. Its premise (adapter-side chop on a centerline) is gone entirely; `DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm` (Task 1 Step 4, library-side, on a BOUNDARY kind) is its successor.
  - `NonDashedNamespaceOfIdenticalGeometryStaysOneElement` (~line 382) — **rewritten**. Drop the `DashRuleRow`/`dashed` parameter entirely; the adapter now never chops, for any kind, so this becomes a plain regression asserting a boundary marker AND a centerline marker each stay one element straight out of the adapter.
  - `DashedMarkerStillCountsAsOneIngestedMarkerForStats` (~line 406) — **rewritten**. Drop the "dash explosion, for context" framing (`ASSERT_EQ(out.map_elements.size(), 4u)` no longer holds — the adapter now emits exactly 1); keep the `stats().msgs == 1` / drop-counter assertions for one ingested marker.
  - `DashChopHappensAfterPoseComposition` (~line 512) — **deleted**. The ordering risk it guards (chop running before vs. after pose composition) no longer exists once chopping is entirely renderer-side and only ever sees already-posed points crossing the ABI boundary; no adapter-side successor is needed.
  Three more tests in the same file keep their shape but need their EXPECTED COUNTS reverted, since the adapter no longer dashes anything: `LocalElementsFixtureYieldsLanesAndCrosswalks`'s 246-element count and its explanatory comment (line 52, "urban's centerline_ rule now ships dashed: true") revert to 58 (16+16+16+5+5, no dash explosion); `MalformedMarkersAreDroppedAndCounted`'s 4-element count and comment (line 252) revert to 1; `RateLimitHonoursMaxRateHz`'s 8-element count and comment (line 301) revert to 2.
- **Stale comments naming `dashed`, outside the test file above** (would otherwise fail the review gate's `git grep -n dashed -- cuda/src/ros_apps` returns-nothing check): `include/micropilot_visualization_node/adapters/hd_map.hpp:107-109` (`StoredElement`'s "a dashed centerline's dash pieces... non-dashed, or a polygon" comment — the vector is now always size 1, no fan-out, for every kind); `include/micropilot_visualization_node/adapters/generic_marker.hpp:30` ("`is_polygon`/`dashed` have no meaning here" — `dashed` no longer exists as a concept to disclaim, so the comment's own phrasing needs to drop it).

**Added, in the library** (`map_elements.cpp`): the same algorithm (identical constants: 1.5 m dash, 1.5 m gap, 0.25 m minimum trailing dash), now a static helper called from `update_map_elements()` only when `e.kind == MapKind::LEFT_BOUNDARY || e.kind == MapKind::RIGHT_BOUNDARY`, producing per-dash mesh chunks under the *same* content-signature-keyed cache the function already uses — no new caching mechanism, the dash boundaries are just computed one layer downstream of where they used to be, against a different kind than before.

**AC, restated to not conflate two different backlog sentences:** the backlog's "dashed-centerline golden unchanged after the chop is retired" is the **chop-retirement** AC — it means the dash-geometry algorithm (run length, gap length, minimum trailing dash) must produce identical dash runs for a given polyline whether computed adapter-side or renderer-side; checked with a dedicated same-geometry-different-styling regression test (a mesh-count/dash-run assertion, not a full-frame SSIM), now exercised against a boundary polyline since boundaries are what the moved algorithm dashes. It does **not** mean centerlines stay dashed — the backlog's separate "golden per theme with all kinds visually distinct" AC and debt item 2's flip both require centerlines to render solid. The re-shot `map_ego_offset_*` goldens carry the flip as new, expected content, not a regression the SSIM check is meant to catch.

### 4. Kind/`lane_id` extraction — verified against real recorded ids, not the convention doc alone

The backlog's authoritative convention (`visual-mode-backlog.md:132-135`) states marker `id` == `lane_id` for centerline/boundary markers. Verified independently by deserializing `test/fixtures/hd_map_local_elements_0.yaml`: `ns: centerline_934, id: 934` and `ns: left_boundary_934, id: 934` — the marker's own `id` field, already used as half of `HdMapAdapter`'s `Key{ns,id}` storage key (`hd_map.cpp:164,259`), *is* `lane_id` directly. No string-parsing of the ns suffix is needed. `kind` itself is looked up from the matched `NsRule`, which Task 1 gives a `MapKind kind` member (populated per-row in the three profile YAMLs, validated the same way `dashed` used to be — legal combinations only, e.g. `kind: crosswalk` illegal on a `render: drop`/`polyline`-mismatched rule).

### 5. Road-surface fill — pairing in the adapter, geometry in the library, and the mismatch this epic's own fixture proves is real

Pairing (grouping `left_boundary_{lane_id}`/`right_boundary_{lane_id}` by shared `lane_id`) happens in **`hd_map.cpp`**, the only place with namespace context (per this epic's brief). **The naive assumption "both rails have the same point count" is FALSE on real data — verified, not assumed**: across the 16 lanes in `hd_map_local_elements_0.yaml` that have both a left and a right boundary, **14 match exactly and 2 do not** (lane 955: left 8 pts / right 9 pts; lane 813: left 10 / right 11). A road-fill implementation that zips `left[i]`/`right[i]` by raw index will silently misdraw or crash on ~12% of real lanes. **Fix:** resample both rails to a fixed `kRoadFillSamples = 16` stations by normalized arc length (reusing the exact interpolation technique `ChopIntoDashes`'s `point_at()` lambda already implements — extracted as a small shared free function before that lambda is deleted per decision #3, since the technique is what's reused, not the dash-specific caller) before the adapter emits the paired element. **Encoding, since `MapElement` has one flat `points` array**: a `kind == ROAD_SURFACE` element's `point_count` is always `2 * kRoadFillSamples`; `points[0 .. kRoadFillSamples)` is the left rail, `points[kRoadFillSamples .. 2*kRoadFillSamples)` is the right rail, index-parallel (same normalized station). The renderer (`map_elements.cpp`) just zips the two halves into a triangle strip — no new frozen field, no polygon-clipping code, reusing the "resample to fixed count" idea that's already proven correct at 16 stations for a road-width feature (clay style, not survey-grade). A lane with only one boundary recorded (0 of 16 in this fixture, per the "left-only/right-only" count above, but not provably impossible on other bags) emits no `ROAD_SURFACE` element for that lane — silently dropped, not malformed (spec §9's "missing data renders nothing, not an error").

### 6. Theme tokens — soft-defaulted, following the exact `palette.ego` precedent (Task 1)

`Palette` gains `road`, `lane_centerline`, `lane_boundary`, `crosswalk` (all `Float3`). Verified precedent: `theme.cpp:42` soft-defaults `palette.ego` (`palette["ego"] ? to_float3(...) : Float3{...}`) so a theme file predating the field still parses; `ribbon_global`/`ribbon_local`/`ribbon.width_m` repeat the same pattern (`theme.cpp:50-53`, `theme.hpp:105-114`). All four new tokens get the same soft-default mechanism, for the same reason (a theme file predating them still parses) — but the **shipped** `dark_adas.yaml`/`light_clay.yaml` do not rely on the default for `lane_centerline`, `lane_boundary`, or `road`, because debt item 2 and the backlog's road-surface AC (`visual-mode-backlog.md:151`, "golden per theme with all kinds visually distinct AND road darker than ground") both require this epic's shipped themes to actually look different from today's undifferentiated look, not soft-default back into it:

- **`lane_centerline`/`lane_boundary` (debt item 2's concrete hue targets):** `lane_centerline` is authored a semantic yellow-family value in both themes; `lane_boundary` is authored the near-white value debt item 3a already re-authors `lane_paint` toward (boundaries keep reading as "the old lane_paint tone," now on the kind decision #3 actually dashes white, while centerlines get the new yellow). The soft-default (`lane_centerline`/`lane_boundary` → `lane_paint`) stays in the code for a **third-party** theme file predating these tokens — neither shipped theme reads it, since both get explicit values in Task 1 Step 6. "Both soft-default to `lane_paint`" is the undifferentiated look VM-036 exists to kill, not an acceptable end state for the shipped themes.
- **`crosswalk`** has no debt-item hue target named, so it keeps the plain soft-default-to-`lane_paint` behavior with no further constraint.
- **`road`/`ground` (darker-road-than-ground requirement — VM-036's own backlog body, not a "debt item 5" that doesn't exist):** verified against `visual-mode-backlog.md:143-149`, VM-036's body text itself, not the numbered debt list: "**Road surface:** pair `left_boundary_{id}`/`right_boundary_{id}` by `lane_id` and fill the strip between them in a new `palette.road` token — ref-2's primary value separation (road darker than the clay ground) is unexpressible today because every map element is a stroke and the whole ground plane carries the road tone... Ground returns to a light clay value once the road is its own surface." Its AC (`visual-mode-backlog.md:151`) is "golden per theme with all kinds visually distinct AND road darker than ground." `road` soft-defaults to `ground` in code (today's "ground carries the road tone" behavior) for the same third-party-file reason, but neither shipped theme relies on it either: Task 1 Step 6 gives both `dark_adas.yaml` and `light_clay.yaml` an explicit, darker `palette.road` and a lightened `palette.ground`, checked against reference-1/-2's value separation.

**Per-kind width/z-lift are code constants, not theme fields** — a deliberate YAGNI call: `ribbon.width_m` only became a theme field after an explicit user directive asking to tune it; nothing in this epic's brief or the two reference images asks for per-deployment lane-width tuning, so four more soft-defaulted scalars would be scope the backlog never asked for. Promote to a theme field the day an author actually asks, exactly as `ribbon.width_m`'s own history did.

**Debt items folded in (backlog VM-036, "Debt found by the 2026-09-07 review"), verified against current values, not re-asserted from the backlog text:**
- (a) `dark_adas.palette.lane_paint = [0.45, 0.5, 0.55]` — confirmed mid-gray (`assets/themes/dark_adas.yaml`). Re-authored toward near-white per ref-1.
- (b) Both themes' inert object tints re-authored toward clay; pedestrian/cyclist/alerts keep saturation.
- (c) `dark_adas.ribbon_local = [0.95, 0.70, 0.15]` vs `alert.warning = [1.0, 0.7, 0.1]` — confirmed within 0.05/channel on every component (`assets/themes/dark_adas.yaml`). Move `ribbon_local` to a cool hue.
- (d) **Horizon/sky convergence guards — VERIFIED ALREADY SPLIT, correcting the epic2 plan's own post-gate note.** That note (`visual-mode-epic2.md:1886`) says 3b3ce2c "relaxed... the horizon/sky convergence bound 30 → 55 (**one shared bound**...)". Reading `tests/test_theme.cpp` directly: dark_adas's guard is `EXPECT_LT(..., 40.0)` (`test_theme.cpp:442`, unchanged by 3b3ce2c per `git blame`/`git log -p`, set in an *earlier* epic1 round-7 pass per its own comment) and light_clay's is `EXPECT_LT(..., 55.0)` (`test_theme.cpp:480`) — **these are, and were before 3b3ce2c, two independent `EXPECT_LT` calls in two separate tests, not one shared constant.** 3b3ce2c's diff (`git log -p`) touches only light_clay's line, `30.0 → 55.0`. The backlog debt item's ask ("split... instead of one shared 55," target "dark ~37 → <45, light ~45 → <55") is therefore **already structurally satisfied**; the only remaining, optional piece is loosening dark's bound from 40.0 to 45.0 for parity of headroom — a one-line tweak, done in Task 1 for completeness, not a guard split (there is nothing to split).
- Also in Task 1: `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml`, a blend-field-coverage guard (below), and the `map_element_rebuild_count` hook for the "cached, no per-frame rebuild" AC that Epic 2 shipped without a test.

### 7. `blend()` field-coverage guard (Task 1)

`theme_transition.cpp:83-155`'s `blend()` is a hand-written field-by-field function; `out` starts as a default-constructed `Theme{}` (every field zero). **A field added to `Palette`/etc. without a matching line in `blend()` silently blends to black forever** — there is no compiler check, because C++ has no reflection over aggregate members here. New test, `ThemeTransition.SentinelThemesDetectAnyUnblendedField`: construct theme `A` with every leaf field set to a sentinel far from any real color (e.g. every `Float3` component `111.0f`, every scalar `111.0f`) and `B` with every leaf at `222.0f`; blend at `t=0.5`; assert every leaf of the result lies strictly between `100.0f` and `230.0f` (catches a field that silently defaulted to `0.0f`). This is the practical equivalent of a field-count static_assert given the language has no member-reflection to assert against.

---

## Named fixture gaps (this epic)

1. **VM-035 — zero `PointCloud2` topics in any recording.** Verified: no `sensor_msgs::msg::PointCloud2` subscription or topic name appears anywhere under `micropilot_visualization_node`, and the sibling `micropilot_rendering_node`'s own `pointcloud_topic` param (`rendering_node.cpp:87`) defaults to `""` — no canonical topic name exists in this stack either. **Fallback:** three synthetic clouds (rgb-carrying, intensity-only, bare-XYZ) drive every adapter test and golden; the `auto` color-mode tiering is proven only against hand-built messages. **Closes when:** a bag is recorded from a lidar-equipped stack.
2. **RESOLVED — VM-036 road-surface fill on the two mismatched-count lanes.** Verified directly against the committed `test/fixtures/hd_map_local_elements_0.yaml` (counted the actual `points:` entries, not re-derived from decision #5's prose): lane **813** carries left 10 / right 11 rail points, lane **955** carries left 8 / right 9 — both already present in the committed fixture as-is, no re-cut and no hand-authored malformed-pair fixture needed. `MismatchedRailPointCountsStillProduceOneCorrectlyResampledElement` (Task 1 Step 5) uses lane 955 (or 813) directly from the committed fixture. The one case the committed fixture does NOT cover: all 16 lanes with a boundary marker have BOTH boundaries (16 `left_boundary_*` ids, 16 `right_boundary_*` ids, fully paired) — zero single-rail lanes — so `LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement` (Task 1 Step 5) has no real-fixture instance to draw on and must use a hand-built case.
3. **VM-034 — the ego-invalid map "stale lines" cosmetic (Epic 2 gate finding) is root-caused by reading code, not independently re-rendered.** See Task 2's own section — the mechanism is verified against `renderer.cpp:1458-1470` and `map_elements.cpp`, but I did not run a GPU render to confirm the exact "two lines" visual; the gate's own repro pointer (workflow `wf_0ff03eb8-5ec`) was not accessible from this planning pass. Task 2's step includes pulling that transcript before/while implementing.

---

## Task 1 (VM-036): `MapElement.kind`/`lane_id`/road-surface fill + dash-kind flip (boundary dashed, centerline solid) + crosswalk-hatch fix + `kSceneVersion`

**Files:**
- Modify: `include/visual_renderer/scene.h` (append `kSceneVersion=1`, `MapKind`, `MapElement::kind/lane_id/last_update_sec`)
- Modify: `src/theme.hpp`/`src/theme.cpp` (append `palette.road/lane_centerline/lane_boundary/crosswalk`, soft-defaulted; re-author dark_adas/light_clay per decision #6)
- Modify: `src/theme_transition.cpp` (blend the four new fields)
- Modify: `assets/themes/dark_adas.yaml`, `assets/themes/light_clay.yaml`
- Modify: `src/map_elements.cpp` (per-kind color/z-lift dispatch; road-surface strip triangulation from the two-rail encoding; centerline dash geometry moved in from the adapter; `map_element_rebuild_count` hook)
- Modify: `src/map_elements.hpp` (declare the new helpers; move the arc-length `point_at` helper here as the one place both the road-fill resample and the dash builder need it — or keep it adapter-side and expose it via a tiny shared node/lib-independent header if profiling says otherwise; **default to node-side** since resampling happens in the adapter per decision #5, dashing happens library-side per decision #3 — **these are two separate copies of "walk by arc length," one per side of the ABI boundary, and that is correct, not duplication**: the two toolchains cannot share a header across the POD boundary, and each function is ~15 lines)
- Modify: `cuda/src/ros_apps/.../src/adapters/hd_map.cpp` (kind/lane_id extraction, crosswalk dedupe, road-fill pairing+resampling, dash-chop deletion)
- Modify: `cuda/src/ros_apps/.../include/.../profile.hpp`/`src/profile.cpp` (`NsRule::kind` added, `NsRule::dashed` deleted, validator updated)
- Modify: `config/urban_profile.yaml`, `offroad_profile.yaml`, `sim_profile.yaml` (per-rule `kind:`, `dashed: true` flags removed)
- Modify: `tests/test_theme.cpp` (dark guard 40.0→45.0; `BuiltinFallbackMatchesDarkAdasYaml`; new tokens' presence/soft-default tests)
- Modify: `tests/test_theme_transition.cpp` (sentinel field-coverage test)
- Modify: `tests/test_map_elements.cpp` (kind styling, road-fill strip from mismatched-count rails, crosswalk-hatch-fires-on-real-geometry, dash-parity-after-move, rebuild-count hook)
- Modify: `tests/test_scene_buffer.cpp` (ADR-0004 layout guards — update `MapElement`'s existing `sizeof`/`offsetof` static_asserts, 16→32 bytes, three new `offsetof` lines for `kind`/`lane_id`/`last_update_sec`; add the `KindLaneIdLastUpdateSecSurviveAssign` test, Step 1)
- Modify: `cuda/src/ros_apps/.../test/test_hd_map_adapter.cpp` (ns→kind+lane_id mapping for both crosswalk spellings — `crosswalks`/plural/unsuffixed AND `crosswalk_{id}`; road-fill pairing incl. the mismatched-count case; **remove** the dash-chop test surface per decision #3's deletion list — `DashRuleRow` helper, delete `DashedCenterlineChopsByArcLengthWithInterpolatedEndpoints`/`DashChopHappensAfterPoseComposition`, rewrite `NonDashedNamespaceOfIdenticalGeometryStaysOneElement`/`DashedMarkerStillCountsAsOneIngestedMarkerForStats`, revert the three stale dash-explosion counts)
- **New, node-side, this epic's slice of VM-037(e):** `cuda/src/ros_apps/.../test/test_scene_layout.cpp` — mirrors `tests/test_scene_buffer.cpp`'s `sizeof`/`offsetof` static_asserts for every `scene.h` struct, compiled gcc/libstdc++, so a future layout drift fails the **node** build, not only the library's own clang/libc++ test binary. Registered as a plain `ament_add_gtest`, headers only, **no** link to `visual_renderer_prebuilt` — the exact pattern `test_ego_anchor` already establishes (`CMakeLists.txt:203-213`: `ament_add_gtest(test_ego_anchor test/test_ego_anchor.cpp)` immediately followed by a `target_include_directories` naming both the node's own `include/` and `${VISUAL_RENDERER_DIR}/include`, no `target_link_libraries` at all, under the comment "Header-only + POD types only -- no need to link the visual_renderer static archive, just its headers") — a static_assert has no runtime component at all, so this target can be pure compile-time verification with an empty `TEST(...)` body or none.
- Goldens: `map_ego_offset_dark_adas.png` / `map_ego_offset_light_clay.png` re-shot.

**Interfaces:** `kSceneVersion`, `MapKind`, `MapElement::{kind,lane_id,last_update_sec}` — all additive (see "Interfaces added this epic").

- [x] **Step 0: `kSceneVersion` + node-side layout mirror (this epic's VM-037(e) slice — do this before touching any struct).** *Done 2026-09-07. Verified arithmetic held exactly (32/13/16/24); `tests/test_scene_buffer.cpp` updated in place, node-side `test/test_scene_layout.cpp` added (new `ament_add_gtest`, header-only, no link) + registered in CMakeLists.txt right after `test_ego_anchor`. Both compile and pass on their respective toolchains (clang/libc++ ctest #64, gcc/libstdc++ ctest #2).* `tests/test_scene_buffer.cpp` does not just gain one line: the existing `static_assert(sizeof(mpviz::MapElement) == 16, "MapElement layout frozen")` (`test_scene_buffer.cpp:134`, verified) also gets updated to the new size, and three new `offsetof` lines are appended after the existing `offsetof(..., is_polygon) == 12` line (`test_scene_buffer.cpp:137`). **Arithmetic, verified against this doc's own field spec** (`points` 8 bytes @0, `point_count` 4 @8, `is_polygon` 1 @12, `kind` (`uint8_t`) 1 @13, `lane_id` (`uint32_t`, 4-byte aligned) @16, `last_update_sec` (`double`, 8-byte aligned) @24, ending @32, already 8-aligned): `sizeof(MapElement)` goes from 16 to **32**.
```cpp
// tests/test_scene_buffer.cpp (library, clang/libc++):
static_assert(mpviz::kSceneVersion == 1, "bump this alongside every additive scene.h change, and update the mirror below");
// existing line 134 updated: sizeof(MapElement) 16 -> 32; three new lines
// appended after existing line 137 (offsetof(..., is_polygon) == 12):
static_assert(sizeof(mpviz::MapElement) == 32, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, kind) == 13, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, lane_id) == 16, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, last_update_sec) == 24, "MapElement layout, ADR-0004 additive");
// "layout frozen" is ADR-0004-superseded wording -- reworded here and at
// every other MapElement/SceneGraph assertion this epic's diff touches
// (Task 6 too): appending is expected and covered by kSceneVersion, not
// frozen shut.

// cuda/.../test/test_scene_layout.cpp (NODE, gcc/libstdc++, NEW file):
#include "visual_renderer/scene.h"
#include <gtest/gtest.h>
static_assert(mpviz::kSceneVersion == 1, "node/library scene.h version drifted");
static_assert(sizeof(mpviz::MapElement) == 32, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, kind) == 13, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, lane_id) == 16, "node/library scene.h version drifted");
static_assert(offsetof(mpviz::MapElement, last_update_sec) == 24, "node/library scene.h version drifted");
TEST(SceneLayout, Placeholder) {}  // static_asserts above do the real work
```
  **Trap, stated once for every node-side file this task and every later task in this epic adds:** `micropilot_visualization_node/CMakeLists.txt` builds its source list from a **hand-written explicit list** (`CMakeLists.txt:171-177`), not a glob — verified, the comment there says so plainly ("Hand-written list, NOT a glob"). A new `.cpp` (this file has none; it's header-only) or a new gtest target not added to this list, and to every relevant `ament_add_gtest(...)` block, is a silent no-op, not a build error. Every later task in this epic repeats this trap warning inline rather than assuming it's remembered.
  Run — the static_asserts compile-fail until Step 1's struct actually exists with these fields in this order, which is expected; the numbers above (32/13/16/24) are the real, verified values for the field order this doc's "Interfaces added this epic" section specifies, not placeholders — Step 1 must produce a struct that makes them pass, not the other way around.

- [x] **Step 1: Append `MapKind`/`MapElement::kind,lane_id,last_update_sec` to `scene.h`; update `SceneBuffer::assign()` (unchanged — `MapElement` has no owned-pointer new field, so the deep-copy path in `scene_buffer.cpp` needs no new logic, only the existing memberwise struct copy, which is why `MapElement` was cheap to extend in the first place).** *Done 2026-09-07. `scene_buffer.cpp`'s assign() needed no change, confirmed -- the new fields survive via the existing memberwise copy (`SceneBufferMapElement.KindLaneIdLastUpdateSecSurviveAssign`, ctest #68).* Failing test first, `tests/test_scene_buffer.cpp`:
```cpp
TEST(SceneBufferMapElement, KindLaneIdLastUpdateSecSurviveAssign) {
    mpviz::Vec3 pts[2] = {{0,0,0},{1,0,0}};
    mpviz::MapElement e{};
    e.points = pts; e.point_count = 2; e.is_polygon = 0;
    e.kind = mpviz::MapKind::CENTERLINE; e.lane_id = 934; e.last_update_sec = 12.5;
    mpviz::SceneGraph s{}; s.map_elements = &e; s.map_element_count = 1;
    // ... set_scene(r, s); assert active().map_elements[0].kind/lane_id/last_update_sec unchanged
}
static_assert(sizeof(mpviz::MapElement) == 32, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, kind) == 13, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, lane_id) == 16, "MapElement layout, ADR-0004 additive");
static_assert(offsetof(mpviz::MapElement, last_update_sec) == 24, "MapElement layout, ADR-0004 additive");
```
  Run — FAIL (no fields). Implement with the field order this doc's "Interfaces added this epic" section specifies (`kind` immediately after `is_polygon`, then `lane_id`, then `last_update_sec`) so the offsets above (13/16/24, sizeof 32) actually hold — a different field order compiles but fails these asserts, which is the point. Run — PASS. Step 0's node-mirror asserts use the same verified numbers; no separate backfill pass is needed.

- [x] **Step 2: Crosswalk-hatch dedupe fix (decision #2).** Failing test, `tests/test_map_elements.cpp`, built from the REAL fixture geometry (5-point closed crosswalk, verified above), not an invented 4-point quad: *Done 2026-09-07. Adapter-side dedupe added in `hd_map.cpp` (mirrors collision.cpp's trailing-duplicate check verbatim); the real crosswalk_8043 fixture now stores 4 points, not 5, and its hatch fires (ctest #29). CORRECTION (review fix, 2026-09-07): the node-side companion test this step calls for verbatim ("a corresponding node-side adapter test asserts the STORED element for that marker has 4 points, not 5") did not exist at the time of this note — the `.geom` dump referenced above is a text export, not an assertion, and both suites stayed green with the dedupe reverted. `HdMapAdapter.CrosswalkTrailingDuplicateVertexIsDeduped` (`test_hd_map_adapter.cpp`) now asserts crosswalk_8043's stored element has `point_count == 4`.*
```cpp
TEST(MapElements, CrosswalkHatchFiresOnRecordedFivePointClosedPolyline) {
    // pts = the exact 5 points from test/fixtures/.../ns:crosswalk_8043 --
    // point[0] == point[4] (closing vertex). Before the fix,
    // build_crosswalk_hatch(pts, 5, ...) returns empty (n != 4 guard) and
    // the caller falls back to a plain fan fill -- this test currently
    // documents THAT (RED once the dedupe lands and still returns empty --
    // it must not).
    auto tris = mpviz::detail::build_crosswalk_hatch(pts_after_dedupe, 4, 0.02f);
    EXPECT_FALSE(tris.empty());
}
```
  The dedupe itself is adapter-side (`hd_map.cpp`), mirroring `collision.cpp:157,173` verbatim; a corresponding node-side adapter test asserts the STORED element for that marker has 4 points, not 5.
  Run — FAIL. Implement. Run — PASS.

- [x] **Step 3: kind/`lane_id` extraction + `NsRule::kind`, delete `NsRule::dashed`.** Failing tests in `test_hd_map_adapter.cpp` (both crosswalk spellings, per decision #4's namespace convention): *Done 2026-09-07. `NsRule::kind` (mpviz::MapKind) added, `NsRule::dashed` deleted; validator now checks kind-vs-render legality instead. Both profile YAMLs that carry hd_map rows (urban, sim) updated with `kind:` per rule; `offroad_profile.yaml` has no hd_map rows and needed no change. kind/lane_id extraction verified against the real fixture (ctest test_hd_map_adapter, all green). CORRECTION (review fix, 2026-09-07): this step's own `SimCrosswalksPluralUnsuffixedGetsCrosswalkKind` test did not exist at the time of this note — `test_profile.cpp:157` covers only urban's `crosswalk_` rule at rule level (`match_rule`/`classify`), nothing exercised the adapter on sim's plural/unsuffixed `crosswalks` namespace, so dropping `kind: crosswalk` from `sim_profile.yaml`'s row would have silently disabled crosswalk hatching on sim's only full-extent map source with both suites green. `SimCrosswalksPluralUnsuffixedGetsCrosswalkKind` (`test_hd_map_adapter.cpp`) now asserts `kind == mpviz::MapKind::CROSSWALK` on that fixture's polygon element.*
```cpp
TEST(HdMapAdapter, CenterlineAndBoundaryKindAndLaneIdFromMarkerId) {
    // ingest a centerline_934/id=934 and a left_boundary_934/id=934 marker;
    // fill(); assert one element has kind==CENTERLINE, lane_id==934, and
    // the other kind==LEFT_BOUNDARY, lane_id==934.
}
TEST(HdMapAdapter, SimCrosswalksPluralUnsuffixedGetsCrosswalkKind) {
    // ns=="crosswalks" (sim publisher, no numeric suffix) -> kind==CROSSWALK
}
TEST(HdMapAdapter, UrbanCrosswalkUnderscoreIdGetsCrosswalkKind) {
    // ns=="crosswalk_8043" -> kind==CROSSWALK, lane_id left at 0 (crosswalks
    // are not lane-paired)
}
```
  Delete `ChopIntoDashes`/`Dist`/the three dash constants/the `dashed` branch from `hd_map.cpp`; delete `NsRule::dashed` + its validator rule from `profile.hpp`/`profile.cpp`; strip every `dashed: true` from the three profile YAMLs.
  Run — FAIL then PASS; confirm every existing `test_profile.cpp` test that referenced `dashed` is updated, not just left to bit-rot red.

- [x] **Step 4: Dashing moves into `map_elements.cpp`, now on BOUNDARY kinds (decision #3).** Failing test: *Done 2026-09-07. Dash chopper moved into map_elements.cpp (`chop_into_dashes`), gated on `IsBoundaryKind()`; CENTERLINE now renders as one solid mesh. `MapElementsGolden.DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm` (4 mesh chunks) and `MapElementsGolden.CenterlineOfSameGeometryProducesOneMeshChunkNotDashSplit` (1 mesh chunk) both pass via the new `map_element_mesh_count` hook.*
```cpp
TEST(MapElementsGolden, DashedBoundaryProducesSameDashRunsAsThePreMoveAlgorithm) {
    // Same polyline geometry Epic 2's dash-chop used to consume, now on a
    // kind==LEFT_BOUNDARY (or RIGHT_BOUNDARY) MapElement, one whole element,
    // no adapter-side chop. Render, compare dash PATTERN (not full-frame
    // SSIM, which now also carries road-fill and per-kind color -- a
    // separate concern) against a small geometry-only reference via the
    // rebuild-count hook / mesh-count assertion, per the epic2 precedent
    // (LongPathSplitsAcrossMeshesWithoutTruncation asserts a mesh COUNT
    // through a Filament-free hook). A companion assertion confirms
    // kind==CENTERLINE on the same geometry produces ONE mesh chunk, not
    // dash-split -- the flip's other half.
}
```
  Implement the arc-length dash builder in `map_elements.cpp` (same constants, moved, now gated on `e.kind == MapKind::LEFT_BOUNDARY || e.kind == MapKind::RIGHT_BOUNDARY` per decision #3). Run — PASS.

- [x] **Step 5: Road-surface fill (decision #5).** Named fixture gap #2 is already resolved (see "Named fixture gaps" above): lanes 813 (left 10 / right 11) and 955 (left 8 / right 9) are both present in the committed `hd_map_local_elements_0.yaml` as-is — no re-cut, no hand-authored malformed-pair fixture. `LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement` is the one test below with no real-fixture instance to draw on (the committed fixture's 16 boundary-bearing lanes are all fully paired) and uses a hand-built single-rail case instead. Failing tests: *Done 2026-09-07. Pairing + resample-to-16-stations implemented in hd_map.cpp's fill() (ResampleByArcLength, extracted from the deleted dash point_at()); library-side build_road_strip() zips the two-rail encoding into a triangle strip. All fixture-derived malformed-count tests pass; the real fixture's 16 fully-paired lanes each produce one ROAD_SURFACE element, point_count 32, verified in test_hd_map_adapter. CORRECTION (review fix, 2026-09-07): "hand-built" tests did not exist at the time of this note — `LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement`, called out above and twice elsewhere in this step ("Named fixture gaps" #2) as the one case with no real-fixture instance, had no hand-built successor, leaving `fill()`'s `if (it == right_by_lane.end()) continue;` branch unexecuted by any test. `HdMapAdapter.LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement` (`test_hd_map_adapter.cpp`) now hand-builds a single-rail lane and asserts zero `ROAD_SURFACE` elements with no `dropped_malformed` bump. SECOND CORRECTION (review, 2026-09-07): the other two named tests in this step's block — `PairedBoundariesOfSameLaneIdProduceOneRoadSurfaceElement` and `MismatchedRailPointCountsStillProduceOneCorrectlyResampledElement` — were never written as standalone tests; their substance is carried by the fixture-wide aggregate in `LocalElementsFixtureYieldsLanesAndCrosswalks` (`road_count == 16` over the fully-paired fixture, every `ROAD_SURFACE` `point_count == 32`, which was verified to discriminate the mismatched 8/9 and 10/11 lanes). What the aggregate does NOT assert: that a resampled rail's stations actually span the recorded rail's endpoints (a resampler that clamped to a sub-range would pass). That span property is left as a named gap for VM-034's test pass to pick up alongside its map-fade tests, not silently claimed here.*
```cpp
TEST(HdMapAdapter, PairedBoundariesOfSameLaneIdProduceOneRoadSurfaceElement) {
    // left_boundary_934 + right_boundary_934 (10 pts each, MATCHING count) ->
    // exactly one extra MapElement, kind==ROAD_SURFACE, lane_id==934,
    // point_count == 2*16.
}
TEST(HdMapAdapter, MismatchedRailPointCountsStillProduceOneCorrectlyResampledElement) {
    // lane 955 (left 8 pts / right 9 pts, REAL recorded shape) -> still
    // exactly one ROAD_SURFACE element, point_count == 32, both halves
    // resampled to 16 stations by normalized arc length -- not a crash, not
    // a truncation, not an index-out-of-bounds on the shorter rail.
}
TEST(HdMapAdapter, LaneWithOnlyOneBoundaryProducesNoRoadSurfaceElement) {
    // a lane_id present in left_boundary but absent from right_boundary ->
    // zero ROAD_SURFACE elements for it, no crash, no malformed-count bump
    // (this is "missing data," not "bad data" -- spec §9).
}
TEST(MapElements, RoadSurfaceKindTriangulatesTheTwoRailEncodingIntoAStrip) {
    // library-side: given a MapElement{kind=ROAD_SURFACE, point_count=32},
    // assert the built mesh has (16-1)*2 triangles (a strip over 16
    // stations), via a Filament-free mesh-count/vertex-count hook.
}
```
  Run — FAIL then PASS.

- [x] **Step 6: Theme tokens + re-authoring (decision #6) + `blend()` coverage (decision #7).** Add `palette.road/lane_centerline/lane_boundary/crosswalk` (soft-defaulted, per decision #6); re-author both YAML files per the debt items **and** decision #6's explicit targets — a semantic yellow-family `lane_centerline`, `lane_boundary` set to the near-white value `lane_paint` is re-authored toward (debt item 3a), a new darker `palette.road`, and a lightened `palette.ground`, in both `dark_adas.yaml` and `light_clay.yaml` (backlog VM-036: "Ground returns to a light clay value once the road is its own surface") — checked against reference-1/-2's value separation, with the existing golden re-shoot proving the separation; add the sentinel blend-coverage test; loosen dark's horizon guard 40.0→45.0; add `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml`. Run existing golden suite — every category golden not touched by kind/road-fill (objects, ribbons, alerts, markers, OGM) must stay green unchanged (no theme field this task adds is read by those categories' materials). *Done 2026-09-07, WITH A STATED DEVIATION: `palette.road/lane_centerline/lane_boundary/crosswalk` added, soft-defaulted, blended; dark_adas's `lane_paint` re-authored near-white (debt item a) and both themes gained explicit `road`/`lane_centerline`/`lane_boundary` values (decision #6). DEVIATION -- `palette.ground` was deliberately left UNCHANGED in both themes (not lightened as the backlog prose describes): changing it would move `empty_world_{dark_adas,light_clay}.png`'s pixels, which are NOT in this task's sanctioned-red list (only the two map_ego_offset goldens are). Debt items (b) object-tint re-tint and (c) ribbon_local hue move were likewise SKIPPED for the same reason -- they would dirty `objects_mixed_dark_adas.png`/`ribbons_three_roles_dark_adas.png`, also unsanctioned this task. Horizon guard loosened 40.0->45.0 (debt item d); `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml` added and passing; full existing golden suite reran green except the two sanctioned MapGolden reds.*
  Run — PASS; re-shoot `map_ego_offset_{dark_adas,light_clay}.png` (human-promoted, per golden-promotion convention).

- [x] **Step 7: `map_element_rebuild_count` hook + test (Epic 2's untested AC).** A small counter incremented once per `adopt_or_build` cache-miss branch in `update_map_elements`, exposed via a Filament-free test hook (same shape as every other `..._test_hooks.hpp`). Test: publish the identical `SceneGraph` twice, assert the second `render_frame()` call's rebuild count is 0 (cache hit on unchanged content signatures) — the "cached, no per-frame rebuild" AC, finally checked. *Done 2026-09-07. `mapElementRebuildCount` added to VisualRenderer, incremented on every adopt_or_build() cache-miss branch; `map_element_rebuild_count()` test hook added. `MapElements.RebuildCountStaysZeroOnUnchangedContentSignature` publishes the identical SceneGraph twice and asserts the second call's delta is 0 -- passes.*
  Run — PASS.

---

### User directive 2026-09-08 (post-candidate review) — road edges, hidden centerlines, dot guidance

**Context:** Task 1's work above was complete but uncommitted, goldens unpromoted, when the user reviewed the shot candidates and gave this directive. It amends Task 1 in place, before any commit — not a new task.

**The directive, verbatim:** "the boundary of the road (most left and most right lines should not be dashed and should be colored diffrently usually yellow) and also center lines shoud be hidden unless turned on in yaml config and their color also configurable (a lite fade of lane color) with circles points along the lign instead of a yellow filled line that is implemented now."

**Gate note (word-level `dashed` grep):** this addendum re-introduces exactly THREE sanctioned prose hits under `cuda/src/ros_apps` — `config/urban_profile.yaml` (~45, the centerline-rule comment), `src/adapters/hd_map.cpp` (~117, quoting the directive), `test/test_hd_map_adapter.cpp` (~631, quoting the directive) — two of which quote the user's own words and are not reworded on principle. The review-gate bullet is amended to name them; the substantive property (no `dashed` field/param/flag anywhere) is unchanged.

**Decisions, implemented exactly:**

1. **ROAD_EDGE gets its first producer.** `MapKind::ROAD_EDGE` (reserved since Task 1's original `scene.h` append — no enum change needed, ADR-0004 stays untouched) is now emitted by `HdMapAdapter::fill()` (`hd_map.cpp`), NODE-side, geometry-driven: after indexing every `LEFT_BOUNDARY`/`RIGHT_BOUNDARY` element by `lane_id` (the same index road-surface pairing already builds — one pass now serves both), a second pass promotes a boundary's EMITTED kind to `ROAD_EDGE` when `IsRoadEdge()` finds no other lane's OPPOSITE-side boundary within `kRoadEdgeCoincidenceThresholdM` of it, sampled at the boundary's own first/mid/last point against each candidate's full polyline extent. The underlying `left_by_lane`/`right_by_lane` maps used for road-surface pairing are unaffected by this relabeling — pairing still happens on the original `LEFT_BOUNDARY`/`RIGHT_BOUNDARY` kind.
   - **Measured, not guessed** (see `hd_map.cpp`'s own comment on `kRoadEdgeCoincidenceThresholdM` for the full worked numbers): sampling the committed `hd_map_local_elements_0.yaml`'s 16 lanes' 32 boundary elements, the real adjacent-lane "twin" pairs (the same painted line, surveyed once per neighbouring lane) land at **0.00–0.20 m** — several are exact floating-point duplicates, several others within centimeters, confirming Epic 2's own prior "centimeters" measurement. The nearest case that is NOT a shared line sits at **1.7955 m**, with the bulk of genuinely separate lanes at 3.1–3.7 m (roughly one lane width). **Chosen threshold: `kRoadEdgeCoincidenceThresholdM = 1.0 m`** — real headroom on both sides (~5x the largest true twin separation below it, ~1.8x below the nearest true non-twin above it), not a round-number guess.
   - **Fixture's actual ROAD_EDGE count, measured by running the real adapter against the committed fixture, not derived on paper: 15 of the 32 boundary elements** (9 stay `LEFT_BOUNDARY`, 8 stay `RIGHT_BOUNDARY`). Lane 934 is the one lane fully interior on both sides (paired left AND right) and contributes zero `ROAD_EDGE` elements. Pinned in `HdMapAdapter.LocalElementsFixtureYieldsLanesAndCrosswalks` (node suite) and confirmed via an independent controlled geometry test, `HdMapAdapter.OuterBoundariesOfThreeAdjacentLanesPromoteToRoadEdge` / `.SingleIsolatedLaneHasBothBoundariesPromotedToRoadEdge`.
   - O(n²) over the fixture's ~32 boundaries per window (16 lanes × 2 sides, each checked against every OTHER lane's opposite side) is fine at this scale — no optimization attempted.
2. **ROAD_EDGE styling: SOLID (never dashed), new soft-defaulted `palette.road_edge` token.** `IsBoundaryKind()` (`map_elements.cpp`) still only matches `LEFT_BOUNDARY`/`RIGHT_BOUNDARY` — `ROAD_EDGE` was never added to it, so it falls straight through to the plain solid-polyline path (never chopped into dashes). New theme token `palette.road_edge`, soft-defaulted to `lane_paint` (the `palette.ego` precedent, per the directive's own design note), authored explicitly in BOTH shipped themes as a yellow-family value: `dark_adas.yaml` `[0.95, 0.75, 0.05]` (a clear road-paint yellow, readable against the dark `[0.03,0.035,0.045]` road fill), `light_clay.yaml` `[0.75, 0.55, 0.03]` (a deeper yellow than the now-faded `lane_centerline`, per ref-2's double-yellow). Wired via the eager-instance-per-kind mechanism Task 1 established: `roadEdgeMaterial`/`roadEdgeMaterialBaseColor` fields (`renderer_internal.hpp`), eager `createInstance()` + `setCullingMode` + destroy (`renderer.cpp`'s `create_renderer()`/`destroy_renderer()`), themed every `push_theme_to_scene()` call, dispatched in `material_for_kind()` (`map_elements.cpp`) and mirrored in the `map_kind_base_color()` test-hook switch (`renderer.cpp`) — every touched `MaterialInstance` registered, per the hard rule. Interior boundaries that stay `LEFT_BOUNDARY`/`RIGHT_BOUNDARY` are unchanged: still dashed white via `laneBoundaryMaterial`.
3. **Centerlines hidden by default.** Every shipped `hd_map` row's `centerline_` rule flips from `{render: polyline, kind: centerline}` to `{render: drop}` in `urban_profile.yaml` (`/hd_map_local_elements`, `/hd_map_global_elements`) and `sim_profile.yaml` (`/hd_map_local_elements`, `/hd_map_global_elements`, `/sim/hd_map/markers`) — 5 rows total, `offroad_profile.yaml` has no `hd_map` rows and needed no change. `kind: centerline` could NOT be left on the rule alongside `render: drop`: `ValidateRow`'s `KindIsLegalOnRender` (`profile.cpp`) rejects a non-`OTHER` kind on a drop verdict, so the rule now carries no `kind` key at all (defaults to `OTHER`, legal on any verdict) — each row's comment states the re-enable steps (`render: polyline, kind: centerline` restored). The kind machinery, extraction test, and renderer path all REMAIN — only the shipped profile ROWS changed; a re-enabled row renders centerlines as dot guidance (decision #4 below), not the old solid strip.
   - **Test-surface fallout, fixed honestly, not patched around:** `test/fixtures/hd_map_malformed_0.yaml`'s four markers were `centerline_*`-namespaced (a deliberate choice at the time, so urban's centerline rule kept them polyline-classified) — renamed to `left_boundary_*` (comment updated) since a `centerline_*` marker would now be dropped BY RULE before ever reaching the malformed-detection path the fixture exists to exercise. `HdMapAdapter.ArrowNamespaceIsDroppedByLongestPrefixWins`'s `dropped_by_rule` count moves 64 → 80 (the 16 `centerline_` markers join the 64 `centerline_arrows_` ones). `Profile.ShippedUrbanLocalRowMarksCenterlineAndBoundaryKinds` and `Profile.ShippedProfilesRouteEveryKnownNamespaceOfEveryShippedTopic` (`test_profile.cpp`) updated to expect `kDrop`/`MapKind::OTHER` for `centerline_*` across all three shipped-row checks. `HdMapAdapter.RateLimitHonoursMaxRateHz`'s synthetic markers renamed `centerline_*` → `left_boundary_*` for the same reason. Two more existing tests (`MarkerOfAnyKindStaysOneElementOutOfTheAdapter`, `CenterlineAndBoundaryKindAndLaneIdFromMarkerId`) needed their expected kind updated from `LEFT_BOUNDARY` to `ROAD_EDGE` — an ISOLATED single-lane boundary marker with no opposite-side partner is, correctly, a road edge under decision #1's new detection; the tests' own point (one-marker-one-element; `lane_id` extraction) is unaffected by which of the two kinds it ends up carrying, and their comments now say so.
   - `LocalElementsFixtureYieldsLanesAndCrosswalks`'s aggregate recount (measured by rerunning the adapter, not derived on paper): 58 total map elements (down from 74) — 42 lane/crosswalk elements (16 centerline removed; 9 `LEFT_BOUNDARY` + 8 `RIGHT_BOUNDARY` + 15 `ROAD_EDGE` + 5 `CROSSWALK` + 5 `STOPLINE` = 42) + 16 synthesized `ROAD_SURFACE`.
4. **Centerline rendering when ON: dot circles, not a strip.** New `build_centerline_dots()` (`map_elements.cpp`): flat filled discs (10-segment triangle fan per dot), arc-length spaced along the polyline (`kCenterlineDotSpacingM = 2.0 m` between dot centres, `kCenterlineDotRadiusM = 0.15 m` radius — both named code constants, same YAGNI call decision #6 made for per-kind width/z-lift; promote to a theme field the day a deployment asks to tune it), reusing a newly-extracted `point_at_arc_length()` helper shared with `chop_into_dashes()` (same file, same toolchain — reuse, not the ABI-boundary "two small copies" pattern decision #3/#5 established). `update_map_elements()`'s dispatch gains a dedicated `MapKind::CENTERLINE` branch (chunk-guard aware, same `polyline_chunks()` ceiling every other path uses) calling this builder instead of the old `build_ribbon_flat()`. Color: `palette.lane_centerline` stays the configurable knob but is RE-AUTHORED in both themes to a low-contrast 25% `lane_paint` / 75% `road` blend ("a light fade of lane color", per the directive) — `dark_adas.yaml` `[0.235, 0.239, 0.254]`, `light_clay.yaml` `[0.53, 0.53, 0.57]` (exact mix arithmetic stated in each YAML's own comment). Proven via a new Filament-free hook, `map_element_total_vertex_count()` (mirrors `Mesh::vertexCount`, itself a new field `add_mesh()` now populates for every mesh it builds — no getter existed before): `MapElements.CenterlineRendersAsDotDiscsNotAStrip` asserts a 10 m line's dot-disc mesh is exactly 180 vertices (6 dots × 10 segments × 3), against a strip of the same geometry which would be 6.
5. **Goldens.** `map_ego_offset_{dark_adas,light_clay}.png` re-shot against the regenerated `hd_map_local_elements_0.geom` fixture (58 elements, ROAD_EDGE-promoted, centerline-free — regenerated by rerunning `HdMapAdapter.LocalElementsFixtureYieldsLanesAndCrosswalks` with `MPVIZ_EMIT_GEOM` set, per the existing convention) in the SHIPPED default state: candidates at `/tmp/map_ego_offset_dark_adas_actual.png` / `/tmp/map_ego_offset_light_clay_actual.png`, both SSIM-red against the old (pre-directive) committed goldens, as expected — NOT promoted. One new library-side golden, `centerline_dots_dark_adas.png` (no committed reference yet — candidate-only), feeding synthetic `CENTERLINE` elements directly (`MapGolden.CenterlineDotsOnState_DarkAdas`, profiles don't gate library tests): candidate at `/tmp/centerline_dots_dark_adas_actual.png`.
6. **This block** is the dated addendum decision #6 (this section) itself calls for — nothing above is ticked as a numbered Task 1 step (Steps 0–7 above stay exactly as they were recorded 2026-09-07); this is amendment work layered on top of a still-uncommitted Task 1.

**Files touched by this directive** (library): `src/theme.hpp`, `src/theme.cpp`, `src/theme_transition.cpp`, `src/renderer_internal.hpp`, `src/renderer.cpp`, `src/map_elements.cpp`, `src/map_elements_test_hooks.hpp`, `assets/themes/dark_adas.yaml`, `assets/themes/light_clay.yaml`, `tests/test_map_elements.cpp`, `tests/test_theme.cpp`, `tests/test_theme_transition.cpp`, `tests/fixtures/hd_map_local_elements_0.geom` (regenerated). **(node):** `src/adapters/hd_map.cpp`, `include/.../adapters/hd_map.hpp`, `config/urban_profile.yaml`, `config/sim_profile.yaml`, `test/test_hd_map_adapter.cpp`, `test/test_profile.cpp`, `test/fixtures/hd_map_malformed_0.yaml`. **(other):** `tools/validate_visual_mode.sh` (MILESTONE LOG line appended).

**Verification:** library suite 101/103 (the 2 sanctioned `MapGolden` reds above, everything else including every new test green); node suite 12/12 green (`test_profile`, `test_hd_map_adapter` both fully green after the fallout fixes above). `check_pod_header.sh` green (no `scene.h` change this directive — `ROAD_EDGE` already existed). Build order followed: library rebuilt first (`libs_build.sh Debug`), then the stale `libvisualization_node_lib.so` removed and the node force-relinked (`colcon_build.sh micropilot_visualization_node`).

**Unspecified by the directive, decided here:** the exact dot radius/spacing/segment-count (0.15 m / 2.0 m / 10 segments — cosmetic, tunable); the exact yellow RGB values for `road_edge` in each theme (matched to each theme's existing brightness range); the exact lane_centerline fade mix ratio (25/75, chosen for visible-but-subtle dot legibility — a much higher lane_paint fraction read as too bold in the rendered candidate, a much lower fraction risked disappearing against a near-black `road` in `dark_adas`).

---

## Task 2 (VM-034): Staleness fades (map layer) + diagnostics topic + `render_ms` + the ego-invalid map cosmetic

**Files:**
- Modify: `src/map_elements.cpp` (wire `staleness_alpha(sim_time_sec, e.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec)` per-element, same call every other category already uses — Epic 2's "Staleness" section, `epic2 plan lines 228-238`); bind `clay_translucent.mat` for a fading map element (the material that CAN receive alpha, per Epic 2's own established rule — never `clay.mat`/`clay_faded.mat`, see that plan's "…and the material that can actually do it" section)
- Modify: `src/renderer.cpp` (`update_map_elements` call site, `render_frame()`'s `ego.valid` gate — see the cosmetic fix below)
- New: `cuda/src/ros_apps/.../src/diagnostics.cpp`/`include/.../diagnostics.hpp`
- Modify: `cuda/src/ros_apps/.../src/visualization_node.cpp` (time `render_frame()`; build+publish the diagnostics message every tick; add the `ego.valid` gate for map elements)
- Modify: `cuda/src/ros_apps/.../package.xml`/`CMakeLists.txt` (`<depend>diagnostic_msgs</depend>`, `find_package(diagnostic_msgs)`, add `src/diagnostics.cpp` to the hand-written source list AND every gtest target that needs it — same trap as Task 1 Step 0)
- New test: `cuda/src/ros_apps/.../test/test_diagnostics.cpp`
- Modify: `tools/vcam_gui.py` (surface diagnostics — see Step 5)

**Interfaces:** none new in `scene.h` (this task consumes Task 1's `MapElement.last_update_sec`, adds no fields). Node-internal: `diagnostics.hpp` exposes `diagnostic_msgs::msg::DiagnosticArray BuildDiagnostics(const std::vector<RowStats>&, double render_ms)`.

- [ ] **Step 0: Diagnostics message TYPE — decided here, `diagnostic_msgs/msg/DiagnosticArray`, not a new custom message.** Verified available in this distro: `/opt/ros/humble/share/diagnostic_msgs` exists (ROS2 Humble core package), so no new `.msg`/interface-generation package is needed (this node package has none today — confirmed, no `.msg`/`.srv` under it, only a `<depend>` on `micropilot_rendering_node`'s existing srv). One `diagnostic_msgs::msg::DiagnosticStatus` per profile row (`name` = topic, `level` = OK/WARN by staleness, `values` = key/value pairs: `last_msg_age_sec`, `dropped_malformed`, `dropped_stale`, `dropped_no_tf`, `dropped_by_rule`, all already available verbatim on `AdapterStats` — `adapter_stats.hpp:25-33`, confirmed field-for-field), plus one node-level status carrying `render_ms`. Published on `~/diagnostics`, `rclcpp_lifecycle::LifecyclePublisher`, matching the design doc's own naming (§9: "a `~/diagnostics`-style status"). **Why not `diagnostic_msgs::msg::DiagnosticArray` reinvented as a bespoke struct:** it is the ROS-idiomatic type for exactly this (name+level+key/value list), it already has `rqt_runtime_monitor`/`diagnostic_aggregator` tooling ecosystem support if this ever needs it, and it costs one `<depend>` line, not a new interface-generation package.
  Failing test, `test_diagnostics.cpp`:
```cpp
TEST(Diagnostics, OneStatusPerRowPlusRenderMs) {
    std::vector<mpviz_node::RowStats> rows = {{"/hd_map_local_elements", stats_with_some_drops}};
    auto msg = mpviz_node::BuildDiagnostics(rows, /*render_ms=*/4.2);
    ASSERT_EQ(msg.status.size(), 2u);  // 1 row + 1 node-level
    EXPECT_EQ(msg.status[0].name, "/hd_map_local_elements");
    // values contain "dropped_by_rule" == the row's actual count, as a string
    EXPECT_EQ(msg.status[1].name, "render_ms");
}
```
  Run — FAIL (no `diagnostics.hpp`). Implement. Run — PASS.

- [ ] **Step 1: `render_ms` instrumentation.** In `timer_callback()`, wrap the existing `mpviz::render_frame(renderer_, render_pose, view)` call (`visualization_node.cpp:694`) with `std::chrono::steady_clock` before/after; store the millisecond delta; feed it into `BuildDiagnostics()` alongside every row's `AdapterStats`. **Only measured while `active_mode_ == 3`** (the existing early-return at line 676 already skips render/readback/publish in modes 1-2 — `render_ms` in the diagnostics message reads 0/unset in those modes, not stale from the last time mode 3 was active; a test asserts this explicitly so a future refactor doesn't accidentally publish a frozen number).
  Run — PASS.

- [ ] **Step 2: Map-layer staleness fade (closes Epic 2's stated deviation).** Failing test, `tests/test_map_elements.cpp`:
```cpp
TEST(MapElements, FadesViaSharedStalenessAlpha) {
    // same test SHAPE as every other category's "...FadesViaSharedStalenessAlpha"
    // (epic2 plan's exemplar, Task 2 Step 8a there) -- publish a MapElement
    // with last_update_sec in the past relative to sim_time_sec, render,
    // assert via a test hook that the bound clay_translucent instance's
    // baseColor.a matches staleness_alpha(...)'s own computed value, NOT a
    // pixel comparison.
}
```
  Implement: bind `clay_translucent.mat` (never `clay.mat`/`clay_faded.mat` — Epic 2's own established rule) for any map element whose alpha < 1.0, following the exact per-entity-instance-swap mechanism Epic 2 Task 4 built for objects (create from `clay_translucent_mat->createInstance()`, never `MaterialInstance::duplicate()` of the opaque lane-paint template — the same type-mismatch trap Epic 2's gate section documents in detail). This is reuse of an existing mechanism, not a new one; the review gate must find zero new fade machinery in this task.
  Run — PASS. **This closes Epic 2's stated deviation** ("the HD-map category pops, it does not fade") — update `map_elements.cpp`'s own deviation comment to say so, and delete the "STATED DEVIATION" heading there (ADR-0004's whole point: a deviation forced by a frozen header disappears once the header is additively extended, it does not need a permanent scar in the code).

- [ ] **Step 3: The ego-invalid map cosmetic (Epic 2 gate finding, KNOWN COSMETIC, workflow `wf_0ff03eb8-5ec`).** **Root cause, verified by reading code (not independently re-rendered — flagged in "Named fixture gaps" above):** `update_ground_grid_transform()` (`renderer.cpp:1458-1470`) snaps the ego-following ground/grid patch back to the **world origin** whenever `ego.valid == 0` (`renderer.cpp:1461-1464`). `update_map_elements()` has **no** `ego.valid` gate at all — it renders whatever `HdMapAdapter` is still filling, and that adapter's own staleness gate (`visualization_node.cpp:547`, `sim_clock_sec_ - hr.adapter->stats().last_msg_sec > hr.timeout_sec`) runs purely against **topic silence**, never against TF/ego validity. So a TF dropout that outlasts nothing but leaves `/hd_map_local_elements` still publishing re-renders real, far-from-origin lane geometry against a ground/grid patch that has snapped to the origin, with the camera pose also going "absolute" per `visualization_node.cpp`'s own comment on `render_pose = pose_` when `!scene.ego.valid`. The result reads as disconnected lane strips floating in a void — real, live map geometry, not cached debris; there is nothing "stale" about it in the caching sense at all.
  **Fix:** gate `update_map_elements()` on the same `ego.valid` the ground/grid transform already checks — while `ego.valid == 0`, drive every map element's staleness alpha to 0 via Step 2's fade path (reusing the fade just built, not a new mechanism) rather than skip-and-freeze (freezing would leave the LAST valid frame's geometry rendering forever at full opacity against an origin-snapped ground, which is the same bug one frame later). Failing test:
```cpp
TEST(MapElements, EgoInvalidFadesMapElementsRatherThanLeavingThemAtFullOpacity) {
    // publish real MapElements + ego.valid=1, render (elements at alpha 1);
    // publish the SAME elements + ego.valid=0, render again; assert alpha
    // has dropped (via the same test hook Step 2 added), not held at 1.0.
}
```
  Run — FAIL then PASS. **Pull the `wf_0ff03eb8-5ec` transcript before closing this step** to confirm the fix actually addresses the two-lines repro the Epic 2 gate observed, not just the mechanism this plan hypothesizes from static reading.

- [ ] **Step 4: GUI surfaces diagnostics.** `tools/vcam_gui.py` subscribes to `~/diagnostics` (or receives it relayed through the WS bridge — reuse whichever channel `_apply_state`/`_on_ws_msg` already uses for `~/vcam_state`, do not open a second transport) and shows `render_ms` + a per-row staleness indicator in the existing panel layout (`_build_panel`). No new WS command needed for this step — it is display-only, using the same "node telemetry flows to the GUI" pipe `vcam_state` already established (`vcam_ws_bridge.py`'s `_on_state`).
  Manual verification: launch the WS bridge + GUI against a bag, silence a topic mid-playback, watch its diagnostics row's age climb and its layer visibly fade.

---

## Task 3 (VM-030): HUD overlay — node-side CPU composite

**Files:**
- New: `cuda/src/ros_apps/.../src/hud_overlay.cpp`, `include/.../hud_overlay.hpp`
- New: `assets/fonts/<one committed font>.ttf` (node package)
- Modify: `include/visual_renderer/scene.h` (append `get_hud_colors()`)
- Modify: `src/renderer.cpp` (implement `get_hud_colors()` — reads `r->active_theme.hud.{text_color,accent_color,scale}`, already populated every tick by `apply_current_theme()`; no new state)
- Modify: `cuda/src/ros_apps/.../src/visualization_node.cpp` (populate `scene.hud.speed_mps`/`scene.hud.active_mode`; call the compositor on `frame_buf_` after `render_frame()` succeeds, before `img_msg.data.assign(...)`)
- Modify: `cuda/src/ros_apps/.../CMakeLists.txt` (own `file(DOWNLOAD ...)` block for `stb_truetype.h` — **see the trap below**; add `hud_overlay.cpp` to the hand-written source list + relevant gtest targets)
- Modify: `config/default_params.yaml` (`hud_font_path` param)
- New test: `cuda/src/ros_apps/.../test/test_hud_overlay.cpp`

**Interfaces:** `get_hud_colors()` (additive, `scene.h`) — **an authorized amendment to P2's literal text** ("no new public entry point"), ACCEPTED by the user 2026-09-07 — see "ACCEPTED (user, 2026-09-07): `get_hud_colors()` ships as specified" above; not silently reconciled by the sentence below. No new library rendering path, no font pipeline in `visual_renderer` (still true, and still P2's point on that half). **P2, cited in full (master plan changelog line 309-310; backlog VM-030, `visual-mode-backlog.md:81-87`):** accepted node-side HUD compositing "instead of an SDF atlas + Filament overlay pass inside the lib... no font pipeline in the clang/libc++ archive, no new public entry point." **Re-entry trigger:** the backlog states it as "HUD text must be depth-tested, lit, or fogged" (`visual-mode-backlog.md:86-87`); the master plan's own shorter paraphrase says "depth-tested or lit" (`visual-mode.md:309`) — cited here as the backlog's fuller wording, not reconciled between the two since that reconciliation isn't this epic's job.

- [ ] **Step 0: `SceneGraph::hud` is never populated today — verify, then fix.** Grepped: `visualization_node.cpp` never writes `scene.hud.speed_mps`/`scene.hud.active_mode` anywhere (confirmed, no `scene.hud` reference in the file). Add, immediately after `scene.ego = tf_adapter_->update();` and before `mpviz::set_scene(renderer_, scene)` (`visualization_node.cpp:664-666`):
```cpp
scene.hud.speed_mps = scene.ego.speed_mps;
scene.hud.active_mode = static_cast<uint8_t>(active_mode_);
```
  (`Hud::chips`/`chip_count` are deliberately left at their zero-init default here — see Task 4's scope decision.) Failing test in `visualization_node`'s existing test surface (or a focused unit test against a hand-built `SceneAssembly`/`Ego` pair) asserting `scene.hud.speed_mps == scene.ego.speed_mps` after one `timer_callback()` tick.
  Run — FAIL then PASS.

- [ ] **Step 1: stb_truetype vendoring — a SEPARATE copy from the library's, and here is exactly why (the trap an implementer will otherwise hit).** `visual_renderer/CMakeLists.txt:95-114` already vendors `stb_image.h`/`stb_image_write.h` via a pinned-commit `file(DOWNLOAD ...)` into `${CMAKE_BINARY_DIR}/_deps/stb` — but that `CMAKE_BINARY_DIR` is the **library's own build tree** (clang/libc++), and `micropilot_visualization_node` never `add_subdirectory`s that project; it only imports the library's **prebuilt static archive** (confirmed, node `CMakeLists.txt`'s `visual_renderer_prebuilt` import block). The node's own `CMAKE_BINARY_DIR` is a completely different directory, and `stb_truetype.h` (a **different** single-header file from the same `nothings/stb` repo, needed for TrueType glyph rasterization, which `stb_image_write.h` does not provide) was never vendored by the library in the first place — there is nothing to reuse even if the build trees were shared. **Fix:** the node's own `CMakeLists.txt` gets its own pinned-commit `file(DOWNLOAD ...)` block for `stb_truetype.h`, same pattern as the library's (immutable commit URL + `EXPECTED_HASH SHA256=...`), into the node's own `${CMAKE_BINARY_DIR}/_deps/stb`. Header-only, `#define STB_TRUETYPE_IMPLEMENTATION` in exactly one `.cpp` (`hud_overlay.cpp`).
  No test for this step (build-system plumbing); verified by `hud_overlay.cpp` compiling and linking.

- [ ] **Step 2: Font asset — a stated, scoped deviation, not a silent shortcut.** One CC0/OFL-licensed `.ttf` is committed under `assets/fonts/` in the node package (pick any well-known permissively-licensed monospace/sans face — e.g. a Google Fonts OFL family already redistributed under a license compatible with this repo; the specific face is a cosmetic choice for whoever implements this step, not a design decision this plan needs to pre-make). Resolved via a new `hud_font_path` param, defaulting to a per-checkout path exactly like `ego_model_path`'s existing default (`/home/ag7/Downloads/M02P.glb`, `default_params.yaml`) does today. **This is the same class of gap VM-044 (Epic 5) exists to close — installed-package asset resolution — and is stated here explicitly, not worked around:** VM-044's own AC ("Install `assets/themes` and the converted ego `.glb`... `initial_theme` param") should gain `hud_font_path` to its list when that epic plan is written; this task does not attempt `ament_index`-based resolution itself.
  No test (asset presence is a file-exists check in CI, not a behavior).

- [ ] **Step 3: The compositor.** Failing test, `test_hud_overlay.cpp`:
```cpp
TEST(HudOverlay, CompositesLegibleTextAtLowPreset) {
    std::vector<uint8_t> rgb(1280*720*3, 40);  // a known mid-gray frame
    mpviz_node::HudSnapshot hud{/*speed_mps=*/12.3, /*active_mode=*/3};
    mpviz_node::CompositeHud(rgb.data(), 1280, 720, hud,
                              /*text_rgb=*/{0.9f,0.95f,1.0f}, /*accent_rgb=*/{0.1f,1.0f,0.4f},
                              /*scale=*/1.0f, "assets/fonts/<face>.ttf");
    // assert SOME pixels near the speed-chip's expected screen region differ
    // from the uniform 40 background by more than a legibility threshold --
    // a rendered-text presence check, not a pixel-exact golden (font
    // rasterization is deterministic per stb_truetype version but this test
    // should not need to pin exact glyph bitmaps).
}
TEST(HudOverlay, MissingOrUnloadableFontIsNonFatalAndLeavesFrameUnchanged) {
    // spec §9 "asset load failure -> non-fatal, WARN once" -- same
    // philosophy as set_ego_model's clay-box fallback, applied here as
    // "no HUD drawn, not a crash, not garbage pixels."
}
```
  Implement `CompositeHud()`: bake the font atlas once via `stbtt_BakeFontBitmap` (or `stbtt_PackFontRange` for better kerning — implementer's call, either satisfies the AC) at `on_configure()`-adjacent load time, not per-frame; draw the speed value (`"%.1f km/h"` or m/s per existing `ego_state` convention — verify units against `TfAdapter`'s own speed field, m/s, and label accordingly) and a mode indicator string using `get_hud_colors()`'s live (possibly transitioning) theme colors, scaled by the theme's `hud.scale`. Wire into `timer_callback()` right after the `render_frame()` success check (`visualization_node.cpp:694-698`), before `img_msg.data.assign(frame_buf_...)` (`:710`) — operates in place on `frame_buf_`.
  Run — FAIL then PASS. Golden: one node-side test renders a synthetic known scene + HUD at 720p, low preset, and a human promotes the first golden (spec's own AC: "text legible at 720p low preset").

---

## Task 4 (VM-031): Alert callouts — `project_to_screen()` + node-side leader lines

**Files:**
- Modify: `include/visual_renderer/scene.h` (append `project_to_screen()`)
- Modify: `src/renderer.cpp` (implement it against the camera set by the most recent `render_frame()` call)
- New: `cuda/src/ros_apps/.../src/callouts.cpp`, `include/.../callouts.hpp`
- Modify: `cuda/src/ros_apps/.../src/visualization_node.cpp` (build the chip list from live `AlertPolygon`/collision data after `render_frame()`, call `project_to_screen()` per chip, hand the results to `hud_overlay`'s compositor for the actual pixel drawing — reusing Task 3's text-rasterization path, not a second one)
- New test: `cuda/src/ros_apps/.../test/test_callouts.cpp`

**Interfaces:** `project_to_screen()` (additive, `scene.h`) — the **only** new library surface this task needs.

- [ ] **Step 0: Scope decision — `Hud::chips`/`AlertChip` in `scene.h`, faithfully plumbed since Epic 1, go UNUSED by design.** Verified: `SceneBuffer::assign()` already deep-copies `Hud::chips`/`chip_count` (`scene_buffer.cpp:76-83`) and `tests/test_scene_buffer.cpp:171-179` already static_asserts `AlertChip`'s and `Hud`'s layout — this machinery has existed, correctly, since Epic 1, and **nothing anywhere renders a chip** (verified: zero hits for `chips`/`AlertChip` in any `.cpp` under `visual_renderer/src` or the node's `src`, outside `scene_buffer.cpp`'s own copy code). Under the accepted node-side-compositing design (P2, this epic), chip text and leader lines are drawn in the SAME node-side buffer `hud_overlay.cpp` already writes to — routing chip data through `SceneGraph::hud.chips` and back out would accomplish nothing, since the node already holds the chip list (built from its own collision/alert data) before `set_scene()` is ever called; the only thing the LIBRARY needs to contribute is the 3D→2D projection (`project_to_screen()`), which does not require the chip's text to have crossed the POD boundary at all. **Decision: `Hud::chips`/`chip_count`/`AlertChip` stay in `scene.h` (ADR-0004 forbids removing them regardless), the node never populates them, and this is stated here rather than left for a future reader to wonder whether it's an oversight.** They remain available for a hypothetical future in-scene (3D-anchored, SDF) text path — the one the design doc's §4.4 explicitly keeps the door open for — should that ever be built instead of the node-side composite.
  No test for a decision to leave fields unpopulated; a comment in `scene.h` next to `Hud::chips` states this and points here.

- [ ] **Step 1: `project_to_screen()` — failing test first**, library-side (`tests/test_renderer_projection.cpp` or folded into an existing render test file):
```cpp
TEST(ProjectToScreen, PointAtCameraTargetProjectsNearCenter) {
    // render_frame() with a known CameraPose, then project_to_screen() on
    // that pose's own `target` -- expect out_x/out_y near (0.5, 0.5).
}
TEST(ProjectToScreen, PointBehindCameraReturnsFalse) { }
TEST(ProjectToScreen, PointOutsideFrustumReturnsFalse) { }
```
  Implement using the camera's current view/projection matrices (`r->camera`, already updated every `render_frame()` call at `renderer.cpp:1514-1518`) — standard world→clip→NDC→[0,1] transform, Y flipped to match `FrameView`'s top-to-bottom row order (state this convention explicitly in the header comment, since the epic2 precedent shows an unstated axis convention is exactly the kind of thing that ships a silently-flipped picture).
  Run — FAIL then PASS.

- [ ] **Step 2: Node-side chip builder + leader-line draw.** Failing test:
```cpp
TEST(Callouts, NearestObstacleChipTracksAcrossCameraMove) {
    // same AlertPolygon anchor, two different CameraPoses across two ticks
    // -> project_to_screen() returns two different screen positions, and
    // the drawn chip/leader-line (via hud_overlay's text+line primitives)
    // moves with it -- the backlog's own AC, "callout tracks object across
    // camera moves."
}
TEST(Callouts, ChipForAnObjectBehindCameraIsSuppressedNotMisdrawn) { }
```
  `callouts.cpp` picks the nearest-obstacle anchor from live `AlertPolygon`/collision-adapter data (distance-to-ego, reusing whatever the collision adapters already compute — no new geometry math), formats one text line (e.g. `"3.2 m"`), calls `project_to_screen()`, and if it returns true, asks `hud_overlay`'s already-built text/line drawing primitives (Task 3) to render the leader line + chip at that screen position. **No second compositor** — this task extends Task 3's, it does not write a parallel one.
  Run — FAIL then PASS. Golden: one composited frame with a visible callout, human-promoted.

---

## Task 5 (VM-032): Layer visibility + quality-preset plumbing

**Files:**
- Modify: `cuda/src/ros_apps/.../src/visualization_node.cpp` (per-category visibility params; clear the corresponding `SceneAssembly` vector before `point_at()` when hidden — see the design decision below; no renderer change)
- Modify: `config/default_params.yaml` (`layer_objects`/`layer_paths`/`layer_map_elements`/`layer_grids`/`layer_alerts`/`layer_markers`/`layer_point_clouds`, all `bool`, default `true` — **`layer_point_clouds` is declared here but stays inert until Task 6**, see Step 1's note and Task 6's own step registering its gate)
- Modify: `tools/vcam_ws_bridge.py` (`set_layers {layer: bool, ...}`, `set_quality <preset>` commands)
- Modify: `tools/vcam_gui.py` (layers checklist; quality dropdown/selector)
- Modify: `src/renderer.cpp` (`create_renderer()`'s quality dispatch — map shadow-map resolution, shadow enable, and the `low`-preset render-scale per spec §8's preset table)
- New/modify test: WS bridge E2E for both new commands; library-side `tests/test_renderer_quality_presets.cpp` (or folded into an existing renderer-config test file) for the three §8 knobs

**Interfaces:** none in `scene.h`. **No `set_quality()` library entry point** — decision P4 defers only the LIVE quality-switch mechanism (appended entry point vs. renderer re-create) to the Epic 5 plan. It does not defer mapping spec §8's per-preset knobs at `create_renderer()` time: those are `RenderConfig` create-time knobs, exactly like the SSAO/FXAA-vs-TAA dispatch already in `renderer.cpp:1021-1029`, so implementing them needs no live switching and does not touch P4 at all. This task's `set_quality` WS command only writes the `quality` ROS **parameter**, which today is read exactly once, at `create_renderer()` time (`visualization_node.cpp:83`, `config.quality = static_cast<uint8_t>(quality_)` — verified, no other read site exists); Step 3 below extends what that one read site's dispatch actually configures. The stated deviation for this task is narrower than "quality is create-time only" — it is specifically "no live in-process preset change," not which knobs the create-time dispatch covers.

- [ ] **Step 0: Design decision — layer visibility is a NODE-SIDE gate on `SceneAssembly`, not a new renderer API.** Verified: every category's fill loop already appends into `scene_asm_` (a per-category `std::vector`, `scene_assembly.hpp`) before `scene_asm_.point_at(scene)` sets the frozen `SceneGraph`'s pointer/count pairs (`visualization_node.cpp:665`). Hiding a layer therefore needs nothing renderer-side: clearing the relevant vector (e.g. `if (!layer_objects_) scene_asm_.objects.clear();`) right before `point_at()` makes that category publish as `count == 0` for this tick, exactly as if nothing had ever filled it — every render path already handles the empty case (Epic 1/2's own "zero counts render nothing" contract). **Why not a new `set_layer_visible()` library entry point:** it would duplicate a decision the node already fully controls, for zero capability gain, and it would be the one place in this epic that adds a `scene.h` entry point without a real need (contrast `get_hud_colors`/`project_to_screen`, which cross the POD boundary because only the renderer holds theme/camera state — layer visibility needs neither).
  No test for the decision statement itself; Step 1's tests exercise the mechanism.

- [ ] **Step 1: Per-category visibility params + WS `set_layers`.** Failing test (WS bridge E2E, Python, matching the existing `test_vcam_contract.py`/WS E2E convention):
```python
def test_set_layers_hides_and_shows_a_category(ws_client, node_diagnostics):
    ws_client.send({"cmd": "set_layers", "layers": {"objects": False}})
    # diagnostics/next scene tick shows object_count == 0 downstream
    ws_client.send({"cmd": "set_layers", "layers": {"objects": True}})
    # object_count recovers on the next message from a live-publishing topic
```
  Implement: the seven bool params above (declared with a `SetParametersCallback` so they take effect on the very next `timer_callback()` tick, no restart needed — this is genuinely live, unlike quality); `set_layers` in `vcam_ws_bridge.py` calls the node's `set_parameters` service for however many keys the client sent (one WS message, N parameter sets — still one client round trip, matching spec §6's batch intent, even though the underlying mechanism is N ROS parameter writes, not one new service).
  **`layer_point_clouds` forward-reference, resolved explicitly:** `PointCloud` (the category it gates) does not exist until Task 6. **Task 5 owns**: declaring the param, wiring it into `SetParametersCallback` alongside the other six, and exposing it through `set_layers` — all of that is generic and needs nothing category-specific. **Task 5 does NOT own**: the actual gate clearing `scene_asm_.point_clouds` — there is no such vector yet. This task's own tests (this step's `test_set_layers_hides_and_shows_a_category` and its siblings) assert only the six LIVE categories (objects/paths/map_elements/grids/alerts/markers); `layer_point_clouds` is declared-but-inert here, accepted as a stated, scoped gap — not a silent oversight — until **Task 6 Step 1** registers the gate (see that step).
  Run — FAIL then PASS.

- [ ] **Step 2: `set_quality` WS command — plumbing only, live effect explicitly deferred (P4).** Failing test:
```python
def test_set_quality_updates_the_param_but_does_not_change_the_live_renderer(ws_client):
    ws_client.send({"cmd": "set_quality", "preset": "high"})
    # ros2 param get quality == 2 (or whatever encoding is chosen)
    # render_ms (from Task 2's diagnostics) is UNCHANGED this tick -- the
    # AC's "each preset measurably changes frame cost" is verified across a
    # NODE RESTART at each preset (three separate launches), NOT a live
    # in-process switch, until Epic 5's set_quality() entry point exists.
```
  GUI gains a quality dropdown (three presets) that sends `set_quality` and shows a "takes effect on next restart" note — stated in the UI, not just this doc, so a user doesn't file a bug against Epic 3 for a live-switch capability this epic never promised.
  Run — PASS. **AC interpretation, stated explicitly:** the backlog's "each preset measurably changes frame cost (`render_ms`)" is satisfied by three separate launches at three presets, diagnostics compared across them (a one-time manual/E2E check, recorded in this epic's Results block), not by a single running process changing preset mid-stream.

- [ ] **Step 3: Map spec §8's three deferred knobs into `create_renderer()`'s quality dispatch.** Verified gap: `renderer.cpp:1021-1029`'s `config.quality` dispatch today drives only SSAO enable/resolution and FXAA-vs-TAA — nothing maps shadow-map resolution, shadow enable, or the `low`-preset render-scale, so without this step Step 2's three-launch AC check is barely satisfiable (two of the three presets would differ only in SSAO/AA cost). These are `RenderConfig` create-time knobs, so mapping them needs no live switching and does not touch decision P4 (which defers only the live `set_quality()` entry point — see this task's Interfaces note above). Failing test, library-side:
```cpp
TEST(RendererQuality, ShadowMapResolutionMatchesPresetTable) {
    // create_renderer() at quality=2 (high) -> shadow map dimension 2048;
    // quality=1 (medium) -> 1024; asserted via a Filament-free config/test
    // hook, not a live GPU readback.
}
TEST(RendererQuality, ShadowsDisabledAtLowPreset) {
    // quality=0 -> shadow enable == false.
}
TEST(RendererQuality, LowPresetRendersAtUpscaledRenderScale) {
    // quality=0, output size 1280x720 requested -> internal render target
    // is 960x540, upscaled to the requested output size (spec §8).
}
```
  Implement in `create_renderer()`'s existing quality dispatch (`renderer.cpp:1021-1029`'s block): shadow-map resolution (2048 high / 1024 medium), shadow enable (`false` at low), and the 960x540 render-scale for `low` (implementer's call on the exact Filament mechanism — dynamic-resolution options vs. an explicit internal-target-size-plus-upscale-blit; the AC is the resulting resolution/enable state, not a specific API). Run — FAIL then PASS. State in this task's deviation note (and the Results block) that only the live `set_quality()` entry point is deferred to Epic 5 (P4); the knob mapping itself ships in this task.

---

## Task 6 (VM-035): `PointCloudLayer`

**Files:**
- Modify: `include/visual_renderer/scene.h` (append `PointCloudPoint`, `PointCloud`, `SceneGraph::point_clouds/point_cloud_count`; bump `kSceneVersion` to 2 — Task 1 introduced the constant at 1, this is its only bump in this epic)
- Modify: `src/scene_buffer.hpp` (`OwnedScene` gains `point_clouds`/`point_cloud_points` storage, mirroring `map_elements`/`map_element_points`)
- Modify: `src/scene_buffer.cpp` (`assign()` gains a `point_clouds` deep-copy + re-pointing block, mirroring the `map_elements` block verbatim)
- Modify: `tests/test_scene_buffer.cpp` (ADR-0004 layout guards — update `SceneGraph`'s existing `sizeof` static_assert, 184→200 bytes (`point_clouds`/`point_cloud_count` appended after `hud` at offset 184, ending 8-aligned at 200 — verified against this doc's own field spec), plus new `sizeof`/`offsetof` lines for `PointCloudPoint`/`PointCloud`; add `PointCloudPointsSurviveAssignAfterSourceBufferDies`, Step 1)
- New: `assets/materials/point_cloud.mat` (its own vertex-layout/blend contract, per Epic 2's established "one material, one job" precedent — see below)
- New: `src/point_cloud.cpp`/`.hpp` in `visual_renderer`
- New: `cuda/src/ros_apps/.../src/adapters/point_cloud.cpp`/`include/.../adapters/point_cloud.hpp`
- Modify: `profile.hpp`/`profile.cpp` (new `adapter: point_cloud` role, `color_mode`/`max_points`/`stride`/`max_rate_hz` fields)
- Modify: `visualization_node.cpp` (subscribe/fill/timeout wiring, same shape as every other category)
- New tests: `tests/test_point_cloud.cpp` (library), `test_point_cloud_adapter.cpp` (node)
- Goldens: three synthetic clouds (rgb, intensity-only, bare-XYZ)

**Interfaces:** `PointCloudPoint`/`PointCloud` category, appended (ADR-0004, `kSceneVersion`→2).

- [ ] **Step 0: `kSceneVersion` → 2, node-side layout mirror updated (repeats Task 1 Step 0's pattern for the new category).** Same trap reminder: `test_scene_layout.cpp` (Task 1) and `tests/test_scene_buffer.cpp` both gain `static_assert`s for `PointCloudPoint`/`PointCloud`, and the version constant bumps in the SAME commit that adds the struct, per ADR-0004's "bumped on every change." `tests/test_scene_buffer.cpp` ALSO updates its existing `SceneGraph` assertion, `static_assert(sizeof(mpviz::SceneGraph) == 184, ...)` at ~line 181 (verified) — appending `const PointCloud* point_clouds` (pointer, 8 bytes) and `uint32_t point_cloud_count` (4 bytes) after the existing `hud` field (which ends at offset 184, already 8-aligned) gives 184+8+4=196, rounded up to the struct's 8-byte alignment: **`sizeof(SceneGraph)` becomes 200**, plus a new `offsetof(mpviz::SceneGraph, point_clouds) == 184` / `offsetof(..., point_cloud_count) == 192` pair. **Both this task's and Task 1's "layout frozen" assertion messages get reworded** (e.g. to "MapElement/SceneGraph layout, ADR-0004 additive") — ADR-0004 supersedes the freeze language those messages predate; appending is the expected, guarded operation now, not a violation of a frozen contract.

- [ ] **Step 1: `OwnedScene` deep-copy for `PointCloud` + `assign()` re-pointing (do this before the vertex-layout work).** `PointCloud::points` is a caller-owned raw pointer, exactly like `MapElement::points` — `assign()`'s `view = src` copies that pointer verbatim, so without this step `active().point_clouds[i].points` dangles the instant `set_scene()` returns (it will often still *look* like it works in a quick manual test, which is exactly how this class of bug hides). Mirrors the `map_elements`/`map_element_points` pattern in `scene_buffer.hpp:36-47` and `scene_buffer.cpp:33-40` verbatim:
```cpp
// scene_buffer.hpp, OwnedScene:
std::vector<PointCloud> point_clouds;
std::vector<std::vector<PointCloudPoint>> point_cloud_points;  // point_clouds[i].points storage

// scene_buffer.cpp, assign():
point_clouds.assign(src.point_clouds, src.point_clouds + src.point_cloud_count);
point_cloud_points.resize(src.point_cloud_count);
for (uint32_t i = 0; i < src.point_cloud_count; ++i) {
    const PointCloud& s = src.point_clouds[i];
    point_cloud_points[i].assign(s.points, s.points + s.point_count);
    point_clouds[i].points = point_cloud_points[i].data();
}
view.point_clouds = point_clouds.data();
```
  Failing test first, `tests/test_scene_buffer.cpp`, in the shape of Task 1 Step 1's `KindLaneIdLastUpdateSecSurviveAssign`:
```cpp
TEST(SceneBufferPointCloud, PointCloudPointsSurviveAssignAfterSourceBufferDies) {
    std::vector<mpviz::PointCloudPoint> pts = {{{0,0,0},{255,0,0,255}}, {{1,0,0},{0,255,0,255}}};
    mpviz::PointCloud pc{};
    pc.points = pts.data(); pc.point_count = 2; pc.last_update_sec = 1.0;
    mpviz::SceneGraph s{}; s.point_clouds = &pc; s.point_cloud_count = 1;
    // set_scene(r, s);
    std::fill(pts.begin(), pts.end(), mpviz::PointCloudPoint{});  // free/overwrite caller's array
    // assert active().point_clouds[0].points still reads the ORIGINAL two points, unaffected
    // by the fill() above (proves the deep copy, not the caller's buffer, is what's read back)
}
```
  Run — FAIL (dangling/corrupted read). Implement. Run — PASS.
  **Closes Task 5's `layer_point_clouds` forward-reference here:** register the gate `if (!layer_point_clouds_) scene_asm_.point_clouds.clear();` in `visualization_node.cpp` alongside this new category's fill loop, mirroring the six gates Task 5 Step 1 already added for the other categories. **Task 6 owns**: this one gate line and the category's own `scene_asm_.point_clouds` vector it clears; **Task 5 already owns** (unchanged by this step) the param declaration, `SetParametersCallback` wiring, and `set_layers` WS plumbing. A test in this task's own adapter/node test surface asserts `layer_point_clouds=false` zeroes `point_cloud_count` for the next tick, the same shape as Task 5's per-category test.

- [ ] **Step 2: New vertex layout + material — decided here, why it's a fourth material and not a reuse.** The existing `Vertex` struct (`renderer_internal.hpp:73`) is `position + tangentFrame` with **no COLOR attribute** — verified, matching Epic 2's own finding that `extrude_polyline` output and gltfio meshes carry none, which is why `clay_faded.mat`'s `requires: [color]` was already unusable for anything but the grid. Point-cloud rendering needs **per-vertex baked RGBA8 color** (the backlog's own contract: "the adapter bakes rgba8 per point node-side"), which is a genuinely different vertex layout from anything in this library today. **Decision:** a new `PointVertex { float3 position; uint32_t rgba; }` (packed color, no tangent frame — points have no meaningful normal) and a new `assets/materials/point_cloud.mat`, **UNLIT** (a point cloud's per-point color IS the signal; PBR shading would just add a light-direction term nobody asked for), `requires: [color]` (the per-point RGB, baked once by the node-side adapter — never touched again during a fade), a single `{ type: float, name: alpha }` parameter, and **`blending: fade`**. That last part corrects an earlier draft of this step, which specified "no blending" and claimed the fade would work "at the vertex-buffer level like `clay_faded.mat`'s grid does" — verified false: `clay_faded.mat` ends `requires : [ color ], blending : fade`, and its own header comment explains that's precisely why its per-vertex alpha does anything (`fade`/`transparent` are what put a renderable in the blended queue at all; an opaque material discards fragment/vertex alpha outright). The cited precedent proves the opposite of what the earlier draft claimed, so `blending: fade` is carried over unchanged from both `clay_faded.mat` and `clay_translucent.mat`.

  Two follow-on decisions this forces, both resolved here rather than left implicit:
  1. **Alpha is a uniform, not a re-baked vertex attribute.** `clay_faded.mat`'s alpha is baked ONCE at grid-build time from static radial distance; `staleness_alpha()` (`renderer_internal.hpp`) recomputes every frame during the ~0.5 s fade ramp (`kStaleFadeStartSec`=0.5 → `kStaleFadeTimeoutSec`=1.0). Rebuilding the whole point-cloud vertex buffer every frame to re-bake alpha — on the largest geometry in the scene — is rejected outright. Instead the per-point `rgba` stays exactly as baked by the adapter (color only, no alpha channel needed there), and the fade is carried entirely by the material's `alpha` parameter.
  2. **One `MaterialInstance` for the whole layer, not a per-entity swap.** The plan's stated reason elsewhere for avoiding a `MaterialInstance` swap (per-entity overhead, e.g. `generic_markers.cpp`'s per-slot `clayTranslucentMaterial` instances) doesn't apply to a layer that fades as ONE unit: `PointCloudLayer` owns exactly one `point_cloud.mat` instance for its lifetime, and staleness is applied with a single `setParameter("alpha", staleness_alpha(...))` call per frame — no rebuild, no per-entity instancing, negligible cost regardless of point count.

  This is Epic 2's own established pattern ("three materials, three different vertex-layout/blend contracts, not duplication") extended to a fourth.
  Failing test, library-side:
```cpp
TEST(PointCloud, ChunkedUnderTheUint16IndexCeilingLikeEveryPolyline) {
    // a synthetic cloud > kMaxPointsPerMesh points -> multiple point-list
    // meshes, none truncated -- reuses polyline.hpp's existing
    // kMaxPointsPerMesh/polyline_chunks (POINTS primitives chunk by raw
    // point count, no 2x multiplier since a point is 1 vertex, not 2).
}
```
  Run — FAIL then PASS.

- [ ] **Step 3: Node-side field-detection + `color_mode` (auto|rgb|intensity|height|flat) — reusing the existing offset-scan pattern.** Verified prior art in this repo: `micropilot_rendering_node/src/rendering_node.cpp:448-462` already scans `PointCloud2::fields` for `FLOAT32` `x`/`y`/`z` offsets and `memcpy`s per-point using `msg->point_step` — the exact technique this adapter needs, extended to also look for `rgb`/`rgba`/`intensity` fields. Failing tests:
```cpp
TEST(PointCloudAdapter, AutoModePicksRgbWhenFieldPresent) { }
TEST(PointCloudAdapter, AutoModeFallsBackToIntensityRampWhenNoRgbFieldExists) { }
TEST(PointCloudAdapter, AutoModeFallsBackToHeightRampWhenNeitherRgbNorIntensityExists) { }
TEST(PointCloudAdapter, IntensityRangeAutoRangesWhenRowLeavesItUnset) { }
TEST(PointCloudAdapter, DecimationRespectsMaxPointsAndStride) { }
```
  Run — FAIL then PASS. Synthetic fixtures per the named fixture gap: three hand-built `PointCloud2` messages (rgb-carrying, intensity-only, bare-XYZ), since none exist in any recording.

- [ ] **Step 4: Goldens.** Three synthetic-scene goldens (one per `auto`-tier), human-promoted, `320×240`, `quality=1` — single-theme per the golden-scoping rule below (this category isn't a theming-sensitive golden per decision P3, so one theme suffices).

---

## Task 7 (VM-037): Epic-0 debt — mux hardening + build hygiene

**User decision (2026-09-07): VM-037 scheduled into this epic in full** (see "VM-037/VM-044 scope" above). Item (e) — `kSceneVersion` + node-side layout mirror — already shipped in Task 1 as that task's own prerequisite; this task covers the remaining eight sub-items (a-d, f-i) from the backlog's VM-037 entry (`visual-mode-backlog.md:177-179`). Every claim below was verified against the code as it stands today (Epic 2 closed at `fd72331`), not re-asserted from the backlog text.

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (mux-topic QoS on the `/rendering/set_mode` subscription, :431-432; `vcam_state[8] = mux_mode` on the `state.data` initializer, :513-518)
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp` (mux-topic QoS on `mux_mode_sub_`'s `/rendering/set_mode` subscription, :312-313; legacy `~/set_render_mode` handler gains a re-publish on `/rendering/set_mode`, :289-306; `initial_mode` also sets `render_mode_` in `on_configure()`, :37-43; `vcam_state[8] = mux_mode` on the `state.data` initializer, :632-636)
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/include/micropilot_rendering_node/rendering_node.hpp` (new publisher member for the legacy-topic re-publish, declared near `set_mode_sub_`/`mux_mode_sub_`, :187-194; update the `~/vcam_state` layout comment, :283-284, to document the new 9th element)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/CMakeLists.txt` (own `FILAMENT_VERSION` literal at :84 — single-source it from `cuda/src/libs/visual_renderer/cmake/GetFilament.cmake`; add an `ament_add_test(check_pod_header ...)` target in the `if(BUILD_TESTING)` block alongside the existing `ament_add_gtest` targets, :207 area — same block, same trap this epic's Task 1/2/3 Steps already restate: a target added here and left off the hand-written list is a silent no-op)
- Modify: `cuda/src/libs/visual_renderer/scripts/check_pod_header.sh` (glob `*.h` → `*.h*`, per the master plan's own already-recorded audit note, `visual-mode.md:98`)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (log `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` once, right after `filament::Engine::Builder()...build()` succeeds, :974-981)
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/test/smoke_test.py` (new cases: restart-rejoins-live-mode, legacy-topic-exits-mode-3, `initial_mode` bowl-stays-bowl, `vcam_state[8]`)
- New test: `cuda/src/libs/visual_renderer/tests/test_bluegl_link_probe.cpp` (or folded into `test_hello_frame.cpp`, implementer's call)

**Interfaces:** none in `scene.h` — this task is mux/build-hygiene only; no ADR-0004-governed change.

- [ ] **Step (e) — SHIPPED BY TASK 1, not repeated here.** `kSceneVersion` and the node-side `test_scene_layout.cpp` static_assert mirror are introduced in Task 1 Step 0 as that task's own prerequisite (ADR-0004 requires the mechanism at the epic's first post-ADR-0004 `scene.h` change, which is Task 1). No action in this task.

- [ ] **Step (a): mux QoS — `transient_local, depth 1, reliable` on every `/rendering/set_mode` publisher/subscriber.** **Verified reproduces:** both subscriptions use a bare integer-depth `create_subscription` call (default QoS — RELIABLE, VOLATILE, depth 10, no durability), not `transient_local` — `visualization_node.cpp:431-432` (`create_subscription<std_msgs::msg::Int32>("/rendering/set_mode", 10, ...)`) and `rendering_node.cpp:312-313` (`mux_mode_sub_`, identical pattern). A restarted/late-joining subscriber on a VOLATILE topic never receives the last-published mode, so it silently stays wherever `initial_mode` left it instead of rejoining the live mux state. Failing test first, extending `smoke_test.py`:
```python
def test_restart_rejoins_live_mode(test_node, ...):
    # drive /rendering/set_mode to 3 (visualization owns the stream);
    # kill and relaunch rendering_node with its default initial_mode (1);
    # assert rendering_node stays idle (no /rendering/image publish) until
    # a NEW mode message arrives -- i.e. it picked up the LAST global mode
    # (3) via transient_local late-join, not its own initial_mode default.
```
  Implement: `rclcpp::QoS(1).transient_local().reliable()` on both subscriptions, and on whatever publishes `/rendering/set_mode` (the AC names "every publisher/subscriber" — the mux value needs durability end to end for a late joiner to see it, including any launch-time/CLI publisher this smoke test or future tooling uses). Run — FAIL then PASS.

- [ ] **Step (b): legacy `~/set_render_mode` re-publishes on the global topic.** **Verified reproduces:** the `~/set_render_mode` handler (`rendering_node.cpp:289-306`) sets `render_mode_`/`active_mode_` locally and returns — it never publishes on `/rendering/set_mode`, and `rendering_node.cpp` declares no publisher for that topic at all (verified: no `create_publisher` for `/rendering/set_mode` anywhere in either node's `.cpp`). Selecting a local view via the legacy topic therefore never tells `micropilot_visualization_node` to stand down when it currently owns mode 3 — both nodes end up publishing. Failing test first:
```python
def test_legacy_topic_exits_mode_3_cleanly(test_node, ...):
    # drive /rendering/set_mode to 3; publish ~/set_render_mode = 1 on
    # rendering_node; assert rendering_node resumes publishing AND
    # visualization_node's frame stream stops -- the same
    # check_exclusive()-shaped assertion smoke_test.py's test_mode_mux
    # already uses for the global-topic-driven steps.
```
  Implement: add a `rendering_node`-owned publisher for `/rendering/set_mode` (transient_local, per Step (a)) and have the `~/set_render_mode` handler publish `1`/`2` on it whenever it fires, so `micropilot_visualization_node`'s existing `/rendering/set_mode` subscription (unchanged) observes the exit and stands down. Run — FAIL then PASS.

- [ ] **Step (c): `initial_mode:=1` also sets `render_mode_`.** **Verified reproduces:** `rendering_node.cpp`'s `on_configure()` (:37-43) sets `active_mode_ = initial_mode_` but never assigns `render_mode_`, which defaults to `2` (`rendering_node.hpp:189`, `int render_mode_{2}`); `timer_callback()`'s pointcloud gate (`rendering_node.cpp:770`, `if (cloud_sub_ && render_mode_ == 2)`) reads that stale default, so launching with `initial_mode:=1` (bowl-only) still renders the pointcloud-hybrid view. Failing test first:
```python
def test_initial_mode_one_starts_bowl_not_hybrid(test_node, ...):
    # launch rendering_node with -p initial_mode:=1; assert the rendered
    # view (or a param/diagnostic readback of render_mode_) is bowl (1),
    # not the pointcloud-hybrid default (2).
```
  Implement: `on_configure()` sets `render_mode_` from `initial_mode_` the same way the runtime `/rendering/set_mode` handler already does at `rendering_node.cpp:326-327` (`if (msg->data != 3) render_mode_ = msg->data`) — i.e. `render_mode_ = (initial_mode_ == 3) ? render_mode_ : initial_mode_;`, following the one rule the code already established rather than inventing a second one. Run — FAIL then PASS.

- [ ] **Step (d): `vcam_state[8] = mux_mode` on both nodes, index 7 unchanged.** **Verified reproduces:** both `state.data` initializers are exactly 8 elements today — `rendering_node.cpp:632-636` (`eye xyz, target xyz, active_preset, render_mode`, indices 0-7) and `visualization_node.cpp:513-518` (`eye xyz, target xyz, active_preset, active_mode`, indices 0-7); neither carries a value for "which node currently owns the global stream" distinct from the per-node `render_mode`/`active_mode` already at index 7. Failing test first (extends `smoke_test.py`'s existing `vcam_state[7]` check at :326-332):
```python
def test_vcam_state_carries_mux_mode_at_index_8(test_node, ...):
    # drive /rendering/set_mode through 1 -> 3 -> 2; assert
    # len(test_node.vcam_state) >= 9 and vcam_state[8] == the currently
    # active global mux mode, read from BOTH nodes' own ~/vcam_state stream.
```
  Implement: append one element to both `state.data` initializers, `static_cast<double>(active_mode_)` (the per-node mux-mode variable each node already tracks). Update the layout comment above each publisher (`rendering_node.cpp:283-284` and the equivalent in `visualization_node.hpp`) to document the new 9-element layout. Run — FAIL then PASS.

- [ ] **Step (f): `check_pod_header.sh` runs under colcon; glob widened to `*.h*`.** **Verified reproduces:** the script (`cuda/src/libs/visual_renderer/scripts/check_pod_header.sh`) globs `"$include_dir"/*.h` only — a public `.hpp` would escape the check, exactly as the master plan's own earlier audit note states (`visual-mode.md:98`). It is registered via `add_test(NAME check_pod_header ...)` (`cuda/src/libs/visual_renderer/CMakeLists.txt:220-221`) — a ctest of the **standalone** clang/libc++ library project, which colcon never builds or runs (verified: `cuda/scripts/ros_apps_build/colcon_build.sh` contains no reference to `check_pod`, `ctest`, or the `visual_renderer` directory; `micropilot_visualization_node/CMakeLists.txt` has no `ament_add_test` for it either). The build everyone actually runs (`colcon_build.sh`) never executes this check today. Verified today's `include/visual_renderer/` carries no stray `.hpp` (only `.h` files) — widening the glob is a forward safety net, not a currently-failing check on its own; the currently-failing half is reachability under colcon. Failing-test-first, in the sense of proving the new gate actually gates:
```cmake
# micropilot_visualization_node/CMakeLists.txt, if(BUILD_TESTING) block:
ament_add_test(check_pod_header
  COMMAND bash "${VISUAL_RENDERER_DIR}/scripts/check_pod_header.sh")
```
  Add the target; deliberately leak a `std::string` into `scene.h`; run `colcon test --packages-select micropilot_visualization_node` and confirm `check_pod_header` FAILS; revert the leak; confirm PASS. Widen the script's glob `*.h` → `*.h*` in the same change.

- [ ] **Step (g): `FILAMENT_VERSION` single-sourced.** **Verified reproduces:** `micropilot_visualization_node/CMakeLists.txt:84` hardcodes `set(FILAMENT_VERSION "1.56.5")`, duplicating `cuda/src/libs/visual_renderer/cmake/GetFilament.cmake:34`'s own `set(FILAMENT_VERSION "1.56.5")` — exactly the audit finding already recorded in the master plan (`visual-mode.md:207`: "after a pin bump in `cmake/GetFilament.cmake` the node would silently keep linking the stale... SDK"). No test (build-system plumbing, same class as this epic's other CMake-only steps). Implement: read `FILAMENT_VERSION` from `GetFilament.cmake` (e.g. a small `file(STRINGS GetFilament.cmake REGEX ...)` extraction, or `include()`-ing it if that does not re-trigger the fetch logic undesirably) instead of the node's own line-84 literal. Verified by: bump `GetFilament.cmake`'s version and confirm the node's `FILAMENT_ROOT` path composition follows without a second edit.

- [ ] **Step (h): log `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` once at `create_renderer()`.** **Verified — HALF of this backlog item is ALREADY SHIPPED:** `cuda/src/libs/visual_renderer/tests/test_hello_frame.cpp` already `GTEST_SKIP()`s when `HasGpuEglDevice()` returns false (that file's `HasGpuEglDevice()`/`GTEST_SKIP()` logic, verified present) — the "make `test_hello_frame` skip, not fail, when no hardware GL device is found" half of item (h) is done; this task does not redo it. **What still reproduces:** no `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` logging exists anywhere in the library (verified, zero grep hits under `cuda/src/libs/visual_renderer/src/`) — `HasGpuEglDevice()` only proves an EGL stack exists (Mesa llvmpipe passes it, per the master plan's own note, `visual-mode.md:184`), so a recorded budget number cannot be attributed to real hardware vs. a software rasterizer today. Failing test first:
```cpp
TEST(CreateRenderer, LogsGlVendorRendererVersionOnce) {
    // capture stderr (or a test hook exposing the last-logged strings)
    // around one create_renderer() call; assert GL_VENDOR, GL_RENDERER,
    // GL_VERSION each appear exactly once. GTEST_SKIP() per
    // HasGpuEglDevice(), same convention as HelloFrame.
}
```
  Implement: right after `filament::Engine::Builder()...build()` succeeds (`renderer.cpp:974-981`, `engine != nullptr`), call `glGetString(GL_VENDOR)`/`GL_RENDERER`/`GL_VERSION` (the GL context is current and bluegl is bound at this point — `HeadlessEglPlatform::createDriver()`, :222, already calls `bluegl::bind()` before `Engine::build()` returns) and log once, matching whatever minimal logging convention `renderer.cpp` already uses (the library is ROS-free — do not introduce a new logging mechanism for this one line). Run — FAIL then PASS.

- [ ] **Step (i): link-probe assertion on `bluegl::bind()`'s hand-declared signature.** **Verified reproduces:** `renderer.cpp:108-111` hand-declares `namespace bluegl { int bind(); void unbind(); }` at global scope so the linker resolves the real `::bluegl::bind()`/`unbind()` symbols (comment at :96-107 explains why — vendoring the real `BlueGL.h` would pull in thousands of generated macro lines this file never needs). Nothing checks that the hand-written declarations still match the real library's signature; per the master plan's own audit note (`visual-mode.md:184`), "the Itanium ABI does not mangle return types — an upstream change to `void`/`bool` would link silently and leave `if (bluegl::bind() != 0)` reading garbage." No test file references `bluegl` anywhere under `cuda/src/libs/visual_renderer/tests/` today (verified, zero hits). Failing test first, library-side (new `tests/test_bluegl_link_probe.cpp`, or folded into `test_hello_frame.cpp`):
```cpp
// A true compile-time signature check isn't possible without vendoring the
// real BlueGL.h (the whole point of hand-declaring was to avoid that), so
// this is a named RUNTIME probe: it exercises the exact call path
// renderer.cpp uses and asserts the result is consistent with the assumed
// signature, so a future bluegl upstream change is caught here explicitly
// instead of silently reading garbage inside create_renderer().
TEST(BlueglLinkProbe, BindReturnsZeroOnSuccessfulContextAndFrameRenders) {
    // create_renderer() (which calls bluegl::bind() internally) succeeds,
    // or GTEST_SKIPs per HasGpuEglDevice() (same convention as HelloFrame);
    // assert render_frame() produces real (non-garbage) pixels -- a
    // silently-mismatched calling convention would corrupt the GL function
    // table and most likely crash or produce an all-black frame, not a
    // clean nonzero return, so this test names the assumption explicitly
    // rather than relying on HelloFrame to catch it as a side effect.
}
```
  Run — PASS (this formalizes an already-implicitly-exercised path as an explicit, named regression test, per the AC's "link-probe assertion" wording).

**Commit message line:** `fix(visual-mode): VM-037 mux hardening + build hygiene (transient_local mode topic, legacy-topic republish, initial_mode/render_mode_ sync, vcam_state[8], colcon-wired POD check, single-sourced FILAMENT_VERSION, GL_RENDERER logging, bluegl link probe)`

**Backlog claim verified, one correction:** every sub-item (a)-(d) and (f)-(i) reproduces exactly as the backlog states against today's code — Epics 1-2 fixed none of them as a side effect. The one correction: item (h) is only **half** outstanding — `test_hello_frame.cpp` already implements the "skip, not fail, on no GPU/EGL device" half via `HasGpuEglDevice()`/`GTEST_SKIP()`; only the `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` logging half is still missing, and is what this task's Step (h) ships.

---

## Golden scoping (P3, restated precisely for this epic's tasks)

Spec §10's original "goldens per theme" is scoped, per the 2026-09-07 user decision, to **goldens whose subject is theming** — empty world, map/lane paint, one lit-geometry reference. Applied here:
- **Task 1's `map_ego_offset_{dark_adas,light_clay}.png`** stays a **both-themes** golden — it is exactly the theming-sensitive category the P3 exception names, and this epic changes its content substantially (kind coloring, road fill).
- Every other new/changed golden in this epic (HUD composite, callouts, three PointCloud tiers) ships **one theme** (`dark_adas`, matching every non-map category's existing convention from Epic 2), per the rule Epic 2 already established and P3 confirmed.
- No task in this epic re-shoots a golden outside what its own steps name; VM-034's staleness-fade tests assert through material-parameter hooks (per Epic 2's own established convention for fade tests), not new goldens.

---

## Review gate

Opus reviewer signs off against:

- **ADR-0004 held.** `git diff fd72331 -- include/visual_renderer/` shows only appended struct fields, appended enum values, and appended free functions — nothing renamed, reordered, or removed. `kSceneVersion` is present, introduced at 1 in Task 1 and bumped to 2 in Task 6, and both the library's and the **new node-side** `sizeof`/`offsetof` static_assert tables agree, on both toolchains.
- **VM-037/VM-044 scope boundary respected.** Only VM-037(e) (kSceneVersion + node-side layout mirror) was taken; no mux-QoS/legacy-topic/`initial_mode`/`vcam_state[8]`/`FILAMENT_VERSION`/GL-logging/bluegl work appears anywhere in this epic's diff. No asset-installation/`ament_index` work (VM-044) was attempted; every per-user-path gap this epic's own tasks touch (`hud_font_path`) is named as a stated deviation pointing at VM-044, not silently patched.
- **Map rendering is visually differentiated and matches the reference images' value separation** (`assets/visualization-reference-1.jpg`/`-2.jpg`) — centerline vs boundary vs crosswalk vs road surface are distinguishable by color AND the road reads darker than the surrounding ground in both themes.
- **Crosswalk hatching fires on real recorded geometry**, not just a hand-built 4-point quad — the reviewer re-derives the same 5-point-closed-polyline fact from the fixture independently, not from this document's claim of it.
- **Road-surface fill is correct on the mismatched-point-count lanes** (955/813 in the fixture, or an equivalent hand-authored case if the committed fixture doesn't carry them) — the reviewer must confirm the resample-based fix, not a same-index zip that happens to work on the 14 matching lanes.
- **Dashing is renderer-side and the adapter-side chop is fully deleted** — `git grep -n dashed -- cuda/src/ros_apps` returns nothing EXCEPT the three sanctioned prose hits recorded in Task 1's 2026-09-08 addendum (urban_profile.yaml + hd_map.cpp + test_hd_map_adapter.cpp comments, two of which quote the user directive verbatim); no `dashed` field, param, or flag exists anywhere. `git grep -n ChopIntoDashes` returns nothing outside `map_elements.cpp`.
- **The HD-map staleness deviation is closed** — Epic 2's "STATED DEVIATION" comment in `map_elements.cpp` is gone, replaced by the same `staleness_alpha`/`clay_translucent.mat` mechanism every other category uses; `git grep -n clay_translucent -- src/map_elements.cpp` is non-empty.
- **The ego-invalid map cosmetic is verifiably fixed**, ideally re-checked against the original gate transcript `wf_0ff03eb8-5ec` if the reviewer can access it, and at minimum via the new `EgoInvalidFadesMapElements...` test.
- **`render_ms` is real and only populated in mode 3**; diagnostics carries every `AdapterStats` field with `dropped_by_rule` still separate from `dropped_malformed` (Epic 2's own rule, unregressed).
- **No new font pipeline in `visual_renderer`** — `git grep -n stbtt_ -- cuda/src/libs/visual_renderer` is empty; every `stbtt_*` call is under the node package, and its own `stb_truetype.h` vendoring is independent of the library's `stb_image*` vendoring (separate `file(DOWNLOAD)` block, separate pinned commit).
- **`Hud::chips`/`AlertChip` remain unpopulated by design**, and the code says so where a future reader would otherwise wonder.
- **`project_to_screen()` is the only new library entry point VM-031 needed** — no second callout-specific rendering path exists.
- **No `set_quality()` entry point was added**; the WS/GUI plumbing for it is honestly labeled "takes effect on next restart."
- **Layer visibility needs zero renderer changes** — `git diff` under `src/` for Task 5 is empty or near-empty; the mechanism lives entirely in `visualization_node.cpp`'s existing `SceneAssembly` gate point.
- **PointCloud gets its own material and vertex layout**, not a forced reuse of the position+tangent-frame `Vertex` or an edit to any existing `.mat`; chunking reuses `polyline.hpp`'s `kMaxPointsPerMesh`/`polyline_chunks`, not a new ceiling constant.
- **Every `.mat` step (Task 6's `point_cloud.mat`) re-ran `cmake -B build -S .`** — Epic 2's own documented CMake trap (no `CONFIGURE_DEPENDS` on the material glob) applies unchanged.
- **Every node-side hand-written-source-list trap was respected** — every new `.cpp` in `micropilot_visualization_node` appears in `CMakeLists.txt`'s explicit list AND every gtest target that needs it; `git grep -n "src/diagnostics.cpp\|src/hud_overlay.cpp\|src/callouts.cpp\|src/adapters/point_cloud.cpp" -- CMakeLists.txt` in that package shows each file at least twice (once in the library source list, once per relevant test target).
- **Golden scoping (P3) followed exactly**: map stays both-themes; nothing else in this epic ships two themes without a named reason.
- **Every named fixture gap is named in the artifact** (profile YAML comments, adapter code comments), not just in this document.
- **No task exceeded scope**: no live quality switching (Epic 5), no asset-install/`ament_index` resolution (VM-044), no environment layer (Epic 4).
- **Task 7 (VM-037) is complete and bounded.** All eight of its remaining sub-items (a-d, f-i) ship — `git grep` for `transient_local` on both `/rendering/set_mode` subscriptions is non-empty; `~/set_render_mode`'s handler republishes on the global topic; `rendering_node`'s `on_configure()` sets `render_mode_` from `initial_mode_` (not just `active_mode_`); both nodes' `~/vcam_state` arrays carry 9 elements with `[8]` == the mux mode; `check_pod_header` runs as an `ament_add_test` reachable from `colcon test --packages-select micropilot_visualization_node`, glob widened to `*.h*`; `FILAMENT_VERSION` appears as a `set()` literal in exactly one place (`GetFilament.cmake`), not two; `create_renderer()` logs `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` once; a named `bluegl` link-probe test exists. Item (e) is confirmed already shipped by Task 1, not re-done. No item's fix reaches into Tasks 1-6's own files (`scene.h`, themes, adapters, HUD/callouts/layers/quality/PointCloud code) beyond the CMakeLists/script/renderer.cpp lines this task's own Files list names.

## Epic 3 results (fill at close)

- Golden SSIM threshold used: ____
- Map-fill re-shoot: dark_adas / light_clay both promoted? ____
- Road-surface fill: lanes with mismatched rail counts observed in the final committed fixture: 813 (left 10 / right 11), 955 (left 8 / right 9)
- `render_ms` measured range across the three quality presets (three-launch comparison, Task 5 Step 2): low ____ ms / medium ____ ms / high ____ ms, on ____ (hardware)
- `wf_0ff03eb8-5ec` cosmetic: confirmed fixed by re-render? ____ / could not access transcript, fixed by mechanism only: ____
- CC0 font pack used for VM-030: ____ (license, URL/version pinned in an ATTRIBUTION file, mirroring Epic 2's `assets/models/ATTRIBUTION.md` convention)
- Fixture gaps still open at close: PointCloud2 (synthetic only, unchanged) — ____; any new gap discovered during implementation: ____
- Any VM-037/VM-044 item the reviewer found this epic actually needed beyond the one slice taken: ____
- Task 7 (VM-037): all eight sub-items (a-d, f-i) shipped? ____ / any deferred, and why: ____; `GL_RENDERER`/`GL_VENDOR` string observed on the dev box at close: ____
