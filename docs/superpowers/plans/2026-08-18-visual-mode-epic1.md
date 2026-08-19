# Visual Mode — Epic 1 Implementation Plan (Core scene & dark theme)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18):** run this epic as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Parent plan:** `docs/superpowers/plans/2026-08-18-visual-mode.md` (Global Constraints bind this doc too — re-read them before starting; not repeated in full here).
**Spec:** `docs/superpowers/specs/2026-08-18-visual-mode-design.md` §4.1/4.3/4.4/9/10.
**Backlog:** `docs/superpowers/specs/2026-08-18-visual-mode-backlog.md` (Epic 1: VM-010, VM-011, VM-014, VM-012, VM-013).
**Prerequisite:** Epic 0 done and review-approved (commits `3cb10ef..e619d41`). `FILAMENT_VERSION = 1.56.5` (pinned, see `cuda/src/libs/visual_renderer/README.md`). Task 6 (on-robot GPU budget) numbers are **pending** — see "Conservative perf assumptions" below.

**Goal:** Replace Epic 0's throwaway hello-frame scene (unlit cube+grid) with the real Epic-1 baseline: a `SceneGraph` domain model with double-buffered freeze-frame semantics and per-entity staleness fade, a data-driven theme system with both shipped themes on a genuinely **lit** clay pipeline (sun + IBL, not Epic 0's unlit deviation), an animated `set_theme` transition, and the ego robot rendered from TF with a smoothed finite-differenced speed. `set_scene`/`set_theme`/the `SceneGraph` POD layout are **frozen for all later epics** after this one (Epic 2 populates the categories Epic 1 leaves empty).

## Known Epic 0 deviation this epic must fix

`cuda/src/libs/visual_renderer/src/renderer.cpp` currently renders with `simple_color.mat`, an **unlit** material (`shadingModel: unlit`, one flat `baseColor` param) — chosen in Epic 0 purely to tell ground/grid/cube apart, since `Engine::getDefaultMaterial()` ignores scene lighting entirely. The sun (`LightManager::Type::SUN`) and the flat-SH "ambient" `IndirectLight` are already built into the scene graph today but **have no visible effect** on an unlit material — dead code. Task 2 below replaces `simple_color.mat` with a real lit clay material and wires the sun + a real (if intentionally simplified — see Task 2) IBL through it, so lighting actually does something for the first time.

## Conservative perf assumptions

Epic 0 Task 6 (on-robot GPU budget measurement) has not run yet. Per project direction: default quality preset = **medium**, target resolution 1280×720@30. Nothing in this epic should be tuned against unmeasured numbers — goldens render at a fixed 320×240 purely for CI speed/determinism, not as a statement about the shipped default. **Stated precisely, not aspirationally: `RenderConfig::quality` is consumed by the renderer for exactly two toggles as of this epic — SSAO on/off+resolution and FXAA-vs-TAA (Task 2 Step 7a) — and nothing else.** Shadow-map resolution and render-resolution scaling (the other two knobs in spec §8's preset table) are still unmapped; that split is VM-032's job (Epic 3, "quality presets end to end") to finish. This isn't a scope creep: goldens are committed in this epic and freeze whatever the post chain does, so SSAO/AA had to be decided now (Task 2 Step 7a) the same way the ego's shadow-cast flags did — shadow *resolution* and render *resolution*, unlike SSAO/AA, don't change what a fixed-320×240, no-shadow-casting-object-yet golden looks like, so those two stay deferred without forcing an early decision. Goldens in this epic are captured at `quality=1` (medium: FXAA + SSAO half-res), matching the stated shipped default above. Do not gold-plate perf work here; Epic 5 (VM-040/041) owns hysteresis/auto-drop/benchmarking.

---

## Interfaces frozen this epic

Everything below is **locked** once Epic 1's review gate passes; Epic 2+ may only *add* new optional fields/entry points, never change these signatures or reorder these struct members (a CI-checked POD-layout snapshot test, added in Task 1, catches accidental breakage).

### `include/visual_renderer/scene.h` (new; extends `api.h`, still POD-only)

Spec §4.1 lists 8 scene categories. All 8 are frozen as POD structs now; Epic 1 only ever populates `ego` (`object_count == 0`, `path_count == 0`, etc. for everything else) — Epic 2 fills the rest without touching this header again.

```c
// scene.h — POD boundary, same rules as api.h (checked by the same
// check_pod_header.sh, extended to glob include/visual_renderer/*.h).
#pragma once
#include <cstdint>
#include <cstddef>
#include "visual_renderer/api.h"

namespace mpviz {

struct Vec3 { double x, y, z; };

enum class ObjectClass : uint8_t {
    CAR = 0, TRUCK_VAN = 1, BUS = 2, PEDESTRIAN = 3, CYCLIST = 4, UNKNOWN = 5
};
enum class PathRole : uint8_t { BEHAVIOR = 0, GLOBAL = 1, LOCAL = 2 };
enum class MarkerPrimitive : uint8_t {
    CUBE = 0, SPHERE = 1, CYLINDER = 2, ARROW = 3, LINE_STRIP = 4,
    LINE_LIST = 5, POINTS = 6, TEXT = 7, TRIANGLE_LIST = 8, MESH = 9
};

// ── Ego (Epic 1 populates this) ─────────────────────────────────────────────
struct EgoState {
    Vec3 position;        // map frame
    double heading_rad;   // yaw about +Z, map frame
    double speed_mps;     // finite-differenced from TF, smoothed (VM-012)
    uint8_t valid;        // 0 = no TF yet -> ego hidden, not a clay box at origin
};

// ── TrackedObject[] (Epic 2: VM-021/022) ────────────────────────────────────
struct TrackedObject {
    uint32_t id;
    ObjectClass cls;
    Vec3 position;
    double heading_rad;
    Vec3 dimensions;              // length(x)/width(y)/height(z), meters
    Vec3 velocity;                // map frame, m/s (velocity arrow, spec §4.1)
    const Vec3* predicted_path;   uint32_t predicted_path_count;
    const char* label;            // nul-terminated, caller-owned, may be nullptr
    double last_update_sec;       // SceneGraph::sim_time_sec at last refresh (staleness)
};

// ── PathRibbon[] (Epic 2: VM-023) ───────────────────────────────────────────
struct PathRibbon {
    PathRole role;
    const Vec3* points;  uint32_t point_count;
    double last_update_sec;
};

// ── MapElement[] (Epic 2: VM-024) ───────────────────────────────────────────
struct MapElement {
    const Vec3* points;  uint32_t point_count;
    uint8_t is_polygon;  // 0 = polyline (lane centerline), 1 = polygon (crosswalk)
};

// ── GroundGrid (OGM layers; Epic 2: VM-025) ─────────────────────────────────
// Spec §4.1 names this category "GroundGrid" (singular); §4.2/§5 says there
// are 2 today (dynamic + gradient OGM) but nothing bounds it to exactly 2 —
// modeled as an array like every other category for the same reason
// GenericMarker exists: tomorrow's third OGM topic is a profile-YAML row,
// not a struct change.
struct GroundGridLayer {
    uint8_t kind;            // 0 = dynamic OGM, 1 = gradient OGM
    Vec3 origin;             // map-frame position of cell (0,0)
    double resolution_m;     // meters per cell edge
    uint32_t width_cells, height_cells;
    const uint8_t* cells;    // width*height, row-major, caller-owned
    double last_update_sec;
};

// ── AlertPolygon[] (Epic 2: VM-026) ─────────────────────────────────────────
struct AlertPolygon {
    const Vec3* points;  uint32_t point_count;
    uint8_t severity;    // 0 info / 1 warning / 2 critical -> theme alert ramp
    double last_update_sec;
};

// ── GenericMarker[] (Epic 2: VM-027, the parity-guarantee fallback) ─────────
struct GenericMarker {
    MarkerPrimitive primitive;
    Vec3 position;  double heading_rad;  Vec3 scale;
    const Vec3* points;  uint32_t point_count;   // LINE_*/POINTS/TRIANGLE_LIST
    const char* text;       // TEXT only, else nullptr
    const char* mesh_path;  // MESH only, else nullptr
    float color[4];         // rgba; theme-neutral default if alpha == 0
    double last_update_sec;
};

// ── Hud (Epic 1: speed+mode only; chips arrive with VM-031) ─────────────────
struct AlertChip {
    const char* text;
    Vec3 anchor;   // map-frame 3D anchor for the leader line (VM-031)
};
struct Hud {
    double speed_mps;
    uint8_t active_mode;     // 1|2|3, mirrors ~/vcam_state
    const AlertChip* chips;  uint32_t chip_count;
};

// ── The root struct ──────────────────────────────────────────────────────────
struct SceneGraph {
    // The single clock all staleness-fade and theme-transition math is
    // computed against (Task 1/3). Caller-supplied, monotonic seconds —
    // never wall-clock-read inside the library, so tests are deterministic.
    double sim_time_sec;

    EgoState ego;
    const TrackedObject*   objects;      uint32_t object_count;
    const PathRibbon*      paths;        uint32_t path_count;
    const MapElement*      map_elements; uint32_t map_element_count;
    const GroundGridLayer* grids;        uint32_t grid_count;
    const AlertPolygon*    alerts;       uint32_t alert_count;
    const GenericMarker*   markers;      uint32_t marker_count;
    Hud hud;
};

// Deep-copies `scene` (and everything its pointers reach) into the renderer's
// internal staging buffer (mpviz::detail::SceneBuffer) and atomically swaps
// which slot is "active". This call touches ONLY that internal buffer — no
// Filament::Engine/Scene/TransformManager/RenderableManager call happens
// here. `scene`'s arrays may be freed/reused the instant this call returns.
//
// Threading (stated precisely, not aspirationally): TODAY THIS IS
// SINGLE-THREADED END TO END — the same single-threaded executor calls both
// set_scene() and render_frame(), from the same thread. SceneBuffer's mutex
// (Task 1) only serializes the active-slot-INDEX read/write; it does NOT
// make set_scene() safe to call from a second thread while render_frame()
// holds a reference from active() across a frame. active() hands back a
// bare `const SceneGraph&` aliased directly into slot storage — a second
// publisher calling set_scene() while a reader still holds that reference
// would overwrite the very slot being read (two publishes between a
// reader's first and last touch of the reference is enough). Do not call
// set_scene() from any thread other than the one calling render_frame()
// until that changes. A future multi-threaded ingest path needs
// SceneBuffer::active() to hand back an owned/refcounted snapshot instead
// (or an equivalent lifetime guarantee) — that is a SceneBuffer design
// change, not a locking tweak, and is out of scope for this epic; the mutex
// here is a cheap hook for that future work, not a guarantee it already
// provides.
// Does NOT render, and does NOT touch Filament: render_frame() always
// re-derives everything Filament-side (ego transform, material params,
// theme blend) from the last-published active() scene, on whichever thread
// owns the Engine — so a tick with no set_scene() call re-renders the
// previous one (freeze-frame, same philosophy as micropilot_rendering_node),
// and all Engine/component calls still happen from a single thread as
// Filament requires.
void set_scene(VisualRenderer*, const SceneGraph& scene);

}  // namespace mpviz
```

### `set_theme` (new in `scene.h`, alongside `set_scene`)

```c
// Eases every themed token (palette, material roughness/metallic, sun/IBL,
// HUD colors) from whatever is currently active toward `theme_name`,
// starting at `at_sec` (a SceneGraph::sim_time_sec value — the caller's
// clock, never wall-clock) and completing after `transition_sec` seconds
// (0.0 -> use the spec default, 0.8s). Smoothstep-eased, Oklab-space color
// lerp (Task 3). A transition retargeted mid-flight (set_theme called again
// before the first finishes) starts a new ease from the CURRENT blended
// state, not from either endpoint — no visible snap.
// Returns false (no-op) if theme_name doesn't match a loaded
// assets/themes/<name>.yaml stem; the active theme is unchanged.
bool set_theme(VisualRenderer*, const char* theme_name, double at_sec,
                double transition_sec);
```

### `RenderConfig` gains two optional fields (`api.h`, additive — appending two trailing pointer fields to an aggregate is source-compatible with every existing construction style: `RenderConfig{w,h,q}` positional-init still compiles, with the two new members value-initialized to `nullptr`. The two real callers, `visualization_node.cpp` and `examples/hello_frame.cpp`, both build via `RenderConfig config{};` + member assignment and need no change at all.)

**Lifetime (stated explicitly — this is the same rule every other `const char*` this plan adds across the POD boundary already needs, but `RenderConfig` didn't have it written down):** both new fields are caller-owned and borrowed only for the duration of the `create_renderer(const RenderConfig&)` call they're passed to. `create_renderer` must copy each into internal `std::string` storage on `VisualRenderer` before returning, and must never retain the raw pointer past that call. This isn't academic: `set_theme` (Task 3) calls `load_theme(theme_assets_dir, name)` again at runtime on every future `~/set_theme` request, long after `create_renderer` returned, and the node (Task 3 Step 9 / Task 4) will construct `theme_assets_dir` from a ROS parameter's `std::string::c_str()` — a temporary whose backing storage does not outlive the `create_renderer` call. Task 2 Step 7 implements the copy-in. (Contrast `scene.h`'s `set_scene`, which already states its deep-copy contract for `const char*` fields like `label`/`text`/`mesh_path`; `RenderConfig` gets the same rule here, spelled out for the first time.)

```c
struct RenderConfig {
    uint32_t width, height;
    uint8_t quality;             // 0=low, 1=med, 2=high
    const char* theme_assets_dir;  // nullable: dir containing *.yaml theme files;
                                    // caller-owned, borrowed only for this call —
                                    // create_renderer copies it into internal
                                    // string storage owned by VisualRenderer
                                    // (Task 2 Step 7)
    const char* initial_theme;     // nullable: default "dark_adas"; same
                                    // borrow-then-copy rule as theme_assets_dir
};
```

### Ego model loading (new in `scene.h`)

```c
// Loads a glTF/GLB ego mesh from `gltf_path` for use whenever
// SceneGraph::ego.valid is true. Non-fatal on failure (bad path, unsupported
// glTF feature, missing file): logs nothing itself (POD boundary — no
// logging crosses it), returns false, and rendering falls back to a clay
// box sized by `fallback_dims` (meters, length/width/height). Call once,
// typically from on_configure(), before the first render_frame().
bool set_ego_model(VisualRenderer*, const char* gltf_path, Vec3 fallback_dims);
```

---

## Task 1 (VM-010): SceneGraph + double buffer + staleness clocks

**Files:**
- Create: `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h` (POD, frozen above)
- Create: `cuda/src/libs/visual_renderer/src/scene_buffer.hpp` (internal, not installed — the double-buffer + deep-copy owner types; std:: usage is fine, it's `.hpp` under `src/`, never shipped)
- Create: `cuda/src/libs/visual_renderer/src/scene_buffer.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/test_scene_buffer.cpp`
- Modify: `cuda/src/libs/visual_renderer/scripts/check_pod_header.sh` (glob `include/visual_renderer/*.h`, not just `api.h`)
- Modify: `cuda/src/libs/visual_renderer/CMakeLists.txt` (give test binaries `-I src` so `test_scene_buffer.cpp` can include the internal `scene_buffer.hpp`)

**Interfaces:** freezes `mpviz::set_scene`'s signature/contract (above) and builds the internal `mpviz::detail::SceneBuffer` class it wraps — Task 1 does NOT define `set_scene()` itself (no `renderer.cpp` touch in this task's Files list, deliberately: `VisualRenderer`/`renderer.cpp` don't exist as a `SceneBuffer`-aware type yet). Task 2 (Step 7) adds a `SceneBuffer` member to `VisualRenderer` and implements `set_scene()` as a thin `publish()` call over it; Task 4 only reads `SceneBuffer::active()` from `render_frame()` for ego transforms, it does not touch `set_scene()` either.

- [ ] **Step 1: Extend the POD check.** `check_pod_header.sh` currently hardcodes `api.h`'s path; change it to `for hdr in "$(dirname "$0")/../include/visual_renderer/"*.h`. Run it now — passes trivially (scene.h doesn't exist yet, glob matches only `api.h`).
- [ ] **Step 2: Write `scene.h`** verbatim per the frozen block above. Run `check_pod_header.sh` — must still PASS (every field is POD/fixed-width/raw-pointer; this is the actual enforcement of the freeze, not just documentation).
- [ ] **Step 3: Failing test — deep copy + freeze-frame.** In `test_scene_buffer.cpp`:

```cpp
#include "scene_buffer.hpp"   // -I src, internal header
#include <gtest/gtest.h>

TEST(SceneBuffer, StageThenActive_ReflectsLastPublishedScene) {
    mpviz::detail::SceneBuffer buf;
    mpviz::SceneGraph g{};
    g.sim_time_sec = 1.0;
    g.ego = {{1, 2, 3}, 0.5, 4.2, /*valid=*/1};
    buf.publish(g);
    const mpviz::SceneGraph& active = buf.active();
    EXPECT_EQ(active.ego.valid, 1);
    EXPECT_DOUBLE_EQ(active.ego.position.x, 1.0);
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerBufferReuse) {
    mpviz::detail::SceneBuffer buf;
    std::vector<mpviz::TrackedObject> objs(1);
    objs[0].id = 7;
    mpviz::SceneGraph g{};
    g.objects = objs.data();
    g.object_count = 1;
    buf.publish(g);
    objs[0].id = 999;  // caller mutates its own buffer after publish() returns
    EXPECT_EQ(buf.active().objects[0].id, 7u);  // must have been deep-copied
}

TEST(SceneBuffer, DeepCopy_SurvivesCallerNestedBufferReuse) {
    // The shallow-copy trap: objs[0].id above lives directly in the
    // TrackedObject struct, so even a flat memcpy-style copy passes the test
    // above. predicted_path/label are pointers INTO caller-owned storage the
    // struct doesn't own — this is the case that actually exercises "deep".
    mpviz::detail::SceneBuffer buf;
    std::vector<mpviz::Vec3> path = {{1, 1, 0}, {2, 2, 0}};
    std::string label = "car-42";
    std::vector<mpviz::TrackedObject> objs(1);
    objs[0].id = 7;
    objs[0].predicted_path = path.data();
    objs[0].predicted_path_count = static_cast<uint32_t>(path.size());
    objs[0].label = label.c_str();
    mpviz::SceneGraph g{};
    g.objects = objs.data();
    g.object_count = 1;
    buf.publish(g);

    // Caller frees/overwrites its nested buffers after publish() returns —
    // set_scene's frozen contract requires the copy to have taken its own
    // storage for these, not just for the flat TrackedObject array.
    path.assign(2, mpviz::Vec3{-9, -9, -9});
    label.assign("OVERWRITTEN");

    const mpviz::TrackedObject& active_obj = buf.active().objects[0];
    ASSERT_EQ(active_obj.predicted_path_count, 2u);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[0].x, 1.0);
    EXPECT_DOUBLE_EQ(active_obj.predicted_path[1].x, 2.0);
    ASSERT_NE(active_obj.label, nullptr);
    EXPECT_STREQ(active_obj.label, "car-42");
}

TEST(SceneBuffer, NoNewPublish_KeepsPreviousActiveScene) {
    mpviz::detail::SceneBuffer buf;
    mpviz::SceneGraph g{};
    g.sim_time_sec = 5.0;
    buf.publish(g);
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // freeze-frame: reading
    EXPECT_DOUBLE_EQ(buf.active().sim_time_sec, 5.0);  // again changes nothing
}
```
  Run — FAIL (link error, `scene_buffer.hpp`/`.cpp` don't exist).
- [ ] **Step 4: Implement `SceneBuffer`.** Deep-copy owner types (`std::vector`-backed, internal-only — never crosses the POD boundary):

```cpp
// scene_buffer.hpp
#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "visual_renderer/scene.h"

namespace mpviz::detail {

// Owns std::vector storage for every array SceneGraph points into, so a
// mpviz::SceneGraph handed out by active() has valid pointers for as long as
// this object isn't republished. NOT itself passed across the POD boundary —
// internal only.
//
// Every category struct in scene.h has, in addition to its own flat array,
// pointers into further caller-owned buffers (TrackedObject::predicted_path/
// label, PathRibbon::points, MapElement::points, GroundGridLayer::cells,
// AlertPolygon::points, GenericMarker::points/text/mesh_path,
// AlertChip::text). A flat std::vector<TrackedObject> copy alone still
// leaves those inner pointers aimed at the CALLER's memory — exactly the use-
// after-free the frozen set_scene contract ("scene's arrays may be freed/
// reused the instant this call returns") promises can't happen. So each
// category gets one parallel "nested storage" vector alongside its flat
// vector, and assign() repoints every entry's pointer fields at its own copy
// after copying.
struct OwnedScene {
    mpviz::SceneGraph view{};  // pointers below point into this object's own vectors
    std::vector<TrackedObject> objects;
    std::vector<std::vector<Vec3>> object_paths;   // objects[i].predicted_path storage
    std::vector<std::string> object_labels;        // objects[i].label storage
    std::vector<PathRibbon> paths;
    std::vector<std::vector<Vec3>> path_points;    // paths[i].points storage
    std::vector<MapElement> map_elements;
    std::vector<std::vector<Vec3>> map_element_points;
    std::vector<GroundGridLayer> grids;
    std::vector<std::vector<uint8_t>> grid_cells;  // grids[i].cells storage
    std::vector<AlertPolygon> alerts;
    std::vector<std::vector<Vec3>> alert_points;
    std::vector<GenericMarker> markers;
    std::vector<std::vector<Vec3>> marker_points;
    std::vector<std::string> marker_texts;         // "" stored for a nullptr text
    std::vector<std::string> marker_mesh_paths;    // "" stored for a nullptr mesh_path
    std::vector<AlertChip> chips;
    std::vector<std::string> chip_texts;
    // Deep-copies `src` — including every nested Vec3[]/uint8_t[]/char*
    // payload reached by the arrays above — into this object's vectors, and
    // repoints view's pointers (both the top-level array pointers AND each
    // entry's own nested pointer fields) at the copies. Pattern, shown once
    // for objects[]/predicted_path — every other category follows the same
    // shape (resize the nested-storage vector to match count, copy element-
    // by-element, repoint):
    //
    //   objects.assign(src.objects, src.objects + src.object_count);
    //   object_paths.resize(src.object_count);
    //   object_labels.resize(src.object_count);
    //   for (uint32_t i = 0; i < src.object_count; ++i) {
    //       const TrackedObject& s = src.objects[i];
    //       object_paths[i].assign(s.predicted_path, s.predicted_path + s.predicted_path_count);
    //       objects[i].predicted_path = object_paths[i].data();
    //       object_labels[i] = s.label ? s.label : "";
    //       objects[i].label = s.label ? object_labels[i].c_str() : nullptr;
    //   }
    //   view.objects = objects.data();
    //   // ... repeat for paths[].points, map_elements[].points,
    //   // grids[].cells, alerts[].points, markers[].points/text/mesh_path,
    //   // chips[].text.
    void assign(const mpviz::SceneGraph& src);
};

class SceneBuffer {
public:
    void publish(const mpviz::SceneGraph& scene);          // deep-copy + atomic swap
    const mpviz::SceneGraph& active() const;                // last-published scene
    // Fade-out multiplier in [0,1] for an entity last touched `last_update_sec`
    // ago relative to `now_sec`: 1.0 while younger than fade_start_sec, ramps
    // to 0.0 by timeout_sec, 0.0 beyond. One function, every stale-able
    // category (TrackedObject, PathRibbon, GroundGridLayer, AlertPolygon,
    // GenericMarker) calls it the same way — no per-category branches.
    static float staleness_alpha(double now_sec, double last_update_sec,
                                  double fade_start_sec, double timeout_sec);

private:
    mutable std::mutex mutex_;   // ponytail: cheap at this call rate (<=30 Hz);
                                  // serializes active_idx_ only. Today's
                                  // single-threaded executor never contends
                                  // it. It does NOT by itself make
                                  // multi-threaded ingest safe — active()
                                  // still hands back a bare reference aliased
                                  // into slot storage, so a second publisher
                                  // could overwrite a slot a reader still
                                  // holds. See set_scene()'s corrected
                                  // threading contract in scene.h: real
                                  // multi-threaded ingest needs active() to
                                  // return an owned/refcounted snapshot, a
                                  // SceneBuffer redesign this epic does not
                                  // attempt.
    OwnedScene slots_[2];
    int active_idx_{0};
};

}  // namespace mpviz::detail
```
  `publish()`: `assign()` into `slots_[1 - active_idx_]`, then swap `active_idx_` under `mutex_`. `active()`: lock, read `active_idx_`, return `slots_[active_idx_].view` (the lock only protects the index read/write, not the whole render — safe ONLY because today's single-threaded executor is both the sole publisher and the sole reader, so nothing mutates a slot while it's active; this stops holding the moment a second thread calls `publish()` while a reader still holds an `active()` reference across a frame — see the corrected threading note on `set_scene()` above, and treat that as a future redesign, not something to silently patch here).
- [ ] **Step 5: Run — PASS.**
- [ ] **Step 6: Failing test — staleness math**, table-driven:

```cpp
TEST(StalenessAlpha, FreshIsFullyOpaque) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(10.0, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, BeforeFadeStartIsFullyOpaque) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(10.4, 10.0, 0.5, 2.0), 1.0f);
}
TEST(StalenessAlpha, MidFadeIsInterpolated) {
    // age=1.25s, fade_start=0.5s, timeout=2.0s -> 50% through the fade window
    EXPECT_NEAR(mpviz::detail::SceneBuffer::staleness_alpha(11.25, 10.0, 0.5, 2.0), 0.5f, 1e-6f);
}
TEST(StalenessAlpha, PastTimeoutIsFullyFaded) {
    EXPECT_FLOAT_EQ(mpviz::detail::SceneBuffer::staleness_alpha(13.0, 10.0, 0.5, 2.0), 0.0f);
}
```
  Run — FAIL (`staleness_alpha` undefined).
- [ ] **Step 7: Implement** (`age = now - last_update; return clamp(1 - (age - fade_start) / (timeout - fade_start), 0, 1)`). Run — PASS.
- [ ] **Step 8: POD-layout snapshot test** (guards the freeze itself — catches an accidental member reorder/resize in review, not just a missing-field build error): `static_assert` a table of `offsetof`/`sizeof` for every struct in `scene.h`, in `test_scene_buffer.cpp` or a new `tests/test_scene_layout.cpp`:

```cpp
static_assert(sizeof(mpviz::EgoState) == 40, "EgoState layout frozen — see Interfaces block");
static_assert(offsetof(mpviz::SceneGraph, ego) == 8, "SceneGraph layout frozen");
// ... one line per struct/field that matters; recompute the real numbers
// when first compiled (Vec3 is 24 bytes, alignment may pad EgoState — write
// down what the compiler actually says, don't hand-guess and then "fix" the
// assert to match without checking why).
```
- [ ] **Step 9: Build + test.**
```bash
cd cuda/src/libs/visual_renderer
cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .
cmake --build build && ctest --test-dir build --output-on-failure
```
- [ ] **Step 10: Commit** `feat(visual): SceneGraph POD model + double-buffered scene + staleness fade (VM-010)`.

---

## Task 2 (VM-011): Theme system on a real lit pipeline + golden-image harness

**Files:**
- Create: `cuda/src/libs/visual_renderer/assets/materials/clay.mat` (lit, opaque, replaces `simple_color.mat` for themed solid surfaces: ground, ego clay-box fallback, ego glTF remap)
- Create: `cuda/src/libs/visual_renderer/assets/materials/clay_faded.mat` (lit, per-vertex-alpha variant, grid-only — see Step 2)
- Create: `cuda/src/libs/visual_renderer/assets/themes/dark_adas.yaml`
- Create: `cuda/src/libs/visual_renderer/assets/themes/light_clay.yaml`
- Create: `cuda/src/libs/visual_renderer/src/theme.hpp`, `src/theme.cpp` (YAML → internal `Theme` struct; NOT POD, internal-only)
- Create: `cuda/src/libs/visual_renderer/src/renderer_internal.hpp` (internal-only, `-I src` visibility, not installed, not POD — the `VisualRenderer` class definition (plus the `Mesh`/`HeadlessEglPlatform`/`Vertex`/`add_mesh` helper types it needs, per Step 7e) extracted out of `renderer.cpp`, so `src/ego.cpp` — a separate translation unit, Task 4, compiled as part of the same library target and so sharing its PRIVATE Filament include access — can see it and `render_frame`'s ego-transform hook. Pulls in `<filament/...>` headers, so it is deliberately NOT included by any `tests/*.cpp` — see Task 4's `ego_test_hooks.hpp` for what tests use instead.)
- Create: `cuda/src/libs/visual_renderer/tests/test_theme.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/test_paths.hpp` (defines `kThemeDir` and documents the `MPVIZ_TEST_DATA_DIR`-prefix convention for golden/fixture paths — see Step 5a; included by every test file in this and later tasks that references either)
- Create: `cuda/src/libs/visual_renderer/tests/golden.cpp`, `tests/golden.hpp` (shared render+compare harness, linked into gtest binaries)
- Create: `cuda/src/libs/visual_renderer/tests/golden.py` (dev-only: regenerate/inspect a committed golden PNG; not on the ctest path)
- Create: `cuda/src/libs/visual_renderer/tests/goldens/empty_world_dark_adas.png`, `tests/goldens/empty_world_light_clay.png` (committed binaries, generated by Step 8)
- Modify: `cuda/src/libs/visual_renderer/include/visual_renderer/api.h` (add `RenderConfig`'s two new fields, `theme_assets_dir`/`initial_theme`, per the frozen block above — Step 7)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (drop the Epic 0 spike scene: unlit cube/`simple_color.mat` ground+grid; replace with themed ground+grid+fog on `clay.mat`, sun + analytic-IBL wired for real)
- Modify: `cuda/src/libs/visual_renderer/CMakeLists.txt` (FetchContent yaml-cpp; exclude `tests/golden.cpp` from the auto-gtest glob, compile it as a plain object linked into the other test binaries instead; hoist the `stb_image_write.h` fetch out of the `if(EXISTS examples)` block and add a matching pinned `stb_image.h` fetch so test binaries can decode PNGs too — see Step 5a; add the `MPVIZ_TEST_DATA_DIR`/`MPVIZ_DEFAULT_THEME_DIR`/`DEFAULT_THEME_ASSETS_DIR` compile definitions — see Step 5a)
- Modify: `cuda/src/libs/visual_renderer/tests/test_hello_frame.cpp` (delete or repoint — Epic 0's cube-scene assertions no longer hold once Task 2 replaces the scene; see Step 1)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/CMakeLists.txt` (teach the prebuilt-archive import about `libyaml-cpp.a` — see Step 5; this is a hard link-error blocker for `colcon_build.sh` the moment `theme.cpp` calls into yaml-cpp, unlike gltfio, see Task 4's Files list note)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (shipped `quality` default 2 -> 1, so the running node matches this epic's frozen `quality=1` goldens — see Step 7f)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (`quality: 2` -> `quality: 1`, same reason — see Step 7f)

**Interfaces:** internal `mpviz::detail::Theme` struct and `Theme load_theme(const char* dir, const char* name)` (used by Task 3's transition code too); no new public POD (theme *names* cross the boundary as `const char*` via `set_theme`/`RenderConfig`, nothing else needs to).

### 2a. Retire the Epic 0 spike scene first

- [ ] **Step 1:** `test_hello_frame.cpp` asserts "sky rows differ from ground rows" against Epic 0's fixed camera pose/scene — that assumption survives (there's still a ground+sky), but the pixel values it silently depended on (unlit flat colors) don't. Re-run it now, before touching `renderer.cpp`, to confirm it's currently green (baseline). Leave it as a coarse smoke check; the real regression protection from here on is the golden-image tests (Step 8+), which is the point of this task.

### 2b. The lit material

- [ ] **Step 2:** Write TWO materials, not one. A single shared "clay" material can't honestly be both (a) opaque, depth-writing, SSAO-participating, no-vertex-color-required — which every solid clay surface needs, including Task 4's gltfio-loaded ego mesh, which has no COLOR vertex attribute and shouldn't be forced to grow one — and (b) alpha-blended with a per-vertex fade, which only the grid actually needs. So: `clay.mat` is the shared solid material (ground, ego clay-box fallback, ego glTF remap all use it), and `clay_faded.mat` is a grid-only variant.

  `clay.mat` — opaque, no vertex-color requirement, usable on ANY renderable in the scene including gltfio-loaded meshes:
```
material {
    name : clay,
    shadingModel : lit,
    parameters : [
        { type : float3, name : baseColor },
        { type : float, name : roughness },
        { type : float, name : metallic }
    ]
    // blending: opaque is Filament's default — no `blending` line needed.
    // No `requires: [color]` — this material never reads a vertex color, so
    // it places zero constraints on the vertex layout of whatever it's
    // bound to (Epic 0's ground/grid Vertex struct, a future gltfio asset,
    // anything).
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        material.baseColor.rgb = materialParams.baseColor;
        material.roughness = materialParams.roughness;
        material.metallic = materialParams.metallic;
    }
}
```
  `clay_faded.mat` — the grid's own material; the ONLY renderable that needs a COLOR vertex attribute is the grid's own dedicated vertex buffer (built once, at grid-construction time, already a distinct `make_vertex_buffer` call from the ground plane's — extending just that one buffer's layout doesn't touch the ground, the ego, or gltfio meshes at all):
```
material {
    name : clay_faded,
    shadingModel : lit,
    parameters : [
        { type : float3, name : baseColor },
        { type : float, name : roughness },
        { type : float, name : metallic }
    ],
    requires : [ color ],
    blending : fade
    // `fade` (not `transparent`): Filament's `transparent` blend mode
    // expects premultiplied alpha and is meant for glass-like surfaces;
    // `fade` is the non-premultiplied "dither this whole surface toward
    // invisible" mode, which is what a distance-faded grid line actually
    // wants, and matches straight (non-premultiplied) alpha values baked
    // into vertex color at grid-build time (Step 7). Both modes put their
    // renderables in the blended queue (no depth write, no SSAO, back-to-
    // front sort) — confining that to the grid alone, instead of every clay
    // surface, keeps the ground/ego/every future opaque object out of it.
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        material.baseColor.rgb = materialParams.baseColor;
        material.roughness = materialParams.roughness;
        material.metallic = materialParams.metallic;
        material.baseColor.a = getColor().a;  // per-vertex distance fade, baked once at grid-build time (Step 7)
    }
}
```
  Both matc-compile via the same `CMakeLists.txt` configure-time block Epic 0 already added for `simple_color.mat` (glob picks up both new `.mat` files automatically). Staleness fade (Task 1's `staleness_alpha`, consumed starting Epic 2 for tracked objects/paths/alerts/markers) is a separate, later decision — Epic 2 picks whichever of these two patterns (or a third: scalar per-`MaterialInstance` alpha) fits fading a whole dynamic object, and is free to add a `clay_alpha.mat` scalar-alpha variant then; Epic 1 does not need to solve that now.
- [ ] **Step 3: Failing test.** `test_theme.cpp`, first case just proves the material loads and differs visibly from flat unlit output under two different light directions (proves it's actually *lit*, not a copy of the unlit bug):
```cpp
TEST(ClayMaterial, RespondsToLightDirection) {
    // render_frame with the sun pointed two different ways at the same
    // ground plane must NOT produce identical pixels — Engine::getDefaultMaterial()
    // and simple_color.mat both failed this trivially (Epic 0 Deviation 2).
}
```
  Run — FAIL (`clay.mat` / theme plumbing don't exist yet in `renderer.cpp`).

### 2c. Theme tokens

- [ ] **Step 4:** Write both YAML files. Full token set from spec §4.3 (palette, material, emissive, grid, hud, sun, ibl) — every field below is consumed generically by `theme.cpp`, no per-theme branch anywhere in the renderer:
```yaml
# dark_adas.yaml
name: dark_adas
palette:
  ground:      [0.05, 0.06, 0.08]
  sky:         [0.02, 0.02, 0.05]
  fog:         [0.02, 0.02, 0.05]
  lane_paint:  [0.45, 0.5, 0.55]
  ribbon_core: [0.10, 1.00, 0.40]
  ribbon_glow: [0.10, 1.00, 0.40]
  object_tints:
    car: [0.25, 0.35, 0.9]
    truck_van: [0.30, 0.35, 0.85]
    bus: [0.85, 0.6, 0.15]
    pedestrian: [0.9, 0.2, 0.2]
    cyclist: [0.9, 0.55, 0.1]
    unknown: [0.5, 0.5, 0.5]
  alert:
    info: [0.2, 0.6, 1.0]
    warning: [1.0, 0.7, 0.1]
    critical: [1.0, 0.15, 0.1]
material: { roughness: 0.85, metallic: 0.0 }
emissive: { ribbon_strength: 4.0 }
grid: { line_color: [0.12, 0.14, 0.18], fade_start_m: 15.0, fade_end_m: 40.0 }
hud: { text_color: [0.9, 0.95, 1.0], accent_color: [0.10, 1.0, 0.4], scale: 1.0 }
sun: { direction: [-0.5, -0.3, -1.0], color: [0.55, 0.6, 0.75], intensity: 15000.0 }
ibl:  # analytic 2-band hemisphere gradient (Step 7) — NOT raw SH coefficients
  sky_color:    [0.05, 0.06, 0.12]
  ground_color: [0.02, 0.02, 0.03]
  intensity: 8000.0
fog: { density: 0.015 }  # matches this plan's originally-authored value —
# NO deviation here (review round 7). Rounds 5/6 spent this knob compensating
# for a fog *color-scale* bug (renderer.cpp's setFogOptions() rendered
# palette.fog ~1.71x brighter than the identical palette.sky, round 5) —
# round 5 raised density to 0.10 (crushed grid/object legibility), round 6
# walked that back to 0.03 once the scale bug was partly addressed. Neither
# was the right fix: the scale conversion itself was still wrong (~1.71x,
# not the 1.0x that "palette.fog == palette.sky" (spec §4.3) implies).
# Round 7 fixes the scale to true 1.0x at the source (renderer.cpp) and
# restores density to this plan's authored 0.015 instead of using density to
# paper over a color bug. The remaining horizon/sky convergence gap this
# leaves (see assets/themes/dark_adas.yaml's own comment, and
# tests/test_theme.cpp's now-loosened dark_adas guard) is real and
# honestly-documented: dark_adas's ground plane is only 40m across
# (kGroundHalfExtent), so even the farthest on-plane ray never reaches the
# near-total fog extinction a true infinite-ground horizon would give —
# density/color-scale tuning alone can't close that gap, and manufacturing
# more fog mass to force it shut is exactly the mistake this round undoes.
```
```yaml
# light_clay.yaml — same key set, no exceptions (that's the "no per-theme
# code branches" AC: if a theme needs a key the other doesn't, the schema
# is wrong, not the loader)
name: light_clay
palette:
  ground:      [0.82, 0.80, 0.76]
  sky:         [0.78, 0.85, 0.92]
  fog:         [0.80, 0.86, 0.92]
  lane_paint:  [0.95, 0.95, 0.92]
  ribbon_core: [0.15, 0.55, 0.95]
  ribbon_glow: [0.15, 0.55, 0.95]
  object_tints:
    car: [0.2, 0.4, 0.85]
    truck_van: [0.35, 0.45, 0.6]
    bus: [0.9, 0.65, 0.2]
    pedestrian: [0.85, 0.25, 0.25]
    cyclist: [0.85, 0.55, 0.15]
    unknown: [0.6, 0.6, 0.6]
  alert:
    info: [0.15, 0.5, 0.9]
    warning: [0.95, 0.6, 0.05]
    critical: [0.9, 0.1, 0.1]
material: { roughness: 0.75, metallic: 0.0 }
emissive: { ribbon_strength: 1.5 }
grid: { line_color: [0.6, 0.58, 0.55], fade_start_m: 15.0, fade_end_m: 40.0 }
hud: { text_color: [0.1, 0.12, 0.15], accent_color: [0.15, 0.55, 0.95], scale: 1.0 }
sun: { direction: [-0.4, -0.2, -1.0], color: [1.0, 0.98, 0.92], intensity: 100000.0 }
ibl:
  sky_color:    [0.75, 0.82, 0.9]
  ground_color: [0.5, 0.48, 0.44]
  intensity: 35000.0
fog: { density: 0.008 }
```
- [ ] **Step 5:** `theme.cpp`/`theme.hpp` — internal `Theme` struct mirroring the YAML 1:1 (plain `float3`/`float`/`std::string` fields, std:: is fine here), `Theme load_theme(const std::string& dir, const std::string& name)` via `FetchContent`'d yaml-cpp (same ABI-safety rationale as GoogleTest in Epic 0 — pin version + sha256, build with the project's own clang/libc++ toolchain, added next to the existing `googletest` `FetchContent_Declare` block in `CMakeLists.txt`):
```cmake
FetchContent_Declare(
    yamlcpp
    URL "https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz"
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yamlcpp)
# link `yaml-cpp` PRIVATE into visual_renderer, same as Filament — never
# leaks past the POD api.h/scene.h boundary.
```
  **This alone is not enough to keep the ROS build green.** `libvisual_renderer.a` will now carry undefined `YAML::*` symbols, and `micropilot_visualization_node/CMakeLists.txt` links it via a hand-written
  `INTERFACE_LINK_LIBRARIES "Filament::filament;${_libcxx_a};${_libcxxabi_a};${_libunwind_a}"`
  on `visual_renderer_prebuilt` (it does not `add_subdirectory`/`find_package` visual_renderer's own build, so it has no automatic visibility into what that build links) — nothing there resolves `YAML::*`, so `colcon_build.sh` (Task 3 Step 13, Task 4 Step 11) fails to link `visualization_node` the first time `theme.cpp` is pulled in. Fix it there too, in the same CMakeLists.txt: resolve the FetchContent-built static archive's path (`${VISUAL_RENDERER_DIR}/build/_deps/yamlcpp-build/libyaml-cpp.a`, matching the existing `VISUAL_RENDERER_LIB` path-construction style a few lines above) and append it to `visual_renderer_prebuilt`'s `INTERFACE_LINK_LIBRARIES`:
```cmake
set(_yamlcpp_a "${VISUAL_RENDERER_DIR}/build/_deps/yamlcpp-build/libyaml-cpp.a")
if(NOT EXISTS "${_yamlcpp_a}")
    message(FATAL_ERROR "yaml-cpp static lib not found at ${_yamlcpp_a} — build visual_renderer first.")
endif()
set_target_properties(visual_renderer_prebuilt PROPERTIES
    INTERFACE_LINK_LIBRARIES "Filament::filament;${_yamlcpp_a};${_libcxx_a};${_libcxxabi_a};${_libunwind_a}")
```
  (Confirm the actual archive path/name once yaml-cpp is first fetched — `FetchContent`'s build-dir naming is deterministic from the `FetchContent_Declare` name (`yamlcpp` → `yamlcpp-build`) but write down what the build actually produced rather than assuming.) For contrast, gltfio needs no equivalent fix: `GetFilament.cmake`'s `file(GLOB _filament_static_libs "${FILAMENT_ROOT}/lib/x86_64/*.a")` already picks up `libgltfio.a`/`libgltfio_core.a`/`libuberarchive.a` (verified present in the pinned 1.56.5 SDK) into the one `Filament::filament` target the node already links — no separate `Filament::gltfio` target or node CMakeLists change is needed for Task 4 (see that task's Files list).
- [ ] **Step 5a: Give test binaries a PNG decoder, and give every test/library `kThemeDir`/`DEFAULT_THEME_ASSETS_DIR` a real, resolvable path.** Two separate gaps, fixed together because both land in the same `CMakeLists.txt` edit:
  - **PNG decode.** Step 9's `render_and_compare` must decode the committed golden PNG to compute SSIM. The only image code in this build today is `stb_image_write.h` (write-only, vendored below the `tests/` block inside `if(EXISTS examples)`, with `STB_IMAGE_WRITE_DIR` granted only to example targets — a test binary can't even `#include` it as things stand). The pinned Filament SDK doesn't fill the gap either: `include/image/` is `LinearImage` ops only, and there is no `libimageio.a` in `lib/x86_64/`. Fix: pin `stb_image.h` the same way as `stb_image_write.h` (immutable commit URL + sha256 — upstream ships no versioned release of either single-header file), and hoist BOTH fetches above the `if(EXISTS tests)` block so both the `tests/` and `examples/` blocks can see them:
    ```cmake
    # Hoisted above both blocks below: PNG codec headers needed by
    # examples/hello_frame.cpp (write-only) and tests/golden.cpp (read+write).
    set(STB_IMAGE_WRITE_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
    set(STB_IMAGE_WRITE_SHA256 "cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05")
    set(STB_IMAGE_COMMIT "${STB_IMAGE_WRITE_COMMIT}")  # same stb tree, both headers
    set(STB_IMAGE_SHA256 "594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3")
    set(STB_DIR "${CMAKE_BINARY_DIR}/_deps/stb")
    set(STB_IMAGE_WRITE_H "${STB_DIR}/stb_image_write.h")
    set(STB_IMAGE_H "${STB_DIR}/stb_image.h")
    if(NOT EXISTS "${STB_IMAGE_WRITE_H}")
        file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${STB_IMAGE_WRITE_COMMIT}/stb_image_write.h"
             "${STB_IMAGE_WRITE_H}" EXPECTED_HASH SHA256=${STB_IMAGE_WRITE_SHA256})
    endif()
    if(NOT EXISTS "${STB_IMAGE_H}")
        file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${STB_IMAGE_COMMIT}/stb_image.h"
             "${STB_IMAGE_H}" EXPECTED_HASH SHA256=${STB_IMAGE_SHA256})
    endif()
    ```
    Delete the now-duplicate fetch block from inside `if(EXISTS examples)` (point its `STB_IMAGE_WRITE_DIR`/`STB_IMAGE_WRITE_H` variable *uses* at `STB_DIR`/the hoisted `STB_IMAGE_WRITE_H` instead of re-declaring them). In the existing `foreach(_test_src ...)` loop inside `if(EXISTS tests)`, add `target_include_directories(${_test_name} PRIVATE "${STB_DIR}")` next to the existing `target_link_libraries` line, so every gtest binary can `#include "stb_image.h"` / `#include "stb_image_write.h"`. `golden.cpp` (Step 9) defines `STB_IMAGE_IMPLEMENTATION`/`STB_IMAGE_WRITE_IMPLEMENTATION` (each in exactly this one `.cpp`, never in a header) to decode the golden PNG to pixels for the SSIM compare and to write the actual-output PNG.
  - **Resolvable paths.** `gtest_discover_tests`'s default `WORKING_DIRECTORY` is the build directory, not this source directory — so any test that opens a bare relative literal like `"tests/goldens/empty_world_dark_adas.png"` or `"tests/fixtures/test_cube.glb"` misses, and `render_and_compare` silently returns its "golden missing" `0.0` (still fails today's `EXPECT_GT`, but for the wrong reason — and would start silently mis-passing the moment anyone runs the binary from this source directory instead of via ctest). Separately, `kThemeDir` — used starting Step 10 below, and by Task 3 Step 5 and Task 4 Step 4 — isn't defined anywhere. Fix both with compile definitions in the same `foreach(_test_src ...)` loop:
    ```cmake
    target_compile_definitions(${_test_name} PRIVATE
        MPVIZ_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        MPVIZ_DEFAULT_THEME_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")
    ```
    and give the library itself the matching default, used by `create_renderer` (Step 7) whenever `RenderConfig::theme_assets_dir` is null:
    ```cmake
    target_compile_definitions(visual_renderer PRIVATE
        DEFAULT_THEME_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")
    ```
    `tests/test_paths.hpp` (new, in the Files list above) turns those macros into the one symbol every later snippet references:
    ```cpp
    #pragma once
    namespace mpviz::testing {
    inline constexpr const char* kThemeDir = MPVIZ_DEFAULT_THEME_DIR;
    }  // namespace mpviz::testing
    using mpviz::testing::kThemeDir;
    ```
    Every `"tests/goldens/...")`/`"tests/fixtures/...")` string literal introduced later in this plan (Task 2 Step 10, Task 3 Step 5/7, Task 4 Step 4/6) is shorthand for that same literal prefixed with `MPVIZ_TEST_DATA_DIR "/"` — e.g. `MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png"` (plain C string-literal concatenation, no runtime work) — apply that substitution when writing the actual test file; one macro, no per-test exception.

    **Installed ROS node.** `micropilot_visualization_node` runs out of `cuda/install`, never out of this source tree, but no step in Task 2 or Task 3 sets `RenderConfig::theme_assets_dir` from the node — it stays null, so the node falls through to the same compiled-in `DEFAULT_THEME_ASSETS_DIR` (this checkout's `assets/themes`) as the tests. That is a deliberate, stated simplification, not an oversight: it works on this single dev box because the `visual_renderer` source checkout stays in place after building, but it is not what a portable ROS install would do (the node's own `CMakeLists.txt` already `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})` for its own params — `assets/themes` gets no equivalent `install()`/`share/` copy here). If the checkout is ever absent (a different machine, a from-scratch build without this library's source tree), `load_theme()` fails exactly like Step 7b's `MissingThemeDir` test and the node falls back to `kFallbackTheme` non-fatally — degraded (no day/night toggle) but never crashing. Making theme assets a proper installed ROS share/ resource is future work; out of scope for this epic's single-box deployment.
- [ ] **Step 6:** Run test_theme.cpp's material test again — still needs `renderer.cpp` wiring (next).

### 2d. Wire it into `renderer.cpp`

- [ ] **Step 7:** Replace the Epic 0 scene construction:
  - Drop `simple_color.mat`/`colorMaterial`/the cube entirely (dead spike geometry).
  - `create_renderer`: `theme_assets_dir`/`initial_theme` are caller-owned `const char*`, borrowed only for this call (Interfaces block above) — the FIRST thing done with each is copy it into a `std::string` member on `VisualRenderer` (default dir = compiled-in `DEFAULT_THEME_ASSETS_DIR` when null, default theme = `"dark_adas"` when null), since `set_theme` (Task 3) re-reads the retained `theme_assets_dir` string on every future call, long after this call's raw pointer is gone. Only then `load_theme()` (see Step 7a below for the failure path — must NOT make `create_renderer` fail), build one `clay.mat` (opaque) `MaterialInstance` for `ground` (flat `baseColor`/`roughness`/`metallic`, no per-vertex color — see Step 2's material split) and one `clay_faded.mat` `MaterialInstance` for `grid`, whose dedicated vertex buffer gets a COLOR attribute added (only that buffer — ground/ego are unaffected) carrying per-vertex alpha baked from radial distance vs. `grid.fade_start_m/fade_end_m` — the "fading with distance" AC from spec §7, computed once at grid-build time since the grid is static geometry, not per-frame.
  - Sun: reuse Epic 0's already-built `LightManager::Type::SUN` entity, but now actually theme-driven: `direction`/`color`/`intensity` from the loaded theme instead of Epic 0's hardcoded constants.
  - IBL: replace Epic 0's flat single-SH-band "ambient" with a real (if deliberately low-frequency) 2-band irradiance IBL, analytically derived from the theme's `ibl.sky_color`/`ibl.ground_color` — a closed-form hemisphere-gradient-to-SH conversion (well-known constants; the "Stupid Spherical Harmonics Tricks" L0/L1 hemisphere gradient), NOT a full prefiltered specular cubemap:
    ```cpp
    // ponytail: clay materials in both reference images are matte/non-
    // reflective — there's no visible specular environment to capture, so a
    // full cmgen-baked prefiltered cubemap (the "real" Filament IBL asset
    // pipeline) buys nothing here and costs an offline bake step + cubemap
    // asset per theme. A 2-band (4-coefficient-per-channel) analytic SH
    // irradiance field from two flat colors gives correct diffuse
    // sky/ground lighting, which is all a matte material samples anyway.
    // Upgrade path if a future epic needs glossy reflections (buildings?
    // wet-road specular?): swap this for cmgen-prefiltered per-theme
    // cubemaps behind the same IndirectLight::Builder call site.
    filament::math::float3 sh[4] = sh_from_hemisphere(theme.ibl.sky_color, theme.ibl.ground_color);
    r->ambient = filament::IndirectLight::Builder().irradiance(2, sh).intensity(theme.ibl.intensity).build(*engine);
    ```
  - Fog: `filament::View::setFogOptions({.color = theme.palette.fog, .density = theme.fog.density, .enabled = true})` (Filament's built-in distance fog — native feature, no custom skybox mesh). **`enabled` defaults to `false`** (it's `FogOptions`' last member in `include/filament/Options.h`) — omit it and both themes' `fog:` token silently renders nothing; the designator order above matches the struct's actual declaration order (`color` before `density` before `enabled`), which clang's designated-initializer rule requires. Also set clear color to `theme.palette.sky` (flat "sky" backdrop, matching the clay/flat aesthetic of both references — no skydome geometry).
  - Shadows: Epic 0's `add_mesh` helper hardcodes `.castShadows(false).receiveShadows(false)` unconditionally on every renderable it builds. Spec §4.2 requires contact shadows on, and Task 4 Step 5 explicitly reuses this same helper for the ego — leaving the flags as-is would freeze a floating, shadowless ego into this task's own committed goldens once Task 4 lands. Give `add_mesh` `cast_shadows`/`receive_shadows` bool parameters (default `false`/`false`, matching today's behavior everywhere this step doesn't override it), and call it here with `receive_shadows = true` for the ground plane (grid stays `false`/`false` — it's alpha-blended, blended renderables don't meaningfully receive shadows and this task adds nothing that casts one onto it yet). Task 4 Step 5 then passes `cast_shadows = true` when building the ego (both the glTF remap and the clay-box fallback), so the ego actually darkens the ground beneath it once the sun is real.
  - `set_scene`: `VisualRenderer` gains a `detail::SceneBuffer` member (Task 1's class; `renderer.cpp` already has `-I src` visibility into it, same as the test binaries). `mpviz::set_scene(VisualRenderer* r, const SceneGraph& scene)` is implemented HERE, in this task, as exactly one line — `r->scene_buffer.publish(scene);` — no `Filament::Engine`/`Scene`/`TransformManager` call, matching scene.h's frozen contract comment verbatim ("This call touches ONLY that internal buffer"). This task's own fog/sun/IBL/theme-blend code and Task 3's transition clock both read `SceneGraph::sim_time_sec` back out via `r->scene_buffer.active()` from `render_frame()` — never from `set_scene()` itself. No ego yet (Task 4 adds the transform-update read of `active().ego`, but does not touch `set_scene()`).
- [ ] **Step 7a: Re-enable post-processing (Epic 0 turned it off; this epic needs it back).** Epic 0's `create_renderer` calls `r->view->setPostProcessingEnabled(false)` — with post-processing off, `setFogOptions` above is a silent no-op (Filament applies fog in the post-process chain) and there is no tone mapping/color grading/gamma encoding at all, so the themes' photometric sun (15,000–100,000 lux) and IBL (8,000–35,000) would clip to flat white instead of rendering as lit. Fix, in `create_renderer`:
  - `r->view->setPostProcessingEnabled(true);`
  - Build and set a `filament::ColorGrading` with ACES tone mapping (spec §4.2): `r->view->setColorGrading(filament::ColorGrading::Builder().toneMapping(filament::ColorGrading::ToneMapping::ACES).build(*engine));`
  - Enable bloom so the emissive path exists before Epic 2 needs it (the theme YAMLs already ship `emissive.ribbon_strength`, which presupposes bloom, even though Epic 1 has no emissive ribbon geometry yet): `r->view->setBloomOptions({.strength = 0.5f /* tuned against goldens, not a spec number */, .enabled = true});` — note the designator order: `BloomOptions::strength` is declared before `enabled` in `include/filament/Options.h`, and clang rejects a designated-initializer list whose order doesn't match declaration order, so `{.enabled = ..., .strength = ...}` (declared the other way round) fails to compile.
  - **SSAO and anti-aliasing (FXAA/TAA): decide and wire now, per `RenderConfig::quality` — not deferred.** Spec §4.2/§8 lists both in the post chain alongside ACES/bloom, which this step DOES enable, and spec §8's preset table already fully specifies both by tier (`high`: TAA + SSAO; `medium`: FXAA + SSAO half-res; `low`: FXAA, no SSAO). Leaving them off (or leaving `quality` un-consumed) would freeze that decision out of this task's own committed goldens exactly the way an unaddressed shadow flag would — and unlike shadow-map *resolution* or render *resolution* (spec §8's other two preset knobs, genuinely deferred to VM-032/Epic 3 above because they don't move a pixel in a fixed-320×240, no-shadow-casting-yet golden), SSAO's contact darkening and FXAA/TAA's edge smoothing DO change golden pixels the moment any edge or corner exists in the scene (the ego box, Task 4). Implement in `create_renderer`, driven by `config.quality` (`0=low, 1=med, 2=high` per the struct comment) — verified against the pinned 1.56.5 `filament/include/filament/Options.h`: `AmbientOcclusionOptions::resolution` must be exactly `0.5` or `1.0`, and `AntiAliasing` is a two-value enum (`NONE`/`FXAA` only — TAA is a separate toggle, `View::setTemporalAntiAliasingOptions`, not a third `AntiAliasing` value):
    ```cpp
    filament::AmbientOcclusionOptions ao{};
    ao.enabled = config.quality >= 1;
    ao.resolution = config.quality >= 2 ? 1.0f : 0.5f;  // Options.h: must be 0.5 or 1.0
    r->view->setAmbientOcclusionOptions(ao);

    if (config.quality >= 2) {
        // high: TAA replaces FXAA (spec §8) — NONE here, TAA enabled separately.
        r->view->setAntiAliasing(filament::AntiAliasing::NONE);
        filament::TemporalAntiAliasingOptions taa{};
        taa.enabled = true;
        r->view->setTemporalAntiAliasingOptions(taa);
    } else {
        // low and medium both use FXAA (spec §8); this is also Filament's own
        // default, so this call is one line of explicitness, not new behavior.
        r->view->setAntiAliasing(filament::AntiAliasing::FXAA);
    }
    ```
    This task's own goldens (Step 10 below, and Task 3's transition goldens) are captured at `quality=1` (medium: FXAA + SSAO half-res) — the project's stated shipped default (see "Conservative perf assumptions" above) — so what ships is what gets frozen, not an arbitrary tier. Shadow-map resolution and render-resolution scaling (spec §8's other two per-preset knobs) stay unmapped, still VM-032's job (Epic 3) to finish — that gap is called out explicitly above, not silently left behind.
  - Set a fixed `Camera::setExposure(aperture, shutterSpeed, sensitivity)` on the render camera — physically-based lighting with the default `getExposure()` (calibrated for a normal 1–10k lux daylight scene) will over/under-expose against the theme's much higher sun/IBL numbers otherwise. Pick values by rendering a golden and tuning until the clay surfaces read as mid-gray-ish, not clipped white or crushed black — this is a one-time calibration captured in the golden, not a per-theme knob.
  - State the color space explicitly (this was previously unstated): every theme palette RGB triple (`ground`/`sky`/`fog`/`lane_paint`/`ribbon_*`/`object_tints`/`alert`/`grid.line_color`/`hud` colors, and `ibl.sky_color`/`ibl.ground_color`) is authored in LINEAR space and fed straight into `materialParams`/`setFogOptions`/`IndirectLight::Builder` with no sRGB decode — matching Filament's own convention that `baseColor`/light/fog colors are linear, and consistent with the sun/IBL `intensity` fields already being physical units (lux), not colors. `theme.cpp`'s loader does no color-space conversion; document this assumption in a comment at the top of `theme.hpp`.
  - This step must land BEFORE Task 2 Step 11 generates the committed goldens — the goldens should capture the real ACES/exposure/bloom output, not the clipped-linear look the Epic 0 deviation would otherwise bake in.
- [ ] **Step 7b: Theme-load failure must be non-fatal (spec §9).** `load_theme()` can fail three ways: `theme_assets_dir` is null/missing/unreadable, the resolved `<name>.yaml` file is missing, or a present file is malformed YAML. In every case `create_renderer` must still succeed — a broken theme-asset install must never take rendering down, mirroring `set_ego_model`'s own non-fatal design elsewhere in this plan. Add a small compiled-in `kFallbackTheme` (a `Theme` struct literal with the same values as `dark_adas.yaml`, kept in `theme.cpp`, never read from disk) and have `create_renderer` use it whenever `load_theme()` fails, instead of propagating failure up to the caller. `load_theme()` itself returns `std::optional<Theme>` (or a bool + out-param — internal-only type, not POD) rather than throwing, so `create_renderer` can make this decision inline. (`set_theme`'s existing "unknown name → return false, no-op" contract is unrelated and unchanged — that's normal runtime behavior for a bad `~/set_theme` request, not an asset-install failure at startup.)
- [ ] **Step 7c: Failing test for the Step 7b fallback**, in `test_theme.cpp`:
```cpp
TEST(ThemeLoad, MissingThemeDir_FallsBackToBuiltinTheme) {
    mpviz::RenderConfig cfg{320, 240, 0, "/nonexistent/theme/dir", "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        // Only acceptable reason for null here is no GPU/EGL, same skip
        // convention as every other renderer test — NOT a missing theme dir.
        GTEST_SKIP() << "no GPU/EGL";
    }
    // create_renderer must have succeeded despite the bad theme_assets_dir —
    // rendering one frame with the built-in fallback theme must not crash.
    mpviz::SceneGraph scene{};
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0,-8,4}, {0,0,0}, 60.0};
    mpviz::FrameView view{/*...*/};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    mpviz::destroy_renderer(r);
}
```
  Run — FAIL (`create_renderer` currently returns nullptr on a bad theme dir, since `load_theme()` failure isn't handled yet).
- [ ] **Step 7d: Run — PASS** (after Step 7b's fallback is implemented).
- [ ] **Step 7e: Extract `VisualRenderer` (and the anonymous-namespace helper types it needs by value) into `src/renderer_internal.hpp`.** Task 4's `src/ego.cpp` (a separate translation unit) needs to see the `class VisualRenderer` definition Step 7 above builds out — including the ego-entity fields Task 4 adds to it and the transform-update hook `render_frame` needs to call — and can't while that class stays defined inside `renderer.cpp` itself.

  **This is NOT a plain class-body relocation** (an earlier draft of this step called it "purely mechanical" — it isn't, against the actual file). `VisualRenderer` has a `HeadlessEglPlatform* platform` member and by-value `Mesh ground/grid/cube` members, and both `HeadlessEglPlatform` and `Mesh` are currently defined inside `renderer.cpp`'s anonymous namespace. A header at `mpviz` scope can't name an anonymous-namespace type, and a namespace-scope forward declaration sharing that name would be a *different* type from the anonymous-namespace one — so a naive move breaks `r->platform = platform;` in `create_renderer` (assigning an incompatible pointer type) and every by-value `Mesh` member (can't even forward-declare a by-value member). Do all of the following together instead:
  - Move `Mesh`'s full definition (needed by value, not just as a pointer) out of the anonymous namespace into `renderer_internal.hpp` at `mpviz` scope. This pulls `<filament/VertexBuffer.h>`, `<filament/IndexBuffer.h>`, and `<utils/Entity.h>` into the header — expected: this header is internal-only (never installed, not POD) and only ever `#include`d from `.cpp` files that are themselves compiled as part of `visual_renderer`'s own library target, which already links `Filament::filament` (PRIVATE) and so already has that include path. `tests/test_ego.cpp` must NOT include this header (see Task 4 Step 4's fix) — test binaries link `visual_renderer` but are never granted its PRIVATE Filament include dir, so a header pulling in `<filament/...>` won't compile in a test TU.
  - Move `HeadlessEglPlatform`'s full class body out of the anonymous namespace to plain `namespace mpviz` scope in `renderer.cpp` (still defined in the `.cpp`, just no longer inside `namespace { ... }`), and forward-declare `class HeadlessEglPlatform;` in `renderer_internal.hpp` at that same `mpviz` scope. Now the header's `HeadlessEglPlatform* platform` member and `renderer.cpp`'s `new HeadlessEglPlatform()` name the identical type.
  - Move `Vertex`, `make_vertex_buffer`, and `make_index_buffer` out of the anonymous namespace into `renderer_internal.hpp`/`renderer.cpp` (declaration/definition split, same pattern as the rest of this header) — `Mesh`-building call sites need them in scope wherever `Mesh`s are built, and Task 4 Step 5 is about to add one outside `renderer.cpp`. Bodies unchanged.
  - Turn the `add_mesh` lambda into a namespace-scope free function, declared in `renderer_internal.hpp` and defined in `renderer.cpp`: `void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts, std::vector<uint16_t> indices, filament::RenderableManager::PrimitiveType primitive, filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows);`. It was a lambda local to `create_renderer` capturing `&engine`/`&em`/`&r` — a lambda can't be called from `ego.cpp`, a different translation unit, which is exactly what Task 4 Step 5 needs to do for the ego's clay-box fallback. Replace the `&engine`/`&em` captures with `r.engine`/`utils::EntityManager::get()` (`em` was always just that singleton accessor, nothing stateful worth threading through) and keep `cast_shadows`/`receive_shadows` as the two bool parameters Step 7 above already added.
  - Everything else — `create_renderer`/`render_frame`/`destroy_renderer`'s bodies, every direct Filament call inside them, `build_ground_plane`/`build_grid_lines`/`build_cube`/`fill_tangent_frames`/`destroy_mesh`/`ReadbackState`/`on_readback_complete` — stays exactly where it is today, in `renderer.cpp`.
  - `#include "renderer_internal.hpp"` from `renderer.cpp` in place of the definitions it now houses.
- [ ] **Step 7f: Make the shipped default match the frozen-in-goldens default.** Today `visualization_node.cpp` declares `quality_ = declare_parameter<int>("quality", 2)` and `config/default_params.yaml` ships `quality: 2` — i.e. the running node takes the `quality >= 2` ("high") branch of Step 7a's SSAO/AA mapping (`AntiAliasing::NONE` + `setTemporalAntiAliasingOptions(enabled=true)`, a temporal-accumulation path), while every golden and unit test in this epic (Step 10 above, Task 3 Step 5, Task 4 Step 6) renders at `quality=1` ("medium": FXAA + SSAO half-res). Nothing in this epic ever exercises the `quality=2`/TAA path, and TAA's jitter/reprojection is exactly the kind of thing that can flicker on this renderer's freeze-frame model (an unchanging scene re-rendered every tick). Fix the mismatch by changing the default, not the goldens: change `declare_parameter<int>("quality", 2)` to `declare_parameter<int>("quality", 1)` in `visualization_node.cpp`, and `quality: 2` to `quality: 1` in `default_params.yaml`. After this, "default quality preset = medium" ("Conservative perf assumptions" above) and "goldens captured at `quality=1`" describe the same running configuration, and `quality=2`/TAA remains reachable (a user can still set the param) but is no longer the untested default.
- [ ] **Step 8: Run — PASS** (`ClayMaterial.RespondsToLightDirection` and any ground/fog assertions).

### 2e. Golden-image harness (used by every later epic — build it right once)

- [ ] **Step 9:** `tests/golden.hpp`/`golden.cpp` — a small reusable helper, **not** a gtest file itself (excluded from the CMakeLists auto-glob-as-gtest-binary by name so it doesn't fight `gtest_main`; instead added as a plain source compiled directly into whichever test binary calls it, same pattern as `scene_buffer.cpp`):
```cpp
// tests/golden.hpp
namespace mpviz::testing {
// Renders ONE frame of `r`'s current active scene/theme state from `pose`,
// writes it to `out_png_path` (stb_image_write, already vendored for
// examples/), and returns a block-SSIM score in [0,1] against
// `golden_png_path` (0 if the golden doesn't exist yet — first run of a new
// golden always fails loudly, never silently "passes" with nothing to
// compare against).
//
// The harness does NOT create a renderer, and does NOT call set_scene/
// set_theme — it only calls render_frame(r, pose, ...) and compares the
// result. The caller owns create_renderer()/destroy_renderer(), and must
// have already driven `r` into whatever scene/theme/transition state it
// wants a golden of via set_scene()/set_theme() BEFORE calling this. This is
// what lets Task 3's mid-transition goldens (a `VisualRenderer` sitting in
// the middle of a set_theme() ease) be captured at all — a harness that
// created its own renderer from a theme *name* could never observe
// transition state, and "midpoint" isn't a loadable
// assets/themes/midpoint.yaml stem anyway.
//
// Returns -1.0 (caller must GTEST_SKIP()) if `r` is null — same
// no-GPU/EGL convention as test_hello_frame.cpp, just checked by the caller
// before create_renderer() rather than inside this function.
double render_and_compare(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path);
}
```
  SSIM implementation: block-wise (8×8, non-overlapping, luminance-only) mean/variance/covariance SSIM averaged over blocks — a deliberately simpler approximation of the full windowed-Gaussian SSIM (unnecessary precision for a pass/fail regression gate at low-preset resolution); document that choice with a `ponytail:` comment in `golden.cpp`.
- [ ] **Step 10:** `tests/test_theme.cpp` golden cases (`#include "test_paths.hpp"` for `kThemeDir`; golden paths use the `MPVIZ_TEST_DATA_DIR`-prefix convention from Step 5a so they resolve regardless of `gtest_discover_tests`'s build-dir working directory):
```cpp
TEST(ThemeGolden, EmptyWorld_DarkAdas) {
    // quality=1 (medium: FXAA + SSAO half-res) — the shipped default (see
    // "Conservative perf assumptions" above), so the committed golden
    // matches what Step 7a actually ships, not an arbitrary tier.
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph scene{};  // empty: ego.valid=0, every count=0
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);           // caller drives scene state...
    // initial_theme is already "dark_adas" from cfg, so no set_theme() call
    // needed here — Task 3's transition tests are what exercise mid-blend.
    mpviz::CameraPose pose{{0,-8,4}, {0,0,0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        r, pose,                          // ...harness only renders + SSIMs `r` as-is
        MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png", "/tmp/empty_world_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
}
TEST(ThemeGolden, EmptyWorld_LightClay) {
    // identical, except cfg.initial_theme = "light_clay" and the golden/out
    // paths point at empty_world_light_clay.png.
}
```
  Run — FAIL (no committed golden PNG yet: `render_and_compare` returns 0 by contract, `EXPECT_GT(0, 0.98)` fails loudly, which is the correct first-run behavior, not a harness bug).
- [ ] **Step 11:** Generate the initial goldens deliberately (not silently from a passing test): run the same binary with an env var/flag that skips the comparison and just writes the PNG (`golden.cpp`'s `render_and_compare` already writes `out_png_path` unconditionally — copy that file to `tests/goldens/...png` after a **human looks at it**, matching the reference images' clay/matte look). `tests/golden.py` is the developer-facing wrapper for this (`python3 tests/golden.py --show /tmp/empty_world_dark_adas_actual.png` opens/prints it for a visual sanity check before committing — this project's stated working style is "user judges by visuals"). Commit the two PNGs.
- [ ] **Step 12:** Run again — PASS (SSIM ≈ 1.0 against the just-committed goldens).
- [ ] **Step 13: Build + full test run.**
```bash
cd cuda/src/libs/visual_renderer
cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .
cmake --build build && ctest --test-dir build --output-on-failure
```
- [ ] **Step 14: Commit** `feat(visual): real lit clay pipeline + theme system + golden-image harness (VM-011)`.

---

## Task 3 (VM-014): Animated theme toggle

**Files:**
- Create: `cuda/src/libs/visual_renderer/src/theme_transition.hpp`, `src/theme_transition.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/test_theme_transition.cpp`
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (`set_theme` entry point; `render_frame` recomputes the blended `Theme` from the active `SceneGraph.sim_time_sec` every call instead of applying a theme once at creation)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/{include,src}/visualization_node.{hpp,cpp}` (`~/set_theme` topic, node clock feeding `sim_time_sec`)
- Modify: `tools/vcam_ws_bridge.py` (`set_theme` WS command)
- Modify: `tools/vcam_gui.py` (day/night toggle button)
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_theme_ws.py`

**Interfaces:** `mpviz::set_theme` (frozen above). Internal: `Theme blend(const Theme& a, const Theme& b, float t)`.

- [ ] **Step 1: Oklab helpers.** `theme_transition.cpp` needs sRGB↔Oklab; small closed-form public-domain conversion (Björn Ottosson's reference formulas — cube-root + two 3×3 matrices each direction, ~15 lines per direction, no dependency):
```cpp
// linear_srgb_to_oklab / oklab_to_linear_srgb — standard published matrices.
// blend_color(a, b, t) = oklab_to_linear_srgb(lerp(linear_srgb_to_oklab(a),
//                                                    linear_srgb_to_oklab(b), t))
```
- [ ] **Step 2: Failing test — mid-transition blend is perceptually plausible, not just component-lerped:**
```cpp
TEST(ThemeTransition, MidpointBlend_IsBetweenEndpointsInOklab) {
    // dark_adas.ground vs light_clay.ground blended at t=0.5 must land near
    // the Oklab midpoint of the two, NOT the naive-sRGB-lerp midpoint (the
    // two differ measurably for colors this far apart — that's the whole
    // reason spec §4.3 calls out Oklab specifically).
}
TEST(ThemeTransition, Smoothstep_EasesInAndOut) {
    // weight(t) at t=0.1 and t=0.9 must be closer to the endpoints than a
    // linear ramp would put them (smoothstep's defining property).
}
```
  Run — FAIL.
- [ ] **Step 3: Implement** `blend(Theme, Theme, float t)`: `w = smoothstep(t)`; every `float3` palette/sun/hud color field → `blend_color`; every scalar (roughness, metallic, ribbon_strength, intensities, fade distances) → linear lerp by `w`. One function, applied uniformly — no per-field special-casing beyond "is this a color or a scalar," which keeps the "no per-theme code branches" property from Task 2 intact (this is "no per-*field* branches" too, just a type dispatch).

  **Deviation from this step's literal text (implemented):** `sun.direction` is a `float3` but is not a color — it's a world-space direction vector (dark_adas: `{-0.5,-0.3,-1.0}`, magnitude ~1.157, already not unit — `renderer.cpp` has never renormalized it). Running it through `blend_color`'s Oklab matrices (calibrated for physically-plausible ~[0,1] linear-sRGB tristimulus values) would be numerically well-defined but photometrically meaningless — it isn't a color, so "is this a color or a scalar" undersells the actual dispatch needed: `sun.direction` gets a third case, a plain component-wise linear lerp by `w`, un-normalized (same convention as the existing static-theme direction feed into `LightManager::Builder::direction()`/`setDirection()`). Every other `sun`/`ibl`/`hud`/`palette`/`grid` float3 (all genuine colors) goes through `blend_color` exactly as written above.

  **Deviation from this step's literal text (implemented):** the "fade distances" half of "every scalar ... → linear lerp by `w`" is not actually true for `grid.fade_start_m`/`fade_end_m`. Those two fields are baked into the grid's vertex buffer's per-vertex alpha exactly once, at `create_renderer()` time (`renderer.cpp`'s `build_grid_lines()`, called once during renderer construction) — `push_theme_to_scene()` (Step 6) never reads `theme.grid.fade_start_m`/`fade_end_m` back out of the blended `Theme` to push anywhere, because there's no live grid-material parameter for it to push into. A `set_theme()` switch therefore can never change grid fade at runtime no matter what `blend()` computes for those two fields, so `blend()` now carries `b`'s (the "to" theme's) values through unchanged instead of lerping — lerping them was dead code creating a false impression that grid fade animates like every other themed token. Both shipped themes (`dark_adas`, `light_clay`) use identical `15.0`/`40.0` fade distances today, so this changes nothing visible. If a future theme ever ships different fade distances, the upgrade path is either re-baking the grid vertex buffer per frame during a transition (cheap given the grid's small vertex count) or moving the fade to a per-frame material parameter (a shader/material change, not just a `blend()` change) — out of scope for this task.
- [ ] **Step 4: Run — PASS.**
- [ ] **Step 5: Failing test — the transition state machine itself**, deterministic clock (this is the "deterministic-clock goldens" AC). `#include "test_paths.hpp"` for `kThemeDir`; golden paths use the same `MPVIZ_TEST_DATA_DIR`-prefix convention as Task 2 Step 10:
```cpp
TEST(ThemeTransition, DeterministicClock_MatchesTargetAtDuration) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};  // medium, same as Task 2's goldens
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);                 // t=0, still dark_adas
    mpviz::set_theme(r, "light_clay", 0.0, 0.8); // begin transition at t=0

    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    // golden at t=0.0 (still ~dark_adas — first tick of the transition).
    // render_and_compare takes `r` directly and renders whatever blended
    // state set_theme()/set_scene() already put it in — it never loads a
    // theme by name itself, so there's no "midpoint" theme file to resolve.
    EXPECT_GT(mpviz::testing::render_and_compare(r,
        kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0.png", "/tmp/t0.png"), 0.98);

    scene.sim_time_sec = 0.4;
    mpviz::set_scene(r, scene);
    // golden at t=0.4s (~50% blended — its own committed midpoint golden,
    // not compared against either endpoint; `r` itself is mid-ease, which is
    // exactly the state a name-based harness could never reach)
    EXPECT_GT(mpviz::testing::render_and_compare(r,
        kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0_4.png", "/tmp/t0_4.png"), 0.98);

    scene.sim_time_sec = 0.8;
    mpviz::set_scene(r, scene);
    // golden at t=0.8s (fully light_clay — transition_sec elapsed)
    EXPECT_GT(mpviz::testing::render_and_compare(r,
        kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0_8.png", "/tmp/t0_8.png"), 0.98);
    mpviz::destroy_renderer(r);
}
TEST(ThemeTransition, RetargetMidFlight_StartsFromCurrentBlendNotEndpoint) {
    // set_theme(A), advance to t=0.4 (mid-blend), set_theme(B) again before
    // it finishes -> immediately after the retarget the rendered frame must
    // still match the t=0.4 A/B blend (continuous), not snap to A.
}
```
  Run — FAIL (`set_theme` doesn't exist; goldens don't exist yet — same deliberate-generation flow as Task 2 Step 11).
- [ ] **Step 6: Implement `set_theme`** in `renderer.cpp`: stores `{from: current_blended_theme(), to: load_theme(theme_name), start_sec: at_sec, duration_sec: transition_sec > 0 ? transition_sec : 0.8}` on the `VisualRenderer`. `render_frame` (or a new internal `apply_current_theme()` called at its top, before touching materials) computes `t = clamp((active_scene.sim_time_sec - start_sec) / duration_sec, 0, 1)`, calls `blend(from, to, t)`, and pushes the result's tokens into the existing material instances/sun/IBL/fog **every call** (cheap — `setParameter` calls, no reloads) — once `t >= 1.0` the transition struct is cleared (idempotent no-op blend after that, avoids recomputing forever). "No frame drop > 1 during switch" (AC) falls out for free: nothing here allocates, blocks, or reloads assets mid-transition, it's pure arithmetic + existing `setParameter` calls that already run every frame.
- [ ] **Step 7: Generate + commit the 3 transition goldens** (same human-look-then-commit flow as Task 2 Step 11).
- [ ] **Step 8: Run — PASS.**

  **Deviation from this step's literal text (implemented):** "pushes the result's tokens into the existing material instances/sun/IBL/fog **every call** (cheap — `setParameter` calls, no reloads)" is accurate for materials (`MaterialInstance::setParameter`), the sun (`LightManager::setDirection`/`setColor`/`setIntensity` — real runtime setters on an already-built component), fog (`View::setFogOptions`), and clear color (`Renderer::setClearOptions`) — but not for the IBL. The pinned Filament 1.56.5 SDK's `IndirectLight` only exposes `setIntensity()`/`setRotation()` at runtime (confirmed against its public header); there is no way to feed new SH coefficients into an already-built instance. Animating `theme.ibl.sky_color`/`ground_color` therefore means destroying and rebuilding the small (4-coefficient, no cubemap) `IndirectLight` object on every `render_frame()` call **while a transition is actually in flight** — bounded to the ~24–30 frames of the default 0.8s transition, and not called at all in steady state (no transition active). This is a real, if small, per-frame allocation during a switch, not the zero-allocation `setParameter`-only path this step describes; `renderer.cpp`'s `push_theme_to_scene()` has the full reasoning and an upgrade path (skip the rebuild when `ibl.sky_color`/`ground_color` haven't actually changed) if this ever shows up in a profile. "No frame drop > 1 during switch" (AC) still held in practice: the real end-to-end WS test (Step 12) measured a 40ms max inter-frame gap (one tick) across a live `set_theme` switch on this dev box.
- [ ] **Step 9: Node wiring.** `visualization_node`: maintain `double sim_clock_sec_` (monotonic, incremented by the timer's own period each tick — `33ms` — rather than reading wall-clock, so the deterministic-clock contract holds all the way to the node too).

  **The call this whole task depends on, made explicit:** in `timer_callback`, EVERY tick, regardless of mode (ingest continues regardless of mode — same philosophy as the mux), build `mpviz::SceneGraph scene{}; scene.sim_time_sec = sim_clock_sec_;` (Task 4 fills in `scene.ego` from the TF adapter here too; nothing else is populated until Epic 2) and call `mpviz::set_scene(renderer_, scene);` — BEFORE the mode gate that decides whether this tick actually renders/publishes an image. Without this call, `SceneBuffer::active().sim_time_sec` never advances, `render_frame`'s `t = clamp((sim_time_sec - start_sec) / duration_sec, 0, 1)` clock is permanently stuck at `t=0`, and a `~/set_theme` request would never visibly finish outside a unit test that drives `set_scene` directly — the animated transition this task builds would be dead code in the running node. This is the node-side counterpart to Task 1/Task 2's `set_scene`/`render_frame` split: the node is the one thing that has to actually call `set_scene` every tick for any of it to matter.

  Then add the theme subscription:
```cpp
theme_sub_ = create_subscription<std_msgs::msg::String>(
    "~/set_theme", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!mpviz::set_theme(renderer_, msg->data.c_str(), sim_clock_sec_, 0.0)) {
            RCLCPP_WARN(get_logger(), "set_theme: unknown theme '%s'", msg->data.c_str());
        }
    });
```
- [ ] **Step 10: WS bridge command.** `tools/vcam_ws_bridge.py`: `{"cmd": "set_theme", "theme": "dark_adas"|"light_clay"}` → publish `std_msgs/String` to `/visualization_node/set_theme` (node-private, mode-3-only concept — no mux needed, harmless if published while mode 1/2 is active, matches the "ingest continues regardless of mode" philosophy).
- [ ] **Step 11: GUI toggle.** `tools/vcam_gui.py`: a day/night button next to the existing mode button (same `Gtk.Button` pattern as `_mode_btn`/`_on_mode_toggle`), sending the WS command above; label reflects last-known theme the way `_mode_btn`'s label already tracks `render_mode` from telemetry (Epic 3's `~/diagnostics`, VM-034, is the eventual place a theme-name echo would live end-to-end — for now the GUI just optimistically flips its own label on click, same as how it doesn't wait for confirmation on preset buttons today).
- [ ] **Step 12: WS E2E test.** `test_theme_ws.py`: launch node + bridge; capture a frame from `/rendering/image` before sending `set_theme light_clay`, send it, wait past `transition_sec` (0.8s + margin), capture a frame after. Assert BOTH (a) no gap in `/rendering/image` frames > 1 tick across the switch, AND (b) the before/after frames actually differ (e.g. mean absolute pixel difference above a small noise-floor threshold). (a) alone passes trivially on a node whose theme never advances (the Step 9 gap this task closes) — a broken `set_scene` wire would never break frame cadence, only the picture. (b) is what actually exercises the Step 9 wiring end to end (reuses the existing bridge E2E harness pattern from Epic 0 Task 5).
```bash
# corrected from the plan's original "cuda/install/setup.bash" — there is no
# such aggregate; colcon_build.sh installs ROS packages under
# cuda/install/ros_apps/ (see smoke_test.py's own INSTALL_DIR), matching
# every other ros_apps test in this repo.
source /opt/ros/humble/setup.bash && source cuda/install/ros_apps/setup.bash
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_theme_ws.py
```
- [ ] **Step 13: Full rebuild + existing-suite regression check** (Global Constraint: everything upstream stays green):
```bash
cd cuda/src/libs/visual_renderer && cmake --build build && ctest --test-dir build --output-on-failure
cd cuda/scripts/ros_apps_build && ./colcon_build.sh
```
- [ ] **Step 14: Commit** `feat(visual): animated set_theme transition (Oklab, smoothstep, 0.8s default) + GUI/WS toggle (VM-014)`.

---

## Task 4 (VM-012): Ego robot — M02P→glTF + TF pose + smoothed speed

**Files:**
- Create: `cuda/src/libs/visual_renderer/scripts/obj2gltf_m02p.py`
- Modify: `requirements.txt` (`trimesh>=4.0` — pure-Python OBJ/glTF conversion; no Blender/Node toolchain needed, matches this repo's plain-pip convention. `pygltflib`/npm `obj2gltf` considered and rejected: this box has no root-installed Blender, and shelling out to npm's `obj2gltf` would add a Node runtime dependency to a C++/Python/ROS repo for a one-time conversion script trimesh already covers.)
- Create: `cuda/src/libs/visual_renderer/src/ego.hpp`, `src/ego.cpp` (glTF/GLB load via Filament gltfio + clay-box fallback)
- Create: `cuda/src/libs/visual_renderer/src/ego_test_hooks.hpp` (internal-only, `-I src` visibility, not installed, not POD — declares `double mpviz::testing::rendered_bounding_box_diagonal(mpviz::VisualRenderer*)` against nothing but `visual_renderer/api.h`'s already-forward-declared opaque `class VisualRenderer` — no Filament includes. This is deliberately a separate header from `renderer_internal.hpp`: `tests/test_ego.cpp` needs to call this hook but is only linked against `visual_renderer`, never granted its PRIVATE Filament include dir, so it can't include `renderer_internal.hpp` — which, per Step 7e, now defines `Mesh` and pulls in `<filament/...>`. `ego.cpp` `#include`s both this header and `renderer_internal.hpp`, and defines the function using the full `VisualRenderer` type.)
- Create: `cuda/src/libs/visual_renderer/tests/test_ego.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/fixtures/test_cube.glb` (tiny, git-trackable glTF fixture — see Step 1/Step 4)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (`VisualRenderer`, now declared in Task 2 Step 7e's `renderer_internal.hpp`, gains the ego entity/state fields Step 5 below stores; `render_frame` updates the ego entity's `TransformManager` transform from `SceneBuffer::active().ego.position/heading_rad` at the top of every call, alongside Task 3's theme-blend apply — `set_scene()` itself is untouched)
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/tf_adapter.cpp`, `include/.../tf_adapter.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_tf_adapter.py` (recorded TF fixture → expected speed)
- Modify: node `CMakeLists.txt` — `find_package(tf2_ros)`, AND add `src/tf_adapter.cpp` to `visualization_node_lib`'s explicit source list (`add_library(visualization_node_lib SHARED src/visualization_node.cpp)` is a hand-written file list, not a glob — a new `.cpp` is invisible to the build until named here; forgetting this step is a silent no-op, not a build error, since `tf_adapter.hpp` can still be included and declared without its `.cpp` ever being compiled in)
- Modify: node `package.xml` — add `<depend>tf2_ros</depend>` (the CMakeLists' `find_package(tf2_ros)` alone does not satisfy `ament`'s package-dependency declaration; a missing `<depend>` is a hard `colcon_build.sh` failure the first time this package is built clean, e.g. in CI or on a fresh workspace, even though it may spuriously succeed on a dev box that already has `tf2_ros` on `CMAKE_PREFIX_PATH` from a prior build)
- Modify: `visualization_node.{hpp,cpp}` (own `tf2_ros::Buffer`/`TransformListener`, `ego_model_path`/`ego_fallback_dims` params, build `SceneGraph.ego` each tick)
- Modify: node `config/default_params.yaml` (`ego_model_path: /home/ag7/Downloads/M02P.glb` — mirrors `micropilot_rendering_node`'s existing `robot_model_path` convention exactly; code default stays `""`)

**Interfaces:** `mpviz::set_ego_model` (frozen above).

### 4a. Conversion script

- [ ] **Step 1:** `obj2gltf_m02p.py` — thin wrapper, not a bespoke parser:
```python
#!/usr/bin/env python3
"""Convert M02P.obj (the ego robot mesh) to a glTF binary for visual_renderer.

Usage: obj2gltf_m02p.py <input.obj> [output.glb]
(output defaults to the input path with its extension swapped to .glb)

The OBJ is assumed authored in meters, matching the ROS convention the map/
base_link frames already use (same assumption micropilot_rendering_node's
existing robot_model_path OBJ loading makes) -- if the ego mesh renders at
the wrong scale, check units here first, don't add a compensating scale
factor blind.
"""
import sys, pathlib
import trimesh

def main() -> int:
    if not 2 <= len(sys.argv) <= 3:
        print(__doc__); return 1
    src = pathlib.Path(sys.argv[1])
    dst = pathlib.Path(sys.argv[2]) if len(sys.argv) == 3 else src.with_suffix(".glb")
    mesh = trimesh.load(src, force="scene")  # force="scene": keep multi-material grouping
    mesh.export(dst)
    print(f"wrote {dst} ({dst.stat().st_size / 1e6:.1f} MB)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
```
  Test (`demo()`-style, per the smallest-check rule — this is a thin CLI wrapper, not business logic, so one manual run stands in for a unit test): run it against a small throwaway test OBJ (a unit cube written inline, NOT the real 143MB M02P.obj — that's a one-time manual step against the user's actual Downloads file, not a CI fixture) and assert the output `.glb` parses back via `trimesh.load` with the same vertex count. **Commit this cube's `.glb` output** as `cuda/src/libs/visual_renderer/tests/fixtures/test_cube.glb` (a few KB, unlike the 143MB M02P asset) — Step 4's `Ego.LoadValidGltf_RendersNonEmptyBoundingBox` test needs a real, always-in-git glTF asset to load, and this self-check already produces exactly that as a side effect.
- [ ] **Step 2:** Run for real against the user's file (manual, one-time, not part of any test suite — the output is a large binary that stays out of git, same as the source OBJ):
```bash
python3 cuda/src/libs/visual_renderer/scripts/obj2gltf_m02p.py /home/ag7/Downloads/M02P.obj
```
  Confirm the `.glb` opens and looks like the robot (visual check — this project's stated working style is "user judges by visuals").

### 4b. gltfio in the renderer

- [ ] **Step 3: Verify gltfio ships in the pinned SDK** before writing any code against it:
```bash
nm cuda/src/libs/visual_renderer/build/_deps/filament-1.56.5/filament/lib/x86_64/libgltfio.a 2>&1 | grep -c AssetLoader
```
  If it's there (expected — gltfio is a headline Filament feature, unlike the backend-internal `PlatformEGLHeadless` Epic 0 found missing), proceed with Step 4. **If not**, the fallback (document which one was needed, don't silently pick): a minimal hand-rolled GLB parser reading only POSITION/NORMAL/indices from the binary chunk (glTF's JSON+binary layout is well-documented and small for a single static mesh with no skinning/animation) — smaller than vendoring gltfio's own dependency tree (cgltf, draco, ktx) for one asset.

  **Deviation from this step's literal text (evidence, not blocked):** the exact command above returns `0`, not a positive count — but gltfio is NOT missing. The pinned SDK splits gltfio's static archive in two: `libgltfio.a` (`ar t`) contains only `JitShaderProvider.cpp.o` (an alternate, JIT-compiled material path this task doesn't use), while the real `AssetLoader`/`ResourceLoader`/`MaterialProvider`/`UbershaderProvider`/etc. implementations — everything Step 5 actually calls — live in a sibling archive, `libgltfio_core.a`, in the same `lib/x86_64/` directory (`ar t libgltfio_core.a` lists `AssetLoader.cpp.o`, `ResourceLoader.cpp.o`, `UbershaderProvider.cpp.o`, ...; `nm libgltfio_core.a | grep -c AssetLoader` → 36). The uberarchive default-material data Step 5 also needs (`UBERARCHIVE_DEFAULT_DATA`/`_SIZE`) is confirmed present too: `include/gltfio/materials/uberarchive.h` declares it and `libuberarchive.a` defines the symbols. Net: gltfio (full `AssetLoader`+`ResourceLoader`+ubershader-material stack) genuinely ships in this SDK, and no CMake change is needed to reach it: `cmake/GetFilament.cmake`'s `file(GLOB _filament_static_libs "${FILAMENT_ROOT}/lib/x86_64/*.a")` (Task 2 Step 5's note) already sweeps every `.a` in that directory — including `libgltfio_core.a` and `libuberarchive.a`, not just `libgltfio.a` — into `Filament::filament`'s `--start-group`/`--end-group` link line. Step 5 gets `AssetLoader`/`ResourceLoader`/`createUbershaderProvider`/`UBERARCHIVE_DEFAULT_DATA` for free from the existing target; no hand-rolled GLB parser fallback needed.
- [ ] **Step 4: Failing test.** `Ego.LoadValidGltf_RendersNonEmptyBoundingBox` loads the fixture committed in Step 1 (`tests/fixtures/test_cube.glb`) — not `{ /* ... */ }`, and not the user's non-git M02P asset. `#include "test_paths.hpp"` for `kThemeDir` and `#include "ego_test_hooks.hpp"` (new this task, see Files list) for `mpviz::testing::rendered_bounding_box_diagonal` — **not** `#include "renderer_internal.hpp"`: per Task 2 Step 7e that header now defines `Mesh` and pulls in `<filament/...>` headers, and `test_ego.cpp`'s test binary is only linked against `visual_renderer` (`target_link_libraries(${_test_name} PRIVATE visual_renderer gtest gtest_main EGL)`), never granted `visual_renderer`'s own PRIVATE Filament include dir — so it wouldn't compile there. `ego_test_hooks.hpp` declares the hook against `api.h`'s already-forward-declared opaque `mpviz::VisualRenderer` instead, which is all this test needs: it only ever holds the pointer, never dereferences the class. The fixture path uses the same `MPVIZ_TEST_DATA_DIR`-prefix convention as Task 2 Step 10:
```cpp
TEST(Ego, LoadValidGltf_RendersNonEmptyBoundingBox) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_TRUE(mpviz::set_ego_model(r, MPVIZ_TEST_DATA_DIR "/tests/fixtures/test_cube.glb", {4.5, 2.0, 1.8}));
    mpviz::SceneGraph scene{}; scene.ego = {{0,0,0}, 0, 0, /*valid=*/1};
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0,-8,3}, {0,0,0.5}, 60};
    mpviz::FrameView view{/*...*/};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));
    EXPECT_GT(mpviz::testing::rendered_bounding_box_diagonal(r), 0.0);  // ego.cpp exposes this test-only introspection hook; not part of api.h/scene.h
}
TEST(Ego, LoadMissingFile_FallsBackToClayBoxNonFatally) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_FALSE(mpviz::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));
    mpviz::SceneGraph scene{}; scene.ego = {{0,0,0}, 0, 0, /*valid=*/1};
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0,-8,3}, {0,0,0.5}, 60};
    mpviz::FrameView view{/*...*/};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));  // must not crash/fail — clay box instead
}
```
  Run — FAIL.

  **Deviation from this task's Files list (implemented, not a scope change):** the Files list at the top of Task 4 lists `Create: src/ego.hpp, src/ego.cpp` but never lists a `scene.h` modification, while the "Interfaces" section near the top of this document (before Task 1) explicitly places `set_ego_model` "new in `scene.h`" — the same section that already put `set_theme` there for Task 3. `test_ego.cpp` (this step) and the eventual node call site (Step 10, out of this scope) both need to call `mpviz::set_ego_model` having included only the POD public headers (`api.h`/`scene.h`) — a declaration living only in the internal, Filament-pulling `src/ego.hpp` would be invisible to both. Added the declaration to `include/visual_renderer/scene.h` instead (function-only addition, no existing struct/field touched — `scene.h`'s frozen POD *layout* is unchanged), matching the precedent `set_theme` already set there. `src/ego.hpp` ended up declaring only the per-frame internal counterpart, `update_ego_transform` (see Step 5's deviation note below) — `set_ego_model` itself is defined in `ego.cpp` directly against its `scene.h` declaration, no forward declaration duplicated in `ego.hpp`.
- [ ] **Step 5: Implement `ego.cpp`.** `#include "renderer_internal.hpp"` (Task 2 Step 7e, visible via the `-I src` Task 1 already added, and via `ego.cpp` being compiled as part of the `visual_renderer` library target itself so it shares that target's PRIVATE Filament include access) to see `class VisualRenderer` — `ego.cpp` is a separate translation unit from `renderer.cpp` and cannot add fields to or read fields off a class it can't see otherwise. Also `#include "ego_test_hooks.hpp"` and define `mpviz::testing::rendered_bounding_box_diagonal` here (it needs the full `VisualRenderer`/`Mesh` definitions from `renderer_internal.hpp` to compute anything, which is exactly why its declaration lives in the separate, Filament-free `ego_test_hooks.hpp` instead — see Step 4). Two Filament gltfio pieces are required for a loaded asset to actually render anything (both easy to miss — exactly the class of gotcha Epic 0 kept hitting), plus a material-remap decision:
  - `gltfio::AssetLoader::create()` needs a `MaterialProvider`. The pinned 1.56.5 SDK ships `gltfio/materials/uberarchive.h` + `libuberarchive.a` (glob-included already, see Task 2 Step 5's note) — use `gltfio::createUbershaderProvider(engine, UBERARCHIVE_DEFAULT_DATA, UBERARCHIVE_DEFAULT_SIZE)` as the `MaterialProvider` passed to `AssetLoader::create()`.
  - After `AssetLoader::create...FromBinary()`/`...FromJson()` returns a non-null `FilamentAsset*`, its geometry is NOT yet uploaded — `gltfio::ResourceLoader::loadResources(asset)` must run (synchronously; the GLB's buffers are embedded so there's no external URI to resolve asynchronously) before the asset has any visible geometry. Skipping this is the "silently renders nothing" trap the finding calls out.
  - Material remap (spec §4.2 says the ego should read as clay, matching the rest of the scene, not keep whatever materials came out of the OBJ→glTF conversion): after `loadResources`, walk `asset->getRenderableEntities()` and call `RenderableManager::setMaterialInstanceAt(entity, primitiveIndex, clay_instance)` for every primitive, pointing at the SAME opaque `clay.mat` `MaterialInstance` (or a per-entity instance of it) Step 2 already builds for the ground/ego-fallback box. This remap is only safe to do blind because Task 2's `clay.mat` fix dropped `requires: [color]` — the gltfio-loaded mesh has POSITION/NORMAL/UV attributes but no vertex COLOR, so remapping it to the OLD `clay.mat` (which required COLOR) would have failed; remapping to the current opaque `clay.mat` does not.
  - On any failure (file missing, parse error, `loadFilamentAsset` returns null), build a themed clay box (reuse Task 2's `clay.mat` ground-instance-creation pattern, sized to `fallback_dims`) by calling the namespace-scope `add_mesh(VisualRenderer&, Mesh&, ...)` free function Task 2 Step 7e promoted out of `create_renderer`'s local lambda for exactly this reason — a lambda local to `renderer.cpp` can't be called from `ego.cpp`, a different translation unit, so it had to become a real function declared in `renderer_internal.hpp` before this step could reuse it. Pass `cast_shadows = true` for both this fallback box and the gltfio remap path above — the ego is the one thing in Epic 1's scene that should actually darken the ground it stands on. Leave `receive_shadows` at its default `false` for the ego (nothing in this epic's scene casts onto the ego itself).
  - Store whichever entity got created on `VisualRenderer`. Per the Finding-3 fix to `set_scene`'s contract, `set_scene` itself must NOT touch this entity's `TransformManager` transform (that would be a Filament/Engine call from what may not be the render thread) — instead, `render_frame` updates the ego transform from `SceneBuffer::active().ego.position/heading_rad` at the top of every call, the same place Task 3's theme blend gets applied. No reloading either way.

  **Deviations from this step's literal text (implemented, evidence-based):**
  - **`AssetLoader::create...FromBinary()`/`...FromJson()` don't exist in the pinned 1.56.5 SDK.** Checked `include/gltfio/AssetLoader.h` directly: the single-instance entry point is `FilamentAsset* AssetLoader::createAsset(const uint8_t* bytes, uint32_t nbytes)` — it accepts either JSON or GLB content (the class comment says so explicitly) and returns one instance, which is all a single static ego mesh needs. Used that instead of the two differently-named methods the step's prose assumed; `createInstancedAsset` (the actual `FromBinary`-adjacent multi-instance API) was not needed.
  - **Shared `MaterialInstance`, not a separate ego one.** Took the step's own "(or a per-entity instance of it)" parenthetical literally in the direction of *reuse*: both the clay-box fallback's `add_mesh(...)` call and the glTF remap's `setMaterialInstanceAt(...)` loop point at `r->groundMaterial` (Task 2's existing clay.mat instance) directly, not a new `r->egoMaterial` instance. Smaller diff (no second theme-push wiring for a second instance to track through `push_theme_to_scene`/`apply_current_theme`), and exactly as correct per spec §4.2's intent (the ego should read as the *same* clay as the ground, not a distinctly-tinted one — no ego-specific palette token exists in `theme.hpp`/the shipped YAMLs).
  - **`ego.valid == 0` hide mechanism (not specified by this step's text at all — `scene.h`'s own `EgoState::valid` comment says "ego hidden," this step doesn't say how):** implemented as a zero-scale `TransformManager` transform (`translation(pos) * rotation(heading) * scaling(valid ? 1.0f : 0.0f)`) in `update_ego_transform`, rather than toggling `Scene::addEntities`/`removeEntities` membership. A zero-scale renderable has no visible extent and casts no shadow, and this avoids having to track "is the glTF asset's full entity list (root + N node entities) currently added to the scene" as separate mutable state. Upgrade path noted inline in `ego.cpp` if a future epic needs to skip the vertex-shader cost too, not just hide the result.
  - **`fill_tangent_frames` duplicated into `ego.cpp`, not promoted into `renderer_internal.hpp`.** Task 2 Step 7e's extraction explicitly named `Mesh`/`Vertex`/`make_vertex_buffer`/`make_index_buffer`/`add_mesh` as what `ego.cpp` needs from `renderer.cpp`'s former anonymous namespace — `fill_tangent_frames` (a small `SurfaceOrientation`-wrapping helper the ego's box-builder also needs) wasn't among them. Rather than widening that already-deliberate extraction surface for one more helper, `ego.cpp` carries its own small, self-contained copy (same body). Noted here rather than silently diverging from the named Step 7e list.
- [ ] **Step 6: Run — PASS** for both `Ego` tests above. Golden: `tests/test_ego.cpp` `EgoGolden_ClayBoxFallback_DarkAdas` (ego at a fixed pose using the clay-box fallback path — no glTF asset — since the CI-committed golden must not depend on the user's non-git M02P asset; `test_cube.glb` is exercised by the correctness test above, not by a committed golden). Same `quality=1` (medium) `RenderConfig` as Task 2/3's goldens — this is the first golden with an actual box in frame, so it's the first to visibly exercise Step 7a's SSAO contact darkening.

  **Status:** all three `Ego`/`EgoGolden` tests pass, alongside the full existing 23-test suite (26/26 green). The golden PNG itself (a themed clay box, front-lit with a visible ground-contact shadow, on the `dark_adas` grid) was rendered and committed to `tests/goldens/ego_clay_box_dark_adas.png` per this step's own EXPECT_GT(ssim, 0.98) — but per this project's stated working style ("user judges by visuals"), the candidate render was also handed to the user for a visual sign-off before this task is treated as done; nothing in this workflow was committed to git regardless (no task in this handoff commits).

  **Additional deviation (review round — `Ego.LoadValidGltf_RendersNonEmptyBoundingBox` strengthened):** the review found `rendered_bounding_box_diagonal()` (Step 5) reads `FilamentAsset::getBoundingBox()`, which gltfio computes from the glTF's JSON accessor `min`/`max` at `createAsset()` time — i.e. purely from parsing, before `ResourceLoader::loadResources()` uploads anything or `Scene::addEntities()` puts the asset in the scene. Deleting either call left the test (and the full 26-test suite) green while the glTF path rendered nothing, exactly the "silently renders nothing" trap this step's own text calls out — the test just wasn't the thing catching it. Fixed with **no change to `ego.cpp`/production code**, only `tests/test_ego.cpp`: the test now also renders a baseline frame from a *second* renderer on which `set_ego_model()` is never called at all (`r->egoTransformEntity` stays null, `update_ego_transform()` is a no-op — a clean "no ego" reference with zero ambiguity from partially-uploaded GPU state), then asserts the glTF-loaded frame's pixels differ from that baseline, in addition to the existing bounding-box check. Verified discriminating by temporarily commenting out the `addEntities()` call in `ego.cpp` and re-running: the strengthened test failed (`differing_bytes` == 0) exactly as the finding predicted, then passed again once the call was restored — evidence, not assertion. (Commenting out `loadResources()` instead segfaults inside Filament rather than silently rendering nothing — the old test would already have caught *that* variant via its own `EXPECT_TRUE(render_frame(...))`; the pixel-diff addition specifically closes the `addEntities()`-skipped gap, which crashes nothing and produces a "successful," empty render.)

### 4c. TF adapter (node side)

- [ ] **Step 7: Failing test.** `test_tf_adapter.py` — recorded fixture: a short sequence of `map→base_link` transforms at known timestamps/positions (straight-line motion at a known constant speed, e.g. 2.0 m/s), fed through the adapter, asserting the finite-differenced+smoothed speed converges to 2.0 m/s within a tolerance after the smoothing filter's settling time (and that a single noisy outlier sample doesn't spike the reported speed — that's what the smoothing is *for*):
```python
# test_tf_adapter.py: publish the fixture's static/dynamic transforms on a
# real tf2 buffer at 20 Hz, run the node, sample ~/vcam_state-adjacent ego
# telemetry (or a dedicated debug topic if simpler), assert convergence.
```
  Run — FAIL (package doesn't exist).

  **Deviations from this step's literal text (implemented, evidence-based):**
  - **`python -m pytest` → `python3 -m pytest`.** This box has no bare `python` binary (only `python3`, even after sourcing `/opt/ros/humble/setup.bash`) — `which python` resolves to nothing, `which python3` resolves to `/usr/bin/python3`. Ran the literal command with `python3` substituted; behavior is otherwise identical (both `test_tf_adapter.py`'s own docstring and Step 11's block below use `python3`).
  - **"sample `~/vcam_state`-adjacent ego telemetry (or a dedicated debug topic if simpler)" → took the parenthetical: added a dedicated `~/ego_state` debug topic.** `~/vcam_state` itself is a fixed 8-float layout (`[eye xyz | target xyz | active_preset | active_mode]`, Task 5's frozen contract surface) with no ego fields and no room to add any without breaking `test_vcam_contract.py`'s exact-layout assertions — piggybacking ego data onto it would have meant widening a contract this epic's review gate freezes, for a debug-only signal. Instead `visualization_node.cpp` gained `pub_ego_state_` (`~/ego_state`, `std_msgs::msg::Float64MultiArray`, `[x, y, z, heading_rad, speed_mps, valid]`), published every timer tick right after `scene.ego` is built — same per-tick `LifecyclePublisher` pattern as `~/vcam_state`, just its own topic. `test_tf_adapter.py` subscribes to `/visualization_node/ego_state`.
  - **Test structure: a real `pytest` test function, not a `main()`/`sys.exit` script.** `test_vcam_contract.py`/`smoke_test.py` (invoked directly via `python3 script.py`) are plain scripts, but Step 11 below invokes *this* file specifically via `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test_tf_adapter.py` — a `main()`-only module has zero collected tests, and pytest exits nonzero (status 5, "no tests collected") in that case, which would make Step 11 fail on a file that "ran clean" by inspection. Wrote `def test_tf_adapter_speed_converges()` with plain `assert`s instead, launching/tearing down the node the same subprocess+lifecycle way the other tests do, internally.
  - **Outlier-ceiling constant tuned empirically, not guessed.** The module derives `OUTLIER_SMOOTHED_CEILING_MPS` from the EMA math (`smoothed_new = smoothed_prev + alpha*(raw - smoothed_prev)`, `raw` for the injected 5 m sample ≈ 100+ m/s) but the exact one-tick jump depends on the real wall-clock `dt` between the node's own 33 ms-cadence reads of a TF buffer fed at 20 Hz — measured empirically at ~36.7 m/s on this box (comfortably far below the ~100+ m/s raw value, proving the EMA damped it, but above an initially-guessed 30.0 ceiling). Set to `60.0` — still an order of magnitude below the undamped raw spike, with headroom over the measured value for run-to-run timing jitter (project note: never assume an idle/uncontended GPU or CPU scheduler).
- [ ] **Step 8: Implement `tf_adapter.cpp`.** Exponential-moving-average smoothing over raw finite-difference speed (`raw = |pos_t - pos_{t-1}| / dt`; `smoothed += alpha * (raw - smoothed)`, `alpha` a small node param e.g. `ego_speed_smoothing_alpha` default `0.2`) — the simplest filter that kills single-sample noise without adding latency-tuning complexity (a Kalman filter would be gold-plating for a HUD speed readout, not a control input). `lookupTransform("map", "base_link", tf2::TimePointZero)` each tick; `tf2::LookupException`/`ExtrapolationException` caught → `ego.valid = 0` (no TF yet), not a crash, not a clay box parked at the origin (spec's non-fatal-failure philosophy extended to "no data" as well as "bad data").

  **Deviations from this step's literal text (implemented, evidence-based):**
  - **Caught `tf2::TransformException` (the shared base class), not the two named subtypes separately.** Both `tf2::LookupException` and `tf2::ExtrapolationException` derive from `tf2::TransformException`; a single `catch (const tf2::TransformException&)` covers both (and `ConnectivityException`/`InvalidArgumentException`, which `Buffer::lookupTransform` can also throw per its own header comment) without duplicating the `ego.valid = 0` handling per subtype. Standard idiom, not a narrowing of the step's intent.
  - **`TfAdapter` doesn't own the `Buffer`/`TransformListener` — takes a `tf2_ros::Buffer&`.** The Files list itself puts this precisely: "`visualization_node.{hpp,cpp}` (own `tf2_ros::Buffer`/`TransformListener` ...)" — constructing a `TransformListener` needs the node's own `NodeInterfaces`, which only the node has. `TfAdapter` (this step) is deliberately the thinner, independently-reasoned-about layer on top: it holds only the small bit of finite-difference/EMA state (previous position/stamp, smoothed speed) and a reference to the node-owned `Buffer`.
  - **Heading from quaternion computed with a manual `atan2` formula, not `tf2::getYaw`.** `tf2::getYaw` needs `tf2_geometry_msgs`/`tf2::Quaternion` conversions for a `geometry_msgs::msg::Quaternion` — one extra header and an extra type round-trip for a single well-known formula (`atan2(2(wz+xy), 1-2(y²+z²))`). No new dependency, same result.
- [ ] **Step 9: Run — PASS.**

  **Status:** `test_tf_adapter_speed_converges` passes (verified twice in a row for flakiness — both green, ~20s each).

  **Additional deviation (review round — missing speed-source preference, spec §7):** `docs/superpowers/specs/2026-08-18-visual-mode-design.md:264` (added by a later commit, after this task's plan text was written, but binding as the spec) requires ego speed to PREFER `/robot/feedback/robot_speed_mps` (Float32, live-sim finding 2026-08-19) with the TF finite-difference/EMA above as the fallback — this task's original Step 8 implemented only the fallback and never subscribed to that topic, silently dropping the spec's preferred path with no deviation note. Fixed: `TfAdapter` gained `set_robot_speed_mps(double)` and an `std::optional<double> topic_speed_mps_` — `update()` now reports `topic_speed_mps_` once set, falling back to the existing `smoothed_speed_` EMA only until the first sample arrives (the EMA keeps running underneath regardless, so it stays warm rather than cold-starting if a future epic adds staleness reversion). The node (`visualization_node.cpp`) owns the actual `rclcpp::Subscription<std_msgs::msg::Float32>` on the global (not `~/...`) topic `/robot/feedback/robot_speed_mps`, forwarding each sample into the adapter — same ROS-machinery-lives-on-the-node split as the `Buffer`/`TransformListener` above. No staleness timeout was added (once a sample arrives, it is preferred permanently, not just while "fresh"): the message type (`std_msgs/Float32`) carries no header/stamp to judge staleness from, and the finding's own failure scenario is about the preference being entirely absent, not about reversion after the topic goes silent — YAGNI given the spec's literal "prefers X with Y as fallback" wording, flagged here as the ceiling to revisit if the feedback topic is ever observed dropping out mid-session. Regression test added: `test_robot_speed_topic_preferred_over_tf_diff` in `test_tf_adapter.py` — TF motion implies ~2.0 m/s while the topic reports a deliberately different 9.0 m/s, asserting the reported `~/ego_state` speed tracks the topic value (proving preference, not just that both paths individually work). Passes, run twice for flakiness — both green (~20s each), alongside the original convergence test.
- [ ] **Step 10: Wire into `visualization_node.cpp`.** `on_configure`: declare `ego_model_path` (default `""`), `ego_fallback_dims` (default `[4.5, 2.0, 1.8]`), call `mpviz::set_ego_model` once (log WARN on `false`, non-fatal, exactly like `robot_model_path` today). `timer_callback`, before `set_scene`: `tf_adapter_->update(sim_clock_sec_)` → fills `SceneGraph.ego`.

  **Deviations from this step's literal text (implemented, evidence-based):**
  - **`tf_adapter_->update()` takes no `sim_clock_sec_` argument.** The adapter finite-differences off the TF transform's own `header.stamp` (real wall-clock time the TF was published, converted to `rclcpp::Time`), not the node's simulated render clock — `sim_clock_sec_` drives `SceneGraph::sim_time_sec`/theme-transition timing (Task 3), an unrelated clock with a different meaning (it advances by a fixed `kTimerPeriodSec` per tick regardless of real elapsed time, per that task's own deterministic-clock contract) and would give wrong-unit finite differences if used here instead of real TF timestamps.
  - **`tf2_ros::TransformListener` constructed with `this` (raw pointer), not `*this`.** Tried `*this` first per the natural reading of "own a TransformListener" — failed to compile: `TransformListener::init()`'s templated `NodeT` body unconditionally does `node->get_node_base_interface()` (pointer-style access only; verified by reading `/opt/ros/humble/include/tf2_ros/tf2_ros/transform_listener.hpp` after the reference-based call failed), so `NodeT` must be pointer-like (raw pointer or `shared_ptr`), not a reference. Passed `this` instead — same object, no behavior change, just the syntax `init()` actually accepts.
  - **Added `~/ego_state` publisher/activation/teardown wiring** (`pub_ego_state_`, activated in `on_activate`, deactivated in `on_deactivate`, reset in `on_cleanup`/`on_shutdown` alongside the other per-tick publishers) — needed by Step 7's deviation above (the debug topic `test_tf_adapter.py` samples); not separately called out in this step's literal text but implied by "before `set_scene`: ... → fills `SceneGraph.ego`" needing to be externally observable for Step 7's test to work at all.
  - **`CMakeLists.txt`/`package.xml` additions not explicitly named by this step** (`find_package(geometry_msgs/tf2/tf2_ros)`, `<depend>` for the same, `src/tf_adapter.cpp` added to `visualization_node_lib`'s source list) — these are exactly what this task's own Files list already calls out as required (`Modify: node CMakeLists.txt`, `Modify: node package.xml`) and are covered under Step 8/10's implementation, not a new deviation — noted here only because package.xml's own comment (added in this change) originally used a literal `--` inside an XML comment (`"declaration -- a missing <depend>"`), which is illegal XML (comments may not contain the two-hyphen sequence) and broke `ament_package_xml`'s parse with `not well-formed (invalid token)`. Reworded to avoid `--`/literal tag-shaped text inside the comment; no content change.
- [ ] **Step 11: Full rebuild + regression.**
```bash
cd cuda/src/libs/visual_renderer && cmake --build build && ctest --test-dir build --output-on-failure
cd cuda/scripts/ros_apps_build && ./colcon_build.sh
source /opt/ros/humble/setup.bash && source ../../install/ros_apps/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest cuda/src/ros_apps/src/micropilot_visualization_node/test/test_tf_adapter.py
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/smoke_test.py   # still green, unmodified
```

  **Status:** all green.
  - `ctest --test-dir build`: 26/26 passed (unchanged from Steps 1-6 — this task's Steps 7-11 touched no library code).
  - `colcon_build.sh` (both the single-package and full/all-packages form): builds clean, `micropilot_visualization_node` links and installs.
  - `python3 -m pytest .../test_tf_adapter.py` (literal `python` substituted per Step 7's deviation note): 1 passed, run twice for flakiness — both green.
  - `smoke_test.py`: unmodified, still passes (mode-3 now also carries a rendered ego — default `ego_model_path=""` → clay-box fallback — with no crash/regression).
  - Additional Global-Constraints regression run beyond this step's own literal list (per this task's own handoff instructions, not a scope add): `test_vcam_contract.py` (PASS, unmodified), `test_theme_ws.py` (PASS, unmodified), `tools/test_vcam_ws_bridge.py` via pytest — 37/37 passed, including `test_bridge_e2e_mode3_orbit_and_frames` (the "WS bridge E2E" the Epic 1 review gate names explicitly).

  **Re-run after the review round above (speed-preference + test-discrimination fixes):** `ctest --test-dir build` still 26/26 (the strengthened `Ego.LoadValidGltf_RendersNonEmptyBoundingBox` is test-only, no new test added/removed). `test_tf_adapter.py` now 2/2 (`test_tf_adapter_speed_converges` + the new `test_robot_speed_topic_preferred_over_tf_diff`), each run twice for flakiness — all green. `colcon_build.sh`, `smoke_test.py`, `test_vcam_contract.py`, `test_theme_ws.py`, and `tools/test_vcam_ws_bridge.py` (37/37) re-run clean, unaffected by either fix.
- [ ] **Step 12: Commit** `feat(visual): ego robot from M02P glTF, TF-driven pose + smoothed speed, clay-box fallback (VM-012)`.

  **Status:** NOT done — no commit made in this handoff (scope explicitly excluded it; Step 2, the manual M02P.obj→glTF conversion against the user's real Downloads asset, is also still pending for the same reason: it's a one-time manual step against a non-git asset, not something this handoff can do).

---

## Task 5 (VM-013): Extract the vcam tween into `src/vcam.cpp`

**Scope addition (2026-08-19 user directive, live validation session):** mode 3's vcam (presets, orbiting, `~/set_look`) now composes with a follow-ego camera instead of fighting it. `presets_`/`cur_`/`src_`/`dst_`/the tween/`on_set_look`/`~/vcam_state` all still operate in an unchanged EGO-RELATIVE OFFSET frame; `visualization_node.cpp`'s `timer_callback` composes the final render pose only at the point it's handed to `render_frame()` — yaw-rotating the tweened offset eye/target about +Z by `(scene.ego.heading_rad - kEgoForwardYaw)` and translating by `scene.ego.position`, when `scene.ego.valid`; falling back to today's absolute-world-pose behavior otherwise (bit-identical for `test_vcam_contract.py` and every no-TF test). This composition is stateless and one-way — it never feeds back into the offset/tween/telemetry — so this task's "move, don't rewrite" extraction of the vcam tween must carry the composition step along with it (it belongs at the render-pose boundary, not inside `Vcam` itself, since `Vcam` only knows the offset frame).

**Known limitation (2026-08-19 user directive, live validation session — deferred, not fixed here):** with the camera now ego-anchored, the ego can drive off the edge of the static ground/grid. `build_ground_plane`/`build_grid_lines` build a fixed 40m extent centered on the *world origin* once, and the grid's line alpha fades by each line's radial distance from that same origin — neither is a function of the ego's live position. Once the ego travels beyond roughly 20m from the origin, the follow-cam frames it against bare void: the ground it's actually standing on has already faded out or ended. This is Epic-2-adjacent (real map/OGM ground content) rather than an Epic 1 defect, so it is being recorded, not fixed, pending a user decision between two candidates: (a) re-centre the ground/grid under the ego every frame, quantizing the re-centre step to the grid spacing so the lines don't visibly swim as the ego moves, or (b) leave the static origin-centered ground as today's placeholder and wait for Epic 2 to supply real map/OGM-derived ground geometry that has no origin-centered extent to outrun.

**Reality check (per the master plan's note — Epic 0 Task 5 already ported and proved the tween math correct via `test_vcam_contract.py`; this task does NOT re-derive or change any behavior):** today `LookPoint`, the preset table, `advance_tween()`, `on_set_virtual_cam()`, `on_set_look()`, and their ROS service/subscription wiring all live inline in `visualization_node.{hpp,cpp}`, mixed in with lifecycle callbacks, the mode-mux subscription, and the render timer. That was fine when the file only had to prove the mux+vcam contract (Epic 0). Epic 1 is about to add scene construction (`SceneGraph` assembly), the TF adapter call, and theme WS wiring to the same timer callback — the file is not "already clean" by the master plan's own criterion, so this task is a small, mechanical, behavior-preserving extraction, not a redesign.

**Files:**
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/vcam.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/vcam.cpp`
- Modify: `visualization_node.{hpp,cpp}` (delete the extracted members/methods, own a `std::unique_ptr<Vcam>` instead)
- Modify: node `CMakeLists.txt` (add `src/vcam.cpp` to `visualization_node_lib`'s sources)

**Interfaces:** none new — this is a pure refactor behind the node's existing ROS-visible surface (`~/set_virtual_cam`, `~/set_look`, `~/vcam_state`), which does not change at all.

- [ ] **Step 1: Confirm the safety net exists and is green before touching anything.**
```bash
source /opt/ros/humble/setup.bash && source cuda/install/setup.bash
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_vcam_contract.py
```
- [ ] **Step 2: Move, don't rewrite.** `vcam.hpp` declares `class Vcam` owning exactly what's listed in the reality-check paragraph (`LookPoint`, `presets_`, `cur_/src_/dst_`, `tween_t_`, `active_preset_`, the `SetVirtualCam` service and `set_look` subscription, `smoothstep()`, `advance_tween()`, `on_set_virtual_cam()`, `on_set_look()`). Constructor takes the owning `rclcpp_lifecycle::LifecycleNode*` (to create the service/subscription against) plus the initial `virtual_pose`/`virtual_vfov_deg`-derived `LookPoint` (seeds `presets_[0]`, identical formula to today's inline code). Public surface: `void advance_tween()`, `const mpviz::CameraPose& pose() const`, `int active_preset() const`. Copy the bodies verbatim (this is find-and-move, not reimplementation — a byte-diff of the tween math against the pre-refactor version should show zero changes, only the enclosing class/file changed).
- [ ] **Step 3: `visualization_node.cpp` shrinks.** `on_configure`: `vcam_ = std::make_unique<Vcam>(this, seed_pose);` replaces the inline preset-table setup. `timer_callback`: `vcam_->advance_tween(); pose_ = vcam_->pose();` replaces the direct member access. `on_cleanup`/`on_shutdown`: `vcam_.reset();` replaces the individual `set_vcam_srv_.reset(); set_look_sub_.reset();`.
- [ ] **Step 4: Rebuild + re-run the exact same contract test — must still PASS, unchanged assertions, unchanged tolerances.**
```bash
cd cuda/scripts/ros_apps_build && ./colcon_build.sh micropilot_visualization_node
source /opt/ros/humble/setup.bash && source ../../install/ros_apps/setup.bash
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/test_vcam_contract.py
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/smoke_test.py
```
- [ ] **Step 5: Commit** `refactor(visual): extract vcam preset/tween into src/vcam.cpp (VM-013)` — no behavior change, so the commit body should say exactly that and point at Step 4's unchanged-test evidence.

---

## Epic 1 review gate

Opus reviewer signs off against:
- Spec §4.1 (all 8 SceneGraph categories present as POD, Ego populated), §4.2 (lit pipeline replacing the Epic 0 unlit deviation — sun + IBL both visibly doing something, not dead code), §4.3 (both themes, `set_theme` animated per the 0.8s/Oklab/smoothstep contract), §4.4 (ego glTF + non-fatal fallback), §9 (asset-load-failure and no-TF-yet paths both non-fatal), §10 (golden-image harness exists and is GPU-skip-clean).
- Global Constraints: existing Epic 0 tests (`test_hello_frame`, `smoke_test.py` ×2 nodes, `test_vcam_contract.py`, the WS bridge E2E) all still green.
- The "Frozen after Epic 1" interfaces (Interfaces block above) match what actually shipped, byte-for-byte — if a reviewer finds a field renamed/reordered mid-epic, that's a blocking finding, not a nit (Epic 2 is scheduled from this doc's frozen signatures).
- No task exceeded its stated scope (e.g., Task 5 stayed a pure refactor; Task 2's IBL simplification is documented, not silently narrower than spec; Task 2's SSAO/FXAA-vs-TAA mapping is wired per `RenderConfig::quality` and baked into this epic's goldens at `quality=1`, while shadow-map-resolution/render-resolution stay an explicit, named deferral to Epic 3's VM-032, not a silent scope drop).

**Epic 1 results (fill on completion):** golden SSIM thresholds actually used = `____`, gltfio-in-prebuilt-SDK outcome (Task 4 Step 3) = `____`, any perf numbers incidentally observed (informational only — Task 6 is still the gate) = `____`.
