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

Epic 0 Task 6 (on-robot GPU budget measurement) has not run yet. Per project direction: default quality preset = **medium**, target resolution 1280×720@30. Nothing in this epic should be tuned against unmeasured numbers — goldens render at the **low** preset (smallest, most deterministic) purely for CI speed/determinism, not as a statement about the shipped default. Do not gold-plate perf work here; Epic 5 (VM-040/041) owns hysteresis/auto-drop/benchmarking.

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
// internal staging buffer and atomically publishes it as the active scene —
// safe to call from the same thread as render_frame() (today's single-
// threaded executor) or, later, from a different ingest thread. `scene`'s
// arrays may be freed/reused the instant this call returns. Does NOT render;
// render_frame() always renders the last-published active scene, so a tick
// with no set_scene() call re-renders the previous one (freeze-frame, same
// philosophy as micropilot_rendering_node).
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

### `RenderConfig` gains two optional fields (`api.h`, additive — existing callers using aggregate-init with named fields are unaffected; positional-init callers must add trailing zeros, there are none yet outside Epic 0's own tests, which Task 2 updates)

```c
struct RenderConfig {
    uint32_t width, height;
    uint8_t quality;             // 0=low, 1=med, 2=high
    const char* theme_assets_dir;  // nullable: dir containing *.yaml theme files
    const char* initial_theme;     // nullable: default "dark_adas"
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

**Interfaces:** produces `mpviz::set_scene` (frozen above) and the internal `mpviz::detail::SceneBuffer` class Task 2/4 build on (ground/grid/fog rendering and ego rendering both read `SceneBuffer::active()`).

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
#include <vector>
#include "visual_renderer/scene.h"

namespace mpviz::detail {

// Owns std::vector storage for every array SceneGraph points into, so a
// mpviz::SceneGraph handed out by active() has valid pointers for as long as
// this object isn't republished. NOT itself passed across the POD boundary —
// internal only.
struct OwnedScene {
    mpviz::SceneGraph view{};  // pointers below point into this object's own vectors
    std::vector<TrackedObject> objects;
    std::vector<PathRibbon> paths;
    std::vector<MapElement> map_elements;
    std::vector<GroundGridLayer> grids;
    std::vector<AlertPolygon> alerts;
    std::vector<GenericMarker> markers;
    std::vector<AlertChip> chips;
    // Deep-copies `src` (including nested Vec3[]/char* payloads reached by
    // the arrays above) into this object's vectors and repoints view's
    // pointers at them.
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
                                  // today's single-threaded executor never
                                  // contends it, but it's what makes a future
                                  // multi-threaded ingest safe without a
                                  // SceneBuffer redesign — matches the
                                  // architecture the spec (§4.1) already commits to.
    OwnedScene slots_[2];
    int active_idx_{0};
};

}  // namespace mpviz::detail
```
  `publish()`: `assign()` into `slots_[1 - active_idx_]`, then swap `active_idx_` under `mutex_`. `active()`: lock, read `active_idx_`, return `slots_[active_idx_].view` (the lock only protects the index read/write, not the whole render — fine, since nothing mutates a slot while it's the active one; the *other* slot is always the one being written).
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
- Create: `cuda/src/libs/visual_renderer/assets/materials/clay.mat` (lit, replaces `simple_color.mat` for themed surfaces)
- Create: `cuda/src/libs/visual_renderer/assets/themes/dark_adas.yaml`
- Create: `cuda/src/libs/visual_renderer/assets/themes/light_clay.yaml`
- Create: `cuda/src/libs/visual_renderer/src/theme.hpp`, `src/theme.cpp` (YAML → internal `Theme` struct; NOT POD, internal-only)
- Create: `cuda/src/libs/visual_renderer/tests/test_theme.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/golden.cpp`, `tests/golden.hpp` (shared render+compare harness, linked into gtest binaries)
- Create: `cuda/src/libs/visual_renderer/tests/golden.py` (dev-only: regenerate/inspect a committed golden PNG; not on the ctest path)
- Create: `cuda/src/libs/visual_renderer/tests/goldens/empty_world_dark_adas.png`, `tests/goldens/empty_world_light_clay.png` (committed binaries, generated by Step 8)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (drop the Epic 0 spike scene: unlit cube/`simple_color.mat` ground+grid; replace with themed ground+grid+fog on `clay.mat`, sun + analytic-IBL wired for real)
- Modify: `cuda/src/libs/visual_renderer/CMakeLists.txt` (FetchContent yaml-cpp; exclude `tests/golden.cpp` from the auto-gtest glob, compile it as a plain object linked into the other test binaries instead)
- Modify: `cuda/src/libs/visual_renderer/tests/test_hello_frame.cpp` (delete or repoint — Epic 0's cube-scene assertions no longer hold once Task 2 replaces the scene; see Step 1)

**Interfaces:** internal `mpviz::detail::Theme` struct and `Theme load_theme(const char* dir, const char* name)` (used by Task 3's transition code too); no new public POD (theme *names* cross the boundary as `const char*` via `set_theme`/`RenderConfig`, nothing else needs to).

### 2a. Retire the Epic 0 spike scene first

- [ ] **Step 1:** `test_hello_frame.cpp` asserts "sky rows differ from ground rows" against Epic 0's fixed camera pose/scene — that assumption survives (there's still a ground+sky), but the pixel values it silently depended on (unlit flat colors) don't. Re-run it now, before touching `renderer.cpp`, to confirm it's currently green (baseline). Leave it as a coarse smoke check; the real regression protection from here on is the golden-image tests (Step 8+), which is the point of this task.

### 2b. The lit material

- [ ] **Step 2:** Write `clay.mat`:
```
material {
    name : clay,
    shadingModel : lit,
    parameters : [
        { type : float3, name : baseColor },
        { type : float, name : roughness },
        { type : float, name : metallic },
        { type : float4, name : vertexAlpha, precision: high }
    ],
    requires : [ color ],
    blending : transparent
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        material.baseColor.rgb = materialParams.baseColor;
        material.roughness = materialParams.roughness;
        material.metallic = materialParams.metallic;
        material.baseColor.a = getColor().a;  // per-vertex fade (grid distance fade, Step 6)
    }
}
```
  (`blending: transparent` + per-vertex color alpha is how the grid's distance fade (Step 6) and later staleness fade (Task 1's `staleness_alpha`, consumed starting Epic 2) both flow through one material — no second "faded" material variant.) matc-compiles via the same `CMakeLists.txt` configure-time block Epic 0 already added for `simple_color.mat` (glob picks it up automatically).
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
fog: { density: 0.015 }
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
    URL_HASH SHA256=<compute-and-record-on-first-fetch>)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yamlcpp)
# link `yaml-cpp` PRIVATE into visual_renderer, same as Filament — never
# leaks past the POD api.h/scene.h boundary.
```
- [ ] **Step 6:** Run test_theme.cpp's material test again — still needs `renderer.cpp` wiring (next).

### 2d. Wire it into `renderer.cpp`

- [ ] **Step 7:** Replace the Epic 0 scene construction:
  - Drop `simple_color.mat`/`colorMaterial`/the cube entirely (dead spike geometry).
  - `create_renderer`: resolve `theme_assets_dir`/`initial_theme` from `RenderConfig` (default dir = compiled-in `DEFAULT_THEME_ASSETS_DIR`, default theme = `"dark_adas"`), `load_theme()`, build one `clay.mat` `MaterialInstance` for `ground` (flat, `vertexAlpha=1` everywhere) and one for `grid` (per-vertex alpha baked from radial distance vs. `grid.fade_start_m/fade_end_m` — the "fading with distance" AC from spec §7, computed once at grid-build time since the grid is static geometry, not per-frame).
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
  - Fog: `filament::View::setFogOptions({.color = theme.palette.fog, .density = theme.fog.density, ...})` (Filament's built-in distance fog — native feature, no custom skybox mesh) and clear color set to `theme.palette.sky` (flat "sky" backdrop, matching the clay/flat aesthetic of both references — no skydome geometry).
  - `set_scene`/ego rendering plug in here too (Task 4) but Task 2 only needs `SceneGraph.sim_time_sec` piped through so Task 3's transition math (next) has a clock — no ego yet.
- [ ] **Step 8: Run — PASS** (`ClayMaterial.RespondsToLightDirection` and any ground/fog assertions).

### 2e. Golden-image harness (used by every later epic — build it right once)

- [ ] **Step 9:** `tests/golden.hpp`/`golden.cpp` — a small reusable helper, **not** a gtest file itself (excluded from the CMakeLists auto-glob-as-gtest-binary by name so it doesn't fight `gtest_main`; instead added as a plain source compiled directly into whichever test binary calls it, same pattern as `scene_buffer.cpp`):
```cpp
// tests/golden.hpp
namespace mpviz::testing {
// Renders one frame of `scene` under `theme_name` at `cfg`/`pose`, writes it
// to `out_png_path` (stb_image_write, already vendored for examples/), and
// returns a block-SSIM score in [0,1] against `golden_png_path` (0 if the
// golden doesn't exist yet — first run of a new golden always fails loudly,
// never silently "passes" with nothing to compare against).
// Returns -1.0 (caller must GTEST_SKIP()) if create_renderer() fails, i.e.
// no GPU/EGL device — identical convention to test_hello_frame.cpp.
double render_and_compare(const mpviz::RenderConfig& cfg, const char* theme_name,
                           const mpviz::SceneGraph& scene, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path);
}
```
  SSIM implementation: block-wise (8×8, non-overlapping, luminance-only) mean/variance/covariance SSIM averaged over blocks — a deliberately simpler approximation of the full windowed-Gaussian SSIM (unnecessary precision for a pass/fail regression gate at low-preset resolution); document that choice with a `ponytail:` comment in `golden.cpp`.
- [ ] **Step 10:** `tests/test_theme.cpp` golden cases:
```cpp
TEST(ThemeGolden, EmptyWorld_DarkAdas) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/0, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    mpviz::SceneGraph scene{};  // empty: ego.valid=0, every count=0
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    mpviz::CameraPose pose{{0,-8,4}, {0,0,0}, 60.0};
    double ssim = mpviz::testing::render_and_compare(
        cfg, "dark_adas", scene, pose,
        "tests/goldens/empty_world_dark_adas.png", "/tmp/empty_world_dark_adas_actual.png");
    EXPECT_GT(ssim, 0.98);
    mpviz::destroy_renderer(r);
}
TEST(ThemeGolden, EmptyWorld_LightClay) { /* identical, theme="light_clay" */ }
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
- [ ] **Step 4: Run — PASS.**
- [ ] **Step 5: Failing test — the transition state machine itself**, deterministic clock (this is the "deterministic-clock goldens" AC):
```cpp
TEST(ThemeTransition, DeterministicClock_MatchesTargetAtDuration) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);                 // t=0, still dark_adas
    mpviz::set_theme(r, "light_clay", 0.0, 0.8); // begin transition at t=0

    scene.sim_time_sec = 0.0;
    // golden at t=0.0 (still ~dark_adas — first tick of the transition)
    EXPECT_GT(mpviz::testing::render_and_compare(cfg, "dark_adas", scene,
        kFixedPose, "tests/goldens/transition_t0.png", "/tmp/t0.png"), 0.98);

    scene.sim_time_sec = 0.4;
    mpviz::set_scene(r, scene);
    // golden at t=0.4s (~50% blended — its own committed midpoint golden,
    // not compared against either endpoint)
    EXPECT_GT(mpviz::testing::render_and_compare(cfg, "midpoint", scene,
        kFixedPose, "tests/goldens/transition_t0_4.png", "/tmp/t0_4.png"), 0.98);

    scene.sim_time_sec = 0.8;
    mpviz::set_scene(r, scene);
    // golden at t=0.8s (fully light_clay — transition_sec elapsed)
    EXPECT_GT(mpviz::testing::render_and_compare(cfg, "light_clay", scene,
        kFixedPose, "tests/goldens/transition_t0_8.png", "/tmp/t0_8.png"), 0.98);
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
- [ ] **Step 9: Node wiring.** `visualization_node`: maintain `double sim_clock_sec_` (monotonic, incremented by the timer's own period each tick — `33ms` — rather than reading wall-clock, so the deterministic-clock contract holds all the way to the node too); feed it into every `SceneGraph.sim_time_sec` (Task 4 wires the rest of the struct). Add:
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
- [ ] **Step 12: WS E2E test.** `test_theme_ws.py`: launch node + bridge, send `set_theme light_clay`, assert no gap in `/rendering/image` frames > 1 tick across the switch (reuses the existing bridge E2E harness pattern from Epic 0 Task 5).
```bash
source /opt/ros/humble/setup.bash && source cuda/install/setup.bash
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
- Modify: `cuda/src/libs/visual_renderer/cmake/GetFilament.cmake` (add `Filament::gltfio` imported target — **Step 1 verifies the prebuilt 1.56.5 SDK actually ships it before assuming so**, mirroring how Epic 0 discovered headless EGL wasn't shipped either)
- Create: `cuda/src/libs/visual_renderer/tests/test_ego.cpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/tf_adapter.cpp`, `include/.../tf_adapter.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_tf_adapter.py` (recorded TF fixture → expected speed)
- Modify: node `CMakeLists.txt` (`find_package(tf2_ros)`), `visualization_node.{hpp,cpp}` (own `tf2_ros::Buffer`/`TransformListener`, `ego_model_path`/`ego_fallback_dims` params, build `SceneGraph.ego` each tick)
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
  Test (`demo()`-style, per the smallest-check rule — this is a thin CLI wrapper, not business logic, so one manual run stands in for a unit test): run it against a small throwaway test OBJ (a unit cube written inline, NOT the real 143MB M02P.obj — that's a one-time manual step against the user's actual Downloads file, not a CI fixture) and assert the output `.glb` parses back via `trimesh.load` with the same vertex count.
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
- [ ] **Step 4: Failing test.**
```cpp
TEST(Ego, LoadValidGltf_RendersNonEmptyBoundingBox) { /* ... */ }
TEST(Ego, LoadMissingFile_FallsBackToClayBoxNonFatally) {
    mpviz::RenderConfig cfg{320, 240, 0, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    EXPECT_FALSE(mpviz::set_ego_model(r, "/nonexistent/path.glb", {4.5, 2.0, 1.8}));
    mpviz::SceneGraph scene{}; scene.ego = {{0,0,0}, 0, 0, /*valid=*/1};
    mpviz::CameraPose pose{{0,-8,3}, {0,0,0.5}, 60};
    mpviz::FrameView view{/*...*/};
    EXPECT_TRUE(mpviz::render_frame(r, pose, view));  // must not crash/fail — clay box instead
}
```
  Run — FAIL.
- [ ] **Step 5: Implement `ego.cpp`.** `set_ego_model`: `gltfio::AssetLoader` loads `gltf_path`; on any failure (file missing, parse error, `loadFilamentAsset` returns null), build a themed clay box (reuse Task 2's `clay.mat` ground/grid instance-creation pattern, sized to `fallback_dims`) via the same `add_mesh`-style helper Epic 0 already has for the ground/grid/cube — generalize that lambda rather than duplicating it. Store whichever entity got created on `VisualRenderer` so per-frame `set_scene` can update its `TransformManager` transform from `SceneGraph.ego.position/heading_rad` without reloading anything.
- [ ] **Step 6: Run — PASS.** Golden: `tests/test_ego.cpp` `EgoGolden_ClayBoxFallback_DarkAdas` (ego at a fixed pose, no real glTF asset committed to the repo — the golden exercises the fallback box path, which is the only ego rendering path CI can exercise without the (non-git) M02P asset present).

### 4c. TF adapter (node side)

- [ ] **Step 7: Failing test.** `test_tf_adapter.py` — recorded fixture: a short sequence of `map→base_link` transforms at known timestamps/positions (straight-line motion at a known constant speed, e.g. 2.0 m/s), fed through the adapter, asserting the finite-differenced+smoothed speed converges to 2.0 m/s within a tolerance after the smoothing filter's settling time (and that a single noisy outlier sample doesn't spike the reported speed — that's what the smoothing is *for*):
```python
# test_tf_adapter.py: publish the fixture's static/dynamic transforms on a
# real tf2 buffer at 20 Hz, run the node, sample ~/vcam_state-adjacent ego
# telemetry (or a dedicated debug topic if simpler), assert convergence.
```
  Run — FAIL (package doesn't exist).
- [ ] **Step 8: Implement `tf_adapter.cpp`.** Exponential-moving-average smoothing over raw finite-difference speed (`raw = |pos_t - pos_{t-1}| / dt`; `smoothed += alpha * (raw - smoothed)`, `alpha` a small node param e.g. `ego_speed_smoothing_alpha` default `0.2`) — the simplest filter that kills single-sample noise without adding latency-tuning complexity (a Kalman filter would be gold-plating for a HUD speed readout, not a control input). `lookupTransform("map", "base_link", tf2::TimePointZero)` each tick; `tf2::LookupException`/`ExtrapolationException` caught → `ego.valid = 0` (no TF yet), not a crash, not a clay box parked at the origin (spec's non-fatal-failure philosophy extended to "no data" as well as "bad data").
- [ ] **Step 9: Run — PASS.**
- [ ] **Step 10: Wire into `visualization_node.cpp`.** `on_configure`: declare `ego_model_path` (default `""`), `ego_fallback_dims` (default `[4.5, 2.0, 1.8]`), call `mpviz::set_ego_model` once (log WARN on `false`, non-fatal, exactly like `robot_model_path` today). `timer_callback`, before `set_scene`: `tf_adapter_->update(sim_clock_sec_)` → fills `SceneGraph.ego`.
- [ ] **Step 11: Full rebuild + regression.**
```bash
cd cuda/src/libs/visual_renderer && cmake --build build && ctest --test-dir build --output-on-failure
cd cuda/scripts/ros_apps_build && ./colcon_build.sh
source /opt/ros/humble/setup.bash && source ../../install/ros_apps/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest cuda/src/ros_apps/src/micropilot_visualization_node/test/test_tf_adapter.py
python3 cuda/src/ros_apps/src/micropilot_visualization_node/test/smoke_test.py   # still green, unmodified
```
- [ ] **Step 12: Commit** `feat(visual): ego robot from M02P glTF, TF-driven pose + smoothed speed, clay-box fallback (VM-012)`.

---

## Task 5 (VM-013): Extract the vcam tween into `src/vcam.cpp`

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
- No task exceeded its stated scope (e.g., Task 5 stayed a pure refactor; Task 2's IBL simplification is documented, not silently narrower than spec).

**Epic 1 results (fill on completion):** golden SSIM thresholds actually used = `____`, gltfio-in-prebuilt-SDK outcome (Task 4 Step 3) = `____`, any perf numbers incidentally observed (informational only — Task 6 is still the gate) = `____`.
