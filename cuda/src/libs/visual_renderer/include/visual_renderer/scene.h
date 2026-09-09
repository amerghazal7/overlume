// scene.h — POD boundary, same rules as api.h (checked by the same
// check_pod_header.sh, extended to glob include/visual_renderer/*.h).
//
// ADDITIVE-ONLY, per ADR-0004 (docs/adr/0004-scene-interface-versioning.md),
// which supersedes the old "frozen after Epic N / freeze lift" language:
// fields are appended to structs, enum values are appended, entry points are
// added -- nothing here is ever renamed, reordered, or removed within a
// major version. `kSceneVersion` below is bumped on every additive change;
// tests/test_scene_buffer.cpp's sizeof/offsetof static_asserts (and the
// node-side test_scene_layout.cpp mirror) are the layout guard that makes a
// version bump without a matching rebuild fail loudly instead of silently
// reading garbage across the ABI boundary.
#pragma once
#include <cstdint>
#include <cstddef>
#include "visual_renderer/api.h"

namespace mpviz {

// Introduced Epic 3 Task 1 (VM-036, ADR-0004). Bumped on every additive
// scene.h change; the node-side static_assert mirror (test_scene_layout.cpp)
// fails loudly on a layout mismatch instead of silently reading garbage
// across the ABI boundary at the node's next rebuild.
constexpr uint32_t kSceneVersion = 2;

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

// ── MapElement[] (Epic 2: VM-024; kind/lane_id/last_update_sec: Epic 3
//    Task 1 / VM-036, ADR-0004 additive) ─────────────────────────────────────
// Namespace convention verified against the recorded bag (see the Epic 3
// plan, Task 1, "Verified: kind/lane_id source"). ROAD_EDGE is emitted by
// HdMapAdapter::fill()'s geometry-driven promotion (user directive
// 2026-09-08): a LEFT_/RIGHT_BOUNDARY with no near-coincident opposite-side
// twin from another lane IS the road's outer edge; no ingest rule produces
// it directly (the disabled /road_markers row may also map to it one day).
// ROAD_SURFACE is adapter-synthesized (never present on the wire) -- see the
// plan's road-fill section for its two-rail point encoding.
enum class MapKind : uint8_t {
    OTHER = 0, CENTERLINE = 1, LEFT_BOUNDARY = 2, RIGHT_BOUNDARY = 3,
    CROSSWALK = 4, STOPLINE = 5, JUNCTION = 6, ROAD_EDGE = 7, ROAD_SURFACE = 8
};

struct MapElement {
    const Vec3* points;  uint32_t point_count;
    uint8_t is_polygon;  // 0 = polyline (lane centerline), 1 = polygon (crosswalk)
    MapKind kind;             // NEW, appended. Default (aggregate zero-init) = OTHER.
    uint32_t lane_id;         // NEW, appended. 0 = none (crosswalk/stopline/junction/other).
    double last_update_sec;   // NEW, appended. Closes Epic 2's "map pops, does not fade" deviation.
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

// ── PointCloud[] (Epic 3 Task 6 / VM-035, ADR-0004 additive) ────────────────
// Node-side PointCloudAdapter bakes one packed rgba8 per point from
// sensor_msgs/PointCloud2 (color_mode: auto|rgb|intensity|height|flat) --
// the renderer never re-derives color from raw sensor fields, and never
// re-bakes this per point_cloud.mat's own header comment (alpha there is a
// per-frame MATERIAL UNIFORM driven by staleness_alpha(), entirely separate
// from this per-vertex color).
//
// Packing convention (mirrored by comment here, in the adapter, and in
// point_cloud.cpp's unpack — same "one ABI constant, several mirror
// comments" shape as ogm.hpp's kUnknownCell): byte 0 (LSB) = r, byte 1 = g,
// byte 2 = b, byte 3 (MSB) = a, i.e.
// `rgba = r | (g << 8) | (b << 16) | (a << 24)`. This is a plain byte-array
// layout on this codebase's little-endian targets, so it copies straight
// into a Filament UBYTE4 vertex attribute with no repacking at the render
// call site.
//
// `a` (alpha) is never touched by the staleness fade (that is the
// material's own uniform) — it is reused instead as a one-bit sentinel:
// a==0 means "no real per-point color was computed" (color_mode: flat, or
// any tier that fell all the way through with nothing to bake), and
// point_cloud.cpp substitutes the theme's neutral token
// (palette.object_tints.unknown — reused, zero new theme fields, same
// token GenericMarker's own alpha==0 sentinel already uses) for every such
// point at mesh-BUILD time. a!=0 (always 255 when baked) means "trust r/g/b
// verbatim". KNOWN LIMITATION: because that substitution happens at
// geometry build time, not every render_frame() call, a flat-mode cloud's
// displayed color follows a live set_theme() transition only on its next
// content-driven rebuild, not smoothly mid-transition -- same class of
// caveat as this library's grid fade-distance bake (scene.h's set_theme()
// doc comment).
struct PointCloudPoint {
    Vec3 position;     // map frame
    uint32_t rgba;      // packed per the convention above
};

struct PointCloud {
    const PointCloudPoint* points;  uint32_t point_count;
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
    // Epic 3 Task 4 (VM-031) Step 0, SCOPE DECISION: stays UNUSED by design.
    // The node builds its callout chip list from its own live
    // AlertPolygon/collision-adapter data and draws it directly (node-side
    // compositing, see project_to_screen()'s own comment below) -- it never
    // routes through here. `chips`/`chip_count`/`AlertChip` stay in scene.h
    // (ADR-0004 forbids removing them regardless) for a hypothetical future
    // in-scene (3D-anchored, SDF) text path, not populated today.
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
    // Appended Epic 3 Task 6 (VM-035, ADR-0004) -- kSceneVersion 1 -> 2, the
    // one bump this epic makes (Task 1 introduced the constant at 1).
    const PointCloud*      point_clouds; uint32_t point_cloud_count;
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

// Eases every themed token (palette, material roughness/metallic, sun/IBL,
// HUD colors) from whatever is currently active toward `theme_name`,
// starting at `at_sec` (a SceneGraph::sim_time_sec value — the caller's
// clock, never wall-clock) and completing after `transition_sec` seconds
// (0.0 -> use the spec default, 0.8s). Smoothstep-eased, Oklab-space color
// lerp (Task 3). A transition retargeted mid-flight (set_theme called again
// before the first finishes) starts a new ease from the CURRENT blended
// state, not from either endpoint — no visible snap.
// Exception: grid fade distances (fade_start_m/fade_end_m) are NOT animated
// by set_theme — they're baked into the grid's vertex buffer once at
// create_renderer() time and take effect only on the next renderer creation
// with a different theme, not on a live set_theme() switch.
// Returns false (no-op) if theme_name doesn't match a loaded
// assets/themes/<name>.yaml stem; the active theme is unchanged.
bool set_theme(VisualRenderer*, const char* theme_name, double at_sec,
                double transition_sec);

// Loads a glTF/GLB ego mesh from `gltf_path` for use whenever
// SceneGraph::ego.valid is true. Non-fatal on failure (bad path, unsupported
// glTF feature, missing file): logs nothing itself (POD boundary — no
// logging crosses it), returns false, and rendering falls back to a clay
// box sized by `fallback_dims` (meters, length/width/height). Call once,
// typically from on_configure(), before the first render_frame() (Epic 1
// Task 4 / VM-012 — see the master plan's "Interfaces frozen this epic",
// which names this as "new in scene.h": this Files list doesn't separately
// list a scene.h modification, only "Create ego.hpp/ego.cpp" — Vec3 is a
// scene.h type and set_ego_model must be reachable from the gcc/libstdc++
// ROS node through a POD public header, same reasoning set_theme's own
// declaration here already established in Task 3, so this is the one
// place it can live).
bool set_ego_model(VisualRenderer*, const char* gltf_path, Vec3 fallback_dims);

// Points the object renderer at a directory of normalized per-class glTF/GLB
// clay models (VM-022). Expected stems: car.glb, truck_van.glb, bus.glb,
// pedestrian.glb, cyclist.glb — one per ObjectClass except UNKNOWN, which is
// always the procedural clay box. Any stem that is missing or fails
// to load is non-fatal: that class falls back to the same procedural clay
// box (a plain unit box today — the fillet was skipped, see
// build_unit_box's ponytail note in objects.cpp),
// scaled to the object's measured bbox exactly as a loaded model would be
// (spec §9, "asset load failure -> clay-box fallback, WARN once"). Returns
// the number of class models successfully loaded (0 is a legal, fully
// functional configuration — see Task 4 Step 0). Call once, from
// on_configure(), before the first render_frame(); `dir` is caller-owned and
// borrowed only for the duration of this call, same rule as
// RenderConfig::theme_assets_dir.
uint32_t set_object_model_dir(VisualRenderer*, const char* dir);

// Gate-review addition (2026-08-20, spec §9 minor): create_renderer()
// silently substitutes the compiled-in kFallbackTheme() whenever
// RenderConfig::theme_assets_dir/initial_theme fails to load (missing dir,
// missing file, malformed YAML) -- non-fatal by design, but until now gave
// the caller no way to know it happened and WARN. Returns true iff the
// theme active right after create_renderer() was actually loaded from disk;
// false if it's the compiled-in fallback. Reflects only the INITIAL load at
// create_renderer() time, not any later set_theme() call (which has its own
// bool return for the same purpose). An additive entry point -- legal under
// the scene.h freeze (see this header's own top comment).
bool theme_assets_loaded(VisualRenderer*);

// Epic 3 Task 3 (VM-030): the node-side HUD compositor (hud_overlay.cpp)
// needs the theme's live HUD colors, including mid-`theme_transition` blend
// values, which exist only inside `active_theme` (renderer_internal.hpp) --
// see the plan's "ACCEPTED (user, 2026-09-07): get_hud_colors() ships as
// specified" for why this is an authorized amendment to P2's "no new public
// entry point," not a silent extension of it. POD, appended per ADR-0004.
struct HudColors {
    float text_color[3];
    float accent_color[3];
    float scale;
};

// Reads `r->active_theme.hud` verbatim (kept live every render_frame() call
// by apply_current_theme(), including mid-transition blend values -- no new
// renderer state added for this). Null `r` -> zero-initialized HudColors
// (scale 0.0), same non-crashing default-on-null shape as this header's
// other pointer-taking calls.
HudColors get_hud_colors(VisualRenderer*);

// Epic 3 Task 4 (VM-031): projects a map-frame world point (same space as
// EgoState::position/TrackedObject::position/AlertPolygon::points) into
// screen-fraction [0, 1] coordinates using the CAMERA STATE THE MOST RECENT
// render_frame() CALL SET (its CameraPose's lookAt()/setProjection(), not a
// separately-passed pose) -- a caller needing a different pose calls
// render_frame() with it first. Standard world -> clip -> NDC -> [0,1]
// pipeline. **Y IS FLIPPED** relative to raw NDC (NDC +Y is up) to match
// FrameView's own top-to-bottom row order: `*out_y == 0.0` is the frame's
// TOP row, `1.0` the bottom -- stated here explicitly (the epic2 precedent
// of an unstated axis convention is exactly the kind of gap that ships a
// silently-flipped picture). Returns false, leaving `*out_x`/`*out_y`
// untouched, when `r`/`out_x`/`out_y` is null, the point is behind the
// camera (clip.w <= 0), or it falls outside the view frustum's x/y bounds
// (|NDC.x| > 1 or |NDC.y| > 1) -- near/far (z) clipping is left to
// Filament's own render-time culling, not duplicated here, since "is this
// point on screen" is fully settled by the x/y bounds alone. True with
// `*out_x`/`*out_y` in [0, 1] otherwise.
bool project_to_screen(VisualRenderer*, Vec3 world_point, float* out_x, float* out_y);

// Epic 2 Task 1 (VM-020) Step 0.3: parses `<dir>/<theme_name>.yaml` with the
// library's OWN bundled yaml-cpp and returns true iff it loaded. Creates no
// Engine, no EGL context, no swapchain -- it is
// `detail::load_theme(dir, name).has_value()` and nothing else
// (renderer.cpp's create_renderer() already calls load_theme() before the
// engine is built, so this wraps an existing GPU-free code path rather than
// adding one). It exists for exactly one reason: proving, in one process,
// that this archive's bundled yaml-cpp and a second, independently-built
// yaml-cpp (the node's gcc/libstdc++ yaml_cpp_vendor) can coexist without a
// GPU -- every other entry point here needs a live VisualRenderer*, which a
// headless CI box cannot provide, so that check would GTEST_SKIP forever
// and the ABI boundary Step 0.2 protects would go untested.
// Null/empty `dir` or `theme_name` -> false. Cheap enough to call from a
// test; not intended for the render loop.
bool theme_parses(const char* dir, const char* theme_name);

}  // namespace mpviz
