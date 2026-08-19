// scene.h — POD boundary, same rules as api.h (checked by the same
// check_pod_header.sh, extended to glob include/visual_renderer/*.h).
//
// Frozen for all Visual Mode epics once Epic 1's review gate passes
// (docs/superpowers/plans/2026-08-18-visual-mode-epic1.md, Task 1/VM-010).
// Spec §4.1 lists 8 scene categories; Epic 1 only ever populates `ego`
// (object_count == 0, path_count == 0, etc. for everything else) — Epic 2
// fills the rest without touching this header again.
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

}  // namespace mpviz
