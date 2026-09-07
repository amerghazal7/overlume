// golden.hpp — Epic 1 Task 2 Step 9: shared render+compare helper used by
// every later epic's golden-image tests. NOT a gtest file itself (see
// CMakeLists.txt: golden.cpp is compiled as a plain extra source into every
// other test binary instead of its own gtest executable).
#pragma once

#include <vector>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Epic 2 Task 2 (VM-024) Step 8: move-only owner of a `.geom` fixture's
// point data AND the `MapElement`s that point into it. Move-only, not
// copyable: `MapElement::points` are raw pointers into `points` — a copy
// would leave the copy's elements aimed at the ORIGINAL's buffer (see
// map_elements.cpp's own comment on this general hazard). A move is safe
// because std::vector's move constructor transfers the underlying buffer
// without reallocating, so every pointer already baked into `elements`
// keeps pointing at valid memory after the move.
struct MapGeom {
    std::vector<mpviz::Vec3> points;
    std::vector<mpviz::MapElement> elements;

    MapGeom() = default;
    MapGeom(const MapGeom&) = delete;
    MapGeom& operator=(const MapGeom&) = delete;
    MapGeom(MapGeom&&) = default;
    MapGeom& operator=(MapGeom&&) = default;
};

// Reads a `.geom` fixture emitted by the node-side HdMapAdapter test
// (MPVIZ_EMIT_GEOM, Task 2 Step 7) -- a plain-text dump, one MapElement per
// line: `<is_polygon> <n> <x1> <y1> <z1> ... <xn> <yn> <zn>`.
// ponytail: a text dump, not a serializer -- this is the whole parser.
// Returns an empty MapGeom (elements.empty()) if `path` can't be opened or
// contains no valid lines -- callers assert non-empty right after this
// call, so a missing/malformed fixture fails loudly, never silently
// "passes" against nothing.
MapGeom load_map_geom(const char* path);

// Epic 2 Task 4 (VM-022) Step 2: move-only owner of a synthetic mixed-class
// scene's `predicted_path` point arrays AND the `TrackedObject`s that point
// into them -- same move-only-not-copyable reasoning as MapGeom above (a
// bare `std::vector<mpviz::TrackedObject>` would dangle: TrackedObject::
// predicted_path is a raw pointer into `path_points`, and a copy would leave
// the copy's objects aimed at the ORIGINAL's buffer).
struct ObjectScene {
    std::vector<mpviz::Vec3> path_points;
    std::vector<mpviz::TrackedObject> objects;

    ObjectScene() = default;
    ObjectScene(const ObjectScene&) = delete;
    ObjectScene& operator=(const ObjectScene&) = delete;
    ObjectScene(ObjectScene&&) = default;
    ObjectScene& operator=(ObjectScene&&) = default;
};

// One TrackedObject per ObjectClass (CAR..UNKNOWN), each with its own bbox
// dims -- the CAR has a nonzero velocity (exercises the velocity-arrow
// path), the TRUCK_VAN has a predicted_path (exercises extrude_polyline),
// and the UNKNOWN object's last_update_sec is 0.9s behind `now` (inside the
// kStaleFadeStartSec=0.5/kStaleFadeTimeoutSec=1.0 fade window -- alpha
// ~0.2), so a single scene exercises instancing, bbox scaling, per-class
// tints, arrows, predicted ribbons AND staleness fade together.
ObjectScene make_mixed_class_objects(double now);

// Epic 2 Task 5 (VM-023) Step 3: move-only owner of a synthetic three-role
// ribbon scene's point arrays AND the `PathRibbon`s that point into them --
// same move-only-not-copyable reasoning as MapGeom/ObjectScene above (a bare
// `std::vector<mpviz::PathRibbon>` would dangle: PathRibbon::points is a raw
// pointer into `point_storage`, and a copy would leave the copy's ribbons
// aimed at the ORIGINAL's buffer). Widening this file's own header comment,
// per the plan: "test scenes must keep the points arrays alive across
// set_scene (golden.cpp move-only owner pattern)" -- this is the ribbon
// case of that pattern, MapGeom/ObjectScene being the map/object ones.
struct RibbonScene {
    std::vector<mpviz::Vec3> point_storage;
    std::vector<mpviz::PathRibbon> ribbons;

    RibbonScene() = default;
    RibbonScene(const RibbonScene&) = delete;
    RibbonScene& operator=(const RibbonScene&) = delete;
    RibbonScene(RibbonScene&&) = default;
    RibbonScene& operator=(RibbonScene&&) = default;
};

// One ribbon per role (BEHAVIOR/GLOBAL/LOCAL), all fresh at `now` unless
// noted, positioned so a camera looking roughly at the world origin sees
// all three -- the RibbonGolden.ThreeRoles_DarkAdas synthetic scene (no
// recording exercises a live GLOBAL ribbon or a second LOCAL row at once,
// see the plan's fixture gap 2).
RibbonScene make_three_role_ribbons(double now);

// Epic 2 Task 7 (VM-026) Step 3: move-only owner of a synthetic
// sweep+predicted alert scene's point arrays AND the `AlertPolygon`s that
// point into them — same move-only-not-copyable reasoning as MapGeom/
// ObjectScene/RibbonScene above (`AlertPolygon::points` is a raw pointer
// into `point_storage`; a copy would leave the copy's polygons aimed at
// the ORIGINAL's buffer).
//
// FIXTURE GAP 4: the five collision-checker topics were silent in the
// recorded bag (a calm scenario, zero messages) — entirely synthetic, the
// same "no recording to draw from" shape as RibbonScene's fixture gap 2.
struct AlertScene {
    std::vector<mpviz::Vec3> point_storage;
    std::vector<mpviz::AlertPolygon> alerts;

    AlertScene() = default;
    AlertScene(const AlertScene&) = delete;
    AlertScene& operator=(const AlertScene&) = delete;
    AlertScene(AlertScene&&) = default;
    AlertScene& operator=(AlertScene&&) = default;
};

// One ego-footprint SWEEP (severity 0/info — the ghost trail, aged 0.75s
// stale so its fade is visibly on, not just its already-low constant
// alpha, same aging convention as Ribbon.StaleRibbonFadesViaSharedStaleness
// Alpha's 0.75s-stale fixture) and one object PREDICTED polygon (severity
// 1/warning, fresh), positioned apart so a human sees both shapes distinctly
// — AlertGolden.SweepPlusPredicted_DarkAdas's synthetic scene.
AlertScene make_sweep_and_predicted_alerts(double now);

// Epic 2 Task 8 (VM-027) Step 1: move-only owner of a synthetic
// one-of-every-primitive-type scene's point storage AND the
// `GenericMarker`s that point into it -- same move-only-not-copyable
// reasoning as MapGeom/ObjectScene/RibbonScene/AlertScene above
// (`GenericMarker::points` is a raw pointer into `point_storage`; a copy
// would leave the copy's markers aimed at the ORIGINAL's buffer). TEXT's
// `text` and MESH's `mesh_path` are plain string literals (static storage
// duration), so — unlike `points` — they need no owned storage here.
//
// FIXTURE GAP 5: 7 of the 12 ROS marker types never appear in the recorded
// bag -- this scene is entirely synthetic BY DESIGN (the backlog AC itself
// asks for one of every primitive type, which real traffic never
// exercises).
struct GenericMarkerScene {
    std::vector<mpviz::Vec3> point_storage;
    std::vector<mpviz::GenericMarker> markers;

    GenericMarkerScene() = default;
    GenericMarkerScene(const GenericMarkerScene&) = delete;
    GenericMarkerScene& operator=(const GenericMarkerScene&) = delete;
    GenericMarkerScene(GenericMarkerScene&&) = default;
    GenericMarkerScene& operator=(GenericMarkerScene&&) = default;
};

// One of every FROZEN MarkerPrimitive (all 10 scene.h enum values) laid
// out in a row along +X so a human can count shapes at a glance, PLUS the
// adapter's CUBE_LIST/SPHERE_LIST fan-out result (a 3-point CUBE_LIST and
// a 3-point SPHERE_LIST both fan out into one GenericMarker CUBE/SPHERE
// per point, Task 8 Step 4's adapter contract) -- hand-built here exactly
// as GenericMarkerAdapter would emit them, since this is a LIBRARY test
// (the node-side fan-out itself is proven by GenericMarkerAdapter.
// CubeListFansOutIntoOneMarkerPerPoint). `mesh_glb_path` is a caller-owned
// string (MPVIZ_TEST_DATA_DIR-prefixed, a compile-time-stable literal at
// every call site) borrowed only for this call -- GenericMarker::mesh_path
// itself is stored as the same pointer, so it must outlive `markers`' use
// exactly like every other GenericMarker string field.
GenericMarkerScene make_all_primitive_markers(double now, const char* mesh_glb_path);

// Mean of every point across every element -- used to place the golden's
// ego/camera FROM the recorded data (see test_map_elements.cpp) rather than
// at a hand-picked coordinate. {0,0,0} if `elems` has no points at all.
mpviz::Vec3 centroid(const std::vector<mpviz::MapElement>& elems);

// Epic 2 Task 6 (VM-025) Step 3: move-only owner of a synthetic two-layer
// OGM scene's cell-byte storage AND the `GroundGridLayer`s that point into
// it -- same move-only-not-copyable reasoning as MapGeom/ObjectScene/
// RibbonScene above (`GroundGridLayer::cells` is a raw pointer into one of
// `cell_storage`'s elements; a copy would leave the copy's layers aimed at
// the ORIGINAL's buffers). Safe to move: moving the OUTER
// `vector<vector<uint8_t>>` relocates only the top-level array, never the
// inner vectors' own heap buffers, so every `cells` pointer already baked
// into `grids` keeps pointing at valid memory after the move (same
// reasoning scene_buffer.hpp's own `OwnedScene::object_paths` — the
// production code's equivalent nested-vector shape — already relies on).
//
// FIXTURE GAP 3 (epic2 plan, Task 6): zero OccupancyGrid topics exist in
// the recorded bag, so unlike MapGeom (which loads a real recorded
// fixture), this is entirely hand-built synthetic data -- see
// make_two_layer_grids()'s own comment.
struct GridScene {
    std::vector<std::vector<uint8_t>> cell_storage;
    std::vector<mpviz::GroundGridLayer> grids;

    GridScene() = default;
    GridScene(const GridScene&) = delete;
    GridScene& operator=(const GridScene&) = delete;
    GridScene(GridScene&&) = default;
    GridScene& operator=(GridScene&&) = default;
};

// One dynamic-OGM layer (kind 0) and one gradient-OGM layer (kind 1),
// both an 8m x 8m footprint (16x16 cells @ 0.5m) centered under the ego at
// the world origin -- GroundGridGolden.TwoLayers_OffroadLightClay's
// synthetic scene. The dynamic layer is a concentric occupancy blob (100%
// at its center, fading to 0% at the edges) with its FOUR CORNER cells
// forced to the 255 unknown sentinel (proving
// UnknownCellsBecomeTheSentinelNotTwoFiftyFive's transfer function reads
// as "ground shows through", not "very occupied", in the actual rendered
// golden — not just the adapter unit test). The gradient layer is a
// smooth left-to-right 0->100 ramp (typical costmap-gradient look), no
// unknown cells, laid out at a slightly different origin so the golden
// shows two DISTINCT, only-partially-overlapping layers rather than one
// fully occluding the other.
GridScene make_two_layer_grids(double now);

// Renders ONE frame of `r`'s current active scene/theme state from `pose`
// at a fixed 320x240 (this epic's goldens are all committed at that size —
// spec: "goldens render at a fixed 320x240 purely for CI speed/
// determinism"), writes it to `out_png_path`, and returns a block-SSIM
// score in [0,1] against `golden_png_path` (0.0 if the golden doesn't exist
// yet or doesn't match 320x240 — first run of a new golden always fails
// loudly, never silently "passes" with nothing to compare against).
//
// The harness does NOT create a renderer, and does NOT call set_scene/
// set_theme — it only calls render_frame(r, pose, ...) and compares the
// result. The caller owns create_renderer()/destroy_renderer(), and must
// have already driven `r` into whatever scene/theme/transition state it
// wants a golden of via set_scene()/set_theme() BEFORE calling this.
//
// Returns -1.0 if `r` is null — same no-GPU/EGL convention as
// test_hello_frame.cpp; callers are expected to GTEST_SKIP() right after
// create_renderer() returns null, before ever reaching this call, same as
// every other renderer test in this codebase.
double render_and_compare(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path);

// Legibility stats for a rendered frame -- the numeric form of the plan's
// Step 7a AC ("clay surfaces read as mid-gray-ish, not clipped white or
// crushed black"). `top_third_mean`/`bottom_third_mean` are luminance means
// of the top/bottom thirds of the frame (sky-ish vs. ground-ish for this
// epic's fixed camera pose looking at the horizon) -- not a scene-aware
// segmentation, just enough to catch "the flat sky backdrop is brighter
// than the supposedly sunlit ground" the way a human glancing at the image
// would. Returns all-zero stats if the PNG can't be loaded.
struct FrameStats {
    double mean = 0.0;
    int distinct_levels = 0;
    double top_third_mean = 0.0;
    double bottom_third_mean = 0.0;
    // sky_row_mean: rows [10,40) -- deep in the flat sky/clear-color
    // backdrop, above any horizon effect, for this epic's fixed 320x240 /
    // CameraPose{{0,-8,4},{0,0,0},60} test setup (same "not scene-aware,
    // just matches this fixed pose" caveat as top/bottom_third_mean above).
    // horizon_row_mean: rows [50,60) -- the far edge of the ground plane
    // (kGroundHalfExtent, only 40m across) as it meets the sky, where
    // distance-fog opacity is at its highest for any on-plane ray (though,
    // being a *finite* plane, never near-total extinction the way a true
    // infinite ground would give). Every shipped theme authors palette.fog
    // == palette.sky (spec §4.3 treats them as one token), so a correctly-
    // scaled fog should pull this band noticeably toward sky_row_mean --
    // this is the guard the epic1 Task 2 fog-scale review round asked for
    // (a flat mean-band check can't express "the ground fades toward the
    // sky", only "isn't crushed/clipped"; see renderer.cpp's setFogOptions
    // comment for why exact equality isn't reachable by color scale alone).
    double sky_row_mean = 0.0;
    double horizon_row_mean = 0.0;
};
FrameStats analyze_png(const char* png_path);

}  // namespace mpviz::testing
