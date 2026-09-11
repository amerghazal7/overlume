#pragma once
/** @file scene_assembly.hpp
 *  @brief Node-owned per-category merge buffer (Epic 2 Task 1 / VM-020).
 *
 *  Exists because there is one adapter INSTANCE per profile row, and the
 *  shipped profiles have multiple rows per category (urban: 3 hd_map, 4
 *  path, 2 ogm, 5 collision rows). If each adapter pointed
 *  `SceneGraph` straight at its own storage, the last adapter to run each
 *  tick would silently erase every earlier one's contribution. So every
 *  adapter's `fill()` APPENDS into the matching vector here instead.
 *
 *  Usage (`VisualizationNode::timer_callback()`):
 *      asm_.clear();
 *      for (auto& a : adapters_) a->fill(asm_);
 *      asm_.point_at(scene);
 *      mpviz::set_scene(renderer_, scene);
 *
 *  The nested payloads each adapter allocates (points/label/predicted_path/
 *  cells/text/mesh_path) live in the ADAPTER's own storage_, not here --
 *  SceneAssembly only holds the flat per-category structs, which alias into
 *  that adapter storage via raw pointers. Those pointers must stay valid
 *  from `point_at()` through `set_scene()` returning (SceneBuffer::assign()
 *  deep-copies them there), which is why nothing between the two may
 *  reallocate an adapter's storage_ -- `point_at()` is const specifically so
 *  it cannot.
 */

#include <vector>

#include "visual_renderer/scene.h"

namespace micropilot::visualization_app
{

struct SceneAssembly
{
    std::vector<mpviz::MapElement> map_elements;
    std::vector<mpviz::TrackedObject> objects;
    std::vector<mpviz::PathRibbon> paths;
    std::vector<mpviz::GroundGridLayer> grids;
    std::vector<mpviz::AlertPolygon> alerts;
    std::vector<mpviz::GenericMarker> markers;
    // Epic 3 Task 6 (VM-035): PointCloud rows append here, same shape as
    // every category above.
    std::vector<mpviz::PointCloud> point_clouds;
    // VM-077: TrajectoryCarpetAdapter rows append here, same shape as every
    // category above.
    std::vector<mpviz::TrajectoryCarpet> trajectory_carpets;
    // Storage for RespineVelocityRibbonOntoLocalPath()'s rebuilt centerline
    // stations (user directive 2026-09-10: the velocity ribbon nests WITHIN
    // the local ribbon, so it must ride the local path's own spine). Owned
    // here, not in an adapter: the respine crosses two adapters' outputs.
    // Lifetime matches the aliasing contract above (cleared by clear(),
    // stable through point_at() -> set_scene()).
    std::vector<std::vector<mpviz::PointCloudPoint>> respined_carpet_points;

    // Cleared at the top of every timer_callback(), before any adapter's
    // fill() runs -- this is what makes "ClearBetweenTicksDoesNotAccumulate"
    // true instead of every entity from every prior tick piling up forever.
    void clear();

    // Sets all six SceneGraph ptr/count pairs to point at this object's own
    // vectors. Const: point_at() must never resize anything, because a
    // reallocation here would dangle the pointers set_scene() is about to
    // deep-copy from.
    void point_at(mpviz::SceneGraph& scene) const;
};

// Epic 3 Task 5 (VM-032) / Task 6 (VM-035): one flag per SceneAssembly
// category, node-side visibility gates. See timer_callback()'s call site for
// why this is a node-side clear rather than a renderer API.
// Re-spines every velocity ribbon (trajectory_carpets) onto the FIRST
// LOCAL-role PathRibbon's own geometry (user directive 2026-09-10: "stacking
// the velocity on top of local and within it"): the carpet trajectory runs
// ~1m laterally offset from the local path, so two independently-spined
// strips crisscross at their edges and every unsynchronized update wiggles
// the overlap boundary. New points = the local path's points; each rgba is
// sampled from the carpet's own stations by arc length (nearest station,
// last color held past the carpet's end). No LOCAL ribbon, or an empty
// carpet -> untouched (the carpet keeps its own spine). Call after every
// fill() and before apply_layer_gates()/point_at().
void respine_velocity_ribbon_onto_local_path(SceneAssembly& a);

struct LayerFlags
{
    bool objects = true;
    bool paths = true;
    bool map_elements = true;
    bool grids = true;
    bool alerts = true;
    bool markers = true;
    bool point_clouds = true;
    // VM-077: gates SceneAssembly::trajectory_carpets. Singular, matching
    // the category's own singular topic/adapter/param name
    // (layer_trajectory_carpet) -- today's shipped profile carries exactly
    // one row.
    bool trajectory_carpet = true;
};

// Clears each SceneAssembly category whose matching LayerFlags member is
// false -- pulled out of timer_callback() so the layer_point_clouds gate
// (and its six siblings) is exercisable by a plain unit test instead of only
// the live-node bridge E2E. Call this right before point_at().
void apply_layer_gates(SceneAssembly& asm_, const LayerFlags& flags);

// Task 4 (VM-093): the node's local render-mode switch, independent of the
// global /rendering/set_mode mux's active_mode_ (mirrors
// micropilot_rendering_node's own render_mode_/active_mode_ split -- see
// that node's rendering_node.hpp for the precedent this one didn't have
// until now).
enum class RenderMode
{
    BOWL = 1,
    HYBRID = 2,
    FREE_LOOK = 3,
};

// USER DIRECTIVE 2026-09-11 (mode content exclusivity, "Decision
// resolutions"): BOWL renders bowl+ego ONLY, HYBRID renders bowl+lidar+ego
// ONLY (the lidar category is point_clouds -- Task 5/VM-094 is what
// actually feeds it; until then this mask still applies, the category is
// just empty), FREE_LOOK renders the full autonomy scene unmasked. Neither
// bowl nor ego is a SceneAssembly category (bowl visibility is
// set_bowl_visible(), ego is scene.ego) -- this mask only ever touches the
// eight SceneAssembly/LayerFlags categories. The environment/buildings layer
// (Epic 4/VM-052) is a THIRD thing outside this mask -- renderer-internal,
// not a SceneAssembly category either -- gated separately in
// timer_callback() (toggling set_environment_source()'s null-source path
// per mode) precisely so it does NOT silently keep rendering in BOWL/HYBRID.
LayerFlags mode_content_mask(RenderMode mode);

// AND `mask` over `user`, field by field -- composes without ever
// overwriting the user's own layer_* params (the same directive: switching
// back to FREE_LOOK must restore the user's persisted layer_* settings
// exactly, not whatever BOWL/HYBRID happened to force them to).
LayerFlags compose_layer_gates(const LayerFlags& user, const LayerFlags& mask);

// Review round 1 (2026-09-11): pulled out of timer_callback() so the actual
// per-mode dispatch is unit-testable (test_scene_assembly.cpp's
// BowlVisibleFor*/OverlaysVisibleFor* cases) instead of only exercised by
// param accept/reject checks and frame-shape smoke tests, neither of which
// fails if this predicate is inverted or deleted.
//
// Bowl is visible in BOWL/HYBRID unconditionally (USER DIRECTIVE
// 2026-09-11: those modes render the CUDA-parity bowl); in FREE_LOOK, visible
// ONLY when the operator's Surround Stitching toggle (`layer_surround_stitching`)
// is on.
bool bowl_visible_for_mode(RenderMode mode, bool surround_stitching);

// HUD and the nearest-obstacle callout are FREE_LOOK-only overlays -- BOWL/
// HYBRID never had either in the CUDA reference (same USER DIRECTIVE), so
// both are force-suppressed there without touching hud_enabled_/
// callouts_enabled_ themselves.
bool overlays_visible_for_mode(RenderMode mode);

}  // namespace micropilot::visualization_app
