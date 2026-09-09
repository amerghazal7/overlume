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
struct LayerFlags
{
    bool objects = true;
    bool paths = true;
    bool map_elements = true;
    bool grids = true;
    bool alerts = true;
    bool markers = true;
    bool point_clouds = true;
};

// Clears each SceneAssembly category whose matching LayerFlags member is
// false -- pulled out of timer_callback() so the layer_point_clouds gate
// (and its six siblings) is exercisable by a plain unit test instead of only
// the live-node bridge E2E. Call this right before point_at().
void apply_layer_gates(SceneAssembly& asm_, const LayerFlags& flags);

}  // namespace micropilot::visualization_app
