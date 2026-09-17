#include "micropilot_visualization_node/scene_assembly.hpp"

#include <cmath>

namespace micropilot::visualization_app
{

void SceneAssembly::clear()
{
    map_elements.clear();
    objects.clear();
    paths.clear();
    grids.clear();
    alerts.clear();
    markers.clear();
    point_clouds.clear();
    trajectory_carpets.clear();
    respined_carpet_points.clear();
}

void respine_velocity_ribbon_onto_local_path(SceneAssembly& a)
{
    if (a.trajectory_carpets.empty()) return;
    const mpviz::PathRibbon* local = nullptr;
    for (const auto& r : a.paths)
    {
        if (r.role == mpviz::PathRole::LOCAL && r.point_count >= 2)
        {
            local = &r;
            break;
        }
    }
    if (local == nullptr) return;  // no local spine this tick -- keep own spine

    // Local path cumulative stations.
    std::vector<double> lcum(local->point_count, 0.0);
    for (uint32_t i = 1; i < local->point_count; ++i)
    {
        const auto& p0 = local->points[i - 1];
        const auto& p1 = local->points[i];
        lcum[i] = lcum[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    for (auto& carpet : a.trajectory_carpets)
    {
        if (carpet.point_count < 2) continue;
        // Carpet's own stations (color lookup key).
        std::vector<double> ccum(carpet.point_count, 0.0);
        for (uint32_t i = 1; i < carpet.point_count; ++i)
        {
            const auto& c0 = carpet.points[i - 1];
            const auto& c1 = carpet.points[i];
            ccum[i] = ccum[i - 1] + std::hypot(c1.position.x - c0.position.x,
                                                c1.position.y - c0.position.y);
        }
        a.respined_carpet_points.emplace_back();
        auto& out = a.respined_carpet_points.back();
        out.reserve(local->point_count);
        uint32_t ci = 0;  // both station arrays are monotone -- one forward walk
        for (uint32_t i = 0; i < local->point_count; ++i)
        {
            while (ci + 1 < carpet.point_count && ccum[ci + 1] <= lcum[i]) ++ci;
            // nearest of ci/ci+1 by station; past the carpet's end this
            // naturally holds the last color.
            uint32_t pick = ci;
            if (ci + 1 < carpet.point_count &&
                (ccum[ci + 1] - lcum[i]) < (lcum[i] - ccum[ci]))
            {
                pick = ci + 1;
            }
            mpviz::PointCloudPoint pt{};
            pt.position = local->points[i];
            pt.rgba = carpet.points[pick].rgba;
            out.push_back(pt);
        }
        carpet.points = out.data();
        carpet.point_count = static_cast<uint32_t>(out.size());
    }
}

void SceneAssembly::point_at(mpviz::SceneGraph& scene) const
{
    scene.map_elements = map_elements.data();
    scene.map_element_count = static_cast<uint32_t>(map_elements.size());
    scene.objects = objects.data();
    scene.object_count = static_cast<uint32_t>(objects.size());
    scene.paths = paths.data();
    scene.path_count = static_cast<uint32_t>(paths.size());
    scene.grids = grids.data();
    scene.grid_count = static_cast<uint32_t>(grids.size());
    scene.alerts = alerts.data();
    scene.alert_count = static_cast<uint32_t>(alerts.size());
    scene.markers = markers.data();
    scene.marker_count = static_cast<uint32_t>(markers.size());
    scene.point_clouds = point_clouds.data();
    scene.point_cloud_count = static_cast<uint32_t>(point_clouds.size());
    scene.trajectory_carpets = trajectory_carpets.data();
    scene.trajectory_carpet_count = static_cast<uint32_t>(trajectory_carpets.size());
}

void apply_layer_gates(SceneAssembly& asm_, const LayerFlags& flags)
{
    if (!flags.objects) asm_.objects.clear();
    if (!flags.paths) asm_.paths.clear();
    if (!flags.map_elements) asm_.map_elements.clear();
    if (!flags.grids) asm_.grids.clear();
    if (!flags.alerts) asm_.alerts.clear();
    if (!flags.markers) asm_.markers.clear();
    if (!flags.point_clouds) asm_.point_clouds.clear();
    if (!flags.trajectory_carpet) asm_.trajectory_carpets.clear();
}

// LayerFlags members default to true, so a 9th category would aggregate-init
// true in the BOWL/HYBRID masks below with no compiler complaint and no test
// failure. This assert is the tripwire.
static_assert(sizeof(LayerFlags) == 8, "LayerFlags gained a category -- extend "
              "mode_content_mask()'s BOWL/HYBRID masks below or it renders in modes 1/2");

LayerFlags mode_content_mask(RenderMode mode)
{
    switch (mode)
    {
        case RenderMode::BOWL:
            // CUDA node's own mode 1: bowl + ego only -- nothing from the
            // autonomy scene.
            return LayerFlags{false, false, false, false, false, false, false, false};
        case RenderMode::HYBRID:
            // CUDA node's own mode 2: bowl + camera-colorized lidar + ego --
            // point_clouds is the one category HYBRID content rides (Task
            // 5/VM-094's visualization_node.cpp replaces this category's
            // content with lidar_colorize.hpp's colorized cloud each tick;
            // this mask only decides visibility, not what fills the row).
            return LayerFlags{false, false, false, false, false, false, true, false};
        case RenderMode::FREE_LOOK:
        default:
            // All-true by NSDMI -- AND-ing this over the user's own flags
            // below is a no-op, so FREE_LOOK sees exactly what the user's
            // layer_* params already said. A 9th category defaults true here
            // for free, same as every existing one.
            return LayerFlags{};
    }
}

bool bowl_visible_for_mode(RenderMode mode, bool surround_stitching)
{
    return mode == RenderMode::BOWL || mode == RenderMode::HYBRID ||
           (mode == RenderMode::FREE_LOOK && surround_stitching);
}

bool overlays_visible_for_mode(RenderMode mode)
{
    return mode == RenderMode::FREE_LOOK;
}

LayerFlags compose_layer_gates(const LayerFlags& user, const LayerFlags& mask)
{
    return LayerFlags{
        user.objects && mask.objects,
        user.paths && mask.paths,
        user.map_elements && mask.map_elements,
        user.grids && mask.grids,
        user.alerts && mask.alerts,
        user.markers && mask.markers,
        user.point_clouds && mask.point_clouds,
        user.trajectory_carpet && mask.trajectory_carpet,
    };
}

}  // namespace micropilot::visualization_app
