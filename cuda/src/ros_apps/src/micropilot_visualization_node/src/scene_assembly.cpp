#include "micropilot_visualization_node/scene_assembly.hpp"

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
}

}  // namespace micropilot::visualization_app
