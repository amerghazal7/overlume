#include "scene_buffer.hpp"

#include <algorithm>

namespace overlume::detail {

void OwnedScene::assign(const overlume::SceneGraph& src) {
    view = src;  // copies every flat scalar field (sim_time_sec, ego, hud
                 // scalars, all *_count fields) — pointer fields below get
                 // overwritten with owned storage next.

    objects.assign(src.objects, src.objects + src.object_count);
    object_paths.resize(src.object_count);
    object_labels.resize(src.object_count);
    for (uint32_t i = 0; i < src.object_count; ++i) {
        const TrackedObject& s = src.objects[i];
        object_paths[i].assign(s.predicted_path, s.predicted_path + s.predicted_path_count);
        objects[i].predicted_path = object_paths[i].data();
        object_labels[i] = s.label ? s.label : "";
        objects[i].label = s.label ? object_labels[i].c_str() : nullptr;
    }
    view.objects = objects.data();

    paths.assign(src.paths, src.paths + src.path_count);
    path_points.resize(src.path_count);
    for (uint32_t i = 0; i < src.path_count; ++i) {
        const PathRibbon& s = src.paths[i];
        path_points[i].assign(s.points, s.points + s.point_count);
        paths[i].points = path_points[i].data();
    }
    view.paths = paths.data();

    map_elements.assign(src.map_elements, src.map_elements + src.map_element_count);
    map_element_points.resize(src.map_element_count);
    for (uint32_t i = 0; i < src.map_element_count; ++i) {
        const MapElement& s = src.map_elements[i];
        map_element_points[i].assign(s.points, s.points + s.point_count);
        map_elements[i].points = map_element_points[i].data();
    }
    view.map_elements = map_elements.data();

    grids.assign(src.grids, src.grids + src.grid_count);
    grid_cells.resize(src.grid_count);
    for (uint32_t i = 0; i < src.grid_count; ++i) {
        const GroundGridLayer& s = src.grids[i];
        const size_t cell_count = static_cast<size_t>(s.width_cells) * s.height_cells;
        grid_cells[i].assign(s.cells, s.cells + cell_count);
        grids[i].cells = grid_cells[i].data();
    }
    view.grids = grids.data();

    alerts.assign(src.alerts, src.alerts + src.alert_count);
    alert_points.resize(src.alert_count);
    for (uint32_t i = 0; i < src.alert_count; ++i) {
        const AlertPolygon& s = src.alerts[i];
        alert_points[i].assign(s.points, s.points + s.point_count);
        alerts[i].points = alert_points[i].data();
    }
    view.alerts = alerts.data();

    markers.assign(src.markers, src.markers + src.marker_count);
    marker_points.resize(src.marker_count);
    marker_texts.resize(src.marker_count);
    marker_mesh_paths.resize(src.marker_count);
    for (uint32_t i = 0; i < src.marker_count; ++i) {
        const GenericMarker& s = src.markers[i];
        marker_points[i].assign(s.points, s.points + s.point_count);
        markers[i].points = marker_points[i].data();
        marker_texts[i] = s.text ? s.text : "";
        markers[i].text = s.text ? marker_texts[i].c_str() : nullptr;
        marker_mesh_paths[i] = s.mesh_path ? s.mesh_path : "";
        markers[i].mesh_path = s.mesh_path ? marker_mesh_paths[i].c_str() : nullptr;
    }
    view.markers = markers.data();

    chips.assign(src.hud.chips, src.hud.chips + src.hud.chip_count);
    chip_texts.resize(src.hud.chip_count);
    for (uint32_t i = 0; i < src.hud.chip_count; ++i) {
        const char* text = src.hud.chips[i].text;
        chip_texts[i] = text ? text : "";
        chips[i].text = text ? chip_texts[i].c_str() : nullptr;
    }
    view.hud.chips = chips.data();

    point_clouds.assign(src.point_clouds, src.point_clouds + src.point_cloud_count);
    point_cloud_points.resize(src.point_cloud_count);
    for (uint32_t i = 0; i < src.point_cloud_count; ++i) {
        const PointCloud& s = src.point_clouds[i];
        point_cloud_points[i].assign(s.points, s.points + s.point_count);
        point_clouds[i].points = point_cloud_points[i].data();
    }
    view.point_clouds = point_clouds.data();

    trajectory_carpets.assign(src.trajectory_carpets,
                               src.trajectory_carpets + src.trajectory_carpet_count);
    trajectory_carpet_points.resize(src.trajectory_carpet_count);
    for (uint32_t i = 0; i < src.trajectory_carpet_count; ++i) {
        const TrajectoryCarpet& s = src.trajectory_carpets[i];
        trajectory_carpet_points[i].assign(s.points, s.points + s.point_count);
        trajectory_carpets[i].points = trajectory_carpet_points[i].data();
    }
    view.trajectory_carpets = trajectory_carpets.data();
}

void SceneBuffer::publish(const overlume::SceneGraph& scene) {
    int next;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        next = 1 - active_idx_;
    }
    slots_[next].assign(scene);
    std::lock_guard<std::mutex> lock(mutex_);
    active_idx_ = next;
}

const overlume::SceneGraph& SceneBuffer::active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slots_[active_idx_].view;
}

float SceneBuffer::staleness_alpha(double now_sec, double last_update_sec,
                                    double fade_start_sec, double timeout_sec) {
    const double age = now_sec - last_update_sec;
    if (age <= fade_start_sec) return 1.0f;
    if (age >= timeout_sec) return 0.0f;
    const double t = (age - fade_start_sec) / (timeout_sec - fade_start_sec);
    return static_cast<float>(std::clamp(1.0 - t, 0.0, 1.0));
}

}  // namespace overlume::detail
