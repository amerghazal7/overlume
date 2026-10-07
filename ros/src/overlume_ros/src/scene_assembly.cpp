// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/scene_assembly.hpp"

#include <cmath>

namespace overlume::ros {

void SceneAssembly::clear() {
    map_elements.clear();
    objects.clear();
    paths.clear();
    grids.clear();
    alerts.clear();
    markers.clear();
    point_clouds.clear();
    trajectory_carpets.clear();
    height_grids.clear();
    respined_carpet_points.clear();
}

void respine_velocity_ribbon_onto_local_path(SceneAssembly& a) {
    if (a.trajectory_carpets.empty()) return;
    const overlume::PathRibbon* local = nullptr;
    for (const auto& r : a.paths) {
        if (r.role == overlume::PathRole::LOCAL && r.point_count >= 2) {
            local = &r;
            break;
        }
    }
    if (local == nullptr) return;

    std::vector<double> lcum(local->point_count, 0.0);
    for (uint32_t i = 1; i < local->point_count; ++i) {
        const auto& p0 = local->points[i - 1];
        const auto& p1 = local->points[i];
        lcum[i] = lcum[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    for (auto& carpet : a.trajectory_carpets) {
        if (carpet.point_count < 2) continue;
        std::vector<double> ccum(carpet.point_count, 0.0);
        for (uint32_t i = 1; i < carpet.point_count; ++i) {
            const auto& c0 = carpet.points[i - 1];
            const auto& c1 = carpet.points[i];
            ccum[i] = ccum[i - 1] +
                      std::hypot(c1.position.x - c0.position.x, c1.position.y - c0.position.y);
        }
        a.respined_carpet_points.emplace_back();
        auto& out = a.respined_carpet_points.back();
        out.reserve(local->point_count);
        uint32_t ci = 0;
        for (uint32_t i = 0; i < local->point_count; ++i) {
            while (ci + 1 < carpet.point_count && ccum[ci + 1] <= lcum[i]) ++ci;
            uint32_t pick = ci;
            if (ci + 1 < carpet.point_count && (ccum[ci + 1] - lcum[i]) < (lcum[i] - ccum[ci])) {
                pick = ci + 1;
            }
            overlume::PointCloudPoint pt{};
            pt.position = local->points[i];
            pt.rgba = carpet.points[pick].rgba;
            out.push_back(pt);
        }
        carpet.points = out.data();
        carpet.point_count = static_cast<uint32_t>(out.size());
    }
}

void SceneAssembly::point_at(overlume::SceneGraph& scene) const {
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
    scene.height_grids = height_grids.data();
    scene.height_grid_count = static_cast<uint32_t>(height_grids.size());
}

void apply_layer_gates(SceneAssembly& asm_, const LayerFlags& flags) {
    if (!flags.objects) asm_.objects.clear();
    if (!flags.paths) asm_.paths.clear();
    if (!flags.map_elements) asm_.map_elements.clear();
    if (!flags.grids) asm_.grids.clear();
    if (!flags.alerts) asm_.alerts.clear();
    if (!flags.markers) asm_.markers.clear();
    if (!flags.point_clouds) asm_.point_clouds.clear();
    if (!flags.trajectory_carpet) asm_.trajectory_carpets.clear();
    if (!flags.height_grids) asm_.height_grids.clear();
}

static_assert(sizeof(LayerFlags) == 9,
              "LayerFlags gained a category -- extend "
              "mode_content_mask()'s BOWL/HYBRID masks below or it renders in modes 1/2");

LayerFlags mode_content_mask(RenderMode mode) {
    switch (mode) {
        case RenderMode::BOWL:
            return LayerFlags{false, false, false, false, false, false, false, false, false};
        case RenderMode::HYBRID:
            return LayerFlags{false, false, false, false, false, false, true, false, false};
        case RenderMode::FREE_LOOK:
        default:
            return LayerFlags{};
    }
}

bool bowl_visible_for_mode(RenderMode mode, bool surround_stitching) {
    return mode == RenderMode::BOWL || mode == RenderMode::HYBRID ||
           (mode == RenderMode::FREE_LOOK && surround_stitching);
}

bool overlays_visible_for_mode(RenderMode mode) { return mode == RenderMode::FREE_LOOK; }

bool environment_effectively_visible(RenderMode mode, bool environment_enabled) {
    return environment_enabled && mode == RenderMode::FREE_LOOK;
}

LayerFlags compose_layer_gates(const LayerFlags& user, const LayerFlags& mask) {
    return LayerFlags{
        user.objects && mask.objects,
        user.paths && mask.paths,
        user.map_elements && mask.map_elements,
        user.grids && mask.grids,
        user.alerts && mask.alerts,
        user.markers && mask.markers,
        user.point_clouds && mask.point_clouds,
        user.trajectory_carpet && mask.trajectory_carpet,
        user.height_grids && mask.height_grids,
    };
}

}
