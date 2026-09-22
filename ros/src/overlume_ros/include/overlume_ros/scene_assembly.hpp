// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <vector>

#include "overlume/scene.h"

namespace overlume::ros {

struct SceneAssembly {
    std::vector<overlume::MapElement> map_elements;
    std::vector<overlume::TrackedObject> objects;
    std::vector<overlume::PathRibbon> paths;
    std::vector<overlume::GroundGridLayer> grids;
    std::vector<overlume::AlertPolygon> alerts;
    std::vector<overlume::GenericMarker> markers;
    std::vector<overlume::PointCloud> point_clouds;
    std::vector<overlume::TrajectoryCarpet> trajectory_carpets;
    std::vector<std::vector<overlume::PointCloudPoint>> respined_carpet_points;

    void clear();

    void point_at(overlume::SceneGraph& scene) const;
};

void respine_velocity_ribbon_onto_local_path(SceneAssembly& a);

struct LayerFlags {
    bool objects = true;
    bool paths = true;
    bool map_elements = true;
    bool grids = true;
    bool alerts = true;
    bool markers = true;
    bool point_clouds = true;
    bool trajectory_carpet = true;
};

void apply_layer_gates(SceneAssembly& asm_, const LayerFlags& flags);

enum class RenderMode {
    BOWL = 1,
    HYBRID = 2,
    FREE_LOOK = 3,
};

LayerFlags mode_content_mask(RenderMode mode);

LayerFlags compose_layer_gates(const LayerFlags& user, const LayerFlags& mask);

bool bowl_visible_for_mode(RenderMode mode, bool surround_stitching);

bool overlays_visible_for_mode(RenderMode mode);

bool environment_effectively_visible(RenderMode mode, bool environment_enabled);

}
