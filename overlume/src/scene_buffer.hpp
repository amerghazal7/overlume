// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "overlume/scene.h"

namespace overlume::detail {

struct OwnedScene {
    overlume::SceneGraph view{};
    std::vector<TrackedObject> objects;
    std::vector<std::vector<Vec3>> object_paths;
    std::vector<std::string> object_labels;
    std::vector<PathRibbon> paths;
    std::vector<std::vector<Vec3>> path_points;
    std::vector<MapElement> map_elements;
    std::vector<std::vector<Vec3>> map_element_points;
    std::vector<GroundGridLayer> grids;
    std::vector<std::vector<uint8_t>> grid_cells;
    std::vector<HeightGridLayer> height_grids;
    std::vector<std::vector<float>> height_grid_cells;
    std::vector<AlertPolygon> alerts;
    std::vector<std::vector<Vec3>> alert_points;
    std::vector<GenericMarker> markers;
    std::vector<std::vector<Vec3>> marker_points;
    std::vector<std::string> marker_texts;
    std::vector<std::string> marker_mesh_paths;
    std::vector<AlertChip> chips;
    std::vector<std::string> chip_texts;
    std::vector<PointCloud> point_clouds;
    std::vector<std::vector<PointCloudPoint>> point_cloud_points;
    std::vector<TrajectoryCarpet> trajectory_carpets;
    std::vector<std::vector<PointCloudPoint>> trajectory_carpet_points;
    void assign(const overlume::SceneGraph& src);
};

class SceneBuffer {
public:
    void publish(const overlume::SceneGraph& scene);
    const overlume::SceneGraph& active() const;
    static float staleness_alpha(double now_sec, double last_update_sec, double fade_start_sec,
                                 double timeout_sec);

private:
    mutable std::mutex mutex_;
    OwnedScene slots_[2];
    int active_idx_{0};
};

}
