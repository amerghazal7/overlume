// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/height_grid.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "overlume_ros/adapters/grid_placement.hpp"

namespace overlume::ros {

float decode_height_cell(int8_t v, bool normalized, double min_m, double max_m, bool& malformed) {
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    if (v == -1) return kNaN;
    if (v < 0 || v > 100) {
        malformed = true;
        return kNaN;
    }
    const double t = normalized ? std::clamp((v - 1) / 99.0, 0.0, 1.0) : v / 100.0;
    return static_cast<float>(min_m + t * (max_m - min_m));
}

HeightGridAdapter::HeightGridAdapter(const ProfileRow& row,
                                     const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), normalized_(row.encoding == "height_normalized") {}

void HeightGridAdapter::ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec) {
    ++stats_.msgs;

    if (msg.info.width == 0 || msg.info.height == 0) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.info.width) * msg.info.height) {
        ++stats_.dropped_malformed;
        return;
    }

    GridPlacement placement{};
    switch (place_grid(msg.header, msg.info.origin, row_.frame_id, tf_, placement)) {
        case PlacementResult::kNoTf:
            ++stats_.dropped_no_tf;
            return;
        case PlacementResult::kNonFinite:
            ++stats_.dropped_malformed;
            return;
        case PlacementResult::kOk:
            break;
    }

    std::vector<float> next(msg.data.size());
    bool malformed = false;
    for (size_t i = 0; i < msg.data.size(); ++i) {
        next[i] = decode_height_cell(msg.data[i], normalized_, row_.height_min_m,
                                     row_.height_max_m, malformed);
    }
    if (malformed) ++stats_.dropped_malformed;

    heights_ = std::move(next);
    origin_ = placement.origin;
    yaw_rad_ = placement.yaw_rad;
    resolution_m_ = msg.info.resolution;
    width_cells_ = msg.info.width;
    height_cells_ = msg.info.height;
    has_grid_ = true;
    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void HeightGridAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (!has_grid_) return;

    overlume::HeightGridLayer g{};
    g.origin = origin_;
    g.yaw_rad = yaw_rad_;
    g.resolution_m = resolution_m_;
    g.width_cells = width_cells_;
    g.height_cells = height_cells_;
    g.heights_m = heights_.data();
    g.last_update_sec = last_update_sec_;
    out.height_grids.push_back(g);
}

}
