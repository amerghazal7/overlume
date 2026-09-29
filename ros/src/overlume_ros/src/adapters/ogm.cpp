// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/ogm.hpp"

#include <cmath>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

uint8_t KindFromRole(const std::string& role) {
    if (role == "gradient_ogm") return 1;
    return 0;
}

bool HasNan(const tf2::Vector3& v) {
    return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
}

uint8_t ConvertCell(int8_t v, bool& malformed) {
    if (v == -1) return kUnknownCell;
    if (v >= 0 && v <= 100) return static_cast<uint8_t>(v);
    malformed = true;
    return kUnknownCell;
}

uint8_t ConvertCostmapCell(int8_t v) {
    const auto cost = static_cast<uint8_t>(v);
    if (cost == 255) return kUnknownCell;
    return static_cast<uint8_t>(std::lround(cost * 100.0 / 254.0));
}

uint8_t ConvertCellFor(const std::string& encoding, int8_t v, bool& malformed) {
    return encoding == "costmap" ? ConvertCostmapCell(v) : ConvertCell(v, malformed);
}

}

OgmAdapter::OgmAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), kind_(KindFromRole(row.role)) {}

void OgmAdapter::ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec) {
    ++stats_.msgs;

    if (msg.info.width == 0 || msg.info.height == 0) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.info.width) * msg.info.height) {
        ++stats_.dropped_malformed;
        return;
    }

    tf2::Transform xform;
    std_msgs::msg::Header header = msg.header;
    if (!row_.frame_id.empty()) header.frame_id = row_.frame_id;
    if (!tf_.lookup(header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }
    const auto& p = msg.info.origin.position;
    const tf2::Vector3 originTf = xform * tf2::Vector3(p.x, p.y, p.z);
    if (HasNan(originTf)) {
        ++stats_.dropped_malformed;
        return;
    }

    std::vector<uint8_t> next(msg.data.size());
    bool malformed = false;
    for (size_t i = 0; i < msg.data.size(); ++i) {
        next[i] = ConvertCellFor(row_.encoding, msg.data[i], malformed);
    }
    if (malformed) ++stats_.dropped_malformed;

    cells_ = std::move(next);
    origin_ = {originTf.x(), originTf.y(), tf_.flatten_z() ? 0.0 : originTf.z()};
    const auto& q = msg.info.origin.orientation;
    const tf2::Quaternion gridInMap =
        xform.getRotation() * tf2::Quaternion(q.x, q.y, q.z, q.w).normalized();
    const tf2::Vector3 xAxis = tf2::Transform(gridInMap) * tf2::Vector3(1.0, 0.0, 0.0);
    yaw_rad_ = std::atan2(xAxis.y(), xAxis.x());
    resolution_m_ = msg.info.resolution;
    width_cells_ = msg.info.width;
    height_cells_ = msg.info.height;
    has_grid_ = true;
    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void OgmAdapter::ingest_update(const map_msgs::msg::OccupancyGridUpdate& msg, double sim_time_sec) {
    ++stats_.msgs;

    if (!has_grid_) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.width == 0 || msg.height == 0) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.width) * msg.height) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.x < 0 || msg.y < 0) {
        ++stats_.dropped_malformed;
        return;
    }
    const uint32_t x0 = static_cast<uint32_t>(msg.x);
    const uint32_t y0 = static_cast<uint32_t>(msg.y);
    if (x0 > width_cells_ || y0 > height_cells_) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.width > width_cells_ - x0 || msg.height > height_cells_ - y0) {
        ++stats_.dropped_malformed;
        return;
    }

    bool malformed = false;
    for (uint32_t row = 0; row < msg.height; ++row) {
        const uint32_t destRowStart = (y0 + row) * width_cells_ + x0;
        const uint32_t srcRowStart = row * msg.width;
        for (uint32_t col = 0; col < msg.width; ++col) {
            cells_[destRowStart + col] =
                ConvertCellFor(row_.encoding, msg.data[srcRowStart + col], malformed);
        }
    }
    if (malformed) ++stats_.dropped_malformed;

    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void OgmAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (!has_grid_) return;

    overlume::GroundGridLayer g{};
    g.kind = kind_;
    g.origin = origin_;
    g.yaw_rad = yaw_rad_;
    g.resolution_m = resolution_m_;
    g.width_cells = width_cells_;
    g.height_cells = height_cells_;
    g.cells = cells_.data();
    g.last_update_sec = last_update_sec_;
    out.grids.push_back(g);
}

}
