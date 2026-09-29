// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "overlume/scene.h"

namespace overlume::ros {

enum class NsRender : uint8_t { kDrop = 0, kPolyline = 1, kPolygon = 2 };

struct NsRule {
    std::string prefix;
    NsRender render{NsRender::kDrop};
    overlume::MapKind kind{overlume::MapKind::OTHER};
};

struct ProfileRow {
    std::string topic;
    std::string type;
    std::string adapter;
    std::string role;
    std::string update_topic;

    double timeout_sec{2.0};
    double max_rate_hz{0.0};

    std::vector<NsRule> namespaces;
    NsRender ns_default{NsRender::kPolyline};

    bool transient_local{false};
    bool best_effort{false};

    bool junction_interior_boundaries{true};

    std::string color_mode{"auto"};
    uint32_t max_points{0};
    uint32_t stride{1};

    double min_z_m{std::numeric_limits<double>::quiet_NaN()};

    std::string frame_id;
    std::string encoding{"occupancy"};
};

struct Profile {
    std::string name;
    std::vector<ProfileRow> rows;
};

struct SubSpec {
    std::string topic;
    std::string type;
    bool best_effort;
    bool transient_local;
};

std::optional<Profile> load_profile(const std::string& path, std::vector<std::string>& errors);
std::optional<Profile> load_profile_string(std::string_view yaml, std::vector<std::string>& errors);

const ProfileRow* find_row(const Profile& profile, std::string_view topic);

NsRender classify(const ProfileRow& row, std::string_view ns);

const NsRule* match_rule(const ProfileRow& row, std::string_view ns);

std::vector<SubSpec> subscriptions_for(const ProfileRow& row);

}
