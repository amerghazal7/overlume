// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <optional>
#include <string>

namespace overlume::mapbox {

inline constexpr double kDefaultRadiusM = 500.0;

struct MapboxSpec {
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;
    double brightness = 1.0;
    std::string fallback_dir;
};

struct MapboxSpecResult {
    std::optional<MapboxSpec> spec;  // nullopt: the URI is refused
    std::string error;               // when refused: one line naming the layer list or the key
    std::string warning;             // when accepted with a caveat (an unknown `materials` value)
};

// Parses the text after "mapbox://": the layer list (only "buildings"), then an optional query
// split at the first '?', pairs on '&', the key at the first '='. Unknown keys and pairs without
// '=' are ignored, the last value of a repeated key wins, and a bad value for a known key refuses
// the whole URI. Pure: no logging, no environment access.
MapboxSpecResult parse_mapbox_spec(const std::string& spec);

}
