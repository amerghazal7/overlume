// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <optional>
#include <string>

namespace overlume::mapbox {

inline constexpr double kDefaultRadiusM = 500.0;

struct MapboxSpec {
    bool buildings = true;           // the layer list carries `buildings`
    bool terrain = false;            // the layer list carries `terrain`
    bool imagery_satellite = false;  // imagery=satellite (requires terrain)
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

// Parses the text after "mapbox://": the layer list, a comma list of `buildings` and `terrain`,
// each at most once, in any order (an empty, repeated or unknown name refuses the URI), then an
// optional query split at the first '?', pairs on '&', the key at the first '='. Unknown keys and
// pairs without '=' are ignored, the last value of a repeated key wins, and a bad value for a
// known key refuses the whole URI. imagery=satellite needs the `terrain` layer; imagery=off is the
// default. Every refusal's error starts with "mapbox://". Pure: no logging, no environment access.
MapboxSpecResult parse_mapbox_spec(const std::string& spec);

}
