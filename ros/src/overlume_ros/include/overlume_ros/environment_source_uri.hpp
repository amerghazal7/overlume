// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <locale>
#include <sstream>
#include <string>

namespace overlume::ros {

inline constexpr double kGroundBiasDefaultM = 0.3;
inline constexpr bool kReplacesGroundDefault = true;
inline constexpr double kMaxTiltDegDefault = 2.0;
inline constexpr double kBrightnessDefault = 1.0;

inline std::string compose_environment_source_uri(
    const std::string& environment_chunks_dir, const std::string& environment_source_uri,
    const std::string& environment_tile_cache_dir, bool follow_terrain = false,
    double ground_bias_m = kGroundBiasDefaultM, bool replaces_ground = kReplacesGroundDefault,
    double max_tilt_deg = kMaxTiltDegDefault, double brightness = kBrightnessDefault) {
    std::string source_uri = environment_chunks_dir;
    if (!environment_source_uri.empty()) {
        source_uri = environment_source_uri;
        if (!environment_tile_cache_dir.empty() &&
            environment_source_uri.find("cache=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "cache=" + environment_tile_cache_dir;
        }
        if (!environment_chunks_dir.empty() &&
            environment_source_uri.find("fallback=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "fallback=" + environment_chunks_dir;
        }
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("follow_terrain=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += follow_terrain ? "follow_terrain=on" : "follow_terrain=off";
        }
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("ground_bias=") == std::string::npos &&
            ground_bias_m != kGroundBiasDefaultM) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << ground_bias_m;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "ground_bias=" + v.str();
        }
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("replaces_ground=") == std::string::npos &&
            !replaces_ground) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "replaces_ground=off";
        }
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("max_tilt_deg=") == std::string::npos &&
            max_tilt_deg != kMaxTiltDegDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << max_tilt_deg;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "max_tilt_deg=" + v.str();
        }
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("brightness=") == std::string::npos &&
            brightness != kBrightnessDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << brightness;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "brightness=" + v.str();
        }
    }
    return source_uri;
}

inline std::string fallback_dir_from_source_uri(const std::string& composed_source_uri) {
    static constexpr char kKey[] = "fallback=";
    const auto key_pos = composed_source_uri.find(kKey);
    if (key_pos == std::string::npos) return {};
    const auto value_start = key_pos + sizeof(kKey) - 1;
    const auto value_end = composed_source_uri.find('&', value_start);
    return composed_source_uri.substr(
        value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
}

}  // namespace overlume::ros
