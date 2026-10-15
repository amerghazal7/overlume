// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <algorithm>
#include <locale>
#include <sstream>
#include <string>

#include "overlume/scene.h"

namespace overlume::ros {

inline constexpr double kGroundBiasDefaultM = 0.3;
inline constexpr bool kReplacesGroundDefault = true;
inline constexpr double kMaxTiltDegDefault = 2.0;
inline constexpr double kBrightnessDefault = 1.0;
inline constexpr double kTileRadiusDefaultM = 700.0;

inline bool is_ion_source_uri(const std::string& uri) { return uri.rfind("ion://", 0) == 0; }

inline bool is_mapbox_source_uri(const std::string& uri) { return uri.rfind("mapbox://", 0) == 0; }

// True when the query carries `key=` as a key of its own: follow_terrain= is not terrain=.
inline bool has_query_key(const std::string& uri, const std::string& key) {
    return uri.find("?" + key + "=") != std::string::npos ||
           uri.find("&" + key + "=") != std::string::npos;
}

// What a value of environment_terrain or environment_imagery gives a source of one scheme, ""
// for nothing: "disable" gives off on both schemes, "ion:<digits>" gives <digits> on ion://, and
// `mapbox_value` ("enable" for terrain, "satellite" for imagery) gives itself on mapbox://.
inline std::string environment_layer_value(const std::string& value, bool ion,
                                           const std::string& mapbox_value) {
    if (value == "disable") return "off";
    if (!ion) return value == mapbox_value ? value : "";
    const std::string id = value.rfind("ion:", 0) == 0 ? value.substr(4) : "";
    return !id.empty() && id.find_first_not_of("0123456789") == std::string::npos ? id : "";
}

// The WARN line the node logs at configure when the composer ignores a layer parameter's value
// for the configured source: a value for the other scheme, or one that fits neither (a source
// that is neither mapbox:// nor ion:// warns only then). "" when nothing is ignored.
inline std::string environment_layer_warning(const std::string& name, const std::string& value,
                                             const std::string& mapbox_value,
                                             const std::string& source_uri) {
    const bool fits_ion = !environment_layer_value(value, true, mapbox_value).empty();
    const bool fits_mapbox = !environment_layer_value(value, false, mapbox_value).empty();
    const bool fits = is_ion_source_uri(source_uri)      ? fits_ion
                      : is_mapbox_source_uri(source_uri) ? fits_mapbox
                                                         : fits_ion || fits_mapbox;
    if (value.empty() || fits) return "";
    return name + " '" + value + "' is ignored for environment source '" + source_uri +
           "' (accepted: \"\", disable, " + mapbox_value +
           " for mapbox://, ion:<asset id> for ion://)";
}

inline std::string compose_environment_source_uri(
    const std::string& environment_chunks_dir, const std::string& environment_source_uri,
    const std::string& environment_tile_cache_dir, bool follow_terrain = false,
    double ground_bias_m = kGroundBiasDefaultM, bool replaces_ground = kReplacesGroundDefault,
    double max_tilt_deg = kMaxTiltDegDefault, double brightness = kBrightnessDefault,
    double tile_radius_m = kTileRadiusDefaultM, const std::string& environment_terrain = "",
    const std::string& environment_imagery = "") {
    std::string source_uri = environment_chunks_dir;
    if (!environment_source_uri.empty()) {
        const bool ion = is_ion_source_uri(environment_source_uri);
        // Mapbox tiles never reach disk (Product Terms), so never cache=; the mapbox:// parser
        // takes none of the ground keys, so they stay ion-only.
        const bool mapbox = is_mapbox_source_uri(environment_source_uri);
        source_uri = environment_source_uri;
        // mapbox:// carries terrain in its layer list, not as a key.
        if (mapbox && environment_layer_value(environment_terrain, false, "enable") == "enable") {
            const size_t list_end = std::min(source_uri.find('?'), source_uri.size());
            const size_t list_start = sizeof("mapbox://") - 1;
            const std::string list =
                "," + source_uri.substr(list_start, list_end - list_start) + ",";
            if (list.find(",terrain,") == std::string::npos) {
                source_uri.insert(list_end, ",terrain");
            }
        }
        if (!mapbox && !environment_tile_cache_dir.empty() &&
            environment_source_uri.find("cache=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "cache=" + environment_tile_cache_dir;
        }
        if (!environment_chunks_dir.empty() &&
            environment_source_uri.find("fallback=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "fallback=" + environment_chunks_dir;
        }
        if (ion && environment_source_uri.find("follow_terrain=") == std::string::npos) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += follow_terrain ? "follow_terrain=on" : "follow_terrain=off";
        }
        if (ion && environment_source_uri.find("ground_bias=") == std::string::npos &&
            ground_bias_m != kGroundBiasDefaultM) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << ground_bias_m;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "ground_bias=" + v.str();
        }
        if (ion && environment_source_uri.find("replaces_ground=") == std::string::npos &&
            !replaces_ground) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "replaces_ground=off";
        }
        if (ion && environment_source_uri.find("max_tilt_deg=") == std::string::npos &&
            max_tilt_deg != kMaxTiltDegDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << max_tilt_deg;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "max_tilt_deg=" + v.str();
        }
        if ((ion || mapbox) && environment_source_uri.find("brightness=") == std::string::npos &&
            brightness != kBrightnessDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << brightness;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "brightness=" + v.str();
        }
        if ((ion || mapbox) && environment_source_uri.find("radius=") == std::string::npos &&
            tile_radius_m != kTileRadiusDefaultM) {
            std::ostringstream v;
            v.imbue(std::locale::classic());
            v << tile_radius_m;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "radius=" + v.str();
        }
        const std::string terrain = environment_layer_value(environment_terrain, true, "enable");
        if (ion && !terrain.empty() && !has_query_key(environment_source_uri, "terrain")) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "terrain=" + terrain;
        }
        const std::string imagery = environment_layer_value(environment_imagery, ion, "satellite");
        if ((ion || mapbox) && !imagery.empty() &&
            !has_query_key(environment_source_uri, "imagery")) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "imagery=" + imagery;
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

// CREDITS: an ion:// source other than Google whose raster overlays carry credits (osm-terrain's
// Bing imagery); it owes those credits alone, with no logo.
enum class AttributionKind { NONE, GOOGLE, MAPBOX, CREDITS };

// Mapbox Product Terms 1.4 wording; the links become plain text on a video frame.
inline constexpr char kMapboxAttributionText[] =
    "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap Improve this map";

inline constexpr char kGoogleAttributionText[] = "3D Tiles data (c) Google";

// The attribution the frame owes. The scheme is checked first: the Google test is a substring
// match that mapbox://buildings?materials=original would pass too.
inline AttributionKind environment_attribution_kind(bool visible, bool enabled,
                                                    const std::string& source_uri,
                                                    overlume::EnvironmentSourceState state,
                                                    bool credits_non_empty = false) {
    if (!visible || !enabled || state != overlume::EnvironmentSourceState::STREAMING) {
        return AttributionKind::NONE;
    }
    if (is_mapbox_source_uri(source_uri)) return AttributionKind::MAPBOX;
    if (source_uri.find("materials=original") != std::string::npos) return AttributionKind::GOOGLE;
    if (credits_non_empty && is_ion_source_uri(source_uri)) return AttributionKind::CREDITS;
    return AttributionKind::NONE;
}

// The line for `kind`: its base text, then each overlay credit ("; " separated, as the library
// reports them) after a space, then Mapbox's "Improve this map". Empty: nothing to draw.
inline std::string environment_attribution_text(AttributionKind kind, const std::string& credits) {
    std::string each;
    for (size_t start = 0; start < credits.size();) {
        const size_t end = std::min(credits.find("; ", start), credits.size());
        if (end > start) each += " " + credits.substr(start, end - start);
        start = end + 2;
    }
    switch (kind) {
        case AttributionKind::MAPBOX:
            return "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap" + each + " Improve this map";
        case AttributionKind::GOOGLE:
            return kGoogleAttributionText + each;
        case AttributionKind::CREDITS:
            return each.empty() ? each : each.substr(1);
        case AttributionKind::NONE:
            break;
    }
    return "";
}

}
