// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include <cmath>
#include <exception>

namespace overlume::mapbox {

namespace {

// Fully consumed, finite and > 0, as parse_ion_spec reads `radius` and `brightness`.
bool parse_positive(const std::string& val, double& out) {
    try {
        size_t used = 0;
        const double v = std::stod(val, &used);
        if (used != val.size() || !std::isfinite(v) || v <= 0.0) return false;
        out = v;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}

MapboxSpecResult parse_mapbox_spec(const std::string& spec) {
    const size_t q = spec.find('?');
    const std::string layers = spec.substr(0, q);
    if (layers != "buildings") {
        return {std::nullopt,
                "mapbox:// supports only the layer list 'buildings', not '" + layers + "'", ""};
    }
    MapboxSpec out;
    std::string warning;
    const std::string rest = q == std::string::npos ? std::string() : spec.substr(q + 1);
    size_t pos = 0;
    while (pos < rest.size()) {
        const size_t amp = rest.find('&', pos);
        const std::string kv = rest.substr(pos, amp == std::string::npos ? amp : amp - pos);
        const size_t eq = kv.find('=');
        if (eq != std::string::npos) {
            const std::string key = kv.substr(0, eq);
            const std::string val = kv.substr(eq + 1);
            // Every other key, the ion-only ones included, is ignored.
            if (key == "materials") {
                out.materials_original = val == "original";
                if (val != "original" && val != "clay")
                    warning = "mapbox:// materials='" + val + "' is unknown, using clay";
            } else if (key == "radius" || key == "brightness") {
                if (!parse_positive(val, key == "radius" ? out.radius_m : out.brightness))
                    return {std::nullopt,
                            "mapbox:// " + key + "='" + val + "' is not a finite number > 0", ""};
            } else if (key == "fallback") {
                // Empty means no fallback, as when absent. It also keeps the node's composer from
                // appending its own, since that skips any URI already naming `fallback=`.
                out.fallback_dir = val;
            } else if (key == "cache" && val != "off") {
                return {std::nullopt,
                        "mapbox:// cache='" + val +
                            "' refused: Mapbox tiles are never written to disk (only cache=off)",
                        ""};
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return {out, "", warning};
}

}
