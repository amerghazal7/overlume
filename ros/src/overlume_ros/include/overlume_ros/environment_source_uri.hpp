// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file environment_source_uri.hpp
 *  @brief Composes the ONE source_uri string set_environment_source()
 *  dispatches on. Pulled out of on_activate()'s anonymous-namespace-free
 *  inline logic so test_environment_source_uri.cpp can exercise it
 *  directly -- the VM-064 gate round 1 finding: a non-empty
 *  environment_tile_cache_dir used to be appended unconditionally, so
 *  composing it against the google preset's "...&cache=off" silently
 *  produced a SECOND "cache=" key. parse_ion_spec()'s key loop
 *  (environment_stream.cpp) is last-wins with no first-wins guard, so the
 *  dir would overwrite "off" and defeat Decision 14 Step 2(b)'s compliance
 *  lever -- exactly the kind of TU-local bug ego_anchor.hpp's header
 *  comment already names as uncatchable without a directly-testable
 *  function.
 */

#include <locale>
#include <sstream>
#include <string>

namespace overlume::ros {

// Pure function: VM-063 Decision 5's "<uri>[?cache=<dir>][&fallback=<dir>]"
// composition, plus 2026-09-21's "[&follow_terrain=on|off]" ("option 2").
// environment_source_uri empty -> environment_chunks_dir verbatim (today's
// pre-VM-063 behavior, and never gets a follow_terrain= key -- a plain
// baked-chunk directory URI has no query string at all). Non-empty -> the
// cache=/fallback=/follow_terrain= keys are appended ONLY when the
// configured URI does not already carry that key -- an inline key in
// environment_source_uri always wins over the separate param, never
// silently overwritten. `follow_terrain` defaults false only for this
// function's own pinned-string unit tests; the node's default is TRUE
// (environment_follow_terrain), so every node call site passes
// environment_follow_terrain_ explicitly -- never rely on the default there.
// Mirrors IonSpec::ground_bias_m's default (environment_stream.cpp).
inline constexpr double kGroundBiasDefaultM = 0.3;
// Mirrors IonSpec::replaces_ground's default (environment_stream.cpp):
// 2026-09-21 live finding, the clay ground plane comes out of the scene
// once a streamed tileset PROVES it has ground under the ego -- true keeps
// that behaviour, false is the escape hatch that keeps the plane always.
inline constexpr bool kReplacesGroundDefault = true;
// Mirrors IonSpec::max_tilt_deg's default (environment_stream.cpp,
// kTerrainMaxTiltRad in degrees): degrees the multi-point terrain fit's
// tilt is clamped to; 0 is the offset-only escape hatch.
inline constexpr double kMaxTiltDegDefault = 2.0;
// Mirrors IonSpec::brightness's default (environment_stream.cpp): 1.0 is a
// no-op gain on the streamed materials' own baseColorFactor.
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
        // ground_bias= is appended only when it differs from the library's own
        // default (0.3 m): the default rides on the parser, so the composed
        // strings pinned by test_environment_source_uri.cpp stay unchanged.
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("ground_bias=") == std::string::npos &&
            ground_bias_m != kGroundBiasDefaultM) {
            std::ostringstream v;
            // Classic locale: a non-C LC_NUMERIC would otherwise compose
            // "ground_bias=0,5" (review minor, 2026-09-21).
            v.imbue(std::locale::classic());
            v << ground_bias_m;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "ground_bias=" + v.str();
        }
        // replaces_ground= is appended only when it differs from the
        // library's own default (true): same "only when it differs from
        // default" shape ground_bias= uses immediately above, so the
        // composed strings pinned by test_environment_source_uri.cpp stay
        // unchanged for every existing call site.
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("replaces_ground=") == std::string::npos &&
            !replaces_ground) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "replaces_ground=off";
        }
        // max_tilt_deg= is appended only when it differs from the
        // library's own default (2.0) -- same "only when it differs from
        // default" shape as ground_bias=/replaces_ground= above, so every
        // pre-existing pinned string in test_environment_source_uri.cpp
        // (all omitting this 7th param) stays unchanged.
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("max_tilt_deg=") == std::string::npos &&
            max_tilt_deg != kMaxTiltDegDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());  // same locale-safety as ground_bias= above
            v << max_tilt_deg;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "max_tilt_deg=" + v.str();
        }
        // brightness= is appended only when it differs from the library's
        // own default (1.0) -- same "only when it differs from default"
        // shape as ground_bias=/max_tilt_deg= above, so every pre-existing
        // pinned string in test_environment_source_uri.cpp (all omitting
        // this 8th param) stays unchanged.
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("brightness=") == std::string::npos &&
            brightness != kBrightnessDefault) {
            std::ostringstream v;
            v.imbue(std::locale::classic());  // same locale-safety as ground_bias= above
            v << brightness;
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "brightness=" + v.str();
        }
    }
    return source_uri;
}

// Pure function: the `fallback=<dir>` value carried by a COMPOSED source_uri
// (the output of compose_environment_source_uri()), or "" when none. Used by
// the STREAMING_FALLBACK WARN to name the dir actually in effect -- an
// inline fallback= on environment_source_uri wins over environment_chunks_dir
// there too, so the WARN must read the composed string, not the dir param.
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
