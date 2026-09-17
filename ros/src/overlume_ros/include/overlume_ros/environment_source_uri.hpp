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

#include <string>

namespace overlume::ros {

// Pure function: VM-063 Decision 5's "<uri>[?cache=<dir>][&fallback=<dir>]"
// composition. environment_source_uri empty -> environment_chunks_dir
// verbatim (today's pre-VM-063 behavior). Non-empty -> the cache=/fallback=
// keys are appended ONLY when the configured URI does not already carry
// that key -- an inline cache= (or fallback=) in environment_source_uri
// always wins over the separate dir param, never silently overwritten.
inline std::string compose_environment_source_uri(const std::string& environment_chunks_dir,
                                                  const std::string& environment_source_uri,
                                                  const std::string& environment_tile_cache_dir) {
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
