// environment_test_hooks.hpp — internal-only, not installed, not POD. Same
// reasoning as map_elements_test_hooks.hpp: tests/test_environment.cpp
// links only against `visual_renderer` and has no access to its PRIVATE
// Filament include dir, so it can't include environment.hpp directly.
#pragma once

#include <cstdint>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// Live chunk count BakedEnvironmentSource currently holds loaded (added to
// r->scene) -- proves distance culling gates LOADING, not merely drawing.
// 0 if `r` is null or no source is configured (set_environment_source()
// never called, or it failed) -- same null-`r` contract as
// map_element_rebuild_count().
uint64_t environment_loaded_chunk_count(mpviz::VisualRenderer* r);

// Epic 6 (VM-062) streaming test hooks. This header stays C++17-safe and
// cesium-free (included by test_environment_stream.cpp, a plain C++17 test
// TU with no cesium/Filament include dirs) -- EVERY function below is
// DEFINED in environment_stream.cpp, the single C++20 TU, per Decision 3's
// quarantine. See that file + the epic plan's Task 3 Interfaces block for
// the full reasoning (why an opaque handle, why construction+install is one
// call, why EnvironmentSource can never appear by value/reference here).
struct FixtureStreamHandle;  // opaque kill-switch handle; owned by the
                             // source it came from, valid until that
                             // source's teardown.

// Builds a streaming source served entirely from a committed fixture dir
// (Decision 13) and installs it on the renderer (teardown-first, replacing
// any current source) -- construction + install in ONE call, entirely
// inside the C++20 TU. No network, no token. false on failure (renderer
// left untouched).
bool install_fixture_streaming_source(mpviz::VisualRenderer* r, const char* fixture_dir,
                                       mpviz::GeoAnchor anchor);

// Same, plus a baked fallback dir and a kill switch for Task 4's
// network-loss e2e. Returns the kill-switch handle; nullptr on failure.
FixtureStreamHandle* install_fixture_streaming_source_with_fallback(
    mpviz::VisualRenderer* r, const char* fixture_dir, const char* fallback_baked_dir,
    mpviz::GeoAnchor anchor);

// Flips the kill switch: every subsequent fixture "network" request fails.
void kill_fixture_network(FixtureStreamHandle* handle);

// Task 3 Step 3 (geo-placement cross-pin): computes the streaming source's
// own ecef_to_map transform (Decision 8) for `anchor`, applies it to the
// WGS84 point (lat_deg, lon_deg, alt_m), and writes the resulting map-frame
// x/y/z through the three out-pointers (each may be null). Not part of the
// plan's literal Interfaces block -- added because Step 3 cannot be tested
// at all otherwise: the transform is built entirely from cesium/glm types
// that never cross the C++17/C++20 quarantine (Decision 3), so this is the
// minimal POD-only probe that exercises it from a plain test TU. Always
// returns true (kept bool, not void, for the same "hook reports success"
// shape as the other hooks here).
bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                        double lat_deg, double lon_deg, double alt_m, double* out_x, double* out_y,
                        double* out_z);

}  // namespace mpviz::testing
