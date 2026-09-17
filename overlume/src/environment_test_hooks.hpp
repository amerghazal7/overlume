// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// environment_test_hooks.hpp — internal-only, not installed, not POD. Same
// reasoning as map_elements_test_hooks.hpp: tests/test_environment.cpp
// links only against `overlume` and has no access to its PRIVATE
// Filament include dir, so it can't include environment.hpp directly.
#pragma once

#include <cstdint>
#include <string>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Live chunk count the installed EnvironmentSource (BakedEnvironmentSource
// or StreamingEnvironmentSource, Decision 12) currently holds loaded --
// proves distance/streaming culling gates LOADING, not merely drawing.
// Independent of visibility: a chunk/tile stays counted here whether it is
// actually in `r`'s Filament scene or not (VM-096 decoupled "loaded" from
// "in the scene" -- see environment_scene_membership_count() below for the
// latter). 0 if `r` is null or no source is configured (set_environment_source()
// never called, or it failed) -- same null-`r` contract as
// map_element_rebuild_count().
uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r);

// VM-096 (vcam GUI Environment Tiles toggle): how many of the
// currently-loaded chunks/tiles are ACTUALLY added to `r`'s Filament scene
// right now -- 0 immediately after set_environment_visible(r, false), back
// to environment_loaded_chunk_count(r)'s own value immediately after
// set_environment_visible(r, true), with NO change in
// environment_loaded_chunk_count(r) itself across either call (hiding
// tears down nothing). 0 if `r` is null or no source is configured, same
// null-safety class as the hook above. Deterministic and
// camera-framing-independent -- unlike a rendered-pixel comparison, it
// doesn't depend on how much screen area a loaded chunk/tile happens to
// cover.
uint64_t environment_scene_membership_count(overlume::VisualRenderer* r);

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
// left untouched). `materials_original` (VM-064, Task 5): defaults false
// (today's clay remap, every pre-VM-064 call site unaffected); true installs
// in original-materials mode (the Google Photorealistic 3D Tiles path).
bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original = false);

// Same, plus a baked fallback dir and a kill switch for Task 4's
// network-loss e2e. Returns the kill-switch handle; nullptr on failure.
FixtureStreamHandle* install_fixture_streaming_source_with_fallback(overlume::VisualRenderer* r,
                                                                    const char* fixture_dir,
                                                                    const char* fallback_baked_dir,
                                                                    overlume::GeoAnchor anchor);

// Flips the kill switch: every subsequent fixture "network" request fails.
void kill_fixture_network(FixtureStreamHandle* handle);

// Un-flips it: requests succeed again. Exists so the e2e can assert the
// one-way property Decision 11 commits to -- a source that has fallen back
// STAYS fallen back for the process's life, even once the "network" returns.
void revive_fixture_network(FixtureStreamHandle* handle);

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

// VM-064 (Task 5) Step 0: true iff the installed source is a
// StreamingEnvironmentSource running in original-materials mode. False if
// `r` is null, no source is installed, or the installed source is not a
// StreamingEnvironmentSource (e.g. BakedEnvironmentSource) -- same
// null-safety class as every other hook in this header.
bool environment_stream_materials_original(overlume::VisualRenderer* r);

// VM-064 gate round 1 finding: the production activation path -- parsing
// "materials=original" out of the ion:// query string -- had zero coverage
// through the real parser (both Step 0 tests reach materials_original via
// install_fixture_streaming_source(), which bypasses parse_ion_spec()
// entirely). Calls parse_ion_spec() directly on `ion_spec` and returns its
// materials_original flag; false if `ion_spec` fails to parse at all (no
// numeric asset id).
bool environment_stream_parse_materials_original(const char* ion_spec);

// VM-064 Step 1: true iff the first currently-loaded tile's first
// renderable's first primitive is bound to r->buildingMaterial (the clay
// remap) -- false in original-materials mode. False on the same
// null/non-streaming conditions as the hook above, or if nothing has
// loaded yet.
bool environment_stream_first_primitive_is_clay(overlume::VisualRenderer* r);

// Finding #0 (security, token redaction) test hooks -- environment_stream.cpp
// only, cesium-free signatures so this header stays includable from a plain
// C++17 test TU (Decision 3).

// Everything this library's own named cesium logger has ever emitted,
// POST-redaction (build_externals()'s RedactingSink) -- empty if no
// build_externals() call has happened yet in this process.
std::string captured_cesium_log_text();

// Drives the real ion-handshake error path (asset_id + access_token
// Tileset ctor) against a file-fixture accessor -- no network, no real
// token. The (bogus) token necessarily appears in the in-flight request
// URL; this hook exists to prove it never reaches this library's own log
// output. Pumps up to `max_ticks`; returns true once the captured log text
// actually contains "access_token=" (not merely once anything was logged --
// the capture is a process-wide singleton an earlier test may have written
// to), false if `max_ticks` elapse first.
bool drive_ion_token_redaction_probe(const char* bogus_token, int64_t asset_id, int max_ticks);

}  // namespace overlume::testing
