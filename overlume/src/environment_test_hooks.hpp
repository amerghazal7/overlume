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
// `follow_terrain` (2026-09-21, "option 2"): defaults false (today's flat
// map frame, every pre-existing call site unaffected); true samples
// Google's own terrain height under the ego, same as the production
// ion:// URI's follow_terrain= key.
bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original = false,
                                      bool follow_terrain = false);

// Same, plus a baked fallback dir and a kill switch for Task 4's
// network-loss e2e. Returns the kill-switch handle; nullptr on failure.
// `follow_terrain`: see install_fixture_streaming_source() above.
FixtureStreamHandle* install_fixture_streaming_source_with_fallback(overlume::VisualRenderer* r,
                                                                    const char* fixture_dir,
                                                                    const char* fallback_baked_dir,
                                                                    overlume::GeoAnchor anchor,
                                                                    bool follow_terrain = false);

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
// `origin_height_m` (2026-09-21 finding, docs/status.md item 4): the
// anchor's own WGS84 ellipsoid height (GeoAnchor::origin_height_m); pass
// 0.0 to reproduce the previous (simulation-only-correct) behaviour.
bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                       double origin_height_m, double lat_deg, double lon_deg, double alt_m,
                       double* out_x, double* out_y, double* out_z);

// Open Follow-up 4 (docs/status.md item 4, streamed-tile ellipsoid-height
// sag): exercises the SAME correction formula
// strip_attributes_and_correct_heights() applies to every streamed vertex
// (environment_stream.cpp), from a plain C++17 test TU. For the WGS84 point
// (lat_deg, lon_deg, alt_m), writes through both out-pointers (each may be
// null): `out_z_uncorrected` is ecef_to_map_probe()'s own z (the rigid
// tangent-plane transform, i.e. the sag) and `out_z_corrected` is that same
// point's ellipsoid height minus `origin_height_m` (2026-09-21 finding: was
// hard-coded 0.0, only correct in simulation -- on the real robot the
// anchor sits ~1.7 m above the ellipsoid, see
// strip_attributes_and_correct_heights()'s own comment). A single call
// proves both that the sag exists (uncorrected) and that the fix removes it
// (corrected). Always returns true.
bool ecef_height_correction_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                                  double origin_height_m, double lat_deg, double lon_deg,
                                  double alt_m, double* out_z_uncorrected, double* out_z_corrected);

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

// 2026-09-21 terrain following ("option 2"): same shape/reasoning as
// environment_stream_parse_materials_original() above, exercising the real
// parser's follow_terrain= key instead. Unlike materials= (which degrades a
// bad value to clay rather than failing), an unrecognized follow_terrain=
// value fails parse_ion_spec() entirely -- `*out_parse_ok` reports that
// separately from the returned bool, so a caller can tell "absent -> false"
// apart from "malformed -> the whole spec failed to parse" (both would
// otherwise read as the same `false`). `out_parse_ok` may be null.
bool environment_stream_parse_follow_terrain(const char* ion_spec, bool* out_parse_ok);

// ponytail: pure-math probe (no live tileset/renderer needed), added to
// keep AGENTS.md's "runnable check that fails when reverted" honest for the
// smoothing/clamp formula even though this pass didn't build the
// height-sampling ground-fixture integration test (see the report this
// shipped with) -- upgrade to a real sampleHeightMostDetailed() e2e is the
// fast-follow once that fixture exists. Applies ONE smoothing step: `snap`
// true reproduces update_ground_offset()'s first-sample snap (returns the
// clamped target immediately); false applies one step of the first-order
// smoothing toward it from `current_offset_m` over `delta_seconds`.
double terrain_ground_offset_probe(double sampled_height_m, double anchor_height_m,
                                   double current_offset_m, float delta_seconds, bool snap);

// VM-064 Step 1: true iff the first currently-loaded tile's first
// renderable's first primitive is bound to r->buildingMaterial (the clay
// remap) -- false in original-materials mode. False on the same
// null/non-streaming conditions as the hook above, or if nothing has
// loaded yet.
bool environment_stream_first_primitive_is_clay(overlume::VisualRenderer* r);

// 2026-09-21 terrain following ("option 2"): the installed
// StreamingEnvironmentSource's own ground_offset_z() -- NaN if `r` is null,
// no source is installed, or the installed source is not a
// StreamingEnvironmentSource, same null-safety pattern as
// environment_stream_materials_original() above (that hook returns a
// meaningful `false` for "no source"; this one can't, since 0.0 is also a
// meaningful in-range offset, so it returns NaN instead).
double environment_terrain_offset_z(overlume::VisualRenderer* r);

// Gate round 1 finding 1: the REAL Filament read-back
// StreamingEnvironmentSource::first_tracked_tile_world_z() exposes (see its
// own declaration comment, environment_stream.hpp) -- unlike
// environment_terrain_offset_z() above, this fails when either the
// tm.setParent(assetRoot, terrainRoot) call or the
// renderResources_->set_ground_offset_z() call is deleted, not just when
// groundOffsetZ_'s own bookkeeping breaks. NaN on the same null/non-
// streaming/nothing-tracked-yet conditions as every other hook here.
double environment_terrain_first_tile_world_z(overlume::VisualRenderer* r);

// 2026-09-21 two-frustum tile selection (coarse-LOD finding, docs/status.md
// item 4 addendum): the installed StreamingEnvironmentSource's own
// last_view_frustum_count() -- 1 before the render camera has ever been
// positioned (synthetic view only), 2 once it has (synthetic + camera). -1
// if `r` is null, no source is installed, or the installed source is not a
// StreamingEnvironmentSource, same null-safety pattern as
// environment_stream_materials_original() above.
int environment_stream_last_view_frustum_count(overlume::VisualRenderer* r);

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
