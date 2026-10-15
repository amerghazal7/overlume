// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r);

uint64_t environment_scene_membership_count(overlume::VisualRenderer* r);

struct FixtureStreamHandle;

bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original = false,
                                      bool follow_terrain = false,
                                      std::optional<double> radius_override_m = std::nullopt);

FixtureStreamHandle* install_fixture_streaming_source_with_fallback(overlume::VisualRenderer* r,
                                                                    const char* fixture_dir,
                                                                    const char* fallback_baked_dir,
                                                                    overlume::GeoAnchor anchor,
                                                                    bool follow_terrain = false);

void kill_fixture_network(FixtureStreamHandle* handle);

void revive_fixture_network(FixtureStreamHandle* handle);

// Make the fixture network reject every tile request with a transport error (no response);
// tileset.json is still served, as with kill_fixture_network.
void break_fixture_network_transport(FixtureStreamHandle* handle);

// A custom-loader source over a 4 x 4 grid of 1 km cells centred on the anchor: column c runs west
// to east, row r north to south, and the anchor is the corner the four central cells share.
struct FixtureGridHandle;
FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection);
// Leaf tiles cesium asked the grid loader to load, as (column, row), in request order.
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle);

bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                       double origin_height_m, double lat_deg, double lon_deg, double alt_m,
                       double* out_x, double* out_y, double* out_z);

bool ecef_height_correction_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                                  double origin_height_m, double lat_deg, double lon_deg,
                                  double alt_m, double* out_z_uncorrected, double* out_z_corrected);

bool environment_stream_materials_original(overlume::VisualRenderer* r);
// NaN when the source is not a StreamingEnvironmentSource.
double environment_stream_brightness(overlume::VisualRenderer* r);
// Whether the open streaming source's tileset reads through the caching asset accessor (a SQLite
// cache database). False when no streaming source is open.
bool environment_stream_has_disk_cache(overlume::VisualRenderer* r);

bool environment_stream_parse_materials_original(const char* ion_spec);

bool environment_stream_parse_follow_terrain(const char* ion_spec, bool* out_parse_ok);

bool environment_stream_parse_replaces_ground(const char* ion_spec, bool* out_parse_ok);

double environment_stream_parse_max_tilt_deg(const char* ion_spec, bool* out_parse_ok);

double environment_stream_parse_brightness(const char* ion_spec, bool* out_parse_ok);

double environment_stream_parse_radius(const char* ion_spec, bool* out_parse_ok);

bool environment_stream_parse_has_radius(const char* ion_spec);

double terrain_ground_offset_probe(double sampled_height_m, double anchor_height_m,
                                   double ground_bias_m, double current_offset_m,
                                   float delta_seconds, bool snap);

void terrain_plane_fit_probe(const double* s, const double* h, int n, double* out_slope,
                             double* out_intercept, double* out_rms);

double terrain_transform_probe(double slope, double intercept, double anchor_height_m,
                               double ground_bias_m, double max_tilt_rad, double heading_rad,
                               double pivot_x, double pivot_y, double s);

bool environment_stream_first_primitive_is_clay(overlume::VisualRenderer* r);

double environment_terrain_offset_z(overlume::VisualRenderer* r);

double environment_terrain_first_tile_world_z(overlume::VisualRenderer* r);

int environment_stream_last_view_frustum_count(overlume::VisualRenderer* r);

bool environment_stream_force_fall_back(overlume::VisualRenderer* r);

std::string captured_cesium_log_text();

bool drive_ion_token_redaction_probe(const char* bogus_token, int64_t asset_id, int max_ticks);

bool environment_stream_provides_ground(overlume::VisualRenderer* r);

bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r);

// The raster-overlay callbacks' counters of the open streaming source's renderer resources;
// zeros when no streaming source is open (and always for content resources, which never attach).
struct OverlayProbe {
    uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed;
};
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);

// Removes every raster overlay from the live streaming source's tileset, which detaches the
// mapped rasters of live tiles; false when no streaming source is live.
bool environment_stream_remove_overlays(overlume::VisualRenderer* r);

// Opens `fixture_dir`'s tileset.json (the file-based fixture tileset, as
// install_fixture_streaming_source) as a terrain tileset at the fixture tests' anchor
// (25.1972 N, 55.2744 E, heading 0, height 0), without a disk cache, under a
// UrlTemplateRasterOverlay on `file://<imagery_dir>/{z}/{x}/{y}.png` capped at `maximum_level`
// (the fixture's leaves ask for level 17). cesium counts {y} from the south edge of the
// web-mercator square.
bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir,
                                               const char* imagery_dir, double brightness = 1.0,
                                               uint32_t maximum_level = 10);

}
