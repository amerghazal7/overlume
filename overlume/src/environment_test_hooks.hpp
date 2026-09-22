// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <string>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r);

uint64_t environment_scene_membership_count(overlume::VisualRenderer* r);

struct FixtureStreamHandle;

bool install_fixture_streaming_source(overlume::VisualRenderer* r, const char* fixture_dir,
                                      overlume::GeoAnchor anchor, bool materials_original = false,
                                      bool follow_terrain = false);

FixtureStreamHandle* install_fixture_streaming_source_with_fallback(overlume::VisualRenderer* r,
                                                                    const char* fixture_dir,
                                                                    const char* fallback_baked_dir,
                                                                    overlume::GeoAnchor anchor,
                                                                    bool follow_terrain = false);

void kill_fixture_network(FixtureStreamHandle* handle);

void revive_fixture_network(FixtureStreamHandle* handle);

bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                       double origin_height_m, double lat_deg, double lon_deg, double alt_m,
                       double* out_x, double* out_y, double* out_z);

bool ecef_height_correction_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad,
                                  double origin_height_m, double lat_deg, double lon_deg,
                                  double alt_m, double* out_z_uncorrected, double* out_z_corrected);

bool environment_stream_materials_original(overlume::VisualRenderer* r);

bool environment_stream_parse_materials_original(const char* ion_spec);

bool environment_stream_parse_follow_terrain(const char* ion_spec, bool* out_parse_ok);

bool environment_stream_parse_replaces_ground(const char* ion_spec, bool* out_parse_ok);

double environment_stream_parse_max_tilt_deg(const char* ion_spec, bool* out_parse_ok);

double environment_stream_parse_brightness(const char* ion_spec, bool* out_parse_ok);

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

}  // namespace overlume::testing
