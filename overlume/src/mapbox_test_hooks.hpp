// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
// anchor's ground, through the Mapbox placement chain (tile frame of its z14 tile, ECEF, the map
// frame of `anchor`, the stream's height correction) → map metres.
bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

}
