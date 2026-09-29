// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

bool ground_grid_ramp_entry(overlume::VisualRenderer* r, uint8_t kind, uint32_t value,
                            overlume::detail::Float3* color, float* alpha);

const void* ground_grid_texture_handle(overlume::VisualRenderer* r, size_t slot);

uint32_t ground_grid_texture_generation(overlume::VisualRenderer* r, size_t slot);

uint32_t ground_grid_texture_upload_count(overlume::VisualRenderer* r, size_t slot);

float ground_grid_material_alpha(overlume::VisualRenderer* r, uint8_t kind);

bool ground_grid_corner(overlume::VisualRenderer* r, size_t slot, size_t corner,
                        overlume::Vec3* out);

}
