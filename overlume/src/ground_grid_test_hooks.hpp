// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"

namespace overlume::testing {

overlume::detail::Float3 ground_grid_free_color(overlume::VisualRenderer* r);
overlume::detail::Float3 ground_grid_occupied_color(overlume::VisualRenderer* r);

const void* ground_grid_texture_handle(overlume::VisualRenderer* r, size_t slot);

uint32_t ground_grid_texture_generation(overlume::VisualRenderer* r, size_t slot);

uint32_t ground_grid_texture_upload_count(overlume::VisualRenderer* r, size_t slot);

float ground_grid_material_alpha(overlume::VisualRenderer* r, uint8_t kind);

}  // namespace overlume::testing
