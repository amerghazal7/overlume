// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

size_t height_grid_slot_count(overlume::VisualRenderer* r);

uint32_t height_grid_vertex_count(overlume::VisualRenderer* r, size_t slot);

uint32_t height_grid_index_count(overlume::VisualRenderer* r, size_t slot);

// World position and CUSTOM0 = (height_m, known) of cell (i, j). The position is the uploaded
// origin-relative position plus the translation Filament's TransformManager currently holds for
// the slot's entity (what was applied, not what the slot bookkeeping says).
bool height_grid_vertex(overlume::VisualRenderer* r, size_t slot, uint32_t i, uint32_t j,
                        float out_pos[3], float out_custom[2]);

uint32_t height_grid_upload_count(overlume::VisualRenderer* r, size_t slot);

// Derived from Filament state: 0 = entity not in the scene (or no renderable), 1 = the shared
// opaque instance is bound, 2 = the slot's own faded instance is bound, -1 = anything else.
int height_grid_material_state(overlume::VisualRenderer* r, size_t slot);

// "roughness" parameter read back from the slot's per-slot faded instance; -1.0 when the slot has
// no faded instance (opaque, removed, or out of range).
float height_grid_fade_roughness(overlume::VisualRenderer* r, size_t slot);

}
