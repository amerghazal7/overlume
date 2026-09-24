// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

size_t trajectory_carpet_mesh_count(overlume::VisualRenderer* r, size_t slot);

size_t trajectory_carpet_vertex_count(overlume::VisualRenderer* r, size_t slot);

float trajectory_carpet_material_alpha(overlume::VisualRenderer* r);

uint32_t trajectory_carpet_vertex_rgba(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx);

float trajectory_carpet_vertex_z(overlume::VisualRenderer* r, size_t slot, size_t vertex_idx);

float trajectory_carpet_half_width_m(overlume::VisualRenderer* r, size_t slot);

float trajectory_carpet_vertex_fade_alpha(overlume::VisualRenderer* r, size_t slot,
                                          size_t vertex_idx);

uint64_t trajectory_carpet_rebuild_count(overlume::VisualRenderer* r);

bool trajectory_carpet_slot_first_point(overlume::VisualRenderer* r, size_t slot,
                                        overlume::Vec3* out);

}
