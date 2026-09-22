// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

overlume::detail::Float3 ribbon_role_base_color(overlume::VisualRenderer* r,
                                                overlume::PathRole role);

size_t ribbon_mesh_count(overlume::VisualRenderer* r, size_t slot);

size_t ribbon_vertex_count(overlume::VisualRenderer* r, size_t slot);

struct RibbonMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
RibbonMaterialInfo ribbon_slot_material_info(overlume::VisualRenderer* r, size_t slot);

float ribbon_slot_half_width_m(overlume::VisualRenderer* r, size_t slot);

bool ribbon_slot_first_point(overlume::VisualRenderer* r, size_t slot, overlume::Vec3* out);

uint64_t ribbon_rebuild_count(overlume::VisualRenderer* r);

}
