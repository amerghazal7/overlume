// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

size_t generic_marker_slot_count(overlume::VisualRenderer* r);

uint32_t generic_marker_alloc_count(overlume::VisualRenderer* r);

uint32_t generic_marker_unknown_count(overlume::VisualRenderer* r);

bool generic_marker_mesh_is_fallback(overlume::VisualRenderer* r, size_t slot);

overlume::detail::Float3 generic_marker_tint(overlume::VisualRenderer* r, size_t slot);

struct GenericMarkerMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
GenericMarkerMaterialInfo generic_marker_material_info(overlume::VisualRenderer* r, size_t slot);

}  // namespace overlume::testing
