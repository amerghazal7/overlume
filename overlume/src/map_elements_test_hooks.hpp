// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

overlume::Vec3 ground_patch_centre(overlume::VisualRenderer* r);

overlume::detail::Float3 lane_material_base_color(overlume::VisualRenderer* r);

overlume::detail::Float3 map_kind_base_color(overlume::VisualRenderer* r, overlume::MapKind kind);

uint64_t map_element_rebuild_count(overlume::VisualRenderer* r);

size_t map_element_mesh_count(overlume::VisualRenderer* r);

size_t map_element_total_vertex_count(overlume::VisualRenderer* r);

struct MapElementMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
MapElementMaterialInfo map_element_material_info(overlume::VisualRenderer* r);

}
