// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

overlume::detail::Float3 object_class_tint(overlume::VisualRenderer* r, overlume::ObjectClass cls);

bool object_in_scene(overlume::VisualRenderer* r, uint32_t id);

uint64_t object_entity_identity(overlume::VisualRenderer* r, uint32_t id);

overlume::Vec3 object_transform_scale(overlume::VisualRenderer* r, uint32_t id);

struct ObjectMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
ObjectMaterialInfo object_material_info(overlume::VisualRenderer* r, uint32_t id);

}  // namespace overlume::testing
