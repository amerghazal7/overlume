// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

overlume::detail::Float3 alert_severity_base_color(overlume::VisualRenderer* r, uint8_t severity);

size_t alert_slot_count(overlume::VisualRenderer* r);

struct AlertMaterialInfo {
    bool bound_to_severity_template = false;
    float alpha = 0.0f;
};
AlertMaterialInfo alert_slot_material_info(overlume::VisualRenderer* r, size_t slot);

}  // namespace overlume::testing
