// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "theme.hpp"
#include "overlume/api.h"

namespace overlume::testing {

double rendered_bounding_box_diagonal(overlume::VisualRenderer* r);

overlume::detail::Float3 ego_material_base_color(overlume::VisualRenderer* r);

}  // namespace overlume::testing
